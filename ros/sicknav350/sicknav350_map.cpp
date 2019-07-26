/*
 * Authors: Nick Hillier and Fred Pauling (CSIRO, 2011)
 *
 * Based on the sicklms.cpp from the sicktoolbox_wrapper ROS package
 * and the sample code from the sicktoolbox manual.
 *
 * Released under BSD license.
 */

#include <iostream>
#include <sicktoolbox/SickNAV350.hh>
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <deque>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include "sensor_msgs/LaserScan.h"
#define DEG2RAD(x) ((x)*M_PI/180.)

double vx = 0.0;
double vy = 0.0;
double vth = 0.0;

namespace OperatingModes{
	enum OperatingMode{
		POWERDOWN = 0,
		STANDBY = 1,
		MAPPING = 2,
		LANDMARK = 3,
		NAVIGATION = 4,
	};
}
typedef OperatingModes::OperatingMode OperatingMode;

namespace ReflectorTypes{
	enum ReflectorType{
		FLAT = 1,
		CYLINDRICAL = 2,
	};
}
typedef ReflectorTypes::ReflectorType ReflectorType;

namespace Users{
	enum User{
		OPERATOR = 2,
		AUTHORIZED_CLIENT = 3,
	};
}
typedef Users::User User;

void OdometryCallback(const nav_msgs::Odometry::ConstPtr& msg){
	vx=msg->twist.twist.linear.x;
	vy=msg->twist.twist.linear.y;
	vth=msg->twist.twist.angular.z;
}

void publish_scan(ros::Publisher *pub, double *range_values,
	uint32_t n_range_values, unsigned int *intensity_values,
	ros::Time start, double scan_time, float angle_min, float angle_max){


		sensor_msgs::LaserScan scan_msg;

		scan_msg.header.frame_id = "laser";
		scan_msg.angle_min = angle_min;
		scan_msg.angle_max = angle_max;
		scan_msg.angle_increment = (scan_msg.angle_max - scan_msg.angle_min) / (double)(n_range_values - 1);
		scan_msg.scan_time = 0.125;
		scan_msg.time_increment = scan_msg.scan_time / n_range_values;
		scan_msg.range_min = 0.1;
		scan_msg.range_max = 250.;
		scan_msg.ranges.resize(n_range_values);
		scan_msg.header.stamp = start;
		for(size_t i = 0; i < n_range_values; i++){
			scan_msg.ranges[i] = (float(range_values[i]/1000));
		}
		scan_msg.intensities.resize(n_range_values);
		for(size_t i = 0; i < n_range_values; i++){
			scan_msg.intensities[i] = (float)intensity_values[i];
		}
		pub->publish(scan_msg);
	}

// A complimentary filter to get a (much) better time estimate, does not
// calibrate out constant network latency delays, but does get rid of
// timming jitter to get better timing estimates than the
// communicated clock resolution (which is only 1ms)
class smoothtime {
protected:
	ros::Time smoothtime_prev, smoothed_timestamp;
	double time_smoothing_factor;
	double error_threshold;
	public:
	smoothtime(){
		time_smoothing_factor = 0.95; /// slowly skew the clocks into sync
		error_threshold = .50; /// 50% jitter is acceptable for , discard data otherwise.
	}
	//! Between 0 and 1, bigger is smoother
	void set_smoothing_factor(double smoothing_factor){
		time_smoothing_factor = smoothing_factor;
	}
	//! Between 0 and 1, threshold on jitter acceptability, higher accepts more jitter before discarding
	void set_error_threshold(double err_threshold){
		error_threshold = err_threshold;
	}
	ros::Time smooth_timestamp(ros::Time recv_timestamp, ros::Duration expctd_dur) {
		if (smoothtime_prev.is_zero() == true) {
			smoothed_timestamp = recv_timestamp;
		} else {
			smoothed_timestamp = smoothtime_prev + expctd_dur;
			double err = (recv_timestamp - smoothed_timestamp).toSec();
			double time_error_threshold = expctd_dur.toSec() * error_threshold;
			if ((time_smoothing_factor > 0) && (fabs(err) < time_error_threshold)){
				ros::Duration correction = ros::Duration(err * (1 - time_smoothing_factor));
				smoothed_timestamp += correction;
			} else {
				// error too high, or smoothing disabled - set smoothtime to last timestamp
				smoothed_timestamp = recv_timestamp;
			}
		}
		smoothtime_prev = smoothed_timestamp;
		return smoothed_timestamp;
	}
};

class averager {
protected:
	std::deque<double> deq;
	unsigned int max_len;
	double sum;
public:
	averager(int max_len = 50){
		this->max_len = max_len;
	}

	void add_new(double data) {
		deq.push_back(data);
		sum += data;
		if (deq.size() > max_len) {
		sum -= deq.front();
		deq.pop_front();
		}
	}

	double get_mean() {
		return sum/deq.size();
	}
};

int main(int argc, char *argv[]) {

	ros::init(argc, argv, "sicknav350");
	int port;
	std::string scan;
	std::string ipaddress;
	double sick_step_angle = 1.5;
	double active_sector_start_angle = 0;
	double active_sector_stop_angle = 360;
	double smoothing_factor, error_threshold;
	int sick_motor_speed = 8;
	ros::NodeHandle nh;
	ros::NodeHandle nh_ns("~");
	nh_ns.param<std::string>("scan", scan, "scan");
	ros::Publisher scan_pub = nh.advertise<sensor_msgs::LaserScan>(scan, 1);
	nh_ns.param("port", port, DEFAULT_SICK_TCP_PORT);
	nh_ns.param("ipaddress", ipaddress, (std::string)DEFAULT_SICK_IP_ADDRESS);
	ros::Rate loop_rate(8);
	int opmode = (int)OperatingModes::MAPPING;

	/* Define buffers for return values */

	double range_values[SickToolbox::SickNav350::SICK_MAX_NUM_MEASUREMENTS] = {0};
	unsigned int intensity_values[SickToolbox::SickNav350::SICK_MAX_NUM_MEASUREMENTS] = {0};

	/* Define buffers to hold sector specific data */
	unsigned int num_measurements = {0};
	unsigned int sector_start_timestamp = {0};
	unsigned int sector_stop_timestamp = {0};
	double sector_step_angle = {0};
	double sector_start_angle = {0};
	double sector_stop_angle = {0};

	/* Instantiate the object */
	SickToolbox::SickNav350 sick_nav350(ipaddress.c_str(),port);
	double last_time_stamp = 0;

	try {
		/* Initialize the device */
		sick_nav350.Initialize();
		sick_nav350.SetAccessMode((int)Users::AUTHORIZED_CLIENT);

		try {
			sick_nav350.GetSickIdentity();
			std::cout << "\n\t\tSick Device Identity" << std::endl;
			std::cout << "\t\tDevice Name : " << sick_nav350.GetSickName() << std::endl;
			std::cout << "\t\tDevice Version : " << sick_nav350.GetSickVersion() << std::endl;
			std::cout << "\t\tDevice Serial Number : " << sick_nav350.GetSickSerialNumber() << std::endl;
			std::cout << "\t\tDevice Firmware Version : " << sick_nav350.GetSickFirmwareVersion() << std::endl;
			std::cout << "\t\tDevice Version : " << sick_nav350.GetSickSoftwareVersion() << std::endl;

			//Change to standby mode for initialization

			sick_nav350.SetOperatingMode((int)OperatingModes::STANDBY);
			sick_nav350.SetCurrentLayer(1);

			/**
			 * SetPoseDataFormat
			 * /param OutputMode {int} - 0 Normal
			 * 				  			 1 Extrapolated
			 * /param ShowOptParam {int} - 0 Suppressed
			 *				  			   1 Enabled
			 */
			sick_nav350.SetPoseDataFormat(1,0);
			sick_nav350.SetReflectorWindow(500,500,500,70000);
			sick_nav350.SetActionRadius(400, 70000);
			sick_nav350.SetReflectorSize(80);
			sick_nav350.SetReflectorType((int)ReflectorTypes::FLAT);

			/**
			 * SetLandmarkMatching
			 * /param Filter {int} - 0 Optimal
			 *						 1 Optimal + angle
			 * 						 2 Optimal + angle + partially covered
			 */
			sick_nav350.SetLandmarkMatching(0);
			sick_nav350.SetMappingConfiguration(1, 1, 0,0,0);

			// Change to mapping mode for mapping
			sick_nav350.SetOperatingMode((int)OperatingModes::MAPPING);

		} catch (...) {
			ROS_ERROR("Configuration error");
			return -1;
		}

		while (ros::ok()) {

			sick_nav350.DoMapping();
			std::cout << "\n=================================" << std::endl;
			std::cout << "ERROR : " << sick_nav350.ReflectorData_.error << std::endl;
			if(sick_nav350.ReflectorData_.error == 0){
				std::cout << "Num Reflector : " << sick_nav350.ReflectorData_.num_reflector << std::endl;
				int temp[sick_nav350.ReflectorData_.num_reflector][7];
				for(int i = 0; i < sick_nav350.ReflectorData_.num_reflector; i++){
					if(sick_nav350.ReflectorData_.cart[i] != 0){
						if(sick_nav350.ReflectorData_.meanEchoAmplitude[i] < 1000){
							temp[i][0] = sick_nav350.ReflectorData_.x[i];
							temp[i][1] = sick_nav350.ReflectorData_.y[i];
							temp[i][2] = sick_nav350.ReflectorData_.type[i];
							temp[i][3] = sick_nav350.ReflectorData_.subtype[i];
							temp[i][4] = sick_nav350.ReflectorData_.size[i];
							temp[i][5] = 1;
							temp[i][6] = 1;
						}
					}
				}

				sick_nav350.AddLandmark(sick_nav350.ReflectorData_.num_reflector, temp);
				for(int i = 0; i < sick_nav350.ReflectorData_.num_reflector; i++){

					std::cout << "-------------------------------------" << std::endl;
					std::cout << "Reflector " << i << std::endl;
					std::cout << "X : " << temp[i][0] << std::endl;
					std::cout << "Y : " << temp[i][1] << std::endl;
					std::cout << "Type : " << temp[i][2] << std::endl;
					std::cout << "Subtype : " << temp[i][3] << std::endl;
					std::cout << "size : " << temp[i][4] << std::endl;
					std::cout << "Layer : " << temp[i][5] << std::endl;
					std::cout << "LayerID : " << temp[i][6] << std::endl;
				}
			}
			std::cout << "=================================\n" << std::endl;
			loop_rate.sleep();
			ros::spinOnce();

		}

		/* Uninitialize the device */
		sick_nav350.Uninitialize();
	}
	catch(...) {
		ROS_ERROR("Error");
		return -1;
	}
	return 0;
}
