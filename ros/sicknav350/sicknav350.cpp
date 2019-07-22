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

void OdometryCallback(const nav_msgs::Odometry::CostPtr& msg){
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
		scann_msg.time_increment = scan_msg.scan_time / n_range_values;
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



int main(int argc, char *argv[]) {

	ros::init(argc, argv, "sicknav350");
	int port;
	std::string ipaddress;
	ros::NodeHandle nh;
	ros::NodeHandle nh_ns("~");
	ros::Rate loop_rate(8);
	ros::Publisher scan_pub = nh.advertise<sensor_msgs::LaserScan>(scan, 1);
	nh_ns.param("port", port, DEFAULT_SICK_TCP_PORT);
	nh_ns.param("ipaddress", ipaddress, (std::string)DEFAULT_SICK_IP_ADDRESS);
	/* Define buffers for return values */

	double range_values[SickToolbox::SickNav350::SICK_MAX_NUM_MEASUREMENTS] = {0};
	unsigned int intensity_values[SickToolbox::SickNav350::SICK_MAX_NUM_MEASUREMENTS] = {0};

	/* Instantiate the object */
	SickToolbox::SickNav350 sick_nav350(ipaddress.c_str(),port);

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
			sick_nav350.SetReflectorWindow(500,500,500,60000);
			sick_nav350.SetReflectorSize(80);
			sick_nav350.SetReflectorType((int)ReflectorTypes::FLAT);

			/**
			 * SetLandmarkMatching
			 * /param Filter {int} - 0 Optimal
			 *						 1 Optimal + angle
			 * 						 2 Optimal + angle + partially covered
			 */
			sick_nav350.SetLandmarkMatching(1);
			sick_nav350.SetActionRadius(400, 70000);
			sick_nav350.SetOperatingMode((int)OperatingModes::NAVIGATION);
			sick_nav350.SetPose(0,0,0);

			//sick_nav350.SetScanDataFormat(1, 1);
		} catch (...) {
			ROS_ERROR("Configuration error");
			return -1;
		}


		while (ros::ok()) {

			sick_nav350.GetDataNavigation(1,2);
			std::cout << "\n=================================" << std::endl;
			std::cout << "Pose X : " << sick_nav350.PoseData_.x << std::endl;
			std::cout << "Pose Y : " << sick_nav350.PoseData_.y << std::endl;
			std::cout << "Pose phi : " << sick_nav350.PoseData_.phi << std::endl;
			std::cout << "Landmark Data follow : " << sick_nav350.PoseData_.optionalLandmarkData << std::endl;
			std::cout << "Num Reflector : " << sick_nav350.ReflectorData_.num_reflector << std::endl;
			for(int i = 0; i < sick_nav350.ReflectorData_.num_reflector;i++){
				if(sick_nav350.ReflectorData_.cart[i]!=0){
					std::cout << "Reflector " << i << std::endl;
					std::cout << "X : " << sick_nav350.ReflectorData_.x[i] << std::endl;
					std::cout << "Y : " << sick_nav350.ReflectorData_.y[i] << std::endl;
				}else if(sick_nav350.ReflectorData_.polar[i]!=0){
					std::cout << "Reflector " << i << std::endl;
					std::cout << "Dist : " << sick_nav350.ReflectorData_.dist[i] << std::endl;
					std::cout << "Phi : " << sick_nav350.ReflectorData_.phi[i] << std::endl;
				}
			}
			std::cout << "=================================\n" << std::endl;

			sick_nav350.GetSickMeasurementsWithRemission(range_values, intensity_values,
				&num_measurements,
				&sector_step_angle,
				&sector_start_angle,
				&sector_stop_angle,
				&sector_start_timestamp,
				&sector_stop_timestamp
			);

			publish_scan(&scan_pub, range_values, num_measurements, intensity_values,
				num_measurements, start_scan_time, scan_duration, sector_start_angle, sector_stop_angle);

			sick_nav350.SetSpeed(vx,vy,vth, sector_start_timestamp,0);
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
