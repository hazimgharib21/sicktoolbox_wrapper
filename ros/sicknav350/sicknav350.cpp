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
#define DEG2RAD(x) ((x)*M_PI/180.)

int main(int argc, char *argv[]) {

	ros::init(argc, argv, "sicknav350");
	int port;
	std::string ipaddress;
	std::string frame_id;
	std::string scan;
	bool inverted;
	int sick_motor_speed = 8;//10; // Hz
	double sick_step_angle = 1.5;//0.5;//0.25; // deg (0.125 = no gaps between spots)
	double active_sector_start_angle = 0;
	double active_sector_stop_angle = 360;//269.75;
	double smoothing_factor, error_threshold;
	ros::NodeHandle nh;
	ros::NodeHandle nh_ns("~");
	ros::Rate loop_rate(8);
	nh_ns.param<std::string>("scan", scan, "scan");
	ros::Publisher scan_pub = nh.advertise<sensor_msgs::LaserScan>(scan, 1);
	nh_ns.param("port", port, DEFAULT_SICK_TCP_PORT);
	nh_ns.param("ipaddress", ipaddress, (std::string)DEFAULT_SICK_IP_ADDRESS);
	nh_ns.param("inverted", inverted, false);
	nh_ns.param<std::string>("frame_id", frame_id, "laser");
	nh_ns.param("timer_smoothing_factor", smoothing_factor, 0.97);
	nh_ns.param("timer_error_threshold", error_threshold, 0.5);
	nh_ns.param("resolution", sick_step_angle, 1.0);
	nh_ns.param("start_angle", active_sector_start_angle, 0.);
	nh_ns.param("stop_angle", active_sector_stop_angle, 360.);
	nh_ns.param("scan_rate", sick_motor_speed, 5);

	/* Define buffers for return values */
	double range_values[SickToolbox::SickNav350::SICK_MAX_NUM_MEASUREMENTS] = {0};
	unsigned int intensity_values[SickToolbox::SickNav350::SICK_MAX_NUM_MEASUREMENTS] = {0};

	/* Instantiate the object */
	SickToolbox::SickNav350 sick_nav350(ipaddress.c_str(),port);

	try {
		/* Initialize the device */
		sick_nav350.Initialize();
		sick_nav350.SetAccessMode(3);

		try {
			sick_nav350.GetSickIdentity();
			std::cout << "\n\t\tSick Device Identity" << std::endl;
			std::cout << "\t\tDevice Name : " << sick_nav350.GetSickName() << std::endl;
			std::cout << "\t\tDevice Version : " << sick_nav350.GetSickVersion() << std::endl;
			std::cout << "\t\tDevice Serial Number : " << sick_nav350.GetSickSerialNumber() << std::endl;
			std::cout << "\t\tDevice Firmware Version : " << sick_nav350.GetSickFirmwareVersion() << std::endl;
			std::cout << "\t\tDevice Version : " << sick_nav350.GetSickSoftwareVersion() << std::endl;

			//Change to standby mode for initialization

			sick_nav350.SetOperatingMode(1);
			sick_nav350.SetCurrentLayer(1);
			sick_nav350.SetPoseDataFormat(1,0);
			sick_nav350.SetReflectorWindow(500,500,500,60000);
			sick_nav350.SetReflectorSize(80);
			sick_nav350.SetReflectorType(1);
			sick_nav350.SetLandmarkMatching(1);
			sick_nav350.SetActionRadius(400, 70000);
			sick_nav350.SetOperatingMode(4);
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
