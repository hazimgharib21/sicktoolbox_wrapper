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

		try {
			sick_nav350.SetOperatingMode(3);
			sick_nav350.SetScanDataFormat(1, 1);
		} catch (...) {
			ROS_ERROR("Configuration error");
			return -1;
		}

		while (ros::ok()) {

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
