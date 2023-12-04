/*
	FILE: fake_detector_node.cpp
	--------------------------
	Run fake detector for simulation
*/

#include <ros/ros.h>
#include <onboard_detector/fakeDetector.h>

int main(int argc, char** argv){
	ros::init(argc, argv, "fake_detector");
	ros::NodeHandle nh;
	onboardDetector::fakeDetector d (nh);
	ros::spin();
	return 0;
}