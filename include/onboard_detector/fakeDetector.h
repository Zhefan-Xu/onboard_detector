/*
	FILE: fakeDetector.h
	---------------------
	fake dynamic obtacle detector for gazebo simulation
*/
#ifndef FAKEDETECTOR_H
#define FAKEDETECTOR_H
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <onboard_detector/utils.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <gazebo_msgs/ModelStates.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Odometry.h>
#include <thread>
#include <mutex>

using std::cout; using std::endl;

namespace onboardDetector{
	class fakeDetector{
	private:
		ros::NodeHandle nh_;
		ros::Timer obstaclePubTimer_;
		ros::Timer visTimer_;
		ros::Subscriber gazeboSub_;
		ros::Publisher visPub_; // publish bounding box
		ros::Subscriber odomSub_;

		std::vector<std::string> targetObstacle_;
		std::vector<int> targetIndex_;
		bool firstTime_;
		std::vector<onboardDetector::box3D> obstacleMsg_;
		std::vector<onboardDetector::box3D> lastObVec_;
		std::vector<ros::Time> lastTimeVec_;
		std::vector<std::vector<double>> lastTimeVel_;

		// visualization:
		nav_msgs::Odometry odom_;
		double colorDistance_;
		visualization_msgs::MarkerArray visMsg_;

	public:
		fakeDetector(const ros::NodeHandle& nh);

		void visCB(const ros::TimerEvent&);
		void stateCB(const gazebo_msgs::ModelStatesConstPtr& allStates);
		void odomCB(const nav_msgs::OdometryConstPtr& odom);
		std::vector<int>& findTargetIndex(const std::vector<std::string>& modelNames);
		void updateVisMsg();
		void publishObstacles();
		void publishVisualization();
		bool isObstacleInSensorRange(const onboardDetector::box3D& ob, double fov);
		void getObstacles(std::vector<onboardDetector::box3D>& obstacles);
		void getObstaclesInSensorRange(double fov, std::vector<onboardDetector::box3D>& obstacles);
	};
}

#endif