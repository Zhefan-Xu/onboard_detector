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
#include <onboard_detector/GetDynamicObstacles.h>
#include <thread>
#include <mutex>
#include <deque>

using std::cout; using std::endl;

namespace onboardDetector{
	class fakeDetector{
	private:
		ros::NodeHandle nh_;
		ros::Timer obstaclePubTimer_;
		ros::Timer visTimer_;
		ros::Timer histTimer_;
		ros::Subscriber gazeboSub_;
		ros::Publisher visPub_; // publish bounding box
		ros::Publisher historyTrajPub_; //publish obstacle history
		ros::Subscriber odomSub_;
		ros::ServiceServer getDynamicObstacleServer_;

		int histSize_;
		std::vector<std::string> targetObstacle_;
		std::vector<int> targetIndex_;
		bool firstTime_;
		std::vector<onboardDetector::box3D> obstacleMsg_;
		std::vector<onboardDetector::box3D> lastObVec_;
		std::vector<ros::Time> lastTimeVec_;
		std::vector<std::vector<double>> lastTimeVel_;
		std::vector<std::deque<onboardDetector::box3D>> obstacleHist_;

		// visualization:
		nav_msgs::Odometry odom_;
		double colorDistance_;
		visualization_msgs::MarkerArray visMsg_;

	public:
		fakeDetector(const ros::NodeHandle& nh);

		bool getDynamicObstacles(onboard_detector::GetDynamicObstacles::Request& req, 
								 onboard_detector::GetDynamicObstacles::Response& res);

		void visCB(const ros::TimerEvent&);
		void stateCB(const gazebo_msgs::ModelStatesConstPtr& allStates);
		void odomCB(const nav_msgs::OdometryConstPtr& odom);
		void histCB(const ros::TimerEvent&);
		std::vector<int>& findTargetIndex(const std::vector<std::string>& modelNames);
		void updateVisMsg();
		void publishObstacles();
		void publishVisualization();
		void publishHistoryTraj();
		bool isObstacleInSensorRange(const onboardDetector::box3D& ob, double fov);
		void getObstacles(std::vector<onboardDetector::box3D>& obstacles, const Eigen::Vector3d &robotSize = Eigen::Vector3d(0.0,0.0,0.0));
		void getObstaclesInSensorRange(double fov, std::vector<onboardDetector::box3D>& obstacles, const Eigen::Vector3d &robotSize = Eigen::Vector3d(0.0,0.0,0.0));
		void getDynamicObstaclesHist(std::vector<std::vector<Eigen::Vector3d>>& posHist, std::vector<std::vector<Eigen::Vector3d>>& velHist, std::vector<std::vector<Eigen::Vector3d>>& sizeHist, const Eigen::Vector3d &robotSize = Eigen::Vector3d(0.0,0.0,0.0));
	};
}

#endif