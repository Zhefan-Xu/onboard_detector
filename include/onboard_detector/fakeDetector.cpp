/*
	FILE: fakeDetector.cpp
	-----------------------
	Function definition of fake detector
*/

#include <onboard_detector/fakeDetector.h>

namespace onboardDetector{
	fakeDetector::fakeDetector(const ros::NodeHandle& nh) : nh_(nh){
		// load ros parameter:
		if (not this->nh_.getParam("target_obstacle", this->targetObstacle_)){
			this->targetObstacle_ = std::vector<std::string> {"person", "obstacle"};
			cout << "[Fake Detector]: No target obstacle param. Use default value." << endl;
		}

		if (not this->nh_.getParam("color_distance", this->colorDistance_)){
			this->colorDistance_ = 5.0; // change color for obstacles in meter distance
			cout << "[Fake Detector]: No color distance param. Use default value: 5.0m." << endl;
		}

		if (not this->nh_.getParam("color_distance", this->colorDistance_)){
			this->colorDistance_ = 5.0; // change color for obstacles in meter distance
			cout << "[Fake Detector]: No color distance param. Use default value: 5.0m." << endl;
		}

		std::string odomTopicName;
		if (not this->nh_.getParam("odom_topic", odomTopicName)){
			odomTopicName = "/CERLAB/quadcopter/odom";
			cout << "[Fake Detector]: No odom topic param. Use default: /CERLAB/quadcopter/odom" << endl;
		}

		// tracking history size
        if (not this->nh_.getParam("history_size", this->histSize_)){
            this->histSize_ = 5;
            std::cout << "[Fake Detector]: No tracking history size parameter found. Use default: 5." << std::endl;
        }
        else{
            std::cout << "[Fake Detector]: The history for tracking is set to: " << this->histSize_ << std::endl;
        }  


		this->firstTime_ = true;
		this->gazeboSub_ = this->nh_.subscribe("/gazebo/model_states", 10, &fakeDetector::stateCB, this);
		this->odomSub_ = this->nh_.subscribe(odomTopicName, 10, &fakeDetector::odomCB, this);
		// this->odomSub_ = this->nh_.subscribe("/mavros/local_position/odom", 10, &fakeDetector::odomCB, this);
		this->historyTrajPub_ = this->nh_.advertise<visualization_msgs::MarkerArray>("onboard_detector/history_trajectories", 10);
		this->histTimer_ = this->nh_.createTimer(ros::Duration(0.033), &fakeDetector::histCB, this);
		this->visPub_ = this->nh_.advertise<visualization_msgs::MarkerArray>("onboard_detector/GT_obstacle_bbox", 10);
		this->visTimer_ = this->nh_.createTimer(ros::Duration(0.05), &fakeDetector::visCB, this);
	
		// get dynamic obstacle service
		this->getDynamicObstacleServer_ = this->nh_.advertiseService("fake_detector/getDynamicObstacles", &fakeDetector::getDynamicObstacles, this);
	}



    bool fakeDetector::getDynamicObstacles(onboard_detector::GetDynamicObstacles::Request& req, 
                                           onboard_detector::GetDynamicObstacles::Response& res) {
        // Get the current robot position
		Eigen::Vector3d currPos (req.current_position.x, req.current_position.y, req.current_position.z);

        // Vector to store obstacles along with their distances
        std::vector<std::pair<double, onboardDetector::box3D>> obstaclesWithDistances;

        // Go through all obstacles and calculate distances
        for (const onboardDetector::box3D& bbox : this->obstacleMsg_) {
            Eigen::Vector3d obsPos(bbox.x, bbox.y, bbox.z);
            Eigen::Vector3d diff = currPos - obsPos;
            diff(2) = 0.;
            double distance = diff.norm();            
			if (distance <= req.range) {
                obstaclesWithDistances.push_back(std::make_pair(distance, bbox));
            }
        }

        // Sort obstacles by distance in ascending order
        std::sort(obstaclesWithDistances.begin(), obstaclesWithDistances.end(), 
                [](const std::pair<double, onboardDetector::box3D>& a, const std::pair<double, onboardDetector::box3D>& b) {
                    return a.first < b.first;
                });

        // Push sorted obstacles into the response
        for (const auto& item : obstaclesWithDistances){
            const onboardDetector::box3D& bbox = item.second;

            geometry_msgs::Vector3 pos;
            geometry_msgs::Vector3 vel;
            geometry_msgs::Vector3 size;

            pos.x = bbox.x;
            pos.y = bbox.y;
            pos.z = bbox.z;

            vel.x = bbox.Vx;
            vel.y = bbox.Vy;
            vel.z = bbox.Vz;

            size.x = bbox.x_width;
            size.y = bbox.y_width;
            size.z = bbox.z_width;

            res.position.push_back(pos);
            res.velocity.push_back(vel);
            res.size.push_back(size);
        }

        return true;
    }


	void fakeDetector::visCB(const ros::TimerEvent&){
		this->publishHistoryTraj();
		this->publishVisualization();
	}

	void fakeDetector::stateCB(const gazebo_msgs::ModelStatesConstPtr& allStates){
		bool update = false;
		if (this->firstTime_){
			this->targetIndex_ = this->findTargetIndex(allStates->name);
			this->firstTime_ = false;
		}
		std::vector<onboardDetector::box3D> obVec;
		onboardDetector::box3D ob;
		geometry_msgs::Pose p;
		geometry_msgs::Twist tw;
		for (int i=0; i<int(this->targetIndex_.size()); ++i){
			std::string name = allStates->name[this->targetIndex_[i]];
			// 1. get position and velocity
			p = allStates->pose[this->targetIndex_[i]];
			tw = allStates->twist[this->targetIndex_[i]];
			ob.x = p.position.x;
			ob.y = p.position.y;
			if (name.size() >= 6 and name.compare(0, 6, "person") == 0){
				ob.z = p.position.z + 0.9;
			}
			else{
				ob.z = p.position.z;
			}
			
			if (this->lastObVec_.size() == 0){
				ob.Vx = 0.0;
				ob.Vy = 0.0;
				ob.Vz = 0.0;
				ros::Time lastTime = ros::Time::now();
				this->lastTimeVec_.push_back(lastTime);
				this->lastTimeVel_.push_back(std::vector<double> {0, 0, 0});
				update = true;
			}
			else{
				ros::Time currTime = ros::Time::now();
				double dT = (currTime.toSec() - this->lastTimeVec_[i].toSec());
				if (dT >= 0.1){
					double vx = (ob.x - this->lastObVec_[i].x)/dT;
					double vy = (ob.y - this->lastObVec_[i].y)/dT;
					double vz = (ob.z - this->lastObVec_[i].z)/dT;
					ob.Vx = vx;
					ob.Vy = vy;
					ob.Vz = vz;
					this->lastTimeVel_[i][0] = vx;
					this->lastTimeVel_[i][1] = vy;
					this->lastTimeVel_[i][2] = vz;
					this->lastTimeVec_[i] = ros::Time::now();
					update = true;
				}
				else{
					ob.Vx = this->lastTimeVel_[i][0];
					ob.Vy = this->lastTimeVel_[i][1];
					ob.Vz = this->lastTimeVel_[i][2];
				}
			}
			// 2. get size (gazebo name contains size):
			double xsize, ysize, zsize;
			int xsizeStartIdx = name.size() - 1 - 1 - 3 * 3;
			std::string xsizeStr = name.substr(xsizeStartIdx, 3);
			xsize = std::stod(xsizeStr);

			int ysizeStartIdx = name.size() - 1 - 3 * 2;
			std::string ysizeStr = name.substr(ysizeStartIdx, 3);
			ysize = std::stod(ysizeStr);

			int zsizeStartIdx = name.size() - 3;
			std::string zsizeStr = name.substr(zsizeStartIdx, 3);
			zsize = std::stod(zsizeStr);
			
			ob.x_width = xsize;
			ob.y_width = ysize;
			ob.z_width = zsize;
			obVec.push_back(ob);
		}
		if (update){
			this->lastObVec_ = obVec;
		}
		this->obstacleMsg_ = obVec;
		// ros::Rate r (60);
		// r.sleep();
	}

	void fakeDetector::odomCB(const nav_msgs::OdometryConstPtr& odom){
		this->odom_ = *odom;
	}

	void fakeDetector::histCB(const ros::TimerEvent&){
		if (this->obstacleHist_.size() == 0){
			this->obstacleHist_.resize(this->obstacleMsg_.size());
		}
		for (int i=0; i<int(this->obstacleMsg_.size());i++){
			if (int(this->obstacleHist_[i].size()) >= this->histSize_){
				this->obstacleHist_[i].pop_back();
			}
			this->obstacleHist_[i].push_front(this->obstacleMsg_[i]);
		}
	}

	std::vector<int>& fakeDetector::findTargetIndex(const std::vector<std::string>& modelNames){
		static std::vector<int> targetIndex;
		int countID = 0;
		for (std::string name : modelNames){
			for (std::string targetName : this->targetObstacle_){
				if (name.compare(0, targetName.size(), targetName) == 0){
					targetIndex.push_back(countID);
				}
			}
			++countID;
		}
		return targetIndex;
	}

	void fakeDetector::updateVisMsg(){
		std::vector<visualization_msgs::Marker> bboxVec;
		int obIdx = 0;
		for (const onboardDetector:: box3D& obstacle : this->obstacleMsg_){

			// 12 lines for each obstacle
			geometry_msgs::Point p1, p2, p3, p4, p5, p6, p7, p8;
			// upper four points
			p1.x = obstacle.x+obstacle.x_width/2; p1.y = obstacle.y+obstacle.y_width/2; p1.z = obstacle.z+obstacle.z_width/2;
			p2.x = obstacle.x-obstacle.x_width/2; p2.y = obstacle.y+obstacle.y_width/2; p2.z = obstacle.z+obstacle.z_width/2;
			p3.x = obstacle.x+obstacle.x_width/2; p3.y = obstacle.y-obstacle.y_width/2; p3.z = obstacle.z+obstacle.z_width/2;
			p4.x = obstacle.x-obstacle.x_width/2; p4.y = obstacle.y-obstacle.y_width/2; p4.z = obstacle.z+obstacle.z_width/2;

			p5.x = obstacle.x+obstacle.x_width/2; p5.y = obstacle.y+obstacle.y_width/2; p5.z = obstacle.z-obstacle.z_width/2;
			p6.x = obstacle.x-obstacle.x_width/2; p6.y = obstacle.y+obstacle.y_width/2; p6.z = obstacle.z-obstacle.z_width/2;
			p7.x = obstacle.x+obstacle.x_width/2; p7.y = obstacle.y-obstacle.y_width/2; p7.z = obstacle.z-obstacle.z_width/2;
			p8.x = obstacle.x-obstacle.x_width/2; p8.y = obstacle.y-obstacle.y_width/2; p8.z = obstacle.z-obstacle.z_width/2;

			std::vector<geometry_msgs::Point> line1Vec {p1, p2};
			std::vector<geometry_msgs::Point> line2Vec {p1, p3};
			std::vector<geometry_msgs::Point> line3Vec {p2, p4};
			std::vector<geometry_msgs::Point> line4Vec {p3, p4};
			std::vector<geometry_msgs::Point> line5Vec {p1, p5};
			std::vector<geometry_msgs::Point> line6Vec {p2, p6};
			std::vector<geometry_msgs::Point> line7Vec {p3, p7};
			std::vector<geometry_msgs::Point> line8Vec {p4, p8};
			std::vector<geometry_msgs::Point> line9Vec {p5, p6};
			std::vector<geometry_msgs::Point> line10Vec {p5, p7};
			std::vector<geometry_msgs::Point> line11Vec {p6, p8};
			std::vector<geometry_msgs::Point> line12Vec {p7, p8};

			std::vector<std::vector<geometry_msgs::Point>> allLines{
				line1Vec,
				line2Vec,
				line3Vec,
				line4Vec,
				line5Vec,
				line6Vec,
				line7Vec,
				line8Vec,
				line9Vec,
				line10Vec,
				line11Vec,
				line12Vec
			};

			int count = 0;
			std::string name = "GT osbtacles" + std::to_string(obIdx);
			for (std::vector<geometry_msgs::Point> lineVec: allLines){
				visualization_msgs::Marker line;

				line.header.frame_id = "map";
				line.ns = name;
				line.points = lineVec;
				line.id = count;
				line.type = visualization_msgs::Marker::LINE_LIST;
				line.lifetime = ros::Duration(0.5);
				line.scale.x = 0.05;
				line.scale.y = 0.05;
				line.scale.z = 0.05;
				line.color.a = 1.0;
				if (this->isObstacleInSensorRange(obstacle, PI_const)){
					line.color.r = 1;
					line.color.g = 0;
					line.color.b = 0;
				}
				else{
					line.color.r = 0;
					line.color.g = 1;
					line.color.b = 0;
				}
				++count;
				bboxVec.push_back(line);
			}
			++obIdx;
		}
		this->visMsg_.markers = bboxVec;
	}

	void fakeDetector::publishHistoryTraj(){
		if (this->obstacleHist_.size() != 0){
			visualization_msgs::MarkerArray trajMsg;
			int countMarker = 0;
			for (size_t i=0; i<this->obstacleHist_.size(); ++i){
				if (this->isObstacleInSensorRange(this->obstacleHist_[i][0],2*M_PI)){
					visualization_msgs::Marker traj;
					traj.header.frame_id = "map";
					traj.header.stamp = ros::Time::now();
					traj.ns = "fake_detector";
					traj.id = countMarker;
					traj.type = visualization_msgs::Marker::LINE_STRIP;
					traj.scale.x = 0.1;
					traj.scale.y = 0.1;
					traj.scale.z = 0.1;
					traj.color.a = 1.0; 
					traj.color.r = 0.0;
					traj.color.g = 1.0;
					traj.color.b = 0.0;
					for (size_t j=0; j<this->obstacleHist_[i].size(); ++j){
						geometry_msgs::Point p1;
						onboardDetector::box3D box1 = this->obstacleHist_[i][j];
						p1.x = box1.x; p1.y = box1.y; p1.z = box1.z;
						traj.points.push_back(p1);
					}

					++countMarker;
					trajMsg.markers.push_back(traj);
				}
			}
			this->historyTrajPub_.publish(trajMsg);
		}
	}


	void fakeDetector::publishVisualization(){
		this->updateVisMsg();
		this->visPub_.publish(this->visMsg_);
	}

	bool fakeDetector::isObstacleInSensorRange(const onboardDetector::box3D& ob, double fov){
		Eigen::Vector3d pRobot (this->odom_.pose.pose.position.x, this->odom_.pose.pose.position.y, this->odom_.pose.pose.position.z);
		Eigen::Vector3d pObstacle (ob.x, ob.y, ob.z);	
		
		Eigen::Vector3d diff = pObstacle - pRobot;
		diff(2) = 0.0;
		double distance = diff.norm();
		double yaw = rpy_from_quaternion(this->odom_.pose.pose.orientation);
		Eigen::Vector3d direction (cos(yaw), sin(yaw), 0);

		double angle = angleBetweenVectors(direction, diff);
		if (angle <= fov/2 and distance <= this->colorDistance_){
			return true;
		}
		else{
			return false;
		}
	
	}

	void fakeDetector::getObstacles(std::vector<onboardDetector::box3D>& obstacles, const Eigen::Vector3d &robotSize){
		obstacles.clear();
		for (onboardDetector::box3D ob : this->obstacleMsg_){
			ob.x_width += robotSize(0);
			ob.y_width += robotSize(1);
			ob.z_width += robotSize(2);
			obstacles.push_back(ob);
		}
		// obstacles = this->obstacleMsg_;
	}

	void fakeDetector::getObstaclesInSensorRange(double fov, std::vector<onboardDetector::box3D>& obstacles, const Eigen::Vector3d &robotSize){
		obstacles.clear();
		for (onboardDetector::box3D obstacle : this->obstacleMsg_){
			if (this->isObstacleInSensorRange(obstacle, fov)){
				obstacle.x_width += robotSize(0);
				obstacle.y_width += robotSize(1);
				obstacle.z_width += robotSize(2);
				obstacles.push_back(obstacle);
			}
		}
	}

	void fakeDetector::getDynamicObstaclesHist(std::vector<std::vector<Eigen::Vector3d>>& posHist, std::vector<std::vector<Eigen::Vector3d>>& velHist, std::vector<std::vector<Eigen::Vector3d>>& sizeHist, const Eigen::Vector3d &robotSize){
		posHist.clear();
        velHist.clear();
        sizeHist.clear();

        if (this->obstacleHist_.size()){
            for (size_t i=0 ; i<this->obstacleHist_.size() ; ++i){
				if (this->isObstacleInSensorRange(this->obstacleHist_[i][0],2*M_PI)){
					std::vector<Eigen::Vector3d> obPosHist, obVelHist, obSizeHist;
					for (size_t j=0; j<this->obstacleHist_[i].size() ; ++j){
						Eigen::Vector3d pos(this->obstacleHist_[i][j].x, this->obstacleHist_[i][j].y, this->obstacleHist_[i][j].z);
						Eigen::Vector3d vel(this->obstacleHist_[i][j].Vx, this->obstacleHist_[i][j].Vy, 0);
						Eigen::Vector3d size(this->obstacleHist_[i][j].x_width, this->obstacleHist_[i][j].y_width, this->obstacleHist_[i][j].z_width);
						size += robotSize;
						obPosHist.push_back(pos);
						obVelHist.push_back(vel);
						obSizeHist.push_back(size);
					}
					posHist.push_back(obPosHist);
					velHist.push_back(obVelHist);
					sizeHist.push_back(obSizeHist);
				}
            }
        }
	}
}