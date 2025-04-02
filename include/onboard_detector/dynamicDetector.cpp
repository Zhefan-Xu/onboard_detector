/*
    FILE: dynamicDetector.cpp
    ---------------------------------
    function implementation of dynamic osbtacle detector
*/
#include <onboard_detector/dynamicDetector.h>

namespace onboardDetector{
    dynamicDetector::dynamicDetector(){
        this->ns_ = "onboard_detector";
        this->hint_ = "[onboardDetector]";
    }

    dynamicDetector::dynamicDetector(const ros::NodeHandle& nh){
        this->ns_ = "onboard_detector";
        this->hint_ = "[onboardDetector]";
        this->nh_ = nh;
        this->initParam();
        this->registerPub();
        this->registerCallback();
    }

    void dynamicDetector::initDetector(const ros::NodeHandle& nh){
        this->nh_ = nh;
        this->initParam();
        this->registerPub();
        this->registerCallback();
    }

    void dynamicDetector::initParam(){
        // localization mode
        if (not this->nh_.getParam(this->ns_ + "/localization_mode", this->localizationMode_)){
            this->localizationMode_ = 0;
            cout << this->hint_ << ": No localization mode option. Use default: pose" << endl;
        }
        else{
            cout << this->hint_ << ": Localizaiton mode: pose (0)/odom (1). Your option: " << this->localizationMode_ << endl;
        }   

        // depth topic name
        if (not this->nh_.getParam(this->ns_ + "/depth_image_topic", this->depthTopicName_)){
            this->depthTopicName_ = "/camera/depth/image_raw";
            cout << this->hint_ << ": No depth image topic name. Use default: /camera/depth/image_raw" << endl;
        }
        else{
            cout << this->hint_ << ": Depth topic: " << this->depthTopicName_ << endl;
        }

        // aligned depth topic name
        if (not this->nh_.getParam(this->ns_ + "/aligned_depth_image_topic", this->alignedDepthTopicName_)){
            this->alignedDepthTopicName_ = "/camera/aligned_depth_to_color/image_raw";
            cout << this->hint_ << ": No aligned depth image topic name. Use default: /camera/aligned_depth_to_color/image_raw" << endl;
        }
        else{
            cout << this->hint_ << ": Aligned depth topic: " << this->alignedDepthTopicName_ << endl;
        }

        if (this->localizationMode_ == 0){
            // odom topic name
            if (not this->nh_.getParam(this->ns_ + "/pose_topic", this->poseTopicName_)){
                this->poseTopicName_ = "/CERLAB/quadcopter/pose";
                cout << this->hint_ << ": No pose topic name. Use default: /CERLAB/quadcopter/pose" << endl;
            }
            else{
                cout << this->hint_ << ": Pose topic: " << this->poseTopicName_ << endl;
            }           
        }

        if (this->localizationMode_ == 1){
            // pose topic name
            if (not this->nh_.getParam(this->ns_ + "/odom_topic", this->odomTopicName_)){
                this->odomTopicName_ = "/CERLAB/quadcopter/odom";
                cout << this->hint_ << ": No odom topic name. Use default: /CERLAB/quadcopter/odom" << endl;
            }
            else{
                cout << this->hint_ << ": Odom topic: " << this->odomTopicName_ << endl;
            }
        }

        std::vector<double> depthIntrinsics (4);
        if (not this->nh_.getParam(this->ns_ + "/depth_intrinsics", depthIntrinsics)){
            cout << this->hint_ << ": Please check camera intrinsics!" << endl;
            exit(0);
        }
        else{
            this->fx_ = depthIntrinsics[0];
            this->fy_ = depthIntrinsics[1];
            this->cx_ = depthIntrinsics[2];
            this->cy_ = depthIntrinsics[3];
            cout << this->hint_ << ": fx, fy, cx, cy: " << "["  << this->fx_ << ", " << this->fy_  << ", " << this->cx_ << ", "<< this->cy_ << "]" << endl;
        }

        // depth scale factor
        if (not this->nh_.getParam(this->ns_ + "/depth_scale_factor", this->depthScale_)){
            this->depthScale_ = 1000.0;
            cout << this->hint_ << ": No depth scale factor. Use default: 1000." << endl;
        }
        else{
            cout << this->hint_ << ": Depth scale factor: " << this->depthScale_ << endl;
        }

        // depth min value
        if (not this->nh_.getParam(this->ns_ + "/depth_min_value", this->depthMinValue_)){
            this->depthMinValue_ = 0.2;
            cout << this->hint_ << ": No depth min value. Use default: 0.2 m." << endl;
        }
        else{
            cout << this->hint_ << ": Depth min value: " << this->depthMinValue_ << endl;
        }

        // depth max value
        if (not this->nh_.getParam(this->ns_ + "/depth_max_value", this->depthMaxValue_)){
            this->depthMaxValue_ = 5.0;
            cout << this->hint_ << ": No depth max value. Use default: 5.0 m." << endl;
        }
        else{
            cout << this->hint_ << ": Depth depth max value: " << this->depthMaxValue_ << endl;
        }

        // depth filter margin
        if (not this->nh_.getParam(this->ns_ + "/depth_filter_margin", this->depthFilterMargin_)){
            this->depthFilterMargin_ = 0;
            cout << this->hint_ << ": No depth filter margin. Use default: 0." << endl;
        }
        else{
            cout << this->hint_ << ": Depth filter margin: " << this->depthFilterMargin_ << endl;
        }

        // depth skip pixel
        if (not this->nh_.getParam(this->ns_ + "/depth_skip_pixel", this->skipPixel_)){
            this->skipPixel_ = 1;
            cout << this->hint_ << ": No depth skip pixel. Use default: 1." << endl;
        }
        else{
            cout << this->hint_ << ": Depth skip pixel: " << this->skipPixel_ << endl;
        }

        // ------------------------------------------------------------------------------------
        // depth image columns
        if (not this->nh_.getParam(this->ns_ + "/image_cols", this->imgCols_)){
            this->imgCols_ = 640;
            cout << this->hint_ << ": No depth image columns. Use default: 640." << endl;
        }
        else{
            cout << this->hint_ << ": Depth image columns: " << this->imgCols_ << endl;
        }

        // depth skip pixel
        if (not this->nh_.getParam(this->ns_ + "/image_rows", this->imgRows_)){
            this->imgRows_ = 480;
            cout << this->hint_ << ": No depth image rows. Use default: 480." << endl;
        }
        else{
            cout << this->hint_ << ": Depth image rows: " << this->imgRows_ << endl;
        }
        this->projPoints_.resize(this->imgCols_ * this->imgRows_ / (this->skipPixel_ * this->skipPixel_));
        this->pointsDepth_.resize(this->imgCols_ * this->imgRows_ / (this->skipPixel_ * this->skipPixel_));
        // ------------------------------------------------------------------------------------


        // transform matrix: body to camera
        std::vector<double> body2CamVec (16);
        if (not this->nh_.getParam(this->ns_ + "/body_to_camera", body2CamVec)){
            ROS_ERROR("[dynamicDetector]: Please check body to camera matrix!");
        }
        else{
            for (int i=0; i<4; ++i){
                for (int j=0; j<4; ++j){
                    this->body2Cam_(i, j) = body2CamVec[i * 4 + j];
                }
            }
        }
        
        std::vector<double> colorIntrinsics (4);
        if (not this->nh_.getParam(this->ns_ + "/color_intrinsics", colorIntrinsics)){
            cout << this->hint_ << ": Please check camera intrinsics!" << endl;
            exit(0);
        }
        else{
            this->fxC_ = colorIntrinsics[0];
            this->fyC_ = colorIntrinsics[1];
            this->cxC_ = colorIntrinsics[2];
            this->cyC_ = colorIntrinsics[3];
            cout << this->hint_ << ": fxC, fyC, cxC, cyC: " << "["  << this->fxC_ << ", " << this->fyC_  << ", " << this->cxC_ << ", "<< this->cyC_ << "]" << endl;
        }

        // transform matrix: body to camera color
        std::vector<double> body2CamColorVec (16);
        if (not this->nh_.getParam(this->ns_ + "/body_to_camera_color", body2CamColorVec)){
            ROS_ERROR("[dynamicDetector]: Please check body to camera color matrix!");
        }
        else{
            for (int i=0; i<4; ++i){
                for (int j=0; j<4; ++j){
                    this->body2CamColor_(i, j) = body2CamColorVec[i * 4 + j];
                }
            }
        }

        // Raycast max length
        if (not this->nh_.getParam(this->ns_ + "/raycast_max_length", this->raycastMaxLength_)){
            this->raycastMaxLength_ = 5.0;
            cout << this->hint_ << ": No raycast max length. Use default: 5.0." << endl;
        }
        else{
            cout << this->hint_ << ": Raycast max length: " << this->raycastMaxLength_ << endl;
        }

        // min num of points for a voxel to be occupied in voxel filter
        if (not this->nh_.getParam(this->ns_ + "/voxel_occupied_thresh", this->voxelOccThresh_)){
            this->voxelOccThresh_ = 10;
            cout << this->hint_ << ": No voxel_occupied_threshold. Use default: 10." << endl;
        }
        else{
            cout << this->hint_ << ": min num of points for a voxel to be occupied in voxel filter is set to be: " << this->voxelOccThresh_ << endl;
        }

        // ground height
        if (not this->nh_.getParam(this->ns_ + "/ground_height", this->groundHeight_)){
            this->groundHeight_ = 0.1;
            std::cout << this->hint_ << ": No ground height parameter. Use default: 0.1m." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": Ground height is set to: " << this->groundHeight_ << std::endl;
        }

        // minimum number of points in each cluster
        if (not this->nh_.getParam(this->ns_ + "/dbscan_min_points_cluster", this->dbMinPointsCluster_)){
            this->dbMinPointsCluster_ = 18;
            cout << this->hint_ << ": No DBSCAN minimum point in each cluster parameter. Use default: 18." << endl;
        }
        else{
            cout << this->hint_ << ": DBSCAN Minimum point in each cluster is set to: " << this->dbMinPointsCluster_ << endl;
        }

        // search range
        if (not this->nh_.getParam(this->ns_ + "/dbscan_search_range_epsilon", this->dbEpsilon_)){
            this->dbEpsilon_ = 0.3;
            cout << this->hint_ << ": No DBSCAN epsilon parameter. Use default: 0.3." << endl;
        }
        else{
            cout << this->hint_ << ": DBSCAN epsilon is set to: " << this->dbEpsilon_ << endl;
        }  

        // IOU threshold
        if (not this->nh_.getParam(this->ns_ + "/filtering_BBox_IOU_threshold", this->boxIOUThresh_)){
            this->boxIOUThresh_ = 0.5;
            cout << this->hint_ << ": No threshold for boununding box IOU filtering parameter found. Use default: 0.5." << endl;
        }
        else{
            cout << this->hint_ << ": The threshold for boununding box IOU filtering is set to: " << this->boxIOUThresh_ << endl;
        }  

        // YOLO overwrite distance
        if (not this->nh_.getParam(this->ns_ + "/yolo_overwrite_distance", this->yoloOverwriteDistance_)){
            this->yoloOverwriteDistance_ = 3.5;
            cout << this->hint_ << ": No threshold for YOLO overwrite distance. Use default: 3.5m." << endl;
        }
        else{
            cout << this->hint_ << ": The YOLO overwrite distance is set to: " << this->yoloOverwriteDistance_ << endl;
        }  

        // tracking history size
        if (not this->nh_.getParam(this->ns_ + "/history_size", this->histSize_)){
            this->histSize_ = 5;
            std::cout << this->hint_ << ": No tracking history isze parameter found. Use default: 5." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": The history for tracking is set to: " << this->histSize_ << std::endl;
        }  

        // time difference
        if (not this->nh_.getParam(this->ns_ + "/time_difference", this->dt_)){
            this->dt_ = 0.033;
            std::cout << this->hint_ << ": No time difference parameter found. Use default: 0.033." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": The time difference for the system is set to: " << this->dt_ << std::endl;
        }  

        // similarity threshold for data association 
        if (not this->nh_.getParam(this->ns_ + "/similarity_threshold", this->simThresh_)){
            this->simThresh_ = 0.9;
            std::cout << this->hint_ << ": No similarity threshold parameter found. Use default: 0.9." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": The similarity threshold for data association is set to: " << this->simThresh_ << std::endl;
        }  

        // similarity threshold for data association 
        if (not this->nh_.getParam(this->ns_ + "/frame_skip", this->skipFrame_)){
            this->skipFrame_ = 5;
            std::cout << this->hint_ << ": No skip frame parameter found. Use default: 5." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": The frames skiped in classification when comparing two point cloud is set to: " << this->skipFrame_ << std::endl;
        }  

        // velocity threshold for dynamic classification
        if (not this->nh_.getParam(this->ns_ + "/dynamic_velocity_threshold", this->dynaVelThresh_)){
            this->dynaVelThresh_ = 0.35;
            std::cout << this->hint_ << ": No dynamic velocity threshold parameter found. Use default: 0.35." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": The velocity threshold for dynamic classification is set to: " << this->dynaVelThresh_ << std::endl;
        }  

        // voting threshold for dynamic classification
        if (not this->nh_.getParam(this->ns_ + "/dynamic_voting_threshold", this->dynaVoteThresh_)){
            this->dynaVoteThresh_ = 0.8;
            std::cout << this->hint_ << ": No dynamic velocity threshold parameter found. Use default: 0.8." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": The voting threshold for dynamic classification is set to: " << this->dynaVoteThresh_ << std::endl;
        }  

        // if the percentage of skipped points(because of being out of previous FOV) are higher than this, it will not be voted as dynamic
        if (not this->nh_.getParam(this->ns_ + "/maximum_skip_ratio", this->maxSkipRatio_)){
            this->maxSkipRatio_ = 0.5;
            std::cout << this->hint_ << ": No maximum_skip_ratio parameter found. Use default: 0.5." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": The the upper limit of points skipping in classification is set to: " << this->maxSkipRatio_ << std::endl;
        }  

        // History threshold for fixing box size
        if (not this->nh_.getParam(this->ns_ + "/fix_size_history_threshold", this->fixSizeHistThresh_)){
            this->fixSizeHistThresh_ = 10;
            std::cout << this->hint_ << ": No history threshold for fixing size parameter found. Use default: 10." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": History threshold for fixing size parameter is set to: " << this->fixSizeHistThresh_ << std::endl;
        }  

        // Dimension threshold for fixing box size
        if (not this->nh_.getParam(this->ns_ + "/fix_size_dimension_threshold", this->fixSizeDimThresh_)){
            this->fixSizeDimThresh_ = 0.4;
            std::cout << this->hint_ << ": No dimension threshold for fixing size parameter found. Use default: 0.4." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": Dimension threshold for fixing size parameter is set to: " << this->fixSizeDimThresh_ << std::endl;
        } 

        // covariance for Kalman Filter
        if (not this->nh_.getParam(this->ns_ + "/e_p", this->eP_)){
            this->eP_ = 0.5;
            std::cout << this->hint_ << ": No covariance parameter found. Use default: 0.5." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": The covariance for kalman filter is set to: " << this->eP_ << std::endl;
        }  

        // noise for prediction for position in Kalman Filter
        if (not this->nh_.getParam(this->ns_ + "/e_q_pos", this->eQPos_)){
            this->eQPos_ = 0.5;
            std::cout << this->hint_ << ": No motion model uncertainty matrix for position parameter found. Use default: 0.5." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": The noise for prediction for position in Kalman Filter is set to: " << this->eQPos_ << std::endl;
        }  

        // noise for prediction for velocity in Kalman Filter
        if (not this->nh_.getParam(this->ns_ + "/e_q_vel", this->eQVel_)){
            this->eQVel_ = 0.5;
            std::cout << this->hint_ << ": No motion model uncertainty matrix for velocity parameter found. Use default: 0.5." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": The noise for prediction for velocity in Kalman Filter is set to: " << this->eQVel_ << std::endl;
        } 

        // noise for prediction in Kalman Filter
        if (not this->nh_.getParam(this->ns_ + "/e_q_acc", this->eQAcc_)){
            this->eQAcc_ = 0.5;
            std::cout << this->hint_ << ": No motion model uncertainty matrix for acceleration parameter found. Use default: 0.5." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": The noise for prediction for acceleration in Kalman Filter is set to: " << this->eQAcc_ << std::endl;
        } 

        // noise for measurement for position in Kalman Filter
        if (not this->nh_.getParam(this->ns_ + "/e_r_pos", this->eRPos_)){
            this->eRPos_ = 0.5;
            std::cout << this->hint_ << ": No measuremnt uncertainty matrix for position parameter found. Use default: 0.5." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": The noise for measurement for position in Kalman Filter is set to: " << this->eRPos_ << std::endl;
        }  

        // noise for prediction for velocity in Kalman Filter
        if (not this->nh_.getParam(this->ns_ + "/e_r_vel", this->eRVel_)){
            this->eRVel_ = 0.5;
            std::cout << this->hint_ << ": No measuremnt uncertainty matrix for velocity parameter found. Use default: 0.5." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": The noise for measurement for velocity in Kalman Filter is set to: " << this->eRVel_ << std::endl;
        } 

        // noise for prediction in Kalman Filter
        if (not this->nh_.getParam(this->ns_ + "/e_r_acc", this->eRAcc_)){
            this->eRAcc_ = 0.5;
            std::cout << this->hint_ << ": No measurement uncertainty matrix for acceleration parameter found. Use default: 0.5." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": The noise for measuremnt for acceleration in Kalman Filter is set to: " << this->eRAcc_ << std::endl;
        } 

        // num of frames used in KF for observation
        if (not this->nh_.getParam(this->ns_ + "/kalman_filter_averaging_frames", this->kfAvgFrames_)){
            this->kfAvgFrames_ = 10;
            std::cout << this->hint_ << ": No number of frames used in KF for observation parameter found. Use default: 10." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": Number of frames used in KF for observation is set to: " << this->kfAvgFrames_ << std::endl;
        } 

        // frames to froce dynamic
        if (not this->nh_.getParam(this->ns_ + "/frames_force_dynamic", this->forceDynaFrames_)){
            this->forceDynaFrames_ = 20;
            std::cout << this->hint_ << ": No range of searching dynamic obstacles in box history found. Use default: 20." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": Range of searching dynamic obstacles in box history is set to: " << this->forceDynaFrames_ << std::endl;
        }  

        if (not this->nh_.getParam(this->ns_ + "/frames_force_dynamic_check_range", this->forceDynaCheckRange_)){
            this->forceDynaCheckRange_ = 30;
            std::cout << this->hint_ << ": No threshold for forcing dynamic obstacles found. Use default: 30." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": Threshold for forcing dynamic obstacles is set to: " << this->forceDynaCheckRange_ << std::endl;
        }  

        // dynamic consistency check
        if (not this->nh_.getParam(this->ns_ + "/dynamic_consistency_threshold", this->dynamicConsistThresh_)){
            this->dynamicConsistThresh_ = 3;
            std::cout << this->hint_ << ": No threshold for dynamic-consistency check found. Use default: 3." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": Threshold for dynamic consistency check is set to: " << this->dynamicConsistThresh_ << std::endl;
        }  

        if ( this->histSize_ < this->forceDynaCheckRange_+1){
            ROS_ERROR("history length is too short to perform force-dynamic");
        }

        // constrain target object size
        if (not this->nh_.getParam(this->ns_ + "/constrain_size", this->constrainSize_)){
            this->constrainSize_ = false;
            std::cout << this->hint_ << ": No target object constrain size param found. Use default: false." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": Target object constrain is set to: " << this->constrainSize_ << std::endl;
        }  

        // object target sizes
        std::vector<double> targetObjectSizeTemp;
        if (not this->nh_.getParam(this->ns_ + "/target_object_size", targetObjectSizeTemp)){
            std::cout << this->hint_ << ": No target object size found. Do not apply target object size." << std::endl;
        }
        else{
            for (size_t i=0; i<targetObjectSizeTemp.size(); i+=3){
                Eigen::Vector3d targetSize (targetObjectSizeTemp[i+0], targetObjectSizeTemp[i+1], targetObjectSizeTemp[i+2]);
                this->targetObjectSize_.push_back(targetSize);
                std::cout << this->hint_ << ": target object size is set to: [" << targetObjectSizeTemp[i+0]  << ", " << targetObjectSizeTemp[i+1] << ", " <<  targetObjectSizeTemp[i+2] << "]." << std::endl;
            }
            
        }
    }

    void dynamicDetector::registerPub(){
        image_transport::ImageTransport it(this->nh_);
        // uv detector depth map pub
        this->uvDepthMapPub_ = it.advertise(this->ns_ + "/detected_depth_map", 1);

        // uv detector u depth map pub
        this->uDepthMapPub_ = it.advertise(this->ns_ + "/detected_u_depth_map", 1);

        // uv detector bird view pub
        this->uvBirdViewPub_ = it.advertise(this->ns_ + "/bird_view", 1);

        // Yolo 2D bounding box on depth map pub
        this->detectedAlignedDepthImgPub_ = it.advertise(this->ns_ + "/detected_aligned_depth_map_yolo", 1);

        // uv detector bounding box pub
        this->uvBBoxesPub_ = this->nh_.advertise<visualization_msgs::MarkerArray>(this->ns_ + "/uv_bboxes", 10);

        // dynamic pointcloud pub
        this->dynamicPointsPub_ = this->nh_.advertise<sensor_msgs::PointCloud2>(this->ns_ + "/dynamic_point_cloud", 10);

        // filtered pointcloud pub
        this->filteredPointsPub_ = this->nh_.advertise<sensor_msgs::PointCloud2>(this->ns_ + "/filtered_depth_cloud", 10);

        // DBSCAN bounding box pub
        this->dbBBoxesPub_ = this->nh_.advertise<visualization_msgs::MarkerArray>(this->ns_ + "/dbscan_bboxes", 10);

        // yolo bounding box pub
        this->yoloBBoxesPub_ = this->nh_.advertise<visualization_msgs::MarkerArray>(this->ns_ + "/yolo_3d_bboxes", 10);

        // filtered bounding box pub
        this->filteredBBoxesPub_ = this->nh_.advertise<visualization_msgs::MarkerArray>(this->ns_ + "/filtered_bboxes", 10);

        // tracked bounding box pub
        this->trackedBBoxesPub_ = this->nh_.advertise<visualization_msgs::MarkerArray>(this->ns_ + "/tracked_bboxes", 10);

        // dynamic bounding box pub
        this->dynamicBBoxesPub_ = this->nh_.advertise<visualization_msgs::MarkerArray>(this->ns_ + "/dynamic_bboxes", 10);

        // history trajectory pub
        this->historyTrajPub_ = this->nh_.advertise<visualization_msgs::MarkerArray>(this->ns_ + "/history_trajectories", 10);

        // velocity visualization pub
        this->velVisPub_ = this->nh_.advertise<visualization_msgs::MarkerArray>(this->ns_ + "/velocity_visualizaton", 10);
    }   

    void dynamicDetector::registerCallback(){
        // depth pose callback
        this->depthSub_.reset(new message_filters::Subscriber<sensor_msgs::Image>(this->nh_, this->depthTopicName_, 50));
        if (this->localizationMode_ == 0){
            this->poseSub_.reset(new message_filters::Subscriber<geometry_msgs::PoseStamped>(this->nh_, this->poseTopicName_, 25));
            this->depthPoseSync_.reset(new message_filters::Synchronizer<depthPoseSync>(depthPoseSync(100), *this->depthSub_, *this->poseSub_));
            this->depthPoseSync_->registerCallback(boost::bind(&dynamicDetector::depthPoseCB, this, _1, _2));
        }
        else if (this->localizationMode_ == 1){
            this->odomSub_.reset(new message_filters::Subscriber<nav_msgs::Odometry>(this->nh_, this->odomTopicName_, 25));
            this->depthOdomSync_.reset(new message_filters::Synchronizer<depthOdomSync>(depthOdomSync(100), *this->depthSub_, *this->odomSub_));
            this->depthOdomSync_->registerCallback(boost::bind(&dynamicDetector::depthOdomCB, this, _1, _2));
        }
        else{
            ROS_ERROR("[dynamicDetector]: Invalid localization mode!");
            exit(0);
        }

        // aligned depth subscriber
        this->alignedDepthSub_ = this->nh_.subscribe(this->alignedDepthTopicName_, 10, &dynamicDetector::alignedDepthCB, this);

        // yolo detection results subscriber
        this->yoloDetectionSub_ = this->nh_.subscribe("yolo_detector/detected_bounding_boxes", 10, &dynamicDetector::yoloDetectionCB, this);

        // detection timer
        this->detectionTimer_ = this->nh_.createTimer(ros::Duration(this->dt_), &dynamicDetector::detectionCB, this);

        // tracking timer
        this->trackingTimer_ = this->nh_.createTimer(ros::Duration(this->dt_), &dynamicDetector::trackingCB, this);

        // classification timer
        this->classificationTimer_ = this->nh_.createTimer(ros::Duration(this->dt_), &dynamicDetector::classificationCB, this);
    
        // visualization timer
        this->visTimer_ = this->nh_.createTimer(ros::Duration(this->dt_), &dynamicDetector::visCB, this);
    }


    void dynamicDetector::depthPoseCB(const sensor_msgs::ImageConstPtr& img, const geometry_msgs::PoseStampedConstPtr& pose){
        // store current depth image
        cv_bridge::CvImagePtr imgPtr = cv_bridge::toCvCopy(img, img->encoding);
        if (img->encoding == sensor_msgs::image_encodings::TYPE_32FC1){
            (imgPtr->image).convertTo(imgPtr->image, CV_16UC1, this->depthScale_);
        }
        imgPtr->image.copyTo(this->depthImage_);

        // store current position and orientation (camera)
        Eigen::Matrix4d camPoseMatrix, camPoseColorMatrix;
        this->getCameraPose(pose, camPoseMatrix, camPoseColorMatrix);

        this->position_(0) = camPoseMatrix(0, 3);
        this->position_(1) = camPoseMatrix(1, 3);
        this->position_(2) = camPoseMatrix(2, 3);
        this->orientation_ = camPoseMatrix.block<3, 3>(0, 0);

        this->positionColor_(0) = camPoseColorMatrix(0, 3);
        this->positionColor_(1) = camPoseColorMatrix(1, 3);
        this->positionColor_(2) = camPoseColorMatrix(2, 3);
        this->orientationColor_ = camPoseColorMatrix.block<3, 3>(0, 0);
    }

    void dynamicDetector::depthOdomCB(const sensor_msgs::ImageConstPtr& img, const nav_msgs::OdometryConstPtr& odom){
        // store current depth image
        cv_bridge::CvImagePtr imgPtr = cv_bridge::toCvCopy(img, img->encoding);
        if (img->encoding == sensor_msgs::image_encodings::TYPE_32FC1){
            (imgPtr->image).convertTo(imgPtr->image, CV_16UC1, this->depthScale_);
        }
        imgPtr->image.copyTo(this->depthImage_);

        // store current position and orientation (camera)
        Eigen::Matrix4d camPoseMatrix, camPoseColorMatrix;
        this->getCameraPose(odom, camPoseMatrix, camPoseColorMatrix);

        this->position_(0) = camPoseMatrix(0, 3);
        this->position_(1) = camPoseMatrix(1, 3);
        this->position_(2) = camPoseMatrix(2, 3);
        this->orientation_ = camPoseMatrix.block<3, 3>(0, 0);

        this->positionColor_(0) = camPoseColorMatrix(0, 3);
        this->positionColor_(1) = camPoseColorMatrix(1, 3);
        this->positionColor_(2) = camPoseColorMatrix(2, 3);
        this->orientationColor_ = camPoseColorMatrix.block<3, 3>(0, 0);
    }

    void dynamicDetector::alignedDepthCB(const sensor_msgs::ImageConstPtr& img){
        cv_bridge::CvImagePtr imgPtr = cv_bridge::toCvCopy(img, img->encoding);
        if (img->encoding == sensor_msgs::image_encodings::TYPE_32FC1){
            (imgPtr->image).convertTo(imgPtr->image, CV_16UC1, this->depthScale_);
        }
        imgPtr->image.copyTo(this->alignedDepthImage_);

        cv::Mat depthNormalized;
        imgPtr->image.copyTo(depthNormalized);
        double min, max;
        cv::minMaxIdx(depthNormalized, &min, &max);
        cv::convertScaleAbs(depthNormalized, depthNormalized, 255. / max);
        depthNormalized.convertTo(depthNormalized, CV_8UC1);
        cv::applyColorMap(depthNormalized, depthNormalized, cv::COLORMAP_BONE);
        this->detectedAlignedDepthImg_ = depthNormalized;
    }

    void dynamicDetector::yoloDetectionCB(const vision_msgs::Detection2DArrayConstPtr& detections){
        this->yoloDetectionResults_ = *detections;
    }

    void dynamicDetector::detectionCB(const ros::TimerEvent&){
        // detection thread
        this->dbscanDetect();
        this->uvDetect();
        this->yoloDetectionTo3D();
        this->filterBBoxes();
        this->newDetectFlag_ = true; // get a new detection
    }

    void dynamicDetector::trackingCB(const ros::TimerEvent&){
        // data association thread
        std::vector<int> bestMatch; // for each current detection, which index of previous obstacle match
        this->boxAssociation(bestMatch);

        // kalman filter tracking
        if (bestMatch.size()){
            this->kalmanFilterAndUpdateHist(bestMatch);
        }
        else {
            this->boxHist_.clear();
            this->pcHist_.clear();
        }
    }

    void dynamicDetector::classificationCB(const ros::TimerEvent&){
        // Identification thread
        std::vector<onboardDetector::box3D> dynamicBBoxesTemp;

        // Iterate through all pointcloud/bounding boxes history (note that yolo's pointclouds are dummy pointcloud (empty))
        // NOTE: There are 3 cases which we don't need to perform dynamic obstacle identification.
        for (size_t i=0; i<this->pcHist_.size() ; ++i){
            // ===================================================================================
            // CASE I: yolo recognized as dynamic dynamic obstacle
            if (this->boxHist_[i][0].is_human){
                dynamicBBoxesTemp.push_back(this->boxHist_[i][0]);
                continue;
            }
            // ===================================================================================


            // ===================================================================================
            // CASE II: history length is not enough to run classification
            int curFrameGap;
            if (int(this->pcHist_[i].size()) < this->skipFrame_+1){
                curFrameGap = this->pcHist_[i].size() - 1;
            }
            else{
                curFrameGap = this->skipFrame_;
            }
            // ===================================================================================


            // ==================================================================================
            // CASE III: Force Dynamic (if the obstacle is classifed as dynamic for several time steps)
            int dynaFrames = 0;
            if (int(this->boxHist_[i].size()) > this->forceDynaCheckRange_){
                for (int j=1 ; j<this->forceDynaCheckRange_+1 ; ++j){
                    if (this->boxHist_[i][j].is_dynamic){
                        ++dynaFrames;
                    }
                }
            }

            if (dynaFrames >= this->forceDynaFrames_){
                this->boxHist_[i][0].is_dynamic = true;
                dynamicBBoxesTemp.push_back(this->boxHist_[i][0]);
                continue;
            }
            // ===================================================================================

            std::vector<Eigen::Vector3d> currPc = this->pcHist_[i][0];
            std::vector<Eigen::Vector3d> prevPc = this->pcHist_[i][curFrameGap];
            Eigen::Vector3d Vcur(0.,0.,0.); // single point velocity 
            Eigen::Vector3d Vbox(0.,0.,0.); // bounding box velocity 
            Eigen::Vector3d Vkf(0.,0.,0.);  // velocity estimated from kalman filter
            int numPoints = currPc.size(); // it changes within loop
            int votes = 0;

            Vbox(0) = (this->boxHist_[i][0].x - this->boxHist_[i][curFrameGap].x)/(this->dt_*curFrameGap);
            Vbox(1) = (this->boxHist_[i][0].y - this->boxHist_[i][curFrameGap].y)/(this->dt_*curFrameGap);
            Vbox(2) = (this->boxHist_[i][0].z - this->boxHist_[i][curFrameGap].z)/(this->dt_*curFrameGap);
            Vkf(0) = this->boxHist_[i][0].Vx;
            Vkf(1) = this->boxHist_[i][0].Vy;

            // find nearest neighbor
            int numSkip = 0;
            for (size_t j=0 ; j<currPc.size() ; ++j){
                // don't perform classification for points unseen in previous frame
                if (!this->isInFov(this->positionHist_[curFrameGap], this->orientationHist_[curFrameGap], currPc[j])){
                    ++numSkip;
                    --numPoints;
                    continue;
                }

                double minDist = 2;
                Eigen::Vector3d nearestVect;
                for (size_t k=0 ; k<prevPc.size() ; k++){ // find the nearest point in the previous pointcloud
                    double dist = (currPc[j]-prevPc[k]).norm();
                    if (abs(dist) < minDist){
                        minDist = dist;
                        nearestVect = currPc[j]-prevPc[k];
                    }
                }
                Vcur = nearestVect/(this->dt_*curFrameGap); Vcur(2) = 0;
                double velSim = Vcur.dot(Vbox)/(Vcur.norm()*Vbox.norm());

                if (velSim < 0){
                    ++numSkip;
                    --numPoints;
                }
                else{
                    if (Vcur.norm()>this->dynaVelThresh_){
                        ++votes;
                    }
                }
            }
            
            
            // update dynamic boxes
            double voteRatio = (numPoints>0)?double(votes)/double(numPoints):0;
            double velNorm = Vkf.norm();

            // voting and velocity threshold
            // 1. point cloud voting ratio.
            // 2. velocity (from kalman filter) 
            // 3. enough valid point correspondence 
            if (voteRatio>=this->dynaVoteThresh_ && velNorm>=this->dynaVelThresh_ && double(numSkip)/double(numPoints)<this->maxSkipRatio_){
                this->boxHist_[i][0].is_dynamic_candidate = true;
                // dynamic-consistency check
                int dynaConsistCount = 0;
                if (int(this->boxHist_[i].size()) >= this->dynamicConsistThresh_){
                    for (int j=0 ; j<this->dynamicConsistThresh_; ++j){
                        if (this->boxHist_[i][j].is_dynamic_candidate){
                            ++dynaConsistCount;
                        }
                    }
                }            
                if (dynaConsistCount == this->dynamicConsistThresh_){
                    // set as dynamic and push into history
                    this->boxHist_[i][0].is_dynamic = true;
                    dynamicBBoxesTemp.push_back(this->boxHist_[i][0]);    
                }
            }
        }

        // filter the dynamic obstacles based on the target sizes
        if (this->constrainSize_){
            std::vector<onboardDetector::box3D> dynamicBBoxesBeforeConstrain = dynamicBBoxesTemp;
            dynamicBBoxesTemp.clear();

            for (onboardDetector::box3D ob : dynamicBBoxesBeforeConstrain){
                bool findMatch = false;
                for (Eigen::Vector3d targetSize : this->targetObjectSize_){
                    double xdiff = std::abs(ob.x_width - targetSize(0));
                    double ydiff = std::abs(ob.y_width - targetSize(1));
                    double zdiff = std::abs(ob.z_width - targetSize(2)); 
                    if (xdiff < 0.5 and ydiff < 0.5 and zdiff < 0.5){
                        findMatch = true;
                    }
                }

                if (findMatch){
                    dynamicBBoxesTemp.push_back(ob);
                }
            }
        }

        this->dynamicBBoxes_ = dynamicBBoxesTemp;
    }

    void dynamicDetector::visCB(const ros::TimerEvent&){
        this->publishUVImages();
        this->publish3dBox(this->uvBBoxes_, this->uvBBoxesPub_, 0, 1, 0);
        std::vector<Eigen::Vector3d> dynamicPoints;
        this->getDynamicPc(dynamicPoints);
        this->publishPoints(dynamicPoints, this->dynamicPointsPub_);
        this->publishPoints(this->filteredPoints_, this->filteredPointsPub_);
        this->publish3dBox(this->dbBBoxes_, this->dbBBoxesPub_, 1, 0, 0);
        this->publishYoloImages();
        this->publish3dBox(this->yoloBBoxes_, this->yoloBBoxesPub_, 1, 0, 1);
        this->publish3dBox(this->filteredBBoxes_, this->filteredBBoxesPub_, 0, 1, 1);
        this->publish3dBox(this->trackedBBoxes_, this->trackedBBoxesPub_, 1, 1, 0);
        this->publish3dBox(this->dynamicBBoxes_, this->dynamicBBoxesPub_, 0, 0, 1);
        this->publishHistoryTraj();
        this->publishVelVis();
    }

    void dynamicDetector::uvDetect(){
        // initialization
        if (this->uvDetector_ == NULL){
            this->uvDetector_.reset(new UVdetector ());
            this->uvDetector_->fx = this->fx_;
            this->uvDetector_->fy = this->fy_;
            this->uvDetector_->px = this->cx_;
            this->uvDetector_->py = this->cy_;
            this->uvDetector_->depthScale_ = this->depthScale_; 
            this->uvDetector_->max_dist = this->raycastMaxLength_ * 1000;
        }

        // detect from depth mapcalBox
        if (not this->depthImage_.empty()){
            this->uvDetector_->depth = this->depthImage_;
            this->uvDetector_->detect();
            this->uvDetector_->extract_3Dbox();

            this->uvDetector_->display_U_map();
            this->uvDetector_->display_bird_view();
            this->uvDetector_->display_depth();

            // transform to the world frame (recalculate the boudning boxes)
            std::vector<onboardDetector::box3D> uvBBoxes;
            this->transformUVBBoxes(uvBBoxes);
            this->uvBBoxes_ = uvBBoxes;
        }
    }

    void dynamicDetector::dbscanDetect(){
        // 1. get pointcloud
        this->projectDepthImage();

        // 2. update pose history
        this->updatePoseHist();

        // 3. filter points
        this->filterPoints(this->projPoints_, this->filteredPoints_);

        // 4. cluster points and get bounding boxes
        this->clusterPointsAndBBoxes(this->filteredPoints_, this->dbBBoxes_, this->pcClusters_, this->pcClusterCenters_, this->pcClusterStds_);
    }

    void dynamicDetector::yoloDetectionTo3D(){
        std::vector<onboardDetector::box3D> yoloBBoxesTemp;
        for (size_t i=0; i<this->yoloDetectionResults_.detections.size(); ++i){
            onboardDetector::box3D bbox3D;
            cv::Rect bboxVis;
            this->getYolo3DBBox(this->yoloDetectionResults_.detections[i], bbox3D, bboxVis);
            cv::rectangle(this->detectedAlignedDepthImg_, bboxVis, cv::Scalar(0, 255, 0), 5, 8, 0);
            yoloBBoxesTemp.push_back(bbox3D);
        }
        this->yoloBBoxes_ = yoloBBoxesTemp;    
    }

    void dynamicDetector::filterBBoxes(){
        std::vector<onboardDetector::box3D> filteredBBoxesTemp;
        std::vector<std::vector<Eigen::Vector3d>> filteredPcClustersTemp;
        std::vector<Eigen::Vector3d> filteredPcClusterCentersTemp;
        std::vector<Eigen::Vector3d> filteredPcClusterStdsTemp; 
        // find best IOU match for both uv and dbscan. If they are best for each other, then add to filtered bbox and fuse.
        for (size_t i=0 ; i<this->uvBBoxes_.size(); ++i){
            onboardDetector::box3D uvBBox = this->uvBBoxes_[i];
            double bestIOUForUVBBox, bestIOUForDBBBox;
            int bestMatchForUVBBox = this->getBestOverlapBBox(uvBBox, this->dbBBoxes_, bestIOUForUVBBox);
            if (bestMatchForUVBBox == -1) continue; // no match at all
            onboardDetector::box3D matchedDBBBox = this->dbBBoxes_[bestMatchForUVBBox]; 
            std::vector<Eigen::Vector3d> matchedPcCluster = this->pcClusters_[bestMatchForUVBBox];
            Eigen::Vector3d matchedPcClusterCenter = this->pcClusterCenters_[bestMatchForUVBBox];
            Eigen::Vector3d matchedPcClusterStd = this->pcClusterStds_[bestMatchForUVBBox];
            int bestMatchForDBBBox = this->getBestOverlapBBox(matchedDBBBox, this->uvBBoxes_, bestIOUForDBBBox);

            // if best match is each other and both the IOU is greater than the threshold
            if (bestMatchForDBBBox == int(i) and bestIOUForUVBBox > this->boxIOUThresh_ and bestIOUForDBBBox > this->boxIOUThresh_){
                onboardDetector::box3D bbox;
                
                // take concervative strategy
                double xmax = std::max(uvBBox.x+uvBBox.x_width/2, matchedDBBBox.x+matchedDBBBox.x_width/2);
                double xmin = std::min(uvBBox.x-uvBBox.x_width/2, matchedDBBBox.x-matchedDBBBox.x_width/2);
                double ymax = std::max(uvBBox.y+uvBBox.y_width/2, matchedDBBBox.y+matchedDBBBox.y_width/2);
                double ymin = std::min(uvBBox.y-uvBBox.y_width/2, matchedDBBBox.y-matchedDBBBox.y_width/2);
                double zmax = std::max(uvBBox.z+uvBBox.z_width/2, matchedDBBBox.z+matchedDBBBox.z_width/2);
                double zmin = std::min(uvBBox.z-uvBBox.z_width/2, matchedDBBBox.z-matchedDBBBox.z_width/2);
                bbox.x = (xmin+xmax)/2;
                bbox.y = (ymin+ymax)/2;
                bbox.z = (zmin+zmax)/2;
                bbox.x_width = xmax-xmin;
                bbox.y_width = ymax-ymin;
                bbox.z_width = zmax-zmin;
                bbox.Vx = 0;
                bbox.Vy = 0;

                filteredBBoxesTemp.push_back(bbox);
                filteredPcClustersTemp.push_back(matchedPcCluster);      
                filteredPcClusterCentersTemp.push_back(matchedPcClusterCenter);
                filteredPcClusterStdsTemp.push_back(matchedPcClusterStd);
            }
        }

        // yolo bounding box filter
        if (this->yoloBBoxes_.size() != 0){ // if no detected or not using yolo, this will not triggered
            std::vector<onboardDetector::box3D> filteredBBoxesTempCopy = filteredBBoxesTemp;
            std::vector<std::vector<Eigen::Vector3d>> filteredPcClustersTempCopy = filteredPcClustersTemp;
            std::vector<Eigen::Vector3d> filteredPcClusterCentersTempCopy = filteredPcClusterCentersTemp;
            std::vector<Eigen::Vector3d> filteredPcClusterStdsTempCopy = filteredPcClusterStdsTemp;
            std::vector<Eigen::Vector3d> emptyPoints {};
            Eigen::Vector3d emptyPcFeat {0,0,0};
            for (size_t i=0; i<this->yoloBBoxes_.size(); ++i){
                onboardDetector::box3D yoloBBox = this->yoloBBoxes_[i]; yoloBBox.is_dynamic = true; yoloBBox.is_human = true; // dynamic obstacle detected by yolo
                Eigen::Vector3d bboxPos (this->yoloBBoxes_[i].x, this->yoloBBoxes_[i].y, this->yoloBBoxes_[i].z);
                double distanceToCamera = (bboxPos - this->position_).norm();
                if (distanceToCamera >= this->raycastMaxLength_){
                    continue; // do not use unreliable YOLO resutls which are distance too far from camera
                }
                double bestIOUForYoloBBox, bestIOUForFilteredBBox;
                int bestMatchForYoloBBox = this->getBestOverlapBBox(yoloBBox, filteredBBoxesTemp, bestIOUForYoloBBox);
                if (bestMatchForYoloBBox == -1){ // no match for yolo bounding boxes with any filtered bbox. 2 reasons: a) distance too far, filtered boxes no detection, b) distance not far but cannot match. Probably Yolo error
                    if (distanceToCamera >= this->yoloOverwriteDistance_){ // a) distance too far, filtered boxes no detection. directly add results
                        filteredBBoxesTempCopy.push_back(yoloBBox); // add yolo bbox because filtered bbox is not able to get detection results at far distance
                        filteredPcClustersTempCopy.push_back(emptyPoints); // no pc need for yolo 
                        filteredPcClusterCentersTempCopy.push_back(emptyPcFeat);
                        filteredPcClusterStdsTempCopy.push_back(emptyPcFeat);
                    }
                    else{ // b) distance not far but cannot match. Probably Yolo error, ignore results
                        continue;
                    }
                }
                else{ // find best match for yolo bbox
                    onboardDetector::box3D matchedFilteredBBox = filteredBBoxesTemp[bestMatchForYoloBBox];
                    int bestMatchForFilteredBBox = this->getBestOverlapBBox(matchedFilteredBBox, this->yoloBBoxes_, bestIOUForFilteredBBox);
                    // if best match is each other and both the IOU is greater than the threshold
                    if (bestMatchForFilteredBBox == int(i) and bestIOUForYoloBBox > this->boxIOUThresh_ and bestIOUForFilteredBBox > this->boxIOUThresh_){
                        onboardDetector::box3D bbox; bbox.is_dynamic = true; bbox.is_human = true;
                        
                        // take concervative strategy
                        double xmax = std::max(yoloBBox.x+yoloBBox.x_width/2, matchedFilteredBBox.x+matchedFilteredBBox.x_width/2);
                        double xmin = std::min(yoloBBox.x-yoloBBox.x_width/2, matchedFilteredBBox.x-matchedFilteredBBox.x_width/2);
                        double ymax = std::max(yoloBBox.y+yoloBBox.y_width/2, matchedFilteredBBox.y+matchedFilteredBBox.y_width/2);
                        double ymin = std::min(yoloBBox.y-yoloBBox.y_width/2, matchedFilteredBBox.y-matchedFilteredBBox.y_width/2);
                        double zmax = std::max(yoloBBox.z+yoloBBox.z_width/2, matchedFilteredBBox.z+matchedFilteredBBox.z_width/2);
                        double zmin = std::min(yoloBBox.z-yoloBBox.z_width/2, matchedFilteredBBox.z-matchedFilteredBBox.z_width/2);
                        bbox.x = (xmin+xmax)/2;
                        bbox.y = (ymin+ymax)/2;
                        bbox.z = (zmin+zmax)/2;
                        bbox.x_width = xmax-xmin;
                        bbox.y_width = ymax-ymin;
                        bbox.z_width = zmax-zmin;
                        bbox.Vx = 0;
                        bbox.Vy = 0;
                        
                        filteredBBoxesTempCopy[bestMatchForYoloBBox] = bbox; // replace the filtered bbox with the new fused bounding box
                        filteredPcClustersTempCopy[bestMatchForYoloBBox] = emptyPoints;      // since it is yolo based, we dont need pointcloud for classification                     
                        filteredPcClusterCentersTempCopy[bestMatchForYoloBBox] = emptyPcFeat;
                        filteredPcClusterStdsTempCopy[bestMatchForYoloBBox] = emptyPcFeat;
                    }
                }
            }
            filteredBBoxesTemp = filteredBBoxesTempCopy;
            filteredPcClustersTemp = filteredPcClustersTempCopy;
            filteredPcClusterCentersTemp = filteredPcClusterCentersTempCopy;
            filteredPcClusterStdsTemp = filteredPcClusterStdsTempCopy;
        }

        this->filteredBBoxes_ = filteredBBoxesTemp;
        this->filteredPcClusters_ = filteredPcClustersTemp;
        this->filteredPcClusterCenters_ = filteredPcClusterCentersTemp;
        this->filteredPcClusterStds_ = filteredPcClusterStdsTemp;
    }

    void dynamicDetector::transformUVBBoxes(std::vector<onboardDetector::box3D>& bboxes){
        bboxes.clear();
        for(size_t i = 0; i < this->uvDetector_->box3Ds.size(); ++i){
            onboardDetector::box3D bbox;
            double x = this->uvDetector_->box3Ds[i].x; 
            double y = this->uvDetector_->box3Ds[i].y;
            double z = this->uvDetector_->box3Ds[i].z;
            double xWidth = this->uvDetector_->box3Ds[i].x_width;
            double yWidth = this->uvDetector_->box3Ds[i].y_width;
            double zWidth = this->uvDetector_->box3Ds[i].z_width;

            Eigen::Vector3d center (x, y, z);
            Eigen::Vector3d size (xWidth, yWidth, zWidth);
            Eigen::Vector3d newCenter, newSize;

            this->transformBBox(center, size, this->position_, this->orientation_, newCenter, newSize);

            // assign values to bounding boxes in the map frame
            bbox.x = newCenter(0);
            bbox.y = newCenter(1);
            bbox.z = newCenter(2);
            bbox.x_width = newSize(0);
            bbox.y_width = newSize(1);
            bbox.z_width = newSize(2);
            bboxes.push_back(bbox);            
        }        
    }

    void dynamicDetector::projectDepthImage(){
        this->projPointsNum_ = 0;

        int cols = this->depthImage_.cols;
        int rows = this->depthImage_.rows;
        uint16_t* rowPtr;

        Eigen::Vector3d currPointCam, currPointMap;
        double depth;
        const double inv_factor = 1.0 / this->depthScale_;
        const double inv_fx = 1.0 / this->fx_;
        const double inv_fy = 1.0 / this->fy_;

        // iterate through each pixel in the depth image
        for (int v=this->depthFilterMargin_; v<rows-this->depthFilterMargin_; v=v+this->skipPixel_){ // row
            rowPtr = this->depthImage_.ptr<uint16_t>(v) + this->depthFilterMargin_;
            for (int u=this->depthFilterMargin_; u<cols-this->depthFilterMargin_; u=u+this->skipPixel_){ // column
                depth = (*rowPtr) * inv_factor;
                
                if (*rowPtr == 0) {
                    depth = this->raycastMaxLength_ + 0.1;
                } else if (depth < this->depthMinValue_) {
                    continue;
                } else if (depth > this->depthMaxValue_) {
                    depth = this->raycastMaxLength_ + 0.1;
                }
                rowPtr =  rowPtr + this->skipPixel_;

                // get 3D point in camera frame
                currPointCam(0) = (u - this->cx_) * depth * inv_fx;
                currPointCam(1) = (v - this->cy_) * depth * inv_fy;
                currPointCam(2) = depth;
                currPointMap = this->orientation_ * currPointCam + this->position_; // transform to map coordinate

                this->projPoints_[this->projPointsNum_] = currPointMap;
                this->pointsDepth_[this->projPointsNum_] = depth;
                this->projPointsNum_ = this->projPointsNum_ + 1;
            }
        } 
    }

    void dynamicDetector::filterPoints(const std::vector<Eigen::Vector3d>& points, std::vector<Eigen::Vector3d>& filteredPoints){
        // currently there is only one filtered (might include more in the future)
        std::vector<Eigen::Vector3d> voxelFilteredPoints;
        this->voxelFilter(points, voxelFilteredPoints);
        filteredPoints = voxelFilteredPoints;
    }


    void dynamicDetector::clusterPointsAndBBoxes(const std::vector<Eigen::Vector3d>& points, std::vector<onboardDetector::box3D>& bboxes, std::vector<std::vector<Eigen::Vector3d>>& pcClusters, std::vector<Eigen::Vector3d>& pcClusterCenters, std::vector<Eigen::Vector3d>& pcClusterStds){
        std::vector<onboardDetector::Point> pointsDB;
        this->eigenToDBPointVec(points, pointsDB, points.size());

        this->dbCluster_.reset(new DBSCAN (this->dbMinPointsCluster_, this->dbEpsilon_, pointsDB));

        // DBSCAN clustering
        this->dbCluster_->run();

        // get the cluster data with bounding boxes
        // iterate through all the clustered points and find number of clusters
        int clusterNum = 0;
        for (size_t i=0; i<this->dbCluster_->m_points.size(); ++i){
            onboardDetector::Point pDB = this->dbCluster_->m_points[i];
            if (pDB.clusterID > clusterNum){
                clusterNum = pDB.clusterID;
            }
        }

        pcClusters.clear();
        pcClusters.resize(clusterNum);
        for (size_t i=0; i<this->dbCluster_->m_points.size(); ++i){
            onboardDetector::Point pDB = this->dbCluster_->m_points[i];
            if (pDB.clusterID > 0){
                Eigen::Vector3d p = this->dbPointToEigen(pDB);
                pcClusters[pDB.clusterID-1].push_back(p);
            }            
        }

        for (size_t i=0 ; i<pcClusters.size() ; ++i){
            Eigen::Vector3d pcClusterCenter(0.,0.,0.);
            Eigen::Vector3d pcClusterStd(0.,0.,0.);
            this->calcPcFeat(pcClusters[i], pcClusterCenter, pcClusterStd);
            pcClusterCenters.push_back(pcClusterCenter);
            pcClusterStds.push_back(pcClusterStd);
        }

        // calculate the bounding boxes based on the clusters
        bboxes.clear();
        // bboxes.resize(clusterNum);
        for (size_t i=0; i<pcClusters.size(); ++i){
            onboardDetector::box3D box;

            double xmin = pcClusters[i][0](0);
            double ymin = pcClusters[i][0](1);
            double zmin = pcClusters[i][0](2);
            double xmax = pcClusters[i][0](0);
            double ymax = pcClusters[i][0](1);
            double zmax = pcClusters[i][0](2);
            for (size_t j=0; j<pcClusters[i].size(); ++j){
                xmin = (pcClusters[i][j](0)<xmin)?pcClusters[i][j](0):xmin;
                ymin = (pcClusters[i][j](1)<ymin)?pcClusters[i][j](1):ymin;
                zmin = (pcClusters[i][j](2)<zmin)?pcClusters[i][j](2):zmin;
                xmax = (pcClusters[i][j](0)>xmax)?pcClusters[i][j](0):xmax;
                ymax = (pcClusters[i][j](1)>ymax)?pcClusters[i][j](1):ymax;
                zmax = (pcClusters[i][j](2)>zmax)?pcClusters[i][j](2):zmax;
            }
            box.id = i;

            box.x = (xmax + xmin)/2.0;
            box.y = (ymax + ymin)/2.0;
            box.z = (zmax + zmin)/2.0;
            box.x_width = (xmax - xmin)>0.1?(xmax-xmin):0.1;
            box.y_width = (ymax - ymin)>0.1?(ymax-ymin):0.1;
            box.z_width = (zmax - zmin);
            bboxes.push_back(box);
        }
    }

    void dynamicDetector::voxelFilter(const std::vector<Eigen::Vector3d>& points, std::vector<Eigen::Vector3d>& filteredPoints){
        const double res = 0.1; // resolution of voxel
        int xVoxels = ceil(2*this->localSensorRange_(0)/res); int yVoxels = ceil(2*this->localSensorRange_(1)/res); int zVoxels = ceil(2*this->localSensorRange_(2)/res);
        int totalVoxels = xVoxels * yVoxels * zVoxels;
        // std::vector<bool> voxelOccupancyVec (totalVoxels, false);
        std::vector<int> voxelOccupancyVec (totalVoxels, 0);

        // Iterate through each points in the cloud
        filteredPoints.clear();
        
        for (int i=0; i<this->projPointsNum_; ++i){
            Eigen::Vector3d p = points[i];

            if (this->isInFilterRange(p) and p(2) >= this->groundHeight_ and this->pointsDepth_[i] <= this->raycastMaxLength_){
                // find the corresponding voxel id in the vector and check whether it is occupied
                int pID = this->posToAddress(p, res);

                // add one point
                voxelOccupancyVec[pID] +=1;

                // add only if thresh points are found
                if (voxelOccupancyVec[pID] == this->voxelOccThresh_){
                    filteredPoints.push_back(p);
                }
            }
        }  
    }

    void dynamicDetector::calcPcFeat(const std::vector<Eigen::Vector3d>& pcCluster, Eigen::Vector3d& pcClusterCenter, Eigen::Vector3d& pcClusterStd){
        int numPoints = pcCluster.size();
        
        // center
        for (int i=0 ; i<numPoints ; i++){
            pcClusterCenter(0) += pcCluster[i](0)/numPoints;
            pcClusterCenter(1) += pcCluster[i](1)/numPoints;
            pcClusterCenter(2) += pcCluster[i](2)/numPoints;
        }

        // std
        for (int i=0 ; i<numPoints ; i++){
            pcClusterStd(0) += std::pow(pcCluster[i](0) - pcClusterCenter(0),2);
            pcClusterStd(1) += std::pow(pcCluster[i](1) - pcClusterCenter(1),2);
            pcClusterStd(2) += std::pow(pcCluster[i](2) - pcClusterCenter(2),2);
        }        

        // take square root
        pcClusterStd(0) = std::sqrt(pcClusterStd(0)/numPoints);
        pcClusterStd(1) = std::sqrt(pcClusterStd(1)/numPoints);
        pcClusterStd(2) = std::sqrt(pcClusterStd(2)/numPoints);
    }


    double dynamicDetector::calBoxIOU(const onboardDetector::box3D& box1, const onboardDetector::box3D& box2){
        double box1Volume = box1.x_width * box1.y_width * box1.z_width;
        double box2Volume = box2.x_width * box2.y_width * box2.z_width;

        double l1Y = box1.y+box1.y_width/2-(box2.y-box2.y_width/2);
        double l2Y = box2.y+box2.y_width/2-(box1.y-box1.y_width/2);
        double l1X = box1.x+box1.x_width/2-(box2.x-box2.x_width/2);
        double l2X = box2.x+box2.x_width/2-(box1.x-box1.x_width/2);
        double l1Z = box1.z+box1.z_width/2-(box2.z-box2.z_width/2);
        double l2Z = box2.z+box2.z_width/2-(box1.z-box1.z_width/2);
        double overlapX = std::min( l1X , l2X );
        double overlapY = std::min( l1Y , l2Y );
        double overlapZ = std::min( l1Z , l2Z );
       
        if (std::max(l1X, l2X)<=std::max(box1.x_width,box2.x_width)){ 
            overlapX = std::min(box1.x_width, box2.x_width);
        }
        if (std::max(l1Y, l2Y)<=std::max(box1.y_width,box2.y_width)){ 
            overlapY = std::min(box1.y_width, box2.y_width);
        }
        if (std::max(l1Z, l2Z)<=std::max(box1.z_width,box2.z_width)){ 
            overlapZ = std::min(box1.z_width, box2.z_width);
        }


        double overlapVolume = overlapX * overlapY *  overlapZ;
        double IOU = overlapVolume / (box1Volume+box2Volume-overlapVolume);
        
        // D-IOU
        if (overlapX<=0 || overlapY<=0 ||overlapZ<=0){
            IOU = 0;
        }
        return IOU;
    }

    void dynamicDetector::getYolo3DBBox(const vision_msgs::Detection2D& detection, onboardDetector::box3D& bbox3D, cv::Rect& bboxVis){
        if (this->alignedDepthImage_.empty()){
            return;
        }

        const Eigen::Vector3d humanSize (0.5, 0.5, 1.8);

        // 1. retrive 2D detection result
        int topX = int(detection.bbox.center.x); 
        int topY = int(detection.bbox.center.y); 
        int xWidth = int(detection.bbox.size_x); 
        int yWidth = int(detection.bbox.size_y); 
        bboxVis.x = topX;
        bboxVis.y = topY;
        bboxVis.height = yWidth;
        bboxVis.width = xWidth;

        // 2. get thickness estimation (double MAD: double Median Absolute Deviation)
        uint16_t* rowPtr;
        double depth;
        const double inv_factor = 1.0 / this->depthScale_;
        int vMin = std::max(topY, this->depthFilterMargin_);
        int uMin = std::max(topX, this->depthFilterMargin_);
        int vMax = std::min(topY+yWidth, this->imgRows_-this->depthFilterMargin_);
        int uMax = std::min(topX+xWidth, this->imgCols_-this->depthFilterMargin_);
        std::vector<double> depthValues;


        // record the depth values in the potential regions
        for (int v=vMin; v<vMax; ++v){ // row
            rowPtr = this->alignedDepthImage_.ptr<uint16_t>(v);
            for (int u=uMin; u<uMax; ++u){ // column
                depth = (*rowPtr) * inv_factor;
                if (depth >= this->depthMinValue_ and depth <= this->depthMaxValue_){
                    depthValues.push_back(depth);
                }
                ++rowPtr;
            }
        }
        if (depthValues.size() == 0){ // in case of out of range
            return;
        }

        // double MAD calculation
        double depthMedian, MAD;
        this->calculateMAD(depthValues, depthMedian, MAD);
        // cout << "MAD: " << MAD << endl;

        double depthMin = 10.0; double depthMax = -10.0;
        // find min max depth value
        for (int v=vMin; v<vMax; ++v){ // row
            rowPtr = this->alignedDepthImage_.ptr<uint16_t>(v);
            for (int u=uMin; u<uMax; ++u){ // column
                depth = (*rowPtr) * inv_factor;
                if (depth >= this->depthMinValue_ and depth <= this->depthMaxValue_){
                    if ((depth < depthMin) and (depth >= depthMedian - 1.5 * MAD)){
                        depthMin = depth;
                    }

                    if ((depth > depthMax) and (depth <= depthMedian + 1.5 * MAD)){
                        depthMax = depth;
                    }
                }
                ++rowPtr;
            }
        }
        
        if (depthMin == 10.0 or depthMax == -10.0){ // in case the depth value is not available
            return;
        }

        // 3. project points into 3D in the camera frame
        Eigen::Vector3d pUL, pBR, center;
        pUL(0) = (topX - this->cxC_) * depthMedian / this->fxC_;
        pUL(1) = (topY - this->cyC_) * depthMedian / this->fyC_;
        pUL(2) = depthMedian;

        pBR(0) = (topX + xWidth - this->cxC_) * depthMedian / this->fxC_;
        pBR(1) = (topY + yWidth- this->cyC_) * depthMedian / this->fyC_;
        pBR(2) = depthMedian;

        center(0) = (pUL(0) + pBR(0))/2.0;
        center(1) = (pUL(1) + pBR(1))/2.0;
        center(2) = depthMedian;

        double xWidth3D = std::abs(pBR(0) - pUL(0));
        double yWidth3D = std::abs(pBR(1) - pUL(1));
        double zWidth3D = depthMax - depthMin; 
        if ((zWidth3D/humanSize(2)>=2.0) or (zWidth3D/humanSize(2) <= 0.5)){ // error is too large, then use the predefined size
            zWidth3D = humanSize(2);
        }       
        Eigen::Vector3d size (xWidth3D, yWidth3D, zWidth3D);

        // 4. transform 3D points into world frame
        Eigen::Vector3d newCenter, newSize;
        this->transformBBox(center, size, this->positionColor_, this->orientationColor_, newCenter, newSize);
        bbox3D.x = newCenter(0);
        bbox3D.y = newCenter(1);
        bbox3D.z = newCenter(2);

        bbox3D.x_width = newSize(0);
        bbox3D.y_width = newSize(1);
        bbox3D.z_width = newSize(2);

        // 5. check the bounding box size. If the bounding box size is too different from the predefined size, overwrite the size
        if ((bbox3D.x_width/humanSize(0)>=2.0) or (bbox3D.x_width/humanSize(0)<=0.5)){
            bbox3D.x_width = humanSize(0);
        }

        if ((bbox3D.y_width/humanSize(1)>=2.0) or (bbox3D.y_width/humanSize(1)<=0.5)){
            bbox3D.y_width = humanSize(1);
        }

        if ((bbox3D.z_width/humanSize(2)>=2.0) or (bbox3D.z_width/humanSize(2)<=0.5)){
            bbox3D.z = humanSize(2)/2.;
            bbox3D.z_width = humanSize(2);
        }
    }


    void dynamicDetector::calculateMAD(std::vector<double>& depthValues, double& depthMedian, double& MAD){
        std::sort(depthValues.begin(), depthValues.end());
        int medianIdx = int(depthValues.size()/2);
        depthMedian = depthValues[medianIdx]; // median of all data

        std::vector<double> deviations;
        for (size_t i=0; i<depthValues.size(); ++i){
            deviations.push_back(std::abs(depthValues[i] - depthMedian));
        }
        std::sort(deviations.begin(), deviations.end());
        MAD = deviations[int(deviations.size()/2)];
    }


    void dynamicDetector::boxAssociation(std::vector<int>& bestMatch){
        int numObjs = this->filteredBBoxes_.size();
        
        if (this->boxHist_.size() == 0){ // initialize new bounding box history if no history exists
            this->boxHist_.resize(numObjs);
            this->pcHist_.resize(numObjs);
            bestMatch.resize(this->filteredBBoxes_.size(), -1); // first detection no match
            for (int i=0 ; i<numObjs ; ++i){
                // initialize history for bbox, pc and KF
                this->boxHist_[i].push_back(this->filteredBBoxes_[i]);
                this->pcHist_[i].push_back(this->filteredPcClusters_[i]);
                MatrixXd states, A, B, H, P, Q, R;       
                this->kalmanFilterMatrixAcc(this->filteredBBoxes_[i], states, A, B, H, P, Q, R);
                onboardDetector::kalman_filter newFilter;
                newFilter.setup(states, A, B, H, P, Q, R);
                this->filters_.push_back(newFilter);
            }
        }
        else{
            // start association only if a new detection is available
            if (this->newDetectFlag_){
                this->boxAssociationHelper(bestMatch);
            }
        }

        this->newDetectFlag_ = false; // the most recent detection has been associated
    }

    void dynamicDetector::boxAssociationHelper(std::vector<int>& bestMatch){
        int numObjs = this->filteredBBoxes_.size();
        std::vector<onboardDetector::box3D> propedBoxes;
        std::vector<Eigen::VectorXd> propedBoxesFeat;
        std::vector<Eigen::VectorXd> currBoxesFeat;
        bestMatch.resize(numObjs);
        std::deque<std::deque<onboardDetector::box3D>> boxHistTemp; 

        // linear propagation: prediction of previous box in current frame
        this->linearProp(propedBoxes);

        // generate feature
        this->genFeat(propedBoxes, numObjs, propedBoxesFeat, currBoxesFeat);

        // calculate association: find best match
        this->findBestMatch(propedBoxesFeat, currBoxesFeat, propedBoxes, bestMatch);
    
    }

    void dynamicDetector::genFeat(const std::vector<onboardDetector::box3D>& propedBoxes, int numObjs, std::vector<Eigen::VectorXd>& propedBoxesFeat, std::vector<Eigen::VectorXd>& currBoxesFeat){
        propedBoxesFeat.resize(propedBoxes.size());
        currBoxesFeat.resize(numObjs);
        this->genFeatHelper(propedBoxesFeat, propedBoxes);
        this->genFeatHelper(currBoxesFeat, this->filteredBBoxes_);
    }

    void dynamicDetector::genFeatHelper(std::vector<Eigen::VectorXd>& features, const std::vector<onboardDetector::box3D>& boxes){ 
        Eigen::VectorXd featureWeights(10); // 3pos + 3size + 1 pc length + 3 pc std
        featureWeights << 2, 2, 2, 1, 1, 1, 0.5, 0.5, 0.5, 0.5;
        for (size_t i=0 ; i<boxes.size() ; i++){
            Eigen::VectorXd feature(10);
            features[i] = feature;
            features[i](0) = (boxes[i].x - this->position_(0)) * featureWeights(0) ;
            features[i](1) = (boxes[i].y - this->position_(1)) * featureWeights(1);
            features[i](2) = (boxes[i].z - this->position_(2)) * featureWeights(2);
            features[i](3) = boxes[i].x_width * featureWeights(3);
            features[i](4) = boxes[i].y_width * featureWeights(4);
            features[i](5) = boxes[i].z_width * featureWeights(5);
            features[i](6) = this->filteredPcClusters_[i].size() * featureWeights(6);
            features[i](7) = this->filteredPcClusterStds_[i](0) * featureWeights(7);
            features[i](8) = this->filteredPcClusterStds_[i](1) * featureWeights(8);
            features[i](9) = this->filteredPcClusterStds_[i](2) * featureWeights(9);
        }
    }

    void dynamicDetector::linearProp(std::vector<onboardDetector::box3D>& propedBoxes){
        onboardDetector::box3D propedBox;
        for (size_t i=0 ; i<this->boxHist_.size() ; i++){
            propedBox = this->boxHist_[i][0];
            propedBox.x += propedBox.Vx*this->dt_;
            propedBox.y += propedBox.Vy*this->dt_;
            propedBoxes.push_back(propedBox);
        }
    }

    void dynamicDetector::findBestMatch(const std::vector<Eigen::VectorXd>& propedBoxesFeat, const std::vector<Eigen::VectorXd>& currBoxesFeat, const std::vector<onboardDetector::box3D>& propedBoxes, std::vector<int>& bestMatch){
        
        int numObjs = this->filteredBBoxes_.size();
        std::vector<double> bestSims; // best similarity
        bestSims.resize(numObjs);

        for (int i=0 ; i<numObjs ; i++){
            double bestSim = -1.;
            int bestMatchInd = -1;
            for (size_t j=0 ; j<propedBoxes.size() ; j++){
                double sim = propedBoxesFeat[j].dot(currBoxesFeat[i])/(propedBoxesFeat[j].norm()*currBoxesFeat[i].norm());
                if (sim >= bestSim){
                    bestSim = sim;
                    bestSims[i] = sim;
                    bestMatchInd = j;
                }
            }

            double iou = this->calBoxIOU(this->filteredBBoxes_[i], propedBoxes[bestMatchInd]);
            if(!(bestSims[i]>this->simThresh_ && iou)){
                bestSims[i] = 0;
                bestMatch[i] = -1;
            }
            else {
                bestMatch[i] = bestMatchInd;
            }
        }
    }


    void dynamicDetector::kalmanFilterAndUpdateHist(const std::vector<int>& bestMatch){
        std::vector<std::deque<onboardDetector::box3D>> boxHistTemp; 
        std::vector<std::deque<std::vector<Eigen::Vector3d>>> pcHistTemp;
        std::vector<onboardDetector::kalman_filter> filtersTemp;
        std::deque<onboardDetector::box3D> newSingleBoxHist;
        std::deque<std::vector<Eigen::Vector3d>> newSinglePcHist; 
        onboardDetector::kalman_filter newFilter;
        std::vector<onboardDetector::box3D> trackedBBoxesTemp;

        newSingleBoxHist.resize(0);
        newSinglePcHist.resize(0);
        int numObjs = this->filteredBBoxes_.size();

        for (int i=0 ; i<numObjs ; i++){
            onboardDetector::box3D newEstimatedBBox; // from kalman filter

            // inheret history. push history one by one
            if (bestMatch[i]>=0){
                boxHistTemp.push_back(this->boxHist_[bestMatch[i]]);
                pcHistTemp.push_back(this->pcHist_[bestMatch[i]]);
                filtersTemp.push_back(this->filters_[bestMatch[i]]);

                // kalman filter to get new state estimation
                onboardDetector::box3D currDetectedBBox = this->filteredBBoxes_[i];

                Eigen::MatrixXd Z;
                this->getKalmanObservationAcc(currDetectedBBox, bestMatch[i], Z);
                filtersTemp.back().estimate(Z, MatrixXd::Zero(6,1));
                
                
                newEstimatedBBox.x = filtersTemp.back().output(0);
                newEstimatedBBox.y = filtersTemp.back().output(1);
                newEstimatedBBox.z = currDetectedBBox.z;
                newEstimatedBBox.Vx = filtersTemp.back().output(2);
                newEstimatedBBox.Vy = filtersTemp.back().output(3);
                newEstimatedBBox.Ax = filtersTemp.back().output(4);
                newEstimatedBBox.Ay = filtersTemp.back().output(5);   
                          

                newEstimatedBBox.x_width = currDetectedBBox.x_width;
                newEstimatedBBox.y_width = currDetectedBBox.y_width;
                newEstimatedBBox.z_width = currDetectedBBox.z_width;
                newEstimatedBBox.is_dynamic = currDetectedBBox.is_dynamic;
                newEstimatedBBox.is_human = currDetectedBBox.is_human;
            }
            else{
                boxHistTemp.push_back(newSingleBoxHist);
                pcHistTemp.push_back(newSinglePcHist);

                // create new kalman filter for this object
                onboardDetector::box3D currDetectedBBox = this->filteredBBoxes_[i];
                MatrixXd states, A, B, H, P, Q, R;    
                this->kalmanFilterMatrixAcc(currDetectedBBox, states, A, B, H, P, Q, R);
                
                newFilter.setup(states, A, B, H, P, Q, R);
                filtersTemp.push_back(newFilter);
                newEstimatedBBox = currDetectedBBox;
                
            }

            // pop old data if len of hist > size limit
            if (int(boxHistTemp[i].size()) == this->histSize_){
                boxHistTemp[i].pop_back();
                pcHistTemp[i].pop_back();
            }

            // push new data into history
            boxHistTemp[i].push_front(newEstimatedBBox); 
            pcHistTemp[i].push_front(this->filteredPcClusters_[i]);

            // update new tracked bounding boxes
            trackedBBoxesTemp.push_back(newEstimatedBBox);
        }

        if (boxHistTemp.size()){
            for (size_t i=0; i<trackedBBoxesTemp.size(); ++i){ 
                if (int(boxHistTemp[i].size()) >= this->fixSizeHistThresh_){
                    if ((abs(trackedBBoxesTemp[i].x_width-boxHistTemp[i][1].x_width)/boxHistTemp[i][1].x_width) <= this->fixSizeDimThresh_ &&
                        (abs(trackedBBoxesTemp[i].y_width-boxHistTemp[i][1].y_width)/boxHistTemp[i][1].y_width) <= this->fixSizeDimThresh_&&
                        (abs(trackedBBoxesTemp[i].z_width-boxHistTemp[i][1].z_width)/boxHistTemp[i][1].z_width) <= this->fixSizeDimThresh_){
                        trackedBBoxesTemp[i].x_width = boxHistTemp[i][1].x_width;
                        trackedBBoxesTemp[i].y_width = boxHistTemp[i][1].y_width;
                        trackedBBoxesTemp[i].z_width = boxHistTemp[i][1].z_width;
                        boxHistTemp[i][0].x_width = trackedBBoxesTemp[i].x_width;
                        boxHistTemp[i][0].y_width = trackedBBoxesTemp[i].y_width;
                        boxHistTemp[i][0].z_width = trackedBBoxesTemp[i].z_width;
                    }

                }
            }
        }
        
        // update history member variable
        this->boxHist_ = boxHistTemp;
        this->pcHist_ = pcHistTemp;
        this->filters_ = filtersTemp;

        // update tracked bounding boxes
        this->trackedBBoxes_=  trackedBBoxesTemp;

    }

    void dynamicDetector::kalmanFilterMatrixVel(const onboardDetector::box3D& currDetectedBBox, MatrixXd& states, MatrixXd& A, MatrixXd& B, MatrixXd& H, MatrixXd& P, MatrixXd& Q, MatrixXd& R){
        states.resize(4,1);
        states(0) = currDetectedBBox.x;
        states(1) = currDetectedBBox.y;
        // init vel and acc to zeros
        states(2) = 0.;
        states(3) = 0.;

        MatrixXd ATemp;
        ATemp.resize(4, 4);
        ATemp <<  0, 0, 1, 0,
                  0, 0, 0, 1,
                  0, 0, 0, 0,
                  0 ,0, 0, 0;
        A = MatrixXd::Identity(4,4) + this->dt_*ATemp;
        B = MatrixXd::Zero(4, 4);
        H = MatrixXd::Identity(4, 4);
        P = MatrixXd::Identity(4, 4) * this->eP_;
        Q = MatrixXd::Identity(4, 4);
        Q(0,0) *= this->eQPos_; Q(1,1) *= this->eQPos_; Q(2,2) *= this->eQVel_; Q(3,3) *= this->eQVel_; 
        R = MatrixXd::Identity(4, 4);
        R(0,0) *= this->eRPos_; R(1,1) *= this->eRPos_; R(2,2) *= this->eRVel_; R(3,3) *= this->eRVel_;

    }

    void dynamicDetector::kalmanFilterMatrixAcc(const onboardDetector::box3D& currDetectedBBox, MatrixXd& states, MatrixXd& A, MatrixXd& B, MatrixXd& H, MatrixXd& P, MatrixXd& Q, MatrixXd& R){
        states.resize(6,1);
        states(0) = currDetectedBBox.x;
        states(1) = currDetectedBBox.y;
        // init vel and acc to zeros
        states(2) = 0.;
        states(3) = 0.;
        states(4) = 0.;
        states(5) = 0.;

        MatrixXd ATemp;
        ATemp.resize(6, 6);

        ATemp <<  1, 0, this->dt_, 0, 0.5*pow(this->dt_, 2), 0,
                  0, 1, 0, this->dt_, 0, 0.5*pow(this->dt_, 2),
                  0, 0, 1, 0, this->dt_, 0,
                  0 ,0, 0, 1, 0, this->dt_,
                  0, 0, 0, 0, 1, 0,
                  0, 0, 0, 0, 0, 1;
        A = ATemp;
        B = MatrixXd::Zero(6, 6);
        H = MatrixXd::Identity(6, 6);
        P = MatrixXd::Identity(6, 6) * this->eP_;
        Q = MatrixXd::Identity(6, 6);
        Q(0,0) *= this->eQPos_; Q(1,1) *= this->eQPos_; Q(2,2) *= this->eQVel_; Q(3,3) *= this->eQVel_; Q(4,4) *= this->eQAcc_; Q(5,5) *= this->eQAcc_;
        R = MatrixXd::Identity(6, 6);
        R(0,0) *= this->eRPos_; R(1,1) *= this->eRPos_; R(2,2) *= this->eRVel_; R(3,3) *= this->eRVel_; R(4,4) *= this->eRAcc_; R(5,5) *= this->eRAcc_;
    }

    void dynamicDetector::getKalmanObservationVel(const onboardDetector::box3D& currDetectedBBox, int bestMatchIdx, MatrixXd& Z){
        Z.resize(4,1);
        Z(0) = currDetectedBBox.x; 
        Z(1) = currDetectedBBox.y;

        // use previous k frame for velocity estimation
        int k = this->kfAvgFrames_;
        int historySize = this->boxHist_[bestMatchIdx].size();
        if (historySize < k){
            k = historySize;
        }
        onboardDetector::box3D prevMatchBBox = this->boxHist_[bestMatchIdx][k-1];

        Z(2) = (currDetectedBBox.x-prevMatchBBox.x)/(this->dt_*k);
        Z(3) = (currDetectedBBox.y-prevMatchBBox.y)/(this->dt_*k);
    }

    void dynamicDetector::getKalmanObservationAcc(const onboardDetector::box3D& currDetectedBBox, int bestMatchIdx, MatrixXd& Z){
        Z.resize(6, 1);
        Z(0) = currDetectedBBox.x;
        Z(1) = currDetectedBBox.y;

        // use previous k frame for velocity estimation
        int k = this->kfAvgFrames_;
        int historySize = this->boxHist_[bestMatchIdx].size();
        if (historySize < k){
            k = historySize;
        }
        onboardDetector::box3D prevMatchBBox = this->boxHist_[bestMatchIdx][k-1];

        Z(2) = (currDetectedBBox.x - prevMatchBBox.x)/(this->dt_*k);
        Z(3) = (currDetectedBBox.y - prevMatchBBox.y)/(this->dt_*k);
        Z(4) = (Z(2) - prevMatchBBox.Vx)/(this->dt_*k);
        Z(5) = (Z(3) - prevMatchBBox.Vy)/(this->dt_*k);
    }
 
    void dynamicDetector::getDynamicPc(std::vector<Eigen::Vector3d>& dynamicPc){
        Eigen::Vector3d curPoint;
        for (size_t i=0 ; i<this->filteredPoints_.size() ; ++i){
            curPoint = this->filteredPoints_[i];
            for (size_t j=0; j<this->dynamicBBoxes_.size() ; ++j){
                if (abs(curPoint(0)-this->dynamicBBoxes_[j].x)<=this->dynamicBBoxes_[j].x_width/2 and 
                    abs(curPoint(1)-this->dynamicBBoxes_[j].y)<=this->dynamicBBoxes_[j].y_width/2 and 
                    abs(curPoint(2)-this->dynamicBBoxes_[j].z)<=this->dynamicBBoxes_[j].z_width/2) {
                        dynamicPc.push_back(curPoint);
                        break;
                    }
            }
        }
    } 
    
    void dynamicDetector::publishUVImages(){
        sensor_msgs::ImagePtr depthBoxMsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", this->uvDetector_->depth_show).toImageMsg();
        sensor_msgs::ImagePtr UmapBoxMsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", this->uvDetector_->U_map_show).toImageMsg();
        sensor_msgs::ImagePtr birdBoxMsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", this->uvDetector_->bird_view).toImageMsg();  
        this->uvDepthMapPub_.publish(depthBoxMsg);
        this->uDepthMapPub_.publish(UmapBoxMsg); 
        this->uvBirdViewPub_.publish(birdBoxMsg);     
    }

    void dynamicDetector::publishYoloImages(){
        sensor_msgs::ImagePtr detectedAlignedImgMsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", this->detectedAlignedDepthImg_).toImageMsg();
        this->detectedAlignedDepthImgPub_.publish(detectedAlignedImgMsg);
    }


    void dynamicDetector::publishPoints(const std::vector<Eigen::Vector3d>& points, const ros::Publisher& publisher){
        pcl::PointXYZ pt;
        pcl::PointCloud<pcl::PointXYZ> cloud;        
        for (size_t i=0; i<points.size(); ++i){
            pt.x = points[i](0);
            pt.y = points[i](1);
            pt.z = points[i](2);
            cloud.push_back(pt);
        }    
        cloud.width = cloud.points.size();
        cloud.height = 1;
        cloud.is_dense = true;
        cloud.header.frame_id = "map";

        sensor_msgs::PointCloud2 cloudMsg;
        pcl::toROSMsg(cloud, cloudMsg);
        publisher.publish(cloudMsg);
    }


    void dynamicDetector::publish3dBox(const std::vector<box3D>& boxes, const ros::Publisher& publisher, double r, double g, double b) {
        // visualization using bounding boxes 
        visualization_msgs::Marker line;
        visualization_msgs::MarkerArray lines;
        line.header.frame_id = "map";
        line.type = visualization_msgs::Marker::LINE_LIST;
        line.action = visualization_msgs::Marker::ADD;
        line.ns = "box3D";  
        line.scale.x = 0.06;
        line.color.r = r;
        line.color.g = g;
        line.color.b = b;
        line.color.a = 1.0;
        line.lifetime = ros::Duration(0.1);
        
        for(size_t i = 0; i < boxes.size(); i++){
            // visualization msgs
            line.text = " Vx " + std::to_string(boxes[i].Vx) + " Vy " + std::to_string(boxes[i].Vy);
            double x = boxes[i].x; 
            double y = boxes[i].y; 
            double z = (boxes[i].z+boxes[i].z_width/2)/2; 

            // double x_width = std::max(boxes[i].x_width,boxes[i].y_width);
            // double y_width = std::max(boxes[i].x_width,boxes[i].y_width);
            double x_width = boxes[i].x_width;
            double y_width = boxes[i].y_width;
            double z_width = 2*z;

            // double z = 
            
            vector<geometry_msgs::Point> verts;
            geometry_msgs::Point p;
            // vertice 0
            p.x = x-x_width / 2.; p.y = y-y_width / 2.; p.z = z-z_width / 2.;
            verts.push_back(p);

            // vertice 1
            p.x = x-x_width / 2.; p.y = y+y_width / 2.; p.z = z-z_width / 2.;
            verts.push_back(p);

            // vertice 2
            p.x = x+x_width / 2.; p.y = y+y_width / 2.; p.z = z-z_width / 2.;
            verts.push_back(p);

            // vertice 3
            p.x = x+x_width / 2.; p.y = y-y_width / 2.; p.z = z-z_width / 2.;
            verts.push_back(p);

            // vertice 4
            p.x = x-x_width / 2.; p.y = y-y_width / 2.; p.z = z+z_width / 2.;
            verts.push_back(p);

            // vertice 5
            p.x = x-x_width / 2.; p.y = y+y_width / 2.; p.z = z+z_width / 2.;
            verts.push_back(p);

            // vertice 6
            p.x = x+x_width / 2.; p.y = y+y_width / 2.; p.z = z+z_width / 2.;
            verts.push_back(p);

            // vertice 7
            p.x = x+x_width / 2.; p.y = y-y_width / 2.; p.z = z+z_width / 2.;
            verts.push_back(p);
            
            int vert_idx[12][2] = {
                {0,1},
                {1,2},
                {2,3},
                {0,3},
                {0,4},
                {1,5},
                {3,7},
                {2,6},
                {4,5},
                {5,6},
                {4,7},
                {6,7}
            };
            
            for (size_t i=0;i<12;i++){
                line.points.push_back(verts[vert_idx[i][0]]);
                line.points.push_back(verts[vert_idx[i][1]]);
            }
            
            lines.markers.push_back(line);
            
            line.id++;
        }
        // publish
        publisher.publish(lines);
    }

    void dynamicDetector::publishHistoryTraj(){
        visualization_msgs::MarkerArray trajMsg;
        int countMarker = 0;
        for (size_t i=0; i<this->boxHist_.size(); ++i){
            visualization_msgs::Marker traj;
            traj.header.frame_id = "map";
            traj.header.stamp = ros::Time::now();
            traj.ns = "dynamic_detector";
            traj.id = countMarker;
            traj.type = visualization_msgs::Marker::LINE_LIST;
            traj.scale.x = 0.03;
            traj.scale.y = 0.03;
            traj.scale.z = 0.03;
            traj.color.a = 1.0; // Don't forget to set the alpha!
            traj.color.r = 0.0;
            traj.color.g = 1.0;
            traj.color.b = 0.0;
            for (size_t j=0; j<this->boxHist_[i].size()-1; ++j){
                geometry_msgs::Point p1, p2;
                onboardDetector::box3D box1 = this->boxHist_[i][j];
                onboardDetector::box3D box2 = this->boxHist_[i][j+1];
                p1.x = box1.x; p1.y = box1.y; p1.z = box1.z;
                p2.x = box2.x; p2.y = box2.y; p2.z = box2.z;
                traj.points.push_back(p1);
                traj.points.push_back(p2);
            }

            ++countMarker;
            trajMsg.markers.push_back(traj);
        }
        this->historyTrajPub_.publish(trajMsg);
    }

    void dynamicDetector::publishVelVis(){ // publish velocities for all tracked objects
        visualization_msgs::MarkerArray velVisMsg;
        int countMarker = 0;
        for (size_t i=0; i<this->trackedBBoxes_.size(); ++i){
            visualization_msgs::Marker velMarker;
            velMarker.header.frame_id = "map";
            velMarker.header.stamp = ros::Time::now();
            velMarker.ns = "dynamic_detector";
            velMarker.id =  countMarker;
            velMarker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            velMarker.pose.position.x = this->trackedBBoxes_[i].x;
            velMarker.pose.position.y = this->trackedBBoxes_[i].y;
            velMarker.pose.position.z = this->trackedBBoxes_[i].z + this->trackedBBoxes_[i].z_width/2. + 0.3;
            velMarker.scale.x = 0.15;
            velMarker.scale.y = 0.15;
            velMarker.scale.z = 0.15;
            velMarker.color.a = 1.0;
            velMarker.color.r = 1.0;
            velMarker.color.g = 0.0;
            velMarker.color.b = 0.0;
            velMarker.lifetime = ros::Duration(0.1);
            double vx = this->trackedBBoxes_[i].Vx;
            double vy = this->trackedBBoxes_[i].Vy;
            double vNorm = sqrt(vx*vx+vy*vy);
            std::string velText = "Vx=" + std::to_string(vx) + ", Vy=" + std::to_string(vy) + ", |V|=" + std::to_string(vNorm);
            velMarker.text = velText;
            velVisMsg.markers.push_back(velMarker);
            ++countMarker;
        }
        this->velVisPub_.publish(velVisMsg);
    }


    void dynamicDetector::transformBBox(const Eigen::Vector3d& center, const Eigen::Vector3d& size, const Eigen::Vector3d& position, const Eigen::Matrix3d& orientation,
                                               Eigen::Vector3d& newCenter, Eigen::Vector3d& newSize){
        double x = center(0); 
        double y = center(1);
        double z = center(2);
        double xWidth = size(0);
        double yWidth = size(1);
        double zWidth = size(2);

        // get 8 bouding boxes coordinates in the camera frame
        Eigen::Vector3d p1 (x+xWidth/2.0, y+yWidth/2.0, z+zWidth/2.0);
        Eigen::Vector3d p2 (x+xWidth/2.0, y+yWidth/2.0, z-zWidth/2.0);
        Eigen::Vector3d p3 (x+xWidth/2.0, y-yWidth/2.0, z+zWidth/2.0);
        Eigen::Vector3d p4 (x+xWidth/2.0, y-yWidth/2.0, z-zWidth/2.0);
        Eigen::Vector3d p5 (x-xWidth/2.0, y+yWidth/2.0, z+zWidth/2.0);
        Eigen::Vector3d p6 (x-xWidth/2.0, y+yWidth/2.0, z-zWidth/2.0);
        Eigen::Vector3d p7 (x-xWidth/2.0, y-yWidth/2.0, z+zWidth/2.0);
        Eigen::Vector3d p8 (x-xWidth/2.0, y-yWidth/2.0, z-zWidth/2.0);

        // transform 8 points to the map coordinate frame
        Eigen::Vector3d p1m = orientation * p1 + position;
        Eigen::Vector3d p2m = orientation * p2 + position;
        Eigen::Vector3d p3m = orientation * p3 + position;
        Eigen::Vector3d p4m = orientation * p4 + position;
        Eigen::Vector3d p5m = orientation * p5 + position;
        Eigen::Vector3d p6m = orientation * p6 + position;
        Eigen::Vector3d p7m = orientation * p7 + position;
        Eigen::Vector3d p8m = orientation * p8 + position;
        std::vector<Eigen::Vector3d> pointsMap {p1m, p2m, p3m, p4m, p5m, p6m, p7m, p8m};

        // find max min in x, y, z directions
        double xmin=p1m(0); double xmax=p1m(0); 
        double ymin=p1m(1); double ymax=p1m(1);
        double zmin=p1m(2); double zmax=p1m(2);
        for (Eigen::Vector3d pm : pointsMap){
            if (pm(0) < xmin){xmin = pm(0);}
            if (pm(0) > xmax){xmax = pm(0);}
            if (pm(1) < ymin){ymin = pm(1);}
            if (pm(1) > ymax){ymax = pm(1);}
            if (pm(2) < zmin){zmin = pm(2);}
            if (pm(2) > zmax){zmax = pm(2);}
        }
        newCenter(0) = (xmin + xmax)/2.0;
        newCenter(1) = (ymin + ymax)/2.0;
        newCenter(2) = (zmin + zmax)/2.0;
        newSize(0) = xmax - xmin;
        newSize(1) = ymax - ymin;
        newSize(2) = zmax - zmin;
    }

    bool dynamicDetector::isInFov(const Eigen::Vector3d& position, const Eigen::Matrix3d& orientation, Eigen::Vector3d& point){
        Eigen::Vector3d worldRay = point - position;
        Eigen::Vector3d camUnitX(1,0,0);
        Eigen::Vector3d camUnitY(0,1,0);
        Eigen::Vector3d camUnitZ(0,0,1);
        Eigen::Vector3d camRay;
        Eigen::Vector3d displacement; 
    
        // z is in depth direction in camera coord
        camRay = orientation.inverse()*worldRay;
        double camRayX = abs(camRay.dot(camUnitX));
        double camRayY = abs(camRay.dot(camUnitY));
        double camRayZ = abs(camRay.dot(camUnitZ));

        double htan = camRayX/camRayZ;
        double vtan = camRayY/camRayZ;
        
        double pi = 3.1415926;
        return htan<tan(42*pi/180) && vtan<tan(28*pi/180) && camRayZ<this->depthMaxValue_;
    }
    
    int dynamicDetector::getBestOverlapBBox(const onboardDetector::box3D& currBBox, const std::vector<onboardDetector::box3D>& targetBBoxes, double& bestIOU){
        bestIOU = 0.0;
        int bestIOUIdx = -1; // no match
        for (size_t i=0; i<targetBBoxes.size(); ++i){
            onboardDetector::box3D targetBBox = targetBBoxes[i];
            double IOU = this->calBoxIOU(currBBox, targetBBox);
            if (IOU > bestIOU){
                bestIOU = IOU;
                bestIOUIdx = i;
            }
        }
        return bestIOUIdx;
    }

    // user functions
    void dynamicDetector::getDynamicObstacles(std::vector<onboardDetector::box3D>& incomeDynamicBBoxes, const Eigen::Vector3d &robotSize){
        incomeDynamicBBoxes.clear();
        for (int i=0; i<int(this->dynamicBBoxes_.size()); i++){
            onboardDetector::box3D box = this->dynamicBBoxes_[i];
            box.x_width += robotSize(0);
            box.y_width += robotSize(1);
            box.z_width += robotSize(2);
            incomeDynamicBBoxes.push_back(box);
        }
    }

    void dynamicDetector::updatePoseHist(){
        if (int(this->positionHist_.size()) == this->skipFrame_){
            this->positionHist_.pop_back();
        }
        else{
            this->positionHist_.push_front(this->position_);
        }
        if (int(this->orientationHist_.size()) == this->skipFrame_){
            this->orientationHist_.pop_back();
        }
        else{
            this->orientationHist_.push_front(this->orientation_);
        }
    }
}


