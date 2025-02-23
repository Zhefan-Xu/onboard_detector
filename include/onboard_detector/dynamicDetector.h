/*
    FILE: dynamicDetector.h
    ---------------------------------
    header file of dynamic obstacle detector
*/
#ifndef ONBOARDDETECTOR_DYNAMICDETECTOR_H
#define ONBOARDDETECTOR_DYNAMICDETECTOR_H

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <Eigen/StdVector>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/MarkerArray.h>
#include <vision_msgs/Detection2DArray.h>
#include <image_transport/image_transport.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <onboard_detector/dbscan.h>
#include <onboard_detector/uvDetector.h>
#include <onboard_detector/lidarDetector.h>
#include <onboard_detector/kalmanFilter.h>
#include <onboard_detector/utils.h>
#include <onboard_detector/GetDynamicObstacles.h>

namespace onboardDetector{
    class dynamicDetector{
    private:
        std::string ns_;
        std::string hint_;

        // ROS
        ros::NodeHandle nh_;
        std::shared_ptr<message_filters::Subscriber<sensor_msgs::Image>> depthSub_;
        std::shared_ptr<message_filters::Subscriber<geometry_msgs::PoseStamped>> poseSub_;
        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, geometry_msgs::PoseStamped> depthPoseSync;
        std::shared_ptr<message_filters::Synchronizer<depthPoseSync>> depthPoseSync_;
        std::shared_ptr<message_filters::Subscriber<nav_msgs::Odometry>> odomSub_;
        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, nav_msgs::Odometry> depthOdomSync;
        std::shared_ptr<message_filters::Synchronizer<depthOdomSync>> depthOdomSync_;
        ros::Subscriber colorImgSub_;
        ros::Subscriber yoloDetectionSub_;
        ros::Subscriber lidarCloudSub_;
        ros::Timer detectionTimer_;
        ros::Timer lidarDetectionTimer_;
        ros::Timer trackingTimer_;
        ros::Timer classificationTimer_;
        ros::Timer visTimer_;
        image_transport::Publisher uvDepthMapPub_;
        image_transport::Publisher uDepthMapPub_;
        image_transport::Publisher uvBirdViewPub_;
        image_transport::Publisher detectedColorImgPub_;
        ros::Publisher uvBBoxesPub_;
        ros::Publisher dbBBoxesPub_;
        ros::Publisher visualBBoxesPub_;
        ros::Publisher lidarBBoxesPub_;
        ros::Publisher filteredBBoxesBeforeYoloPub_;
        ros::Publisher filteredBBoxesPub_;
        ros::Publisher trackedBBoxesPub_;
        ros::Publisher dynamicBBoxesPub_;
        ros::Publisher filteredDepthPointsPub_;
        ros::Publisher lidarClustersPub_;
        ros::Publisher filteredPointsPub_;
        ros::Publisher dynamicPointsPub_;
        ros::Publisher rawDynamicPointsPub_;
        ros::Publisher downSamplePointsPub_;
        ros::Publisher rawLidarPointsPub_;
        ros::Publisher historyTrajPub_;
        ros::Publisher velVisPub_;
        ros::ServiceServer getDynamicObstacleServer_;
    
        // DETECTOR
        std::shared_ptr<onboardDetector::UVdetector> uvDetector_;
        std::shared_ptr<onboardDetector::DBSCAN> dbCluster_;
        std::shared_ptr<onboardDetector::lidarDetector> lidarDetector_;

        // SENSOR INFO
        // CAMERA DEPTH
        double fx_, fy_, cx_, cy_; // depth camera intrinsics
        double depthScale_; // value / depthScale
        double depthMinValue_, depthMaxValue_;
        double raycastMaxLength_;
        int depthFilterMargin_, skipPixel_; // depth filter margin
        int imgCols_, imgRows_;
        Eigen::Matrix4d body2CamDepth_; // from body frame to camera frame

        // CAMERA COLOR
        double fxC_, fyC_, cxC_, cyC_;
        Eigen::Matrix4d body2CamColor_;

        // LIDAR
        Eigen::Matrix4d body2Lidar_;

        // PARAMETETER
        // Topics
        int localizationMode_;
        std::string depthTopicName_;
        std::string colorImgTopicName_;
        std::string lidarTopicName_;
        std::string poseTopicName_;
        std::string odomTopicName_;

        // System
        double dt_;

        // DBSCAN Common
        double groundHeight_;
        double roofHeight_;
        
        // DBSCAN visual param
        double voxelOccThresh_;
        int dbMinPointsCluster_;
        double dbEpsilon_;
        
        // DBSCAN LiDAR param
        int lidarDBMinPoints_;
        double lidarDBEpsilon_;
        int gaussianDownSampleRate_;
        int downSampleThresh_;

        // LiDAR Visual Filtering
        double boxIOUThresh_;

        // Tracking and data association
        double maxMatchRange_;
        double maxMatchSizeRange_;
        Eigen::VectorXd featureWeights_;
        int histSize_;
        int fixSizeHistThresh_;
        double fixSizeDimThresh_;
        double eP_; // kalman filter initial uncertainty matrix
        double eQPos_; // motion model uncertainty matrix for position
        double eQVel_; // motion model uncertainty matrix for velocity
        double eQAcc_; // motion model uncertainty matrix for acceleration
        double eRPos_; // observation uncertainty matrix for position
        double eRVel_; // observation uncertainty matrix for velocity
        double eRAcc_; // observation uncertainty matrix for acceleration
        int kfAvgFrames_;

        // Classification
        int skipFrame_;
        double dynaVelThresh_;
        double dynaVoteThresh_;
        int forceDynaFrames_;
        int forceDynaCheckRange_;
        int dynamicConsistThresh_;

        // Constrain size
        bool constrainSize_;
        std::vector<Eigen::Vector3d> targetObjectSize_; 
        Eigen::Vector3d maxObjectSize_; 

        // SENSOR DATA
        cv::Mat depthImage_;
        Eigen::Vector3d position_; // robot position
        Eigen::Matrix3d orientation_; // robot orientation
        Eigen::Vector3d positionDepth_; // depth camera position
        Eigen::Matrix3d orientationDepth_; // depth camera orientation
        Eigen::Vector3d positionColor_; // color camera position
        Eigen::Matrix3d orientationColor_; // color camera orientation
        Eigen::Vector3d positionLidar_; // color camera position
        Eigen::Matrix3d orientationLidar_; // color camera orientation
        bool hasSensorPose_;
        Eigen::Vector3d localSensorRange_ {5.0, 5.0, 5.0};
        Eigen::Vector3d localLidarRange_ {10.0, 10.0, 5.0};

        //LIDAR DATA
        sensor_msgs::PointCloud2ConstPtr latestCloud_;
        pcl::PointCloud<pcl::PointXYZ>::Ptr lidarCloud_ = NULL; 
        std::vector<onboardDetector::Cluster> lidarClusters_;

        // DETECTOR DATA
        std::vector<onboardDetector::box3D> uvBBoxes_; // uv detector bounding boxes
        int projPointsNum_ = 0;
        std::vector<Eigen::Vector3d> projPoints_; // projected points from depth image
        std::vector<double> pointsDepth_;
        std::vector<Eigen::Vector3d> filteredDepthPoints_; // filtered point cloud data
        std::vector<onboardDetector::box3D> dbBBoxes_; // DBSCAN bounding boxes
        std::vector<std::vector<Eigen::Vector3d>> pcClustersVisual_; // pointcloud clusters
        std::vector<Eigen::Vector3d> pcClusterCentersVisual_; // pointcloud cluster centers
        std::vector<Eigen::Vector3d> pcClusterStdsVisual_; // pointcloud cluster standard deviation in each axis      
        std::vector<onboardDetector::box3D> filteredBBoxesBeforeYolo_; // filtered bboxes before yolo
        std::vector<onboardDetector::box3D> filteredBBoxes_; // filtered bboxes
        std::vector<std::vector<Eigen::Vector3d>> filteredPcClusters_; // pointcloud clusters after filtering by UV and DBSCAN fusion
        std::vector<Eigen::Vector3d> filteredPcClusterCenters_; // filtered pointcloud cluster centers
        std::vector<Eigen::Vector3d> filteredPcClusterStds_; // filtered pointcloud cluster standard deviation in each axis
        std::vector<onboardDetector::box3D> visualBBoxes_; // visual bobxes detected by camera
        std::vector<onboardDetector::box3D> lidarBBoxes_; // bboxes detected by lidar (have static and dynamic)
        std::vector<onboardDetector::box3D> trackedBBoxes_; // bboxes tracked from kalman filtering
        std::vector<onboardDetector::box3D> dynamicBBoxes_; // boxes classified as dynamic

        // TRACKING AND ASSOCIATION DATA
        bool newDetectFlag_;
        std::vector<std::deque<onboardDetector::box3D>> boxHist_; // data association result: history of filtered bounding boxes for each box in current frame
        std::vector<std::deque<std::vector<Eigen::Vector3d>>> pcHist_; // data association result: history of filtered pc clusteres for each pc cluster in current frame
        std::vector<std::deque<Eigen::Vector3d>> pcCenterHist_; 
        std::vector<onboardDetector::kalman_filter> filters_; // kalman filter for each objects

        // YOLO RESULTS
        vision_msgs::Detection2DArray yoloDetectionResults_; // yolo detected 2D results
        cv::Mat detectedColorImage_;

    public:
        dynamicDetector();
        dynamicDetector(const ros::NodeHandle& nh);
        void initDetector(const ros::NodeHandle& nh);

        void initParam();
        void registerPub();
        void registerCallback();

        // service
		bool getDynamicObstacles(onboard_detector::GetDynamicObstacles::Request& req, 
								 onboard_detector::GetDynamicObstacles::Response& res);

        // callback
        void depthPoseCB(const sensor_msgs::ImageConstPtr& img, const geometry_msgs::PoseStampedConstPtr& pose);
        void depthOdomCB(const sensor_msgs::ImageConstPtr& img, const nav_msgs::OdometryConstPtr& odom);
        void colorImgCB(const sensor_msgs::ImageConstPtr& img);
        void yoloDetectionCB(const vision_msgs::Detection2DArrayConstPtr& detections);
        void lidarCloudCB(const sensor_msgs::PointCloud2ConstPtr& cloudMsg);
        void detectionCB(const ros::TimerEvent&);
        void lidarDetectionCB(const ros::TimerEvent&);
        void trackingCB(const ros::TimerEvent&);
        void classificationCB(const ros::TimerEvent&);
        void visCB(const ros::TimerEvent&);

        // detect function
        void uvDetect();
        void dbscanDetect();
        void lidarDetect();
        void filterLVBBoxes(); // filter lidar and vision bounding boxes
        void transformUVBBoxes(std::vector<onboardDetector::box3D>& bboxes);
        
        // Visual DBSCAN Detector Functions
        void projectDepthImage();
        void filterPoints(const std::vector<Eigen::Vector3d>& points, std::vector<Eigen::Vector3d>& filteredPoints);
        void clusterPointsAndBBoxes(const std::vector<Eigen::Vector3d>& points, std::vector<onboardDetector::box3D>& bboxes, std::vector<std::vector<Eigen::Vector3d>>& pcClusters, std::vector<Eigen::Vector3d>& pcClusterCenters, std::vector<Eigen::Vector3d>& pcClusterStds);
        void voxelFilter(const std::vector<Eigen::Vector3d>& points, std::vector<Eigen::Vector3d>& filteredPoints);
        
        // detection helper functions
        void calcPcFeat(const std::vector<Eigen::Vector3d>& pcCluster, Eigen::Vector3d& pcClusterCenter, Eigen::Vector3d& pcClusterStd);
        double calBoxIOU(const onboardDetector::box3D& box1, const onboardDetector::box3D& box2, bool ignoreZmin=false);

        // Data association and tracking functions
        void boxAssociation(std::vector<int>& bestMatch);
        void boxAssociationHelper(std::vector<int>& bestMatch);
        void genFeatHelper(const std::vector<onboardDetector::box3D>& boxes, const std::vector<Eigen::Vector3d>& pcCenters, std::vector<Eigen::VectorXd>& feature);
        void getPrevBBoxes(std::vector<onboardDetector::box3D>& prevBoxes, std::vector<Eigen::Vector3d>& prevPcCenters);
        void linearProp(std::vector<onboardDetector::box3D>& propedBoxes, std::vector<Eigen::Vector3d>& propedPcCenters);
        void findBestMatch(const std::vector<onboardDetector::box3D>& prevBBoxes, const std::vector<Eigen::VectorXd>& prevBoxesFeat, const std::vector<onboardDetector::box3D>& propedBoxes, const std::vector<Eigen::VectorXd>& propedBoxesFeat, const std::vector<Eigen::VectorXd>& currBoxesFeat, std::vector<int>& bestMatch);
        void kalmanFilterAndUpdateHist(const std::vector<int>& bestMatch);
        void kalmanFilterMatrixVel(const onboardDetector::box3D& currDetectedBBox, MatrixXd& states, MatrixXd& A, MatrixXd& B, MatrixXd& H, MatrixXd& P, MatrixXd& Q, MatrixXd& R);
        void kalmanFilterMatrixAcc(const onboardDetector::box3D& currDetectedBBox, MatrixXd& states, MatrixXd& A, MatrixXd& B, MatrixXd& H, MatrixXd& P, MatrixXd& Q, MatrixXd& R);
        void getKalmanObservationVel(const onboardDetector::box3D& currDetectedBBox, int bestMatchIdx, MatrixXd& Z);
        void getKalmanObservationAcc(const onboardDetector::box3D& currDetectedBBox, int bestMatchIdx, MatrixXd& Z);


        // visualization
        void getDynamicPc(std::vector<Eigen::Vector3d>& dynamicPc);
        void publishUVImages(); 
        void publishColorImages();
        void publishPoints(const std::vector<Eigen::Vector3d>& points, const ros::Publisher& publisher);
        void publish3dBox(const std::vector<onboardDetector::box3D>& bboxes, const ros::Publisher& publisher, double r, double g, double b);
        void publishHistoryTraj();
        void publishVelVis();
        void publishLidarClusters();
        void publishFilteredPoints();
        void publishRawDynamicPoints();

        // helper function
        void transformBBox(const Eigen::Vector3d& center, const Eigen::Vector3d& size, const Eigen::Vector3d& position, const Eigen::Matrix3d& orientation,
                                  Eigen::Vector3d& newCenter, Eigen::Vector3d& newSize);
        int getBestOverlapBBox(const onboardDetector::box3D& currBBox, const std::vector<onboardDetector::box3D>& targetBBoxes, double& bestIOU);

        // user functions
        void getDynamicObstacles(std::vector<onboardDetector::box3D>& incomeDynamicBBoxes, const Eigen::Vector3d &robotSize = Eigen::Vector3d(0.0,0.0,0.0));
        void getDynamicObstaclesHist(std::vector<std::vector<Eigen::Vector3d>>& posHist, 
									 std::vector<std::vector<Eigen::Vector3d>>& velHist, 
									 std::vector<std::vector<Eigen::Vector3d>>& sizeHist, const Eigen::Vector3d &robotSize = Eigen::Vector3d(0.0,0.0,0.0));

        // inline helper functions
        bool isInFilterRange(const Eigen::Vector3d& pos);
        void posToIndex(const Eigen::Vector3d& pos, Eigen::Vector3i& idx, double res);
        int indexToAddress(const Eigen::Vector3i& idx, double res);
        int posToAddress(const Eigen::Vector3d& pos, double res);
        void indexToPos(const Eigen::Vector3i& idx, Eigen::Vector3d& pos, double res);
        void getCameraPose(const geometry_msgs::PoseStampedConstPtr& pose, Eigen::Matrix4d& camPoseDepthMatrix, Eigen::Matrix4d& camPoseColorMatrix);
        void getCameraPose(const nav_msgs::OdometryConstPtr& odom, Eigen::Matrix4d& camPoseDepthMatrix, Eigen::Matrix4d& camPoseColorMatrix);
        void getLidarPose(const geometry_msgs::PoseStampedConstPtr& pose, Eigen::Matrix4d& lidarPoseMatrix);
        void getLidarPose(const nav_msgs::OdometryConstPtr& odom, Eigen::Matrix4d& lidarPoseMatrix);
        onboardDetector::Point eigenToDBPoint(const Eigen::Vector3d& p);
        Eigen::Vector3d dbPointToEigen(const onboardDetector::Point& pDB);
        void eigenToDBPointVec(const std::vector<Eigen::Vector3d>& points, std::vector<onboardDetector::Point>& pointsDB, int size);       
    };


    inline bool dynamicDetector::isInFilterRange(const Eigen::Vector3d& pos){
        if ((pos(0) >= this->position_(0) - this->localSensorRange_(0)) and (pos(0) <= this->position_(0) + this->localSensorRange_(0)) and 
            (pos(1) >= this->position_(1) - this->localSensorRange_(1)) and (pos(1) <= this->position_(1) + this->localSensorRange_(1)) and 
            (pos(2) >= this->position_(2) - this->localSensorRange_(2)) and (pos(2) <= this->position_(2) + this->localSensorRange_(2))){
            return true;
        }
        else{
            return false;
        }        
    }

    inline void dynamicDetector::posToIndex(const Eigen::Vector3d& pos, Eigen::Vector3i& idx, double res){
        idx(0) = floor( (pos(0) - this->position_(0) + localSensorRange_(0)) / res);
        idx(1) = floor( (pos(1) - this->position_(1) + localSensorRange_(1)) / res);
        idx(2) = floor( (pos(2) - this->position_(2) + localSensorRange_(2)) / res);
    }

    inline int dynamicDetector::indexToAddress(const Eigen::Vector3i& idx, double res){
        return idx(0) * ceil(2*this->localSensorRange_(1)/res) * ceil(2*this->localSensorRange_(2)/res) + idx(1) * ceil(2*this->localSensorRange_(2)/res) + idx(2);
        // return idx(0) * ceil(this->localSensorRange_(0)/res) + idx(1) * ceil(this->localSensorRange_(1)/res) + idx(2);
    }

    inline int dynamicDetector::posToAddress(const Eigen::Vector3d& pos, double res){
        Eigen::Vector3i idx;
        this->posToIndex(pos, idx, res);
        return this->indexToAddress(idx, res);
    }

    inline void dynamicDetector::indexToPos(const Eigen::Vector3i& idx, Eigen::Vector3d& pos, double res){
		pos(0) = (idx(0) + 0.5) * res - localSensorRange_(0) + this->position_(0);
		pos(1) = (idx(1) + 0.5) * res - localSensorRange_(1) + this->position_(1);
		pos(2) = (idx(2) + 0.5) * res - localSensorRange_(2) + this->position_(2);
	}
    
    inline void dynamicDetector::getCameraPose(const geometry_msgs::PoseStampedConstPtr& pose, Eigen::Matrix4d& camPoseDepthMatrix, Eigen::Matrix4d& camPoseColorMatrix){
        Eigen::Quaterniond quat;
        quat = Eigen::Quaterniond(pose->pose.orientation.w, pose->pose.orientation.x, pose->pose.orientation.y, pose->pose.orientation.z);
        Eigen::Matrix3d rot = quat.toRotationMatrix();

        // convert body pose to camera pose
        Eigen::Matrix4d map2body; map2body.setZero();
        map2body.block<3, 3>(0, 0) = rot;
        map2body(0, 3) = pose->pose.position.x; 
        map2body(1, 3) = pose->pose.position.y;
        map2body(2, 3) = pose->pose.position.z;
        map2body(3, 3) = 1.0;

        camPoseDepthMatrix = map2body * this->body2CamDepth_;
        camPoseColorMatrix = map2body * this->body2CamColor_;
    }

    inline void dynamicDetector::getCameraPose(const nav_msgs::OdometryConstPtr& odom, Eigen::Matrix4d& camPoseDepthMatrix, Eigen::Matrix4d& camPoseColorMatrix){
        Eigen::Quaterniond quat;
        quat = Eigen::Quaterniond(odom->pose.pose.orientation.w, odom->pose.pose.orientation.x, odom->pose.pose.orientation.y, odom->pose.pose.orientation.z);
        Eigen::Matrix3d rot = quat.toRotationMatrix();

        // convert body pose to camera pose
        Eigen::Matrix4d map2body; map2body.setZero();
        map2body.block<3, 3>(0, 0) = rot;
        map2body(0, 3) = odom->pose.pose.position.x; 
        map2body(1, 3) = odom->pose.pose.position.y;
        map2body(2, 3) = odom->pose.pose.position.z;
        map2body(3, 3) = 1.0;

        camPoseDepthMatrix = map2body * this->body2CamDepth_;
        camPoseColorMatrix = map2body * this->body2CamColor_;
    }

    inline void dynamicDetector::getLidarPose(const geometry_msgs::PoseStampedConstPtr& pose, Eigen::Matrix4d& lidarPoseMatrix){
        Eigen::Quaterniond quat;
        quat = Eigen::Quaterniond(pose->pose.orientation.w, pose->pose.orientation.x, pose->pose.orientation.y, pose->pose.orientation.z);
        Eigen::Matrix3d rot = quat.toRotationMatrix();

        // convert body pose to camera pose
        Eigen::Matrix4d map2body; map2body.setZero();
        map2body.block<3, 3>(0, 0) = rot;
        map2body(0, 3) = pose->pose.position.x; 
        map2body(1, 3) = pose->pose.position.y;
        map2body(2, 3) = pose->pose.position.z;
        map2body(3, 3) = 1.0;

        lidarPoseMatrix = map2body * this->body2Lidar_;
    }

    inline void dynamicDetector::getLidarPose(const nav_msgs::OdometryConstPtr& odom, Eigen::Matrix4d& lidarPoseMatrix){
        Eigen::Quaterniond quat;
        quat = Eigen::Quaterniond(odom->pose.pose.orientation.w, odom->pose.pose.orientation.x, odom->pose.pose.orientation.y, odom->pose.pose.orientation.z);
        Eigen::Matrix3d rot = quat.toRotationMatrix();

        // convert body pose to camera pose
        Eigen::Matrix4d map2body; map2body.setZero();
        map2body.block<3, 3>(0, 0) = rot;
        map2body(0, 3) = odom->pose.pose.position.x; 
        map2body(1, 3) = odom->pose.pose.position.y;
        map2body(2, 3) = odom->pose.pose.position.z;
        map2body(3, 3) = 1.0;

        lidarPoseMatrix = map2body * this->body2Lidar_;
    }
    
    inline onboardDetector::Point dynamicDetector::eigenToDBPoint(const Eigen::Vector3d& p){
        onboardDetector::Point pDB;
        pDB.x = p(0);
        pDB.y = p(1);
        pDB.z = p(2);
        pDB.clusterID = -1;
        return pDB;
    }

    inline Eigen::Vector3d dynamicDetector::dbPointToEigen(const onboardDetector::Point& pDB){
        Eigen::Vector3d p;
        p(0) = pDB.x;
        p(1) = pDB.y;
        p(2) = pDB.z;
        return p;
    }

    inline void dynamicDetector::eigenToDBPointVec(const std::vector<Eigen::Vector3d>& points, std::vector<onboardDetector::Point>& pointsDB, int size){
        for (int i=0; i<size; ++i){
            Eigen::Vector3d p = points[i];
            onboardDetector::Point pDB = this->eigenToDBPoint(p);
            pointsDB.push_back(pDB);
        }
    }
}

#endif
