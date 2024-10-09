/*
    FILE: lidarDetector.cpp
    ------------------
    class function definitions for lidar-based obstacle detector
*/
#include <onboard_detector/lidarDetector.h>
namespace onboardDetector{
    lidarDetector::lidarDetector(){
        this->eps_ = 0.5;
        this->minPts_ = 10;
        this->cloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
    }

    void lidarDetector::setParams(double eps, int minPts){
        this->eps_ = eps;
        this->minPts_ = minPts;
    }

    void lidarDetector::getPointcloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud){
        this->cloud_ = cloud;
    }

    void lidarDetector::lidarDBSCAN(){
        if(!cloud_ || cloud_->empty()){
            ROS_WARN("Empty pointcloud");
            return;
        }

        std::vector<Point> points;
        for(size_t i=0; i<cloud_->size(); ++i){
            Point p;
            p.x = cloud_->points[i].x;
            p.y = cloud_->points[i].y;
            p.z = cloud_->points[i].z;
            p.clusterID = UNCLASSIFIED;
            points.push_back(p);
        }

        DBSCAN dbscan(minPts_, eps_, points);
        dbscan.run();

        const std::vector<Point>& clusterResult = dbscan.m_points;
        this->clusters_.clear();
        for(size_t i=0; i<clusterResult.size(); ++i){
            if(clusterResult[i].clusterID != NOISE){
                Cluster cluster;
                cluster.cluster_id = clusterResult[i].clusterID;
                cluster.points->push_back(cloud_->points[i]);
                this->clusters_.push_back(cluster);
            }
        }

        for(auto& cluster : this->clusters_){
            Eigen::Vector4f centroid;
            pcl::compute3DCentroid(*cluster.points, centroid);
            cluster.centroid = centroid;
            pcl::PointXYZ minPt, maxPt;
            pcl::getMinMax3D(*cluster.points, minPt, maxPt);
            cluster.dimensions = Eigen::Vector3f(maxPt.x - minPt.x, maxPt.y - minPt.y, maxPt.z - minPt.z); 
        }

        ROS_INFO("DBSCAN clustering finished. %d clusters found.", this->clusters_.size());
    }
}
