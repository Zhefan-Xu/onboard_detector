/*
    FILE: lidarDetector.h
    ---------------------------------
    header file of lidar-based obstacle detector
*/
#ifndef ONBOARDDETECTOR_LIDARDETECTOR_H
#define ONBOARDDETECTOR_LIDARDETECTOR_H

#include <ros/ros.h>
#include <onboard_detector/dbscan.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/Eigen>
namespace onboardDetector{
    struct Cluster
    {
        int cluster_id;               
        Eigen::Vector4f centroid;      
        pcl::PointCloud<pcl::PointXYZ>::Ptr points; // pointcloud of the cluster      

        // Geometry information
        Eigen::Vector3f dimensions;    // bbox size
        Eigen::Matrix3f eigen_vectors; // PCA eigen vectors
        Eigen::Vector3f eigen_values;  // PCA eigen values

        Cluster():
            cluster_id(-1),
            centroid(Eigen::Vector4f::Zero()),
            points(new pcl::PointCloud<pcl::PointXYZ>()) {}
    };

    class lidarDetector{
    private:
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_; //current pointcloud
        std::vector<onboardDetector::Cluster> clusters_; //current cluster list
        //lidar DBSCAN parameters
        double eps_;
        int minPts_;
        
    public:
        lidarDetector();
        void setParams(double eps, int minPts);
        void getPointcloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
        void lidarDBSCAN();
        const std::vector<Cluster>& getClusters(std::vector<Cluster>& clusters) const {clusters = this->clusters_; return clusters_;}
    };
}


#endif
