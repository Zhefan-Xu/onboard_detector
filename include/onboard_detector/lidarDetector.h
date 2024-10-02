/*
    FILE: lidarDetector.h
    ---------------------------------
    header file of lidar-based obstacle detector
*/
#ifndef ONBOARDDETECTOR_LIDARDETECTOR_H
#define ONBOARDDETECTOR_LIDARDETECTOR_H

#include <ros/ros.h>
#include <onboard_detector/dbscan.h>
namespace onboardDetector{
    class lidarDetector{
    private:

    public:
        lidarDetector();
    };
}


#endif