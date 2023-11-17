# Dynamic Object Detection and Tracking for Autonomous Robots  
This repository contains the implementation of Dynamic Obstacle Detection and Tracking (DODT) algorithm which aims at detecting and tracking dynamic obstacles for robots with extremely constraint computational resources.

The related paper can be found on:

**Zhefan Xu\*, Xiaoyang Zhan\*, Yumeng Xiu, Christopher Suzuki, Kenji Shimada, "Onboard dynamic-object detection and tracking for autonomous robot navigation with RGB-D camera,” IEEE Robotics and Automation Letters (RA-L), 2023.** [\[arxiv preprint\]](https://arxiv.org/pdf/2303.00132.pdf)

*The authors contribute equally.



https://github.com/Zhefan-Xu/onboard_detector/assets/55560905/d5e794d3-d446-43da-ab1d-0c9e188993fe

## Installation
This package has been tested on Ubuntu 18.04/20.04 LTS with ROS Melodic/Noetic on [Intel Realsense D435i](https://www.intelrealsense.com/depth-camera-d435i/) and NVIDIA Jetson [Xavier NX](https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/jetson-xavier-series/), [Orin NX](https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/jetson-orin/) and [Intel NUC](https://www.intel.com/content/www/us/en/products/details/nuc.html). Make sure you have installed the compatible ROS version. 
```
cd ~/catkin_ws/src
git clone https://github.com/Zhefan-Xu/onboard_detector.git
cd ..
catkin_make
```

## Run demo



## Issue
for ```ImportError: No module named yaml``` on Ubuntu 20.04, please run: 
```
sudo ln -sf /usr/bin/python3 /usr/local/bin/python
```

## Citation and Reference:
