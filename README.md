# Onboard Dynamic Object Detection and Tracking for Autonomous Mobile Robots  
## I. Introduction
This repository contains the implementation of Dynamic Obstacle Detection and Tracking (DODT) algorithm which aims at detecting and tracking dynamic obstacles for robots with extremely constraint computational resources.

If you find this work helpful, kindly show your support by giving us a free ⭐️. Your recognition is truly valued.

This repo can be used as a standalone package and also comes as a module of our [autonomy framework](https://github.com/Zhefan-Xu/CERLAB-UAV-Autonomy).

The related paper can be found on:

**Zhefan Xu\*, Xiaoyang Zhan\*, Yumeng Xiu, Christopher Suzuki, Kenji Shimada, "Onboard dynamic-object detection and tracking for autonomous robot navigation with RGB-D camera”, IEEE Robotics and Automation Letters (RA-L), 2024.** [\[paper\]](https://ieeexplore.ieee.org/document/10323166) [\[video\]](https://youtu.be/9dKX3BRnxyw).

*The authors contributed equally.



https://github.com/Zhefan-Xu/onboard_detector/assets/55560905/2f736e5d-ffbb-4e31-a440-6a5c984fa87f



## II. Installation
This package has been tested on Ubuntu 18.04/20.04 LTS with ROS Melodic/Noetic on [Intel Realsense D435i](https://www.intelrealsense.com/depth-camera-d435i/) and NVIDIA Jetson [Xavier NX](https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/jetson-xavier-series/), [Orin NX](https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/jetson-orin/) and [Intel NUC](https://www.intel.com/content/www/us/en/products/details/nuc.html). Make sure you have installed the compatible ROS version. 
```
# this package needs ROS vision_msgs package
sudo apt install ros-noetic-vision-msgs

cd ~/catkin_ws/src
git clone https://github.com/Zhefan-Xu/onboard_detector.git
cd ..
catkin_make
```

## III. Run DEMO
### a. Run on dataset
Please download the rosbag file from this [link](https://cmu.box.com/s/aiixv3p3pzufodsrcv8a2yqpiibu28ds):
```
rosbag play -l multiple-objects.bag
roslaunch onboard_detector run_detector.launch
```
- Example with single dynamic object:



https://github.com/Zhefan-Xu/onboard_detector/assets/55560905/be2d6103-1579-4daf-aefb-9d18f42e2dfe



- Example with multiple dynamic objects:



https://github.com/Zhefan-Xu/onboard_detector/assets/55560905/3b6a0feb-7b2c-4d67-8696-3a489abb9043








### b. Run on your device
Please adjust the configuration file under ```cfg/detector_param.yaml``` of your camera device. Also, change the color image topic name in ```scripts/yolo_detector/yolo_detector.py```

From the parameter file, you can find that the algorithm expects the following data from the robot:

- Depth image: ```/camera/depth/image_rect_raw```

- Robot pose: ```/mavros/local_position/pose```

- Robot odom (optional): ```/mavros/local_position/odom```

- Color image (optional if YOLO is applied): ```/camera/color/image_rect_raw```

- Aligned depth image (optional): ```/camera/aligned_depth_to_color/image_raw```

```
# Launch your device first. Make sure it has the above data.
roslaunch onboard_detector run_detector.launch
```

## IV. Issue
For ```ImportError: No module named yaml``` on Ubuntu 20.04, please run: 
```
sudo ln -sf /usr/bin/python3 /usr/local/bin/python
```

## V. Citation and Reference
If you find this work useful, please cite the paper:
```
@article{xu2023onboard,
  title={Onboard dynamic-object detection and tracking for autonomous robot navigation with RGB-D camera},
  author={Xu, Zhefan and Zhan, Xiaoyang and Xiu, Yumeng and Suzuki, Christopher and Shimada, Kenji},
  journal={IEEE Robotics and Automation Letters},
  volume={9},
  number={1},
  pages={651--658},
  year={2023},
  publisher={IEEE}
}
```
