#!/usr/bin/env python3
import sys
sys.path.append('/home/jhy/catkin_ws/src/CERLAB-UAV-Autonomy/onboard_detector/scripts/yolo_detector')
import rospy
import numpy as np
from yolo.yolo_detector import *


def main():
	rospy.init_node("yolo_detector_node")
	yolo_detector()
	rospy.spin()

if __name__=="__main__":
	main()
	
