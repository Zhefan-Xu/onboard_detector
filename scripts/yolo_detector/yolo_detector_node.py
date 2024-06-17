#!/usr/bin/env python3

import rospy
import numpy as np
from yolo_detector import *


def main():
	rospy.init_node("yolo_detector_node")
	yolo_detector()
	rospy.spin()

if __name__=="__main__":
	main()
	
