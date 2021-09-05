#!/usr/bin/python
#-*- encoding: utf-8 -*-

import cv2, rospy, time
import numpy as np
import math
from sensor_msgs.msg import Image
from darknet_ros_msgs.msg import BoundingBox, BoundingBoxes
from cv_bridge import CvBridge


# global variables
bridge = CvBridge()
img = np.empty(shape=[0])
# image callback
def image_callback(img_data):
	global bridge
	global img
	img = bridge.imgmsg_to_cv2(img_data, "bgr8")
	rospy.loginfo("check image_callback")

# boundig_box callback
def bounding_callback(msg):
	detected_list = []
	bbox_num = len(msg.bounding_boxes)
	bbox = msg.bounding_boxes
	for box in bbox:
		if box.Class == "yellow cone":
			global yellow_class
			yellow_class = box.Class
			yellow_xmin = box.xmin
			yellow_xmax = box.xmax
			yellow_ymin = box.ymin
			yellow_ymax = box.ymax
			rospy.loginfo(
         	   "class_name : {}, xmin: {}, xmax: {}, ymin: {}, ymax: {}".format(
               	 yellow_class, yellow_xmin, yellow_xmax, yellow_ymin, yellow_ymax
         	   ))

		elif box.Class == "blue cone":
			global blue_class
			blue_class = box.Class
			blue_xmin = box.xmin
			blue_xmax = box.xmax
			blue_ymin = box.ymin
			blue_ymax = box.ymax
			rospy.loginfo(
         	   "class_name : {}, xmin: {}, xmax: {}, ymin: {}, ymax: {}".format(
               	 blue_class, blue_xmin, blue_xmax, blue_ymin, blue_ymax
         	   ))
		
         	
	
		
def main():
	while not rospy.is_shutdown():


		rospy.init_node('warp2')
		image_sub = rospy.Subscriber("/usb_cam/image_rect_color/", Image, image_callback)
		#image_sub = rospy.Subscriber("/usb_cam/image_raw/", Image, image_callback)
		rospy.loginfo("Subscribers set usb_cam")

		bbox_sub = rospy.Subscriber("/darknet_ros/bounding_boxes/", BoundingBoxes, bounding_callback)
		rospy.loginfo("Subscribers set darknet_ros")

		rospy.spin

if __name__ == '__main__':
	try:
		main()
		
		
	except rospy.ROSInterruptException:
		pass	

	#while not rospy.is_shutdown():
	#	if img.size != (640*480*3):
    #                continue

#********************************************************************************#

