#!/usr/bin/python
#-*- encoding: utf-8 -*-

import cv2, rospy, time
import numpy as np
import math

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

bridge = CvBridge()
img = np.empty(shape=[0])

pixel = 80.0/200.0 # 0.4cm
center = np.array([298,480,1], np.float32)
invisible_distance = 205

def image_callback(img_data):
	global bridge
	global img
  img = bridge.imgmsg_to_cv2(img_data, "bgr8")

def calculate(tf_center, tf_rubbercone_center):
	distance = tf_center- tf_rubbercone_center
	distance = distance * pixel
	distance[1] += invisible_distance
	print("distance", distance)
	print("distance[0]", int(distance[0]))
	return distance	

#center_visualization
def check_center(image):
	cv2.circle(image,(320,240),5, (122,0,255),-1)
	return image
	


if __name__ == '__main__':
	rospy.init_node('warp')
	rospy.Subscriber("/usb_cam/image_raw/", Image, image_callback)
	while not rospy.is_shutdown():
    if img.size != (640*480*3):
      continue
    up_left = [230,350]
    up_right = [363,349]
    down_left = [216,415]
    down_right = [380,413]
    corner_points_array = np.float32([up_left,up_right,down_left,down_right])
    
    cv2.circle(img, (230,350), 5, (255,0,0),-1)
    cv2.circle(img, (363,349), 5, (0,255,0),-1)
    cv2.circle(img, (216,415), 5, (0,0,255),-1)
	  cv2.circle(img, (380,413), 5, (0,0,0),-1)

		#rubbercone location
		rubbercone_center = np.array([120,400,1],np.float32)
		cv2.circle(img, (120,400), 5, (255,255,255),-1)
    
    width=640
    height=480

	  # Create an array with the parameters (the dimensions) required to build the matrix
		img_up_left = [220,150] #[400,600]
		img_up_right = [420,150] #[600,600]
		img_down_left = [220,350] #[600,800]
		img_down_right = [420,350] #[400,800]
		img_params = np.float32([img_up_left,img_up_right,img_down_left,img_down_right])

	  # Compute and return the transformation matrix
	  matrix = cv2.getPerspectiveTransform(corner_points_array,img_params)
	  print("matrix",matrix)

		np_matrix = np.array(matrix)


	  img_transformed = cv2.warpPerspective(img,matrix,(width,height))

		tf_rubbercone_center = np.matmul(np_matrix, rubbercone_center)
		tf_rubbercone_center /= tf_rubbercone_center[2]

		print("trffic_rubber : ", tf_rubbercone_center)

		tf_center = np.matmul(np_matrix, center)
		tf_center /= tf_center[2]
		print("tf_center: ", tf_center)

		distance = calculate(tf_center, tf_rubbercone_center)
		img = check_center(img)

    cv2.imshow("display", img)
    cv2.imshow("warp", img_transformed)
    cv2.waitKey(33)
	cv2.destroyAllWindows()
