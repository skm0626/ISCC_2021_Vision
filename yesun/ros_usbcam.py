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

up_left = [230,350]
up_right = [363,349]
down_left = [216,415]
down_right = [380,413]


def image_callback(img_data):
	global bridge
	global img
	img = bridge.imgmsg_to_cv2(img_data, "bgr8")

def calculate(tf_center, tf_rubbercone_center):
	distance = tf_center - tf_rubbercone_center
	distance = distance * pixel
	distance[1] += invisible_distance
	print("distance", distance)
	print("distance[0]", int(distance[0]))
	return distance	

#center_visualization
def check_center(image):
	cv2.circle(image,(320,240),5, (122,0,255),-1)
	return image
	
# get matrix
def get_matrix(image, ul, ur, dl, dr):
	corner_points_array = np.float32([ul, ur, dl, dr])
	
	# visualization
	cv2.circle(img, (230,350), 5, (255,0,0), -1) # up_left
    	cv2.circle(img, (363,349), 5, (0,255,0), -1) # up_right
    	cv2.circle(img, (216,415), 5, (0,0,255), -1) # down_left
    	cv2.circle(img, (380,413), 5, (0,0,0), -1) # down_right
	
	# Create an array with the parameters (the dimensions) required to build the matrix
	img_up_left = [220,150] #[400,600]
	img_up_right = [420,150] #[600,600]
	img_down_left = [220,350] #[600,800]
	img_down_right = [420,350] #[400,800]
	img_params = np.float32([img_up_left, img_up_right, img_down_left, img_down_right])

    	# Compute and return the transformation matrix
    	matrix = cv2.getPerspectiveTransform(corner_points_array, img_params)
    	print("matrix", matrix)

	return matrix


# get object's center coordinates in warp image (이 함수의 input으로 yolo 정보 넣을 것!)
def get_object_center(image, xmin, ymin, xmax, ymax, matrix): # 일단은 특정 객체의 좌표를 넣어서 구할 수 있도록 구현 
	object_center = np.array([(xmin + xmax) / 2, ymax, 1], np.float32)
	cv2.circle(img, ((xmin + xmax) / 2, ymax), 5, (255,255,255), -1)

	tf_object_center = np.matmul(matrix, object_center)
	tf_object_center /= tf_object_center[2]
	print("trffic_rubber : ", tf_object_center)

	return tf_object_center



if __name__ == '__main__':
	rospy.init_node('warp')
	rospy.Subscriber("/usb_cam/image_raw/", Image, image_callback)
	while not rospy.is_shutdown():
		if img.size != (640*480*3):
                    continue
		
		width = 640
	    	height = 480

		matrix = get_matrix(img, up_left, up_right, down_left, down_right)
	    	img_transformed = cv2.warpPerspective(img, matrix, (width,height))
		np_matrix = np.array(matrix)
		

		#rubbercone location (임의로 정함)
		xmin = 297
		ymin = 281
		xmax = 353
		ymax = 364

		tf_rubbercone_center = get_object_center(img, xmin, ymin, xmax, ymax, np_matrix)

		tf_center = np.matmul(np_matrix, center)
		tf_center /= tf_center[2]
		print("tf_center: ", tf_center)

		distance = calculate(tf_center, tf_rubbercone_center)
		img = check_center(img)


    		cv2.imshow("display", img)
    		cv2.imshow("warp", img_transformed)
    		cv2.waitKey(33)

	cv2.destroyAllWindows()
