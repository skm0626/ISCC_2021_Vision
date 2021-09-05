#!/usr/bin/python
#-*- encoding: utf-8 -*-

######################################################
#################### delivery #######################
######################################################


import cv2, rospy, time
import numpy as np
import math
from darknet_ros_msgs.msg import BoundingBox, BoundingBoxes
from sensor_msgs.msg import Image
from vision_distance.msg import Delivery, DeliveryArray
from geometry_msgs.msg import Point
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

box_class = None
box_xmin = None
box_xmax = None
box_ymin = None
box_ymax = None

matrix_path = '/home/foscar/ISCC_2021/src/vision_distance/src/ISCC_2021_Vision/yesun/matrix'
matrix = None

delivery_pub = rospy.Publisher('delivery', DeliveryArray, queue_size=10)

def image_callback(img_data):
	global bridge
	global img
	img = bridge.imgmsg_to_cv2(img_data, "bgr8")

# boundig_box callback
def bounding_callback(msg):
	global box_class, box_xmin, box_xmax, box_ymin, box_ymax
	bbox_num = len(msg.bounding_boxes)
	bbox = msg.bounding_boxes
	# yellow_lst = []
	# blue_lst = []
	tmp_list = []
	if np.any(matrix) == None: return
	
	data_list = []
	for idx, box in enumerate(bbox):
		box_class = box.Class
		box_xmin = box.xmin
		box_xmax = box.xmax
		box_ymin = box.ymin
		box_ymax = box.ymax

		# b1(0), b2(1), b3(2)
		if box_class == "b1": b_flag = 0
		if box_class == "b2": b_flag = 1
		if box_class == "b3": b_flag = 2

		tf_object_center = get_object_center2(box_xmin, box_ymin, box_xmax, box_ymax)
		tf_center = np.matmul(matrix, center)
		tf_center /= tf_center[2]
		

		distance = calculate(tf_center, tf_object_center)
		# print("{}) tf_center: {}, distance: {}".format(idx, tf_center, distance))
		
		delivery = Delivery()
		delivery.flag = b_flag
		delivery.x = distance[0]
		delivery.y = distance[1]

		data_list.append(delivery)

	delivery_array = DeliveryArray()
	delivery_array.visions = data_list
	delivery_pub.publish(delivery_array)


def calculate(tf_center, tf_object_center):
	distance = tf_center - tf_object_center
	distance = distance * pixel
	distance[1] += invisible_distance
	#print("distance", distance)
	# print("distance[0]", int(distance[0]), int(distance[1]))
	return distance	

#center_visualization
def check_center(image):
	cv2.circle(image,(320,240),5, (122,0,255),-1)
	return image
	

# get object's center coordinates in warp image (이 함수의 input으로 yolo 정보 넣을 것!)
def get_object_center2(xmin, ymin, xmax, ymax): # 일단은 특정 객체의 좌표를 넣어서 구할 수 있도록 구현 
	object_center = np.array([(xmin + xmax) / 2, ymax, 1], np.float32)
	tf_object_center = np.matmul(matrix, object_center)
	tf_object_center /= tf_object_center[2]
	# print("trffic_rubber : ", tf_object_center)

	return tf_object_center


if __name__ == '__main__':
	global matrix
	rospy.init_node('warp')
	image_sub = rospy.Subscriber("/usb_cam/image_raw/", Image, image_callback)
	bbox_sub = rospy.Subscriber("/darknet_ros/bounding_boxes/", BoundingBoxes, bounding_callback)
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		if img.size != (640*480*3):
                    continue
		
		width = 640
	    height = 480
		
		# load matrix
		# if np.any(matrix) == None:
		# 	matrix = np.load(matrix_path + '.npy')
		# #print('load matrix', matrix)
	    # 	img_transformed = cv2.warpPerspective(img, matrix, (width,height))
		# np_matrix = np.array(matrix)
		# if box_xmin == None or box_ymin == None or box_xmax == None or box_ymax == None: continue 

		# xmin = float(box_xmin)
		# ymin = float(box_ymin)
		# xmax = float(box_xmax)
		# ymax = float(box_ymax)

		# tf_object_center = get_object_center(img, xmin, ymin, xmax, ymax, np_matrix)

		# tf_center = np.matmul(np_matrix, center)
		# tf_center /= tf_center[2]
		# #print("tf_center: ", tf_center)

		# distance = calculate(tf_center, tf_object_center)
		# img = check_center(img)
		# # print('class name', box_class)


    	# 	cv2.imshow("display", img)
    	# 	cv2.imshow("warp", img_transformed)
    	#	cv2.waitKey(33)
		
		rate.sleep()

	cv2.destroyAllWindows()
