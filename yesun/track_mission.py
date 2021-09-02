#!/usr/bin/python
#-*- encoding: utf-8 -*-

import cv2, rospy, time
import numpy as np
import math
from darknet_ros_msgs.msg import BoundingBox, BoundingBoxes
from sensor_msgs.msg import Image
from vision_distance.msg import Colorcone, ColorconeArray
from geometry_msgs.msg import Point
from cv_bridge import CvBridge

from datetime import datetime

bridge = CvBridge()
img = np.empty(shape=[0])

now = datetime.now()

pixel = 80.0/100.0 # 0.8cm # 0.4cm
center = np.array([288,480,1], np.float32)
invisible_distance = 207

up_left = [227,313]
up_right = [346,313]
down_left = [212,383]
down_right = [362,383]

box_class = None
box_xmin = None
box_xmax = None
box_ymin = None
box_ymax = None

matrix = None

cone_pub = rospy.Publisher('color_cone', ColorconeArray, queue_size=10)

def image_callback(img_data):
	global bridge
	global img
	img = bridge.imgmsg_to_cv2(img_data, "bgr8")

# boundig_box callback
def bounding_callback(msg):
	global box_class, box_xmin, box_xmax, box_ymin, box_ymax
	global data_list
	global bounding_list
	global matrix
	bbox_num = len(msg.bounding_boxes)
	bbox = msg.bounding_boxes
	# yellow_lst = []
	# blue_lst = []
	# bounding_list = []
	if np.any(matrix) == None: return
	
	data_list = []
	for idx, box in enumerate(bbox):
		box_class = box.Class
		box_xmin = box.xmin
		box_xmax = box.xmax
		box_ymin = box.ymin
		box_ymax = box.ymax

		# blue(1), yellow(0)		
		cone_flag = 1
		if box_class == "yellow cone": cone_flag = 0

		tf_center = np.matmul(matrix, center)
		tf_center /= tf_center[2]
		'''
		cv2.circle(img,(box_xmin,box_ymin),5,(122,0,0),-1)
		cv2.circle(img,(box_xmax,box_ymin),5,(122,0,0),-1)
		cv2.circle(img,(box_xmin,box_ymax),5,(122,0,0),-1)
		cv2.circle(img,(box_xmax,box_ymax),5,(122,0,0),-1)		
		'''
		cone_x = (box_xmin+box_xmax)/2
		cone_y = box_ymax
		warp_cone = np.array([cone_x, cone_y, 1], np.float32)
		warp_cone = np.matmul(np_matrix, warp_cone)
		warp_cone /= warp_cone[2]

		cone = Colorcone()
		cone.flag = cone_flag
		cone.x = warp_cone[0]
		cone.y = warp_cone[1]
		# cone.dist_x = distance[0]
		# cone.dist_y = distance[1]

		data_list.append(cone)
		# bounding_list.append([box_class])
		# print("Data_list", data_list)
		# print("data_list len", len(data_list))

	cone_array = ColorconeArray()
	cone_array.visions = data_list
	cone_pub.publish(cone_array)
	
		

#center_visualization
def check_center(image):
	cv2.circle(image,(288,240),5, (122,0,255),-1)
	return image


if __name__ == '__main__':
	global matrix
	# global img
	rospy.init_node('warp')
	image_sub = rospy.Subscriber("/usb_cam/image_raw/", Image, image_callback)
	# cap = cv2.VideoCapture("/home/foscar/ISCC_2021/src/vision_distance/src/ISCC_2021_Vision/yesun/8-31/origin_2021-8-31-19-42.avi")
	bbox_sub = rospy.Subscriber("/darknet_ros/bounding_boxes/", BoundingBoxes, bounding_callback)
	
	out = cv2.VideoWriter('/home/foscar/ISCC_2021/src/vision_distance/src/ISCC_2021_Vision/yesun/8-31/origin_{}-{}-{}-{}-{}.avi'.format(now.year,now.month, now.day, now.hour, now.minute), cv2.VideoWriter_fourcc(*'MJPG'),30,(640,480))
	out2 = cv2.VideoWriter('/home/foscar/ISCC_2021/src/vision_distance/src/ISCC_2021_Vision/yesun/8-31/dot_origin_{}-{}-{}-{}-{}.avi'.format(now.year,now.month, now.day, now.hour, now.minute), cv2.VideoWriter_fourcc(*'MJPG'),30,(640,480))
	out3 = cv2.VideoWriter('/home/foscar/ISCC_2021/src/vision_distance/src/ISCC_2021_Vision/yesun/8-31/warp_{}-{}-{}-{}-{}.avi'.format(now.year,now.month, now.day, now.hour, now.minute), cv2.VideoWriter_fourcc(*'MJPG'),30,(1000,850))
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():  # cap.isOpened()
		# ret, img = cap.read()
		# img = cv2.resize(img, (640,480))
		# print(img.shape)
		# print(ret)
		if img.size != (1280*720*3):
                    continue

		try:
			out.write(img)
		except:
			pass
		
		width = 1000
	    	height = 850
		
		img_up_left = [450,650] # [220,150] # [400,600]
		img_up_right = [550,650] # [420,150] # [600,600]
		img_down_left = [450,750] # [220,350] # [600,800]
		img_down_right = [550,750] # [420,350] # [400,800]
		img_params = np.float32([img_up_left, img_up_right, img_down_left, img_down_right])

	    	# Compute and return the transformation matrix
	    	matrix = cv2.getPerspectiveTransform(corner_points_array, img_params)
		np_matrix = np.array(matrix)
	    	img_transformed = cv2.warpPerspective(img, matrix, (width,height))
		
		black_img = np.zeros((width, height, 3), np.uint8)
	
		if box_xmin==None or box_ymin==None or box_xmax==None or box_ymax==None: continue 

		xmin = float(box_xmin)
		ymin = float(box_ymin)
		xmax = float(box_xmax)
		ymax = float(box_ymax)

		warp_xymin = np.array([xmin,ymin,1], np.float32)
		warp_xymax = np.array([xmax,ymax,1], np.float32)

		warp_xymin = np.matmul(np_matrix, warp_xymin)
		warp_xymax = np.matmul(np_matrix, warp_xymax)
		warp_xymin /= warp_xymin[2]
		warp_xymax /= warp_xymax[2]

		tf_center = np.matmul(np_matrix, center)
		tf_center /= tf_center[2]
		#print("tf_center: ", tf_center)

		img = check_center(img)
		# print('class name', box_class)

		cv2.circle(img,(up_left[0],up_left[1]),5,(255,0,0),-1)
		cv2.circle(img,(up_right[0],up_right[1]),5,(0,255,0),-1)
		cv2.circle(img,(down_left[0],down_left[1]),5,(0,0,255),-1)
		cv2.circle(img,(down_right[0],down_right[1]),5,(0,0,0),-1)

		cv2.circle(img, (box_xmin,box_ymin),5,(122,0,0),-1)
		cv2.circle(img,(box_xmax,box_ymin),5,(122,0,0),-1)
		cv2.circle(img,(box_xmin,box_ymax),5,(122,0,0),-1)
		cv2.circle(img,(box_xmax,box_ymax),5,(122,0,0),-1)

		cv2.circle(img, (288,480), 5, (255,0,0),-1) #center

		cv2.circle(img_transformed,(int(warp_xymin[0]),int(warp_xymin[1])),5,(122,122,0),-1)
		cv2.circle(img_transformed,(int(warp_xymin[0]),int(warp_xymax[1])),5,(122,122,0),-1)
		cv2.circle(img_transformed,(int(warp_xymax[0]),int(warp_xymin[1])),5,(122,122,0),-1)
		cv2.circle(img_transformed,(int(warp_xymax[0]),int(warp_xymax[1])),5,(122,122,0),-1)

		print("warp_xymin",warp_xymin)
		
		#yolo center visualization
		if (len(data_list) > 0):
			print("data_list**************", data_list)
			yellow_arr = []
			blue_arr = []
			print("len(data_list) :", len(data_list))
			
			yello_cnt = 0
			blue_cnt = 0
			
			for i in range (0, len(data_list)):
				print("i", i)
				print("data list i", data_list[i])
				if (data_list[i].flag == 0):
					yello_cnt += 1
					print("yello_cnt : ", yello_cnt)
					yellow_arr.append([data_list[i].flag, data_list[i].x, data_list[i].y])
					# print("yellow_arr", yellow_arr)
					cv2.circle(img_transformed, (int(data_list[i].x), int(data_list[i].y)), 5, (0,122,122), -1)
				elif (data_list[i].flag == 1):
					blue_cnt+=1
					print("blue_cnt : ", blue_cnt)
					blue_arr.append([data_list[i].flag, data_list[i].x, data_list[i].y])	
					# print("blue_arr", blue_arr)
					cv2.circle(img_transformed,(int(data_list[i].x),int(data_list[i].y)),5,(0,122,122),-1)
				# print("cone_center", cone_center_x, cone_center_y)
				
			# sort by Y
			yellow_arr = sorted(yellow_arr, key=lambda x:(x[2],x[1],x[0]))
			print("sort_yellow", yellow_arr)
			blue_arr = sorted(blue_arr, key=lambda x:(x[2],x[1],x[0]))	
			print("sort_blue", blue_arr)
			
			# print("lennnnnnnnnnnnnnnnnn", len(data_list))
						
		try:
			out2.write(img)
			out3.write(img_transformed)
		except:
			pass


    		cv2.imshow("display", img)
    		cv2.imshow("warp", img_transformed)
		#if cv2.waitKey(1) & 0xFF == ord('q'):
		#	break    		
		cv2.waitKey(33)
		rate.sleep()

	cv2.destroyAllWindows()
