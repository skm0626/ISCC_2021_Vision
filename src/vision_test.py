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

center = np.array([288,480,1], np.float32)

up_left = [227,313]
up_right = [346,313]
down_left = [212,383]
down_right = [362,383]
corner_points_array = np.float32([up_left,up_right,down_left,down_right])

box_class = None
box_xmin = None
box_xmax = None
box_ymin = None
box_ymax = None

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
	bbox_num = len(msg.bounding_boxes)
	bbox = msg.bounding_boxes
	# yellow_lst = []
	# blue_lst = []
	bounding_list = []
	if np.any(matrix) == None: return
	
	data_list = []
	for idx, box in enumerate(bbox):
		box_class = box.Class
		box_xmin = box.xmin
		box_xmax = box.xmax
		box_ymin = box.ymin
		box_ymax = box.ymax

		# blue(0), yellow(1)		
		cone_flag = 0
		if box_class == "yellow cone": cone_flag = 1


		'''
		cv2.circle(img,(box_xmin,box_ymin),5,(122,0,0),-1)
		cv2.circle(img,(box_xmax,box_ymin),5,(122,0,0),-1)
		cv2.circle(img,(box_xmin,box_ymax),5,(122,0,0),-1)
		cv2.circle(img,(box_xmax,box_ymax),5,(122,0,0),-1)		
'''
		distance = calculate(tf_center, tf_object_center)
		print("{}) tf_center: {}, distance: {}".format(idx, tf_center, distance))
		
		cone = Colorcone()
		cone.flag = cone_flag
		cone.dist_x = distance[0]
		cone.dist_y = distance[1]

		data_list.append(cone)
		bounding_list.append([box_class])
		#print("Data_list", data_list)
		#print("data_list len", len(data_list))

	cone_array = ColorconeArray()
	cone_array.visions = data_list
	cone_pub.publish(cone_array)
	
#center_visualization
def check_center(image):
	cv2.circle(image,(288,240),5, (122,0,255),-1)
	return image
	

if __name__ == '__main__':
	global matrix
	global img
	#rospy.init_node('warp')
	#image_sub = rospy.Subscriber("/usb_cam/image_raw/", Image, image_callback)
	cap = cv2.VideoCapture("/home/foscar/ISCC_2021/src/vision_distance/src/ISCC_2021_Vision/yesun/8-31/origin_2021-8-31-18-34.avi")
	#bbox_sub = rospy.Subscriber("/darknet_ros/bounding_boxes/", BoundingBoxes, bounding_callback)
	
	out = cv2.VideoWriter('/home/foscar/ISCC_2021/src/vision_distance/src/ISCC_2021_Vision/yesun/8-31/origin_{}-{}-{}-{}-{}.avi'.format(now.year,now.month, now.day, now.hour, now.minute), cv2.VideoWriter_fourcc(*'MJPG'),30,(1280,720))
	out2 = cv2.VideoWriter('/home/foscar/ISCC_2021/src/vision_distance/src/ISCC_2021_Vision/yesun/8-31/dot_origin_{}-{}-{}-{}-{}.avi'.format(now.year,now.month, now.day, now.hour, now.minute), cv2.VideoWriter_fourcc(*'MJPG'),30,(1280,720))
	out3 = cv2.VideoWriter('/home/foscar/ISCC_2021/src/vision_distance/src/ISCC_2021_Vision/yesun/8-31/warp_{}-{}-{}-{}-{}.avi'.format(now.year,now.month, now.day, now.hour, now.minute), cv2.VideoWriter_fourcc(*'MJPG'),30,(1000,1000))
	rate = rospy.Rate(10)
	while cap.isOpened():
		ret, img = cap.read()
		print(ret)
		if img.size != (1280*720*3):
                    continue

		try:
			out.write(img)
		except:
			pass
		
		width = 1000
	    	height = 1000

		img_up_left = [450,650]#[220,150] #[400,600]
		img_up_right = [550,650]#[420,150] #[600,600]
		img_down_left = [450,750]#[220,350] #[600,800]
		img_down_right = [550,750]#[420,350] #[400,800]
		img_params = np.float32([img_up_left, img_up_right, img_down_left, img_down_right])

	    	# Compute and return the transformation matrix
	    	matrix = cv2.getPerspectiveTransform(corner_points_array, img_params)

	    	img_transformed = cv2.warpPerspective(img, matrix, (width,height))
		np_matrix = np.array(matrix)
		if box_xmin==None or box_ymin==None or box_xmax==None or box_ymax==None: continue 

		xmin = float(box_xmin)
		ymin = float(box_ymin)
		xmax = float(box_xmax)
		ymax = float(box_ymax)

		warp_xymin = np.array([xmin,ymin,1], np.float32)
		warp_xymax = np.array([xmax,ymax,1], np.float32)

		warp_xymin = np.matmul(np_matrix,warp_xymin)
		warp_xymax = np.matmul(np_matrix,warp_xymax)
		warp_xymin /= warp_xymin[2]
		warp_xymax /= warp_xymax[2]

		img = check_center(img)
		# print('class name', box_class)

		cv2.circle(img,(227,313),5,(255,0,0),-1)
		cv2.circle(img,(346,313),5,(0,255,0),-1)
		cv2.circle(img,(212,383),5,(0,0,255),-1)
		cv2.circle(img,(362,383),5,(0,0,0),-1)

		cv2.circle(img, (box_xmin,box_ymin),5,(122,0,0),-1)
		cv2.circle(img,(box_xmax,box_ymin),5,(122,0,0),-1)
		cv2.circle(img,(box_xmin,box_ymax),5,(122,0,0),-1)
		cv2.circle(img,(box_xmax,box_ymax),5,(122,0,0),-1)

		cv2.circle(img, (288,480), 5, (255,0,0),-1 ) #center

		cv2.circle(img_transformed,(int(warp_xymin[0]),int(warp_xymin[1])),5,(122,122,0),-1)
		cv2.circle(img_transformed,(int(warp_xymin[0]),int(warp_xymax[1])),5,(122,122,0),-1)
		cv2.circle(img_transformed,(int(warp_xymax[0]),int(warp_xymin[1])),5,(122,122,0),-1)
		cv2.circle(img_transformed,(int(warp_xymax[0]),int(warp_xymax[1])),5,(122,122,0),-1)

		print("warp_xymin",warp_xymin)
		
		#yolo center visualization
		if (len(bounding_list)!=0):
			print("bounding_list**************",bounding_list[0][0])
			for i in range (0,len(bounding_list)):
				if (box_class =='yellow cone'):
					cone_center_x = (warp_xymin[0]+warp_xymax[0])/2
					cone_center_y = warp_xymax[1]
				elif (box_class =='blue cone'):
					cone_center_x = (warp_xymin[0]+warp_xymax[0])/2
					cone_center_y = warp_xymax[1]
				cv2.circle(img_transformed,(int(cone_center_x),int(cone_center_y)),5,(0,122,122),-1)
				#print("cone_center", cone_center_x, cone_center_y)
	
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
