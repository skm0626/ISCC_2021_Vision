#!/usr/bin/python
#-*- encoding: utf-8 -*-

# video : /home/foscar/ISCC_2021/src/vision_distance/src/ISCC_2021_Vision/yesun/9-2/origin_2021-9-2-13-41.avi
import cv2, rospy, time
import numpy as np
import math
import copy
from darknet_ros_msgs.msg import BoundingBox, BoundingBoxes
from sensor_msgs.msg import Image
from vision_distance.msg import Delivery, DeliveryArray
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
from datetime import datetime
from SlidingWindow import SlidingWindow

bridge = CvBridge()
slidingwindow = SlidingWindow()
img = np.empty(shape=[0])

now = datetime.now()

pixel = 80.0/200.0 # 0.4cm
invisible_distance = 366
center = np.array([320,480,1], np.float32)

# black square points
up_left = [268,297]
up_right = [365,297]
down_left = [260,318]
down_right = [377,318]
corner_points_array = np.float32([up_left,up_right,down_left,down_right])

box_class = None
box_xmin = None
box_xmax = None
box_ymin = None
box_ymax = None

matrix_path = '/home/foscar/ISCC_2021/src/vision_distance/src/delivery_matrix'
matrix = None

delivery_pub = rospy.Publisher('delivery', DeliveryArray, queue_size=10)

data_list = []

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
		b_flag = -1
		box_class = box.Class
		box_xmin = box.xmin
		box_xmax = box.xmax
		box_ymin = box.ymin
		box_ymax = box.ymax

		# b1(0), b2(1), b3(2)
		if box_class == "B1": b_flag = 0
		if box_class == "B2": b_flag = 1
		if box_class == "B3": b_flag = 2
	

		if box_class == "A1": b_flag = 3

		tf_object_center = get_object_center2(box.Class,box_xmin, box_ymin, box_xmax, box_ymax)
		tf_center = np.matmul(matrix, center)
		tf_center /= tf_center[2]

		distance = calculate(tf_center, tf_object_center)
		# print("{}) tf_center: {}, distance: {}".format(idx, tf_center, distance))
		
		if b_flag != -1:
			delivery = Delivery()
			delivery.flag = b_flag
			delivery.x = distance[0]
			delivery.y = distance[1]

			data_list.append(delivery)
	
	if len(data_list) > 0:
		delivery_array = DeliveryArray()
		delivery_array.visions = data_list
		delivery_pub.publish(delivery_array)
	

def get_object_center2(box_class,xmin, ymin, xmax, ymax):  
	object_center = np.array([(xmin + xmax) / 2, ymax, 1], np.float32)
	tf_object_center = np.matmul(matrix, object_center)
	tf_object_center /= tf_object_center[2]

	#if box_class == "B1":
		#print("B1 : ", tf_object_center)
	#elif box_class == "B2":
		#print("B2 : ", tf_object_center)
	#elif box_class == "B3":
		#print("B3 : ", tf_object_center)

	return tf_object_center

def calculate(tf_center, tf_object_center):
	distance = tf_center - tf_object_center
	distance = distance * pixel
	distance[1] += invisible_distance
	#print("distance", distance)
	# print("distance[0]", int(distance[0]), int(distance[1]))
	return distance	


# center_visualization
def check_center(image):
	cv2.circle(image, (288,240), 5, (122,0,255), -1)
	return image
	


if __name__ == '__main__':
	global matrix
	#global data_list
	#global img
	rospy.init_node('warp')
	# Video 용 Subscriber
	# image_sub = rospy.Subscriber("/videofile/image_raw/", Image, image_callback)
	# USB CAMERA 용 Subscriber	
	image_sub = rospy.Subscriber("/usb_cam/image_raw/", Image, image_callback)
	
	#cap = cv2.VideoCapture("/home/foscar/ISCC_2021/src/vision_distance/src/ISCC_2021_Vision/yesun/8-31/origin_2021-8-31-19-42.avi")
	bbox_sub = rospy.Subscriber("/darknet_ros/bounding_boxes/", BoundingBoxes, bounding_callback)
	
	#out = cv2.VideoWriter('/home/foscar/ISCC_2021/src/vision_distance/src/ISCC_2021_Vision/yesun/9-2/origin_{}-{}-{}-{}-{}.avi'.format(now.year,now.month, now.day, now.hour, now.minute), cv2.VideoWriter_fourcc(*'MJPG'),30,(640,480))
	#out2 = cv2.VideoWriter('/home/foscar/ISCC_2021/src/vision_distance/src/ISCC_2021_Vision/yesun/9-2/dot_origin_{}-{}-{}-{}-{}.avi'.format(now.year,now.month, now.day, now.hour, now.minute), cv2.VideoWriter_fourcc(*'MJPG'),30,(640,480))
	#out3 = cv2.VideoWriter('/home/foscar/ISCC_2021/src/vision_distance/src/ISCC_2021_Vision/yesun/9-2/warp_{}-{}-{}-{}-{}.avi'.format(now.year,now.month, now.day, now.hour, now.minute), cv2.VideoWriter_fourcc(*'MJPG'),30,(1000,850))
	rate = rospy.Rate(10)
	while not rospy.is_shutdown(): #cap.isOpened()
		#ret, img = cap.read()
		#img = cv2.resize(img, (640,480))
		#print(img.shape)
		#print(ret)
		if img.size != (640 * 480 * 3):
                    continue

		try:
			out.write(img)
		except:
			pass
		
		width = 1000
	    	height = 850
		
		new_data_list = []
		
		if len(data_list) > 0:
			new_data_list = copy.deepcopy(data_list)
			#new_data_list = data_list.copy()
			#print("new_data_list %%%%%%%%%%%%%% ", new_data_list)
		#else:
		#	continue
				

		img_up_left = [400,600]#[450, 650] #[220,150] #[400,600]
		img_up_right = [600,600]#[550, 650] #[420,150] #[600,600]
		img_down_left = [400,800]#[450, 750] #[220,350] #[400,800]
		img_down_right = [600,800]#[550, 750] #[420,350] #[600,800]
		img_params = np.float32([img_up_left, img_up_right, img_down_left, img_down_right])

	    	# Compute and return the transformation matrix
	    	matrix = cv2.getPerspectiveTransform(corner_points_array, img_params)
		np_matrix = np.array(matrix)
		np.save(matrix_path, np_matrix)
		# print(np_matrix)
	    	img_transformed = cv2.warpPerspective(img, matrix, (width, height))

		if box_xmin == None or box_ymin == None or box_xmax == None or box_ymax == None: continue 

		xmin = float(box_xmin)
		ymin = float(box_ymin)
		xmax = float(box_xmax)
		ymax = float(box_ymax)

		warp_xymin = np.array([xmin, ymin, 1], np.float32)
		warp_xymax = np.array([xmax, ymax, 1], np.float32)

		warp_xymin = np.matmul(np_matrix, warp_xymin)
		warp_xymax = np.matmul(np_matrix, warp_xymax)
		warp_xymin /= warp_xymin[2]
		warp_xymax /= warp_xymax[2]

		img = check_center(img)
		# print('class name', box_class)

		cv2.circle(img, (up_left[0], up_left[1]), 5, (255,0,0), -1)
		cv2.circle(img, (up_right[0], up_right[1]), 5, (0,255,0), -1)
		cv2.circle(img, (down_left[0], down_left[1]), 5, (0,0,255), -1)
		cv2.circle(img, (down_right[0], down_right[1]), 5, (0,0,0), -1)

		cv2.circle(img, (box_xmin, box_ymin), 5, (122,0,0), -1)
		cv2.circle(img, (box_xmax, box_ymin), 5, (122,0,0), -1)
		cv2.circle(img, (box_xmin, box_ymax), 5, (122,0,0), -1)
		cv2.circle(img, (box_xmax, box_ymax), 5, (122,0,0), -1)

		cv2.circle(img, (288,480), 5, (255,0,0),-1 ) #center

		#cv2.circle(img_transformed, (int(warp_xymin[0]), int(warp_xymin[1])), 25, (0,0,255), -1)
		#cv2.circle(img_transformed, (int(warp_xymin[0]), int(warp_xymax[1])), 5, (122,122,0), -1)
		#cv2.circle(img_transformed, (int(warp_xymax[0]), int(warp_xymin[1])), 5, (122,122,0), -1)
		#cv2.circle(img_transformed, (int(warp_xymax[0]), int(warp_xymax[1])), 5, (122,122,0), -1)

		#print("warp_xymin", warp_xymin)
		# yolo center visualization
		
		if (len(new_data_list) > 0):
			#print("data_list**************", data_list)
			#print("len(new_data_list) :", len(new_data_list))
			
			#for i in range (0, len(new_data_list)):
				#print("i", i)
				#print("data list i", new_data_list[i])

			left_point = np.array([70.18,520,1], np.float32) #[184.18,520,1]
			right_point = np.array([570.4,520,1], np.float32) #[389.4,520,1]
			warp_left_point = np.matmul(np_matrix, left_point)
			warp_left_point /= warp_left_point[2]
			warp_right_point = np.matmul(np_matrix, right_point)
			warp_right_point /= warp_right_point[2]

			#print("warp_left_point:", warp_left_point)
			#print("warp_right_point:", warp_right_point)
			#('warp_left_point:', array([ 290.31703522,  886.70705631,    1.        ]))
			#('warp_right_point:', array([ 775.20601084,  886.70705631,    1.        ]))

		try:
			out2.write(img)
			out3.write(img_transformed)
		except:
			pass

		#cv2.imshow("display", img)
		#cv2.imshow("warp", img_transformed)
		#cv2.imshow('out_img', out_img)

		#if cv2.waitKey(1) & 0xFF == ord('q'):
		#	break    		
		cv2.waitKey(33)
		rate.sleep()

	cv2.destroyAllWindows()
