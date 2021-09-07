import cv2
import numpy as np
import matplotlib.pyplot as plt

class SlidingWindow:
    def __init__(self):
	self.x_lower = 200 
        self.x_upper = 850
        self.y_lower = 600
	self.y_upper = 650 

    def slidingwindow(self, img):
		x_location = None

		height = img.shape[0]
		width = img.shape[1]

		out_img = np.zeros((height, width, 3), np.uint8)
		out_img = np.uint8(out_img)

		#window_height = 10
		#window_num = 15
                window_height = 10
		window_num = 7

		nonzero = img.nonzero()

		nonzero_y = np.array(nonzero[0])
		nonzero_x = np.array(nonzero[1])

		margin = 20
		minpix = 10

		lane = []

		good_line = ((nonzero_x >= self.x_lower) & (nonzero_y >= 700) & (nonzero_x <= self.x_upper)).nonzero()[0]

		# draw square
		square = np.array([[self.x_lower, height], [self.x_lower, self.y_lower], [self.x_upper, self.y_upper], [self.x_upper, height]], np.int32)

		cv2.polylines(out_img, [square], False, (0,255,0), 1)
		cv2.circle(out_img, (self.x_lower, height), 8, (255,0,255), -1)
		cv2.circle(out_img, (self.x_upper, height), 8, (255,0,255), -1)
		cv2.circle(out_img, (self.x_lower, self.y_lower), 8, (255,0,255), -1)
		cv2.circle(out_img, (self.x_upper, self.y_upper), 8, (255,0,255), -1)
		# draw (0.340)

		y_represent = None
		x_represent = None


		#print("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@", len(good_line))
		if (len(good_line) != 0):
		    x_represent = np.int(np.mean(nonzero_x[good_line]))
		    y_represent = np.int(np.mean(nonzero_y[good_line]))		
                else:
                    return out_img, None

                # visualization --> circle valid inds
		for i in range(len(good_line)):
                    cv2.circle(out_img, (nonzero_x[good_line[i]], nonzero_y[good_line[i]]), 1, (0,255,255), -1)

		cv2.circle(out_img, (x_represent, y_represent), 15, (0,255,255), -1)

		# window sliding and draw
		for window in range(0, window_num):
		    # rectangle x,y range
		    win_y_low = y_represent - (window + 1) * window_height
		    win_y_high = y_represent - (window) * window_height
		    win_x_low = x_represent - margin
		    win_x_high = x_represent + margin
		    # draw rectangle
		    cv2.rectangle(out_img, (win_x_low, win_y_low), (win_x_high, win_y_high), (0, 255, 0), 1)
		    cv2.rectangle(out_img, (win_x_low + int(width * 0.54), win_y_low), (win_x_high + int(width * 0.54), win_y_high), (255, 0, 0), 1)

		    good_line = ((nonzero_y >= win_y_low) & (nonzero_y < win_y_high) & (nonzero_x >= win_x_low) & (nonzero_x < win_x_high)).nonzero()[0]

		    if len(good_line) > minpix:
			#print("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@")
		        x_represent = np.int(np.mean(nonzero_x[good_line]))
			#print("@@@@@@@@@@@@@@@@@@@@@@@@@@", x_represent)
		    elif nonzero_y[lane] != [] and nonzero_x[lane] != []:
		        p_left = np.polyfit(nonzero_y[lane], nonzero_x[lane], 3)
		        x_represent = np.int(np.polyval(p_left, win_y_high))
                    # 600 700
		    if win_y_low >= 700 and win_y_low < 800:
		        x_location = x_represent + 230 #245#150 #0.2

		    lane.extend(good_line)
		    #print("X_LOCATIONNNNNN", x_location)

		return out_img, x_location
























