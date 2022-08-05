#!/usr/bin/env python
import rospy
import cv2
import numpy as np
import math
import time
#import imutils
from geometry_msgs.msg import Point

from std_msgs.msg import Header

from sensor_msgs.msg import Image
from cv_bridge import CvBridge


from blob_detector.msg import Blobs
from blob_detector.msg import Blob




k = 1
pos_x = [0]
pos_y = [0]

HEIGHT = 960
WIDTH = 1280

c_mtx = np.array([
[586.828741, 0.000000, 658.171801],
[ 0.000000, 591.121354, 437.937937],
[0.0, 0.0, 1.0]])
MARKER_COLORS = rospy.get_param("/abebe/marker_mapping")    
MARKER_POSITIONS = rospy.get_param("/abebe/marker_positions")

dist_c = np.array([-0.241325, 0.036928, -0.001418, -0.003593, 0.000000])
camera_mtx, roi = cv2.getOptimalNewCameraMatrix(c_mtx, dist_c, (WIDTH, HEIGHT), 1, (WIDTH, HEIGHT))
map1, map2 = cv2.initUndistortRectifyMap(c_mtx, dist_c, None, camera_mtx, (WIDTH, HEIGHT), 5)
#roi = 405, 65, 740, 344 #roi - for some reason I'm not particularly interested in investigating, returned ROI doesn't work
roi = 250, 150, 850, 700
COLORS = {'red' : [np.array([174, 90, 110], np.uint8), np.array([179, 255, 255], np.uint8)],
    'blue' :      [np.array([83,  114, 57], np.uint8), np.array([128, 255, 255], np.uint8)],
    'yellow' :    [np.array([20,  80,  50], np.uint8),  np.array([40, 255, 255], np.uint8)],
    'pink' :      [np.array([132, 90,  70], np.uint8), np.array([173, 255, 255], np.uint8)],
  'white' :     [np.array([0, 0,  205], np.uint8), np.array([179, 74, 255], np.uint8)]
}
ROI_X, ROI_Y, ROI_W, ROI_H = roi
kernel = np.ones((5, 5), np.uint8)

DEBUG = True

def largest_n_contours(contours, num):
    area_contours = []
    if len(contours) > 0:
        for contour in contours:
    	    area = cv2.contourArea(contour)
	    area_contours.append([area, contour])
	area_contours = sorted(area_contours, key=lambda x : x[0], reverse=True)
	area_contours = area_contours[:num if num < len(area_contours) else len(area_contours)]
    return area_contours

def undistort(img):
	global HEIGHT, WIDTH, DEBUG
	bridge = CvBridge()
	
	
	u_img = cv2.remap(img, map1, map2, cv2.INTER_LINEAR)

	img = u_img[ROI_Y:ROI_Y+ROI_H, ROI_X:ROI_X+ROI_W] #croping to region of interest

	img_height, img_width = np.shape(img)[:2]


	#total_time = 0
	img = cv2.resize(img, (img_width/2, img_height/2), 1)
	img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
	#color = 'blue'
	#hsv_range = COLORS[color]
	a_x = []
	a_y = []
	for color, hsv_range in COLORS.iteritems():
	
		img_bin = cv2.inRange(img_hsv, hsv_range[0], hsv_range[1])
		img_dil1 = cv2.morphologyEx(img_bin, cv2.MORPH_OPEN, kernel)
		img_dil = cv2.resize(img_dil1, (img_width, img_height), 1)
		if DEBUG:
		    cv2.imwrite(color+'.jpg', img_dil)
		x, y = conts(img_dil,color)
		#print img_dil.shape
		'''if color == 'pink':
			give_img = img_dil'''
		a_x.append(x)
		a_y.append(y)
	
	return img_hsv, a_x, a_y

def conts(img_dil, color):
	x = 0
	y = 0
	if float(cv2.__version__[:3]) < 3.0:
            contours, hierarchy= cv2.findContours(img_dil, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        else:
            _, contours, hierarchy= cv2.findContours(img_dil, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

	largest_contours = largest_n_contours(contours, 3)
        for idx in xrange(len(largest_contours)):
            blob = Blob()
            if color == MARKER_COLORS['ball'][0]:
                (x,y),radius = cv2.minEnclosingCircle(largest_contours[idx][1])
                radius = int(radius)
                
            else: #if not circle then rectangle
                (x, y), (w, h), r_angle = cv2.minAreaRect(largest_contours[idx][1])
            

	return x,y

def canny(image):
	gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
	blurred = cv2.GaussianBlur(gray, (5, 5), 0)
	edged = cv2.Canny(blurred, 50, 200, 255)
	img_bin = cv2.inRange(image, COLORS['pink'][0], COLORS['pink'][1])
	cnts = cv2.findContours(img_bin.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
	cnts = cnts[0]
	mat_x = []
	mat_y = []
	mat_w = []
	mat_h = []
	for c in cnts:
		(x, y, w, h) = cv2.boundingRect(c)
		mat_x.append(x)
		mat_y.append(y)
		mat_w.append(w)
		mat_h.append(h)

	return mat_x, mat_y, mat_w, mat_h


def view_image(imgx):
	global pos_y, pos_x
	bridge = CvBridge()
	imgx = bridge.imgmsg_to_cv2(imgx, "bgr8")
	des = imgx.copy()
	sou = des.copy()
	sou, mx, my = undistort(sou)
	#sou = cv2.resize(sou,(425,350), cv2.INTER_CUBIC)
	#des = cv2.resize(des,(850,700), cv2.INTER_CUBIC)
	dx, dy, w, h = canny(des)
	#print '----'
	#print '----'
	
	for i in range(len(my)):
		x = int(mx[i]*0.5)
		y = int(my[i]*0.5)
		cv2.putText(sou, str(x)+'x+'+str(y)+'y',(x,y), cv2.FONT_HERSHEY_SIMPLEX,0.35,(0,255,0),1)
		cv2.line(sou, (212,350), (212,0),(0,255,0),1)
		cv2.line(sou, (0,175), (425,175),(0,255,0),1)
	'''for i in range(len(dx)):
		cv2.putText(des, str(dx[i])+'x+'+str(dy[i])+'y',(dx[i],dy[i]), cv2.FONT_HERSHEY_SIMPLEX,0.35,(0,255,0),1)
		cv2.rectangle(des, (dx[i], dy[i]), (dx[i] + w[i], dy[i] + h[i]), (0, 0, 255), 1)
	'''#print mx,my
	#print pos_x, pos_y
	print '----'
	pos_y = [0]
	pos_x = [0]																		  
	cv2.imshow('wall',sou)
	#cv2.imshow('sou',des)
	cv2.waitKey(33)
	sou = des.copy()
	#cv2.destroyWindow('wall')

def give_pos(blobs):
	global pos_x, pos_y
	for b in blobs.blobs:
		pos_x.append(b.x)
		pos_y.append(b.y)


def main():
	rospy.init_node('sunday_node',anonymous=True)
	rospy.Subscriber('/abebe/camera/image_raw', Image, view_image)
	rospy.Subscriber('/abebe/blobs', Blobs, give_pos)


	rospy.spin()


if __name__ == '__main__':
	main()

