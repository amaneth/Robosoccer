#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Header
from blob_mapping.msg import Marker
from blob_mapping.msg import Observed_Markers


####################################################################################################
																									######
COLORS = {'Red' : [np.array([0, 127, 20], np.uint64), np.array([8, 255, 128], np.uint64)],		#RED
		'Blue' :      [np.array([83,  114, 57], np.uint64), np.array([128, 255, 255], np.uint64)],	#BLUE
		'Yellow' :    [np.array([20,  56,  50], np.uint64),  np.array([40, 255, 255], np.uint64)],	#YELLOW
		'Pink' :      [np.array([132, 90,  70], np.uint64), np.array([173, 255, 255], np.uint64)],	#PINK
		'White' :     [np.array([84, 0,  186], np.uint64), np.array([173, 74, 255], np.uint64)]		#WHITE
}																									######
color_words = {1:'Red', 2:'Blue', 3:'Yellow', 4:'Pink', 5:'White'}

maps = {'Yellow-Blue':'marker_top_left' ,'Yellow-White': 'marker_top_center', 'Yellow-Pink':'marker_top_right' ,
		'Pink-Yellow': 'marker_bot_right', 'White-Yellow': 'marker_bot_center', 'Blue-Yellow':'marker_bot_left' ,
		'Red':'ball', 'Pink':'home_goal', 'Blue':'visit_goal'}

####################################################################################################
WIDTH = 1280
HEIGHT = 960
c_mtx = np.array([
[586.828741, 0.000000, 658.171801],
[ 0.000000, 591.121354, 437.937937],
[0.0, 0.0, 1.0]])
dist_c = np.array([-0.241325, 0.036928, -0.001418, -0.003593, 0.000000])
camera_mtx, roi = cv2.getOptimalNewCameraMatrix(c_mtx, dist_c, (WIDTH, HEIGHT), 1, (WIDTH, HEIGHT))
map1, map2 = cv2.initUndistortRectifyMap(c_mtx, dist_c, None, camera_mtx, (WIDTH, HEIGHT), 5)
ROI_X = 250
ROI_Y = 150
ROI_W = 850
ROI_H = 700

####################################################################################################

Markers = rospy.Publisher('/abebe/Observed_Markers', Observed_Markers, queue_size=1)

####################################################################################################

min_area = 300
max_area = 52000

####################################################################################################
ix = 1
def detector(sou):
	global camera_mtx, ROI_Y, ROI_X, ROI_W, ROI_H, min_area, max_area, ix
	im_name = 'still_image/image' + str(ix) + '.png'
	ix += 1

	u_img = cv2.remap(sou, map1, map2, cv2.INTER_LINEAR)
	img = u_img[ROI_Y:ROI_Y+ROI_H, ROI_X:ROI_X+ROI_W] #croping to region of interest
	hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
	#cv2.imwrite(im_name, img)
	#print '============================>>>',hsv_img.shape
	kernel = np.ones((5,5), np.uint8)

	A = []
	C = []
	D = []	
	HA = []
	VA = []
	print '******'
	for color, ranges in COLORS.items():
		
		thresh = cv2.inRange(hsv_img,ranges[0] , ranges[1] )
		im3 =  cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)
		#im3 =  cv2.erode(thresh, kernel, iterations = 1)
		im2, contours, hierarchy, = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
		if color == 'Red':
			cv2.drawContours(img, contours, -1, (255,0,0), 1)	
		
		
		n = len(contours)
		
		for i in range(n):
			if color == 'Red':
				(x,y), radius = cv2.minEnclosingCircle(contours[i])
				print 'Color = ', color, 'Position = ', x,y, 'Area = ', cv2.contourArea(contours[i])
			else:
				(x,y), (w,h), r_angle = cv2.minAreaRect(contours[i])
			area = cv2.contourArea(contours[i])
			
			if area > min_area and area < max_area :
				print 'Color = ', color, 'Position = ', x,y, 'Area = ', area
				print '------'
				A.append(area)
				C.append(color)
				HA.append(x)
				VA.append(y)
				cv2.putText(img, 'x', (int(x),int(y)), cv2.FONT_HERSHEY_SIMPLEX,0.35,(255,0,0),3)
			
				
				d = 5632/area
				D.append(d)
	print '******'
	sou = cv2.resize(img, (425,350), cv2.INTER_LINEAR)
	y =  175 - (-35 * (350/90) )
	cv2.line(sou, (0,int(y)), (425, int(y)), (255,0,0), 2)
	return A, C, D, HA, VA, sou

			


def Mapper(image_data):
	bridge = CvBridge()
	img = bridge.imgmsg_to_cv2(image_data, "bgr8")
	Area, Color, Distance, HA, VA, img = detector(img)

	n = len(Area)
	markers = Observed_Markers()
	markers.header = Header()
	markers.header.stamp = rospy.Time.now()
	markers.header.seq = 2
	accumlator = []
	used_index = []
	#=======================Isolate The ball======================#
	for i in range(n):
		if Color[i] == 'Red':
			used_index.append(i)
			mark = Marker()
			mark.name = maps['Red']
			mark.angle_horizontal = HA[i]/850 *160 - 80
			mark.angle_vertical = 45 - VA[i]/700 * 90
			mark.area = Area[i]
			mark.distance = 5632/mark.area #* an                  uncomment this for the distortion consideration
			markers.markers.append(mark)
			print 'Name = ', maps[Color[i]]
			print 'Area = ', Area[i]
			print 'Position_1 = ',mark.angle_horizontal, mark.angle_vertical
			print 'Distance = ', Distance[i]
			print '--------------------------------------------------------------'

	#======================Detect Markers=========================#
	for i in range(n):
		for j in range(n):
			t1 = abs(HA[i] - HA[j]) < 65
			map_name = 'm'
			if VA[i] > VA[j]:
				map_name = Color[j] + '-' + Color[i]
			else :
				map_name = Color[i] + '-' + Color[j]
			t2 = map_name in maps
			t3 = not(map_name in accumlator)
			t4 = abs(VA[i] - VA[j]) < 120
			t5 = not(i in used_index or j in used_index)


			if t1 and t2 and t3 and t4 and t5:
				used_index.append(i)
				used_index.append(j)
				accumlator.append(map_name)

				

				mark = Marker()
				mark.name = maps[map_name]
				a_h = (HA[i] + HA[j]) / 2
				a_v = (VA[i] + VA[j]) / 2

				a_h = abs((a_h / 850) * 180 - 90)
				a_v = abs(45 - (a_v/ 700) * 90)
				mark.angle_horizontal = (HA[i] + HA[j]) / 2
				mark.angle_vertical = (VA[i] + VA[j]) / 2
				mark.area = (Area[i] + Area[j]) / 2
				an = a_h * 0.0168 + a_v * -0.0103
				an = np.exp(an)
				mark.distance = 5632/mark.area #* an                  uncomment this for the distortion consideration

				markers.markers.append(mark)
				print 'Name = ', map_name
				print 'Area = ', mark.area
				print 'Position_1 = ', HA[i], VA[i]
				print 'Position_2 = ', HA[j], VA[j]
				print 'Distance = ', mark.distance 
				print 'Factor = ', an
				print '--------------------------------------------------------------'
	m1 = [0,0]
	m2 = [0,0]
	#==========================Goals===============================#
	for i in range(n):
		if (not(i in used_index)):
			if Color[i] == 'Blue':
				if Area[i]>m1[0]:
					m1[0], m1[1] = Area[i], i
			if Color[i] == 'Pink':
				if Area[i]>m2[0]:
					m2[0], m2[1] = Area[i], i
	if (m1[0] > 0):
		i = m1[1]
		print 'Name = ', maps[Color[i]]
		print 'Area = ', Area[i]
		print 'Position = ', HA[i], VA[i]
		print 'Distance = ', Distance[i]
		print '--------------------------------------------------------------'
	if (m2[0] > 0):
		i = m2[1]
		print 'Name = ', maps[Color[i]]
		print 'Area = ', Area[i]
		print 'Position = ', HA[i], VA[i]
		print 'Distance = ', Distance[i]
		print '--------------------------------------------------------------'

	
				
	#rate = rospy.Rate(0.5)
	#rate.sleep()
	Markers.publish(markers)
	cv2.namedWindow('Output')
	cv2.imshow('Output', img)
	cv2.waitKey(24)
	print '=============================================================='
	






	
def main():
	rospy.init_node('blob_mappe', anonymous = True)
	rospy.Subscriber('/abebe/camera/image_raw', Image, Mapper)
	rospy.spin()


if __name__ == '__main__':
	main()

