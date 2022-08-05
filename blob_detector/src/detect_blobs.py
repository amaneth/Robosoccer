#!/usr/bin/env python
'''
    Blob detection
 
    This node looks for the colors in the COLORS dictionary 
    from every frame and publishes it on a ros topic with
    CmVision Blobs message type.

    Image coming from the subscribed topic is first binarized
    to the color it looks for then erosion and dialation is 
    performed for filtering then xy location in frame and 
    area of the region is obtained from the largest contours.

    ACKNOWLEDGMENT
    Some code taken from Daniel Fekadu's and Abenezer Negussie's 
    previous code.
'''
import rospy
import cv2
import numpy as np
import math
import time

from geometry_msgs.msg import Point

from std_msgs.msg import Header

from sensor_msgs.msg import Image
from cv_bridge import CvBridge


from blob_detector.msg import Blobs
from blob_detector.msg import Blob



DEBUG = True


MARKER_COLORS = rospy.get_param("marker_mapping")    
MARKER_POSITIONS = rospy.get_param("marker_positions")
COLORS = {'red' : [np.array([174, 90, 110], np.uint8), np.array([179, 255, 255], np.uint8)],
    'blue' :      [np.array([83,  114, 57], np.uint8), np.array([128, 255, 255], np.uint8)],
    'yellow' :    [np.array([20,  80,  50], np.uint8),  np.array([40, 255, 255], np.uint8)],
    'pink' :      [np.array([132, 90,  70], np.uint8), np.array([173, 255, 255], np.uint8)],
  'white' :     [np.array([0, 0,  205], np.uint8), np.array([179, 74, 255], np.uint8)]
}


#undistortion parameters
c_mtx = np.array([
[586.828741, 0.000000, 658.171801],
[ 0.000000, 591.121354, 437.937937],
[0.0, 0.0, 1.0]])

HEIGHT = 960
WIDTH = 1280

dist_c = np.array([-0.241325, 0.036928, -0.001418, -0.003593, 0.000000])
camera_mtx, roi = cv2.getOptimalNewCameraMatrix(c_mtx, dist_c, (WIDTH, HEIGHT), 1, (WIDTH, HEIGHT))
map1, map2 = cv2.initUndistortRectifyMap(c_mtx, dist_c, None, camera_mtx, (WIDTH, HEIGHT), 5)
#roi = 405, 65, 740, 344 #roi - for some reason I'm not particularly interested in investigating, returned ROI doesn't work
roi = 250, 150, 850, 700

ROI_X, ROI_Y, ROI_W, ROI_H = roi




kernel = np.ones((5, 5), np.uint8)
#kernel = cv2.getStructuringElement(cv2.MORPH_CROSS,(5,5))        
    

def largest_n_contours(contours, num):
    #print 'here'
    area_contours = []
    if len(contours) > 0:
        for contour in contours:
    	    area = cv2.contourArea(contour)
	    area_contours.append([area, contour])
	area_contours = sorted(area_contours, key=lambda x : x[0], reverse=True)
	area_contours = area_contours[:num if num < len(area_contours) else len(area_contours)]
    return area_contours

    
def shape_class(cnt):
    mmnt = cv2.moments(cnt)
    cx = int((M["m10"] / M["m00"]))
    cy = int((M["m01"] / M["m00"]))
    p = cv2.arcLength(cnt, True)
    vert = cv2.approxPolyDP(cnt, 0.04*p, True)
        
    


def image_cb(data):
    i = 0
    #print 'The Start'
    img = bridge.imgmsg_to_cv2(data, "bgr8")
    #undistorting image
    u_img = cv2.remap(img, map1, map2, cv2.INTER_LINEAR)
    img = u_img[ROI_Y:ROI_Y+ROI_H, ROI_X:ROI_X+ROI_W] #croping to region of interest

    img_height, img_width = np.shape(img)[:2]

    if DEBUG:
        img_p = bridge.cv2_to_imgmsg(img, "bgr8")
        pub_debug.publish(img_p)
    
    blobs = Blobs()
    blobs.header = Header()
    blobs.header.stamp = rospy.Time.now()
    blobs.header.seq = data.header.seq
    blobs.image_height = np.shape(img)[0]
    blobs.image_width = np.shape(img)[1]
    
    total_time = 0
    img = cv2.resize(img, (img_width/2, img_height/2), 1)
    img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    for color, hsv_range in COLORS.iteritems():
        #print 'The color starts here.', color
        st_time = time.time()

	img_bin = cv2.inRange(img_hsv, hsv_range[0], hsv_range[1])
	img_dil = cv2.morphologyEx(img_bin, cv2.MORPH_OPEN, kernel)
	img_dil = cv2.resize(img_dil, (img_width, img_height), 1)
	if DEBUG:
	    cv2.imwrite(color+'.jpg', img_dil)
	#get contours of binarized blobs
        contours, hierarchy = 0, 0
        if float(cv2.__version__[:3]) < 3.0:
            contours, hierarchy= cv2.findContours(img_dil, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        else:
            _, contours, hierarchy= cv2.findContours(img_dil, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

	largest_contours = largest_n_contours(contours, 3)
        for idx in xrange(len(largest_contours)):
            blob = Blob()
            i = i + 1
            print i, color
            if color == MARKER_COLORS['ball'][0]:
                (x,y),radius = cv2.minEnclosingCircle(largest_contours[idx][1])
                radius = int(radius)
                blob.x = int(x)
                blob.y = int(y)
            else: #if not circle then rectangle
                (x, y), (w, h), r_angle = cv2.minAreaRect(largest_contours[idx][1])
                #this works even when the rectangle is rotated... no bounding box problem
                #like faced with the CmVision shit... the params are
                #center (x, y) , width and height(w,h) and angle of rotation
                #except for the center, idk how to use the others 
                blob.x = int(x)
                blob.y = int(y)
            blob.name = color
            blob.area = largest_contours[idx][0]
            blob.top, blob.bottom, blob.right, blob.left = 0,0,0,0
            print 'x, y', blob.x,blob.y
            blobs.blobs.append(blob)
        if DEBUG:
	    time_taken = time.time() - st_time
	    total_time += time_taken
	    print "++++++++++++++++++++++++++++++TIME TAKENx = " + str(time_taken)

    if DEBUG:
        print "********************************TOTAL TIME TAKEN = "  + str(total_time)
    blobs.blob_count = len(blobs.blobs)
    pub.publish(blobs)



rospy.init_node('blob_detector', anonymous=False, log_level=rospy.INFO)
rospy.loginfo("Starting Blob Detector")
print 'Not passed'
bridge = CvBridge()
pub = rospy.Publisher('blobs', Blobs, queue_size=1)

if DEBUG:
    pub_debug = rospy.Publisher('filtered_image', Image, queue_size=1)


rospy.Subscriber('camera/image_raw', Image, callback=image_cb, queue_size=1)

try:
    print 'Passed-1'
    rospy.spin()
    print 'Passed-2'

except KeyboardInterrupt:
    rospy.loginfo("Stopping Blob Detector")
