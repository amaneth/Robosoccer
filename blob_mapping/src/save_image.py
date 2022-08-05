#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Header
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

i = 0

look = {1114033:1, 1114034:2, 1114035:3, 1114036:4, 1114037:5, 1114038:6, 1048608:'SPACE',
		 -1:None, 1114039:7, 1114040:8, 1114041:9, 1113937:'LARROW', 1113938:'UARROW',
		  1113940:'DARROW', 1113939:'RARROW', 1114032:0, 65455:'/', 1114031:'/', 65450:'*',
		  65453:'-', 65451:'+', 65366:'PGDN', 65365:'PGUP', 65367:'END', 65360:'HOME', 
		  27:'ESC', 32:'SPACE', 10:'ENTER', 97:'A', 115:'S', 100:'D', 119:'W', 65507:'LCTRL',
		  65508:'RCTRL', 65506:'RSHIFT', 65505:'LSHIFT', 1114030:'.', 1113997:'NENTER', 
		  1113983:'NLOCK', 65407:'NLOCK', 1114026:'*', 1114029:'-', 1114027:'+', 1113942:'PGDN',
		  1113941:'PGUP', 1113943:'END', 1113936:'HOME', 1048603:'ESC', 1048586:'ENTER',
		  1048673:'A', 1048691:'S', 1048676:'D', 1048695:'W', 1114083:'LCTRL', 1114084:'RCTRL',
		  1114082:'RSHIFT', 1114081:'LSHIFT', 1114089:'LALT', 65513:'LALT', 1114090:'RALT', 
		  65514:'RALT', 65436:1, 65433:2, 65435:3, 65430:4, 65437:5, 65432:6, 65429:7,
		  65431:8, 65434:9, 65438:0, 48:0, 45:'-', 61:'=', 112:'P', 91:'[', 93:']', 108:'L', 
		  59:';', 39:'QUOTE', 44:',', 46:'.', 47:'/', 85:'PGUP', 86:'PGDN', 80:'HOME', 87:'END',
		  255:'DEL', 99:'INS', 233:'LALT'}

def save_it(data):
	global i
	
	img = CvBridge().imgmsg_to_cv2(data, "bgr8")
	des = cv2.resize(img,(425,350),cv2.INTER_LINEAR)
	cv2.imshow('Output',des)
	a = cv2.waitKey(33)
	if look[a] == 'SPACE':
		wrd = 'Image_' + str(i) + '.jpg'
		i+=1
		cv2.imwrite(wrd,img)
		print 'WRITTEN ', str(i), ' image!'


rospy.init_node('save_image', anonymous=False, log_level=rospy.INFO)
rospy.Subscriber("/abebe/camera/image_raw", Image, save_it) 
rospy.spin()  