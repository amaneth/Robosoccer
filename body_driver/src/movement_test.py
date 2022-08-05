#!/usr/bin/env python
import rospy
import numpy as np
import cv2
import time
from geometry_msgs.msg import Pose2D, Point
from std_msgs.msg import String
from blob_mapping.msg import Observed_Markers
from blob_mapping.msg import Marker


h_angle = 0
h_angle = 0
exist = 0

def toggle(flag, number):
	if (number > 0):
		side = 'TURN_RIGHT_STEP'
	else:
		side = 'TURN_LEFT_STEP'

	if flag == 0:
		return 1, 'STEP_FWD'
	else :
		return 0, side
def wait(delay):
	t = time.time()
	while(time.time() - t < delay):
		pass
	return None

def turn():
	print 'Turning in Progress ...'
	pub_command = rospy.Publisher('/send_move_command', String, queue_size = 1)
	st = 'TURN_LEFT_STEP'
	for i in range(4):
		print 'Command=', st
		pub_command.publish(st)
		wait(2.7)
	st = 'STEP_FWD'
	for i in range(4):
		print 'Command=', st
		pub_command.publish(st)
		wait(2.7)
	st = 'TURN_RIGHT_STEP'
	for i in range(4):
		print 'Command=', st
		pub_command.publish(st)
		wait(2.7)
	
s_i = 0
r_i = 0
a_i = 0
t = time.time()
ha = 0
va = 0
d = 0
state = 'search'
traj = False
def set_angel(all_markers):
	print 'Entereds'
	global ha, va, d, s_i, r_i, a_i, t, state, traj
	ha_p,va_p, d_p = 0, 0, 0
	exist = 0
	pub_command = rospy.Publisher('/send_move_command', String, queue_size = 1)
	command = 'STOP'
	#--------Check to see if the ball is seen with the current frame-------#
	for i in all_markers.markers:
		print i.name
		if (i.name == 'ball'):
			exist = 1
			ha_p = i.angle_horizontal
			va_p = i.angle_vertical
			d_p = i.distance
			break
		else :
			exist = 0

	#-------If there is a new data, is it plausable to change from previous(nothing done yet)------#
	t_de = ((d == 0) and (va == 0) and (ha == 0))	
	if (exist):
		t2 = abs(ha - ha_p) > 10					#---in trial
		ha = ha_p
		va = va_p
		d = d_p
		t = time.time()
	#---------set of tests to determine the state of the motion-----------#
	t_t = (t - time.time()) > 15		#----the time test
	t_d = d < 0.8						#----the 'how far test'
	t_ha = ha < 0
	t_center = abs(ha) < 20				#----approximately, is the ball somewhere in the center
	t_center_r = abs(ha) < 10
	t_va = va < -35
	t_tu = va < -18
	t_tf = (t - time.time())  > 5

	#
	if (t_t or t_de):
		state = 'search'
	elif (t_tu and not(traj)):
		state = 'turn'
	elif ((t_va or t_d) and traj):
		state = 'reach'
	else :
		state = 'approch'

	#
	if (state == 'search'):
		print 'Because of time',t_t,'or defa', t_de 
		s_i,command = toggle(s_i, -9)

	elif (state == 'reach'):
		if (t_tf or t_center_r):
			command = 'STEP_FWD'
		else :
			r_i,command = toggle(r_i,ha)

	elif (state = 'turn'):
		turn()
		traj = True
	elif (state == 'approch'):
		if (t_tf or t_center):
			command = 'STEP_FWD'
		else :
			r_i,command = toggle(r_i,ha)

	else:
		pass
	print 'State = ',state, 'Command=', command, 'va=',va, 'ha=', ha 
	
	
	rate = rospy.Rate(0.5)
	pub_command.publish(command)
	rate.sleep()
	
		

	

#--------------------Subscribe to both Mapper and Localizer----------------------#
rospy.init_node('Movements',anonymous=True)

#rospy.Subscriber('position/self2',Pose2D, execute)
rospy.Subscriber('/abebe/Observed_Markers',Observed_Markers, set_angel, queue_size=1)
rospy.spin()

	
