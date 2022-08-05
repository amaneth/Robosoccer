#!/usr/bin/env python
import rospy
import time
import sys
from std_msgs.msg import String
def wait(delay):
	t = time.time()
	while(time.time() - t < delay):
		pass
	return None

def turn():
	l,j,k = int(sys.argv[1]), int(sys.argv[2]), int(sys.argv[3])
	print 'Turning in Progress ...'
	pub_command = rospy.Publisher('/send_move_command', String, queue_size = 1)
	st = 'TURN_LEFT_STEP'
	for i in range(l):
		print 'Command=', st
		pub_command.publish(st)
		wait(2.7)
	st = 'STEP_FWD'
	for i in range(j):
		print 'Command=', st
		pub_command.publish(st)
		wait(2.7)
	st = 'TURN_RIGHT_STEP'
	for i in range(k):
		print 'Command=', st
		pub_command.publish(st)
		wait(2.7)

rospy.init_node('Movements',anonymous=True)
turn()