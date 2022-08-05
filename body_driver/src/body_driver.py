#!/usr/bin/env python
import rospy
import time 
from std_msgs.msg import String, Int32
import RPi.GPIO as GPIO
IR_PIN  = 17
HIGH  = 1
LOW = 0
NODE_NAME = "body_driver"
MOVE_TIME = 2
IR_CODES = rospy.get_param('IR_CODES')

def decode_command(cmd):
	if cmd in IR_CODES.keys():
		return IR_CODES[cmd]
	return -1	

def send_command(data):
	cmd  = decode_command(data.data)
        if cmd != -1:	
		tell(cmd)
		time.sleep(MOVE_TIME)
		tell(IR_CODES['STOP']) #stop
	else:
	     print "invalid command!"

def delay(units):
	time.sleep(833 * units/1000000.0)

def tell(arg) :
        command = arg
	GPIO.output(IR_PIN,LOW)
	delay(8)
	#Pulse the 8 bit command. Delay 4 slices for 1-bit and 1 slice for a 0-bit.
	for b in range(7,-1,-1):
		GPIO.output(IR_PIN,HIGH);
		delay( 4 if((command & 128) != 0) else 1)
		GPIO.output(IR_PIN,LOW)
		delay(1)
		command <<= 1
		print 4 if((command & 128) != 0) else 1
	GPIO.output(IR_PIN,HIGH)
def initial() :
	GPIO.setmode(GPIO.BCM)
	GPIO.cleanup(IR_PIN)
	GPIO.setup(0, GPIO.IN)
	GPIO.setup(IR_PIN, GPIO.OUT)
	GPIO.output(IR_PIN, HIGH)
	rospy.init_node(NODE_NAME, anonymous=False)
	subscriber = rospy.Subscriber("send_move_command", String, send_command)
	
initial()
try:
	rospy.spin()
except KeyboardInterrupt:
        GPIO.cleanup(IR_PIN)
	rospy.loginfo("Stopping " + NODE_NAME)










