#!/usr/bin/env python
import rospy
import smbus
import time
import math
import time
from std_msgs.msg import Int32
bus = smbus.SMBus(1)
address = 0x1e


def read_byte(adr):
    return bus.read_byte_data(address, adr)

def read_word(adr):
    high = bus.read_byte_data(address, adr)
    low = bus.read_byte_data(address, adr+1)
    val = (high << 8) + low
    return val

def read_word_2c(adr):
    val = read_word(adr)
    if (val >= 0x8000):
        return -((65535 - val) + 1)
    else:
        return val

def write_byte(adr, value):
    bus.write_byte_data(address, adr, value)


def read_compass():
        x_out = read_word_2c(3) * scale
        y_out = read_word_2c(7) * scale
        z_out = read_word_2c(5) * scale
	return math.atan2(y_out, x_out) / 1.745329252 * 100

def full_range():
	bearing = read_compass()
	if bearing < 0:
		bearing += 360
	return bearing

write_byte(0, 0b01110000) # Set to 8 samples @ 15Hz
write_byte(1, 0b00100000) # 1.3 gain LSb / Gauss 1090 (default)
write_byte(2, 0b00000000) # Continuous sampling

scale = 0.92
NODE_NAME = 'compass'
rospy.init_node(NODE_NAME, anonymous=False)
publisher = rospy.Publisher('compass',Int32, queue_size=1)
rate = rospy.Rate(2)

#here's just a little work around to offset reading with the first ever reading taken as zero
MAX = 180
init_reading = read_compass()
w_range, sign = ((-MAX, -MAX + (init_reading - 1)), 1) if init_reading >= 0 else ((MAX - (abs(init_reading) - 1), MAX), -1)
diffr = 360 - (abs(init_reading) - 1)

try:
    while True and not rospy.is_shutdown():
	bearing = read_compass()
	if w_range[0] <= bearing <= w_range[1]:
		bearing = sign * (diffr - abs(bearing))
	else:
		bearing -= init_reading
	publisher.publish(bearing)
	rate.sleep()
except rospy.ROSInterruptException:
    print "Exiting..."
