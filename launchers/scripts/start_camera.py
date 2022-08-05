#!/usr/bin/env python 

import rospy
from std_srvs.srv import Empty
print '-------------------HERE------------------'
rospy.wait_for_service('camera/start_capture')
st = rospy.ServiceProxy('camera/start_capture', Empty)
st()
