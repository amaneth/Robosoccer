#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import math

from std_msgs.msg import Header

from blob_detector.msg import Blobs
from blob_detector.msg import Blob

from blob_mapping.msg import MappedBlobs
from blob_mapping.msg import MappedBlob

from blob_mapping.msg import Marker
from blob_mapping.msg import Observed_Markers

"""
  This node will take as input blobs perceived by cmvision and calculate
  normalized angle and area using the camera_mapping parameters.

  The output will be blobs corrected for lens warp to be consumed by
  robot_localization.

"""

rospy.init_node('blob_mapping', anonymous=False, log_level=rospy.INFO)
rospy.loginfo("Blob Mapping Node Launching...")

# grab the coefficients for mapping    
fov_h = float(rospy.get_param("camera_mapping/fov_horizontal"))
fov_v = float(rospy.get_param("camera_mapping/fov_vertical"))
distortion_coeff = float(rospy.get_param("camera_mapping/distortion_coeff"))
markers_map = rospy.get_param("marker_mapping")


mblob_pub = rospy.Publisher('mapped_blobs', MappedBlobs, queue_size=1)
marker_pub = rospy.Publisher('observed_markers', Observed_Markers, queue_size=1)

#constants
V_ANGLE_THRESHOLD = 40  	# ignore everythings above this angle
MERGE_THRESHOLD = 300	# threshold 
AREA_THRESHOLD = 400
MARKER_AREA = 7.04
FOCAL_LENGTH = 800 

PAIR_PARAM = [0.9, 0.7, 0.9, 0.65]
DEBUG = True


def func_1(val1, val2, thresh):
    z = abs(val1 - val2)/float(max(abs(val1), abs(val2)))
    return 2*((1./3)/(1 + math.exp(abs(z*thresh*10))))

def func_2(val1, val2):
    z = val1 - val2
    return ((1./3) / (1+math.exp(-z)))


def can_pair(pair):
    '''
    Any two blobs can be paired if:
        it's not the same blob AND
    	the vertical angle of the first is greater than the second AND
	the difference between the horizontal angle of the two is less than some threshold AND
	the difference between their areas is less than some threshold AND
	the pair is known to exist in some configuration file (from robot_localization node)
    '''
    #if pair of blobs are not the same blob twice
    if pair[0] == pair[1]:
         if DEBUG:
             print ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>SAME BLOB"
         False
	 return False

    #if pair of colors are not known to exist in the map -> NO PAIR
    if markers_map.values().count([pair[0].name, pair[1].name]) == 0:
        if DEBUG:
            print ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>DON'T EXIST IN MAP PROBLEM"
	return False
    
    #TODO this here needs more testing. - 
    #TODO noticed somthing weird... it's could think it's looking at the same marker
    #twice or more so when this happens pop both of them out
    v1 = func_1(pair[0].area, pair[1].area, 0.5)
    v2 = func_2(pair[0].angle_vertical, pair[1].angle_vertical)
    v3 = func_1(pair[0].angle_horizontal, pair[1].angle_horizontal, 0.1)
    V = (PAIR_PARAM[0] * v1) + (PAIR_PARAM[1] * v2) + (PAIR_PARAM[2] * v3)
    if DEBUG:
        print '>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> V VALUE =', V
    if V > PAIR_PARAM[3]:
        return True
    else:
        return False



def is_irrelevant(blob):
    if blob.area < AREA_THRESHOLD or \
      blob.angle_vertical > V_ANGLE_THRESHOLD:
        return True

    return False
	




def blobs_cb(blobs):
    mapped_blobs = MappedBlobs()
    mapped_blobs.header = Header()
    mapped_blobs.header.stamp = rospy.Time.now()
    mapped_blobs.header.seq = blobs.header.seq
    
    for blob in blobs.blobs:
        mapped_blob = MappedBlob()
        mapped_blob.name = blob.name

        normalized_x = float(blob.x) / blobs.image_width 
        normalized_y = float(blob.y) / blobs.image_height
        mapped_blob.angle_horizontal = (normalized_x * fov_h) - fov_h / 2.0
        mapped_blob.angle_vertical = fov_v / 2.0 - (normalized_y * fov_v) 
        #TODO we may need to do some correction on the area here.
        mapped_blob.area = blob.area
	
        #check relevance
        if not is_irrelevant(mapped_blob):
            mapped_blobs.blobs.append(mapped_blob)

    o_markers = Observed_Markers()
    o_markers.header = Header()
    o_markers.header.stamp = rospy.Time.now()
    o_markers.header.seq = blobs.header.seq

    #backup array to check for goal and ball
    blobs_bck = list(mapped_blobs.blobs)
    o_mrkr_names = []

    for blob1 in mapped_blobs.blobs:
        for blob2 in mapped_blobs.blobs:
            pair = [blob1, blob2]
            if can_pair(pair):
                mrkr = Marker()
                mrkr.name = markers_map.keys()[markers_map.values().index([pair[0].name, pair[1].name])]
                mrkr.angle_vertical = (pair[0].angle_vertical + pair[1].angle_vertical) / 2.0
                mrkr.angle_horizontal = (pair[0].angle_horizontal + pair[1].angle_horizontal) / 2.0
                mrkr.area = (pair[0].area + pair[1].area) / 2.0
		mrkr.distance = (MARKER_AREA * FOCAL_LENGTH) / mrkr.area
		#noticed that it mistakenly thinks it's seeing the same blob more than once.
		#remove all same observed markers cuz I can't tell which one is the right one.
		if o_mrkr_names.count(mrkr.name) > 0:
		    for m in list(o_markers.markers):
                        if m.name == mrkr.name:
                            o_markers.markers.pop(o_markers.markers.index(m))
                            break
                else:
                    o_markers.markers.append(mrkr)
		    o_mrkr_names.append(mrkr.name)
                #poop out the blob from backup blobs array
                if blobs_bck.count(pair[0]) != 0:
                    blobs_bck.pop(blobs_bck.index(pair[0]))
                if blobs_bck.count(pair[1]) != 0:
                    blobs_bck.pop(blobs_bck.index(pair[1]))
    
    #these are th remaining blobs
    big_one = 0
    for blob in blobs_bck:
        #placed blob name in [] when indexing because markers_map has values in lists.
        #check laboratory-map config file in robot_localization to see
        if markers_map.values().count([blob.name]) != 0:
            mrkr = Marker()
            mrkr.name = markers_map.keys()[markers_map.values().index([blob.name])]
            mrkr.angle_vertical = blob.angle_vertical
            mrkr.angle_horizontal = blob.angle_horizontal
            mrkr.area = blob.area
            #take the biggest f them all. this is idiotic. it doesn't even consider balls.
	    #XXX fix me
            if big_one < mrkr.area:
	        o_markers.markers.append(mrkr)
                big_one = mrkr.area

    mapped_blobs.blob_count = len(mapped_blobs.blobs)
    mblob_pub.publish(mapped_blobs)
    o_markers.marker_count = len(o_markers.markers)
    marker_pub.publish(o_markers)


rospy.Subscriber("blobs", Blobs, blobs_cb)                            
rospy.spin()
