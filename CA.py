#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''-------------------------- Program Header ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------''
  ' Add all important details regarding the program here
  '
  ' Project             : BAE Swarm Challenge - Group Design Project
  ' Program name        : SAMPLE_CODE_gdpa.py
  ' Author              : Romain Delabeye
  ' Adm. structure      : Cranfield University
  ' Date (YYYYMMDD)     : 20200205
  ' Purpose             : provide code structure
  ' User                : A_Team
  '
  ' Revision History    : 1.0
  '
  ' Date YYYYMMDD |  Author          | Ref       |Revision comment
  '-------------------------------------------------------------------
  ' 20200205      |  Romain DELABEYE |           | Developing & Structuring - howto publish&subscribe; struct
  ' 20200206      |  Kester Broatch  |           | Add Collision Avoidance
  '               |                  |           |
  '               |                  |           |


TODO:
""" Put here all your tasks """
(You can also add TODO anywhere in the code if you need to come back to a certain line later on)

-
-
-

''-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------'''
'''
'------------------------------------------------------------------------------------------------------------------------
'    import libraries
'------------------------------------------------------------------------------------------------------------------------
'''
## import libraries - comment the undesired libraries
# import os
# import time

# import math
# import numpy as np
# from numpy import linalg as npla
# # from scipy import linalg
# # import matplotlib.pyplot as plt
# # from mpl_toolkits.mplot3d import Axes3D

# import rospy
# #from std_msgs.msg import *
import message_filters

'''
'------------------------------------------------------------------------------------------------------------------------
'    Toolbox
'------------------------------------------------------------------------------------------------------------------------
'''
'''
Put here your toolbox (functions)
'''
import rospy
from std_msgs.msg import String
from test.msg import position
from test.msg import multipositions
from cv_bridge import CvBridge,CvBridgeError
import sys
ros_path = '/opt/ros/melodic/lib/python2.7/dist-packages'

if ros_path in sys.path:
    sys.path.remove(ros_path)

import cv2
import math
import numpy as np

'''
'------------------------------------------------------------------------------------------------------------------------
'    Classes
'------------------------------------------------------------------------------------------------------------------------
'''
'''
Put here your classes.
'''


'''
'------------------------------------------------------------------------------------------------------------------------
'    Main
'------------------------------------------------------------------------------------------------------------------------
'''
camera_fx = 383.599
camera_fy = 383.599
camera_cx = 320.583
camera_cy = 238.327

def fun(x,y,z,d,a1,a2):
    pass

def getDistanceAngle(pixel_x, pixel_y, real_z):
    z = np.float(real_z)
    x = (pixel_x-camera_cx)*z/camera_fx
    y = (pixel_y-camera_cy)*z/camera_fy

    horizon_angle = math.atan2(x,z)
    vertical_angle = math.atan2(y,z)
    absolute_distance = math.sqrt(x*x+y*y+z*z)
    print("x: ",x)
    print("y: ",y)
    print("z: ",z)
    print("absolut_distance: ",absolute_distance)
    print("horizon_angle: ",horizon_angle)
    print("vertical_angle: ",vertical_angle)
    #pass


def callback(data):
    '''444 Callback Function'''
    global pub

    try:
        # ----------------------- LOOP --------------------- [main loop]
        # Your data processing here


        # Calculating distance and angle to target #################################
        #print("I hear")
        #print(data.lists)
        for item in data.lists:
            mid_u = (item.right-item.left)/2+item.left
            mid_v = (item.bottom-item.top)/2+item.top
            getDistanceAngle(mid_u,mid_v,item.distance)
        
	    # Publishing Placeholder ######################################################
        var2publish1 = rospy.get_caller_id() + 'I heard %s', data.lists

        # Publish your SA_multipositions here
        rospy.loginfo(var2publish1)  # [optional] print & save data in the node's terminal
        pub.publish(var2publish1)
        
        ##################################################################
        rate.sleep() # to reach rospy.Rate(...) frequency.
        pass
    except rospy.ROSInterruptException:
        pass
    

if __name__ == "__main__":

    # Setup node (name)
    rospy.init_node('CA', anonymous=False)

    # do nothing
    quickFix_header = Header()
    quickFix_header.stamp = rospy.Time.now()
    String.header = quickFix_header # adds a header in String to synchronize subscribing


    # Setup publishers (name, dataType)
    pub = rospy.Publisher('pub_name', String, queue_size=10)

    rate = rospy.Rate(10) # setup publishing frequency, in Hz

    #TO DO - change code below to subscribe using message filters instead

    # Setup subscribers (name, dataType)
    #ts = message_filters.TimeSynchronizer([message_filters.Subscriber('bbox_position2', multipositions)],10)
                                           # message_filters.Subscriber('subs_name2', String)], 10)

    # callback for treatment of data from subscribers above
    #ts.registerCallback(callback)

    rospy.Subscriber("bbox_position2", multipositions, callback)



    # Prevent undesired program ending
    rospy.spin()
    pass
