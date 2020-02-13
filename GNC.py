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
  ' 20200206      |  Kester Broatch  |           | Added GNC 
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
import os
import time

import math
import numpy as np
from numpy import linalg as npla
import rospy
from std_msgs.msg import *
import message_filters

'''
'------------------------------------------------------------------------------------------------------------------------
'    Toolbox
'------------------------------------------------------------------------------------------------------------------------
'''
from dronekit import *
from sensor_msgs.msg import NavSatFix, Imu
from test.msg import position, multipositions
from CA_functions import getDistanceAngle
from uptime import uptime

'''
'------------------------------------------------------------------------------------------------------------------------
'    Main
'------------------------------------------------------------------------------------------------------------------------
'''


def callback_VehicleState(data):
    global vehicle

    # Imports vehicle state from pixhawk and publishes vehicle states to ros
    try:

        #TODO get xyz, gps and imu from vehicle object

        # Publishing GPS
        gps = NavSatFix()
        gps_pub = rospy.Publisher('VehicleState/gps', NavSatFix, queue_size=10)
        gps_rate = rospy.Rate(10) # setup publishing frequency, in Hz
        #rospy.loginfo(gps)  # [optional] print & save data in the node's terminal
        gps_pub.publish(gps)
        gps_rate.sleep() # to reach rospy.Rate(...) frequency.

        # Publishing Imu
        imu = Imu()
        imu_pub = rospy.Publisher('VehicleState/imu', Imu, queue_size=10)
        imu_rate = rospy.Rate(10) # setup publishing frequency, in Hz
        #rospy.loginfo(imu)  # [optional] print & save data in the node's terminal
        imu_pub.publish(imu)
        imu_rate.sleep() # to reach rospy.Rate(...) frequency.    

        pass
    except rospy.ROSInterruptException:
        pass
    
def callback_GNCTaskManager(data): #TODO subcribe to TC, SA, MC
    global vehicle

    # Decides which GNC algorithm should be used, based upon TC, SA, MC
    try:

        pass
    except rospy.ROSInterruptException:
        pass

def callback_CollisionAvoidance(data): 
    global vehicle

    # Implements collision avoidance 
    try:
        # collect SA info from multiple targets
        for item in data.lists:
            mid_u = (item.right-item.left)/2+item.left
            mid_v = (item.bottom-item.top)/2+item.top
            dist_cm = 100*getDistanceAngle(mid_u,mid_v,item.distance)[0]

            dist_msg = vehicle.message_factory.distance_sensor_encode(
                np.uint32(1000*uptime()),          # time since system boot in ms
                np.uint16(10),                     # min distance sensor can measure in cm
                np.uint16(1000),                   # max distance sensor can measure in cm
                np.uint16(dist_cm),                # Current distance reading in cm
                np.uint8(4),                       # Mav_distance_sensor, 4 = unknown
                np.uint8(1),                       # onboard sensor id (arbitray)
                np.uint8(0),                       # camera orientation (0 roll, 0 pitch, 0 yaw)
                np.uint8(255))                     # measurement covariance (unknown)
            #print(dist_msg)
            vehicle.send_mavlink(dist_msg)
            
            @vehicle.on_message('HWSTATUS') # if no message found it will not display but will not error
            def listener(self,name,message):
                print(message)  

        pass
    except rospy.ROSInterruptException:
        pass

def callback_Test(data): 
    global vehicle

    # Implements collision avoidance 
    try:
        # vehicle.message_factory.send(
        # vehicle.message_factory.distance_sensor_send(
        # vehicle.message_factory.collision_encode(
        # vehicle.message_factory.set_mode_encode(
        # vehicle.message_factory.rangefinder_encode(
        
        # Send distance information to pixhawk, DISTANCE_SENSOR mavlink object
        # TODO - use vehicle parameters to fill in fields
        # dist = 2000
        # dist_msg = vehicle.message_factory.distance_sensor_encode(
        #     np.uint32(1000*uptime()),          # time since system boot in ms
        #     np.uint16(10),                     # min distance sensor can measure in cm
        #     np.uint16(1000),                   # max distance sensor can measure in cm
        #     np.uint16(dist),                   # Current distance reading in cm
        #     np.uint8(4),                       # Mav_distance_sensor, 4 = unknown
        #     np.uint8(1),                       # onboard sensor id (arbitray)
        #     np.uint8(0),                       # camera orientation (0 roll, 0 pitch, 0 yaw)
        #     np.uint8(255))                     # measurement covariance (unknown)
        # vehicle.send_mavlink(dist_msg)
        
        
        ##Show MAVLINK messages (which are sent from pixhawk) in terminal - '*' shows all or type specific required
        # @vehicle.on_message('*') # if no message found it will not display but will not error
        # def listener(self,name,message):
        #     print(message)   

        pass
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":


    ## ----------------------- SETUP --------------------- [Code to run only once]
    ## Setup node (name)
    rospy.init_node('GNC', anonymous=False)

    # Connect to vehicle
    # vehicle = connect('/dev/ttyTHS1', wait_ready=True, baud=115200)
    # ṢITL
    vehicle = connect('tcp:127.0.0.1:5762', wait_ready=True) 

    # Testing 
    print('Connection successful')
    vehicle.mode = VehicleMode("GUIDED")
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)
    vehicle.armed = True
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)
    time.sleep(1)
    vehicle.simple_takeoff(15)
    time.sleep(5)
    vehicle.airspeed = 3
    point1 = LocationGlobalRelative(-35.36137565, 149.16472783, 20)
    vehicle.simple_goto(point1)

    time.sleep(30)


    # -35.36137565 149.16472783

    # TODO - load a config file of all parameters

    # do nothing
    quickFix_header = Header()
    quickFix_header.stamp = rospy.Time.now()
    String.header = quickFix_header # adds a header in String to synchronize subscribing
    Float64.header = quickFix_header

    ## ----------------------- PUBLISHERS --------------------- [Code to run only once]




    ## ----------------------- SUBSCRIBERS/CALLBACKS --------------------- [Code to run only once]

    # VEHICLE STATE
    ###############
    # ts_VehicleState = message_filters.TimeSynchronizer(message_filters.Subscriber('clock/time', Float64),10)
    # ts_VehicleState.registerCallback(callback_VehicleState)
    rospy.Subscriber("time",Float64,callback_VehicleState)
    
    # GNC TASK MANAGER
    ##################
    # ts_GNCTaskManager = message_filters.TimeSynchronizer([message_filters.Subscriber('SA', String),
    #                                        message_filters.Subscriber('TC', String)], 10)
    # ts_GNCTaskManager.registerCallback(callback_GNCTaskManager)
    rospy.Subscriber("time",Float64,callback_GNCTaskManager)

    # COLISION AVOIDANCE
    #####################
    # ts_CollisionAvoidance = message_filters.TimeSynchronizer(message_filters.Subscriber('bbox_position2', Float64),10)
    # ts_CollisionAvoidance.registerCallback(callback_CollisionAvoidance)
    rospy.Subscriber("bbox_position2", multipositions, callback_CollisionAvoidance)

    rospy.Subscriber("time",Float64, callback_Test)

    # Prevent undesired program ending
    rospy.spin()

    # Close vehicle object
    #vehicle.close()

    pass

