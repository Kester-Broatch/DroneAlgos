#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''-------------------------- Program Header ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------''
  ' Add all important details regarding the program here
  '
  ' Project             : BAE Swarm Challenge - Group Design Project
  ' Program name        : GNC.py
  ' Author              : Romain Delabeye & Kester Broatch
  ' Adm. structure      : Cranfield University
  ' Date (YYYYMMDD)     : 20200205
  ' Purpose             : provide code structure
  ' User                : A_Team
  '
  ' Revision History    : 1.0
  '
  ' Date YYYYMMDD |  Author          | Ref       |Revision comment
  '-------------------------------------------------------------------
  ' 20200205      |  Romain DELABEYE |           | Developing & Structuring
  ' 20200206      |  Kester Broatch  |           | Added GNC 
  ' 20200212      |  Romain DELABEYE |           | Merge CA with wp guidance, state, TM, etc.
  '               |                  |           | 


TODO:



''-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------'''
'''
'------------------------------------------------------------------------------------------------------------------------
'    import libraries
'------------------------------------------------------------------------------------------------------------------------
'''

## System
import os
import time
from uptime import uptime


# Math
import math
import numpy as np
from numpy import linalg as npla
from geomdl import NURBS

# ROS & piloting
import rospy
from sensor_msgs.msg import *
from std_msgs.msg import *
import message_filters
from dronekit import *
try : 
    from test.msg import Action, Agent, position_detection, multipositions, testimage
    from CA_functions import getDistanceAngle
except Exception as e:
    print("!Could not import some stuff\n",e)

# Perso
import config as cf
from CA_functions import getDistanceAngle


'''
'------------------------------------------------------------------------------------------------------------------------
'    Toolbox
'------------------------------------------------------------------------------------------------------------------------
'''

def NED2WF(north_east_down, pos_offset_Float32MultiArray = cf.POS_OFFSET):
    offset_to_origin = np.array(pos_offset_Float32MultiArray.data)
    current = np.array(north_east_down)
    newloc = Float32MultiArray()
    newloc.data = np.array(offset_to_origin-current).tolist()
    return newloc

def WF2NED(north_east_down, pos_offset_Float32MultiArray = cf.POS_OFFSET):
    offset_to_origin = np.array(pos_offset_Float32MultiArray.data)
    current = np.array(north_east_down)
    newloc = Float32MultiArray()
    newloc.data = np.array( offset_to_origin + current).tolist()
    return newloc
    
def spline(wi, wi1, wi2, deg = 2, delta = .05):
    # w_previous = np.array(w_previous)
    # wi = np.array(wi)
    # w_next = np.array(w_next)
    curve = NURBS.Curve()
    curve.degree = deg
    curve.ctrlpts = [wi, wi1, wi2]
    curve.knotvector = [0, 0, 0, 1, 1, 1]
    curve.delta = delta
    return curve.evalpts

def calc_heading_speed(Wi, Wnxt, pos, current_heading, current_velocity, K_carrot = 1., Delta_carrot = .1):
    '''
    provide the new heading & velocity:
    INPUT:
    - Wi , Wnxt , pos : horizontal waypoint positions & current location
    - current_heading , current_velocity : heading & vel
    - K_carrot : velocity tuning
    - Delta_carrot : heading tuning
    OUTPUT:
    - new_heading , new_velocity
    '''
    if len(current_velocity) >1: current_velocity = npla.norm(np.array(current_velocity[:2])) # take the norm of the velocity if given as a vector
    Ru = npla.norm(np.array(Wi[:2])-np.array(pos[:2]))
    theta = np.arctan2(Wnxt[1] - Wi[1], Wnxt[0] - Wi[0])
    thetau = np.arctan2(pos[1] - Wi[1], pos[0] - Wi[0])
    delta_theta = theta-thetau
    R = np.sqrt(Ru*Ru - (Ru*np.sin(delta_theta))*(Ru*np.sin(delta_theta)))
    xt , yt = (R+Delta_carrot)*np.cos(theta) , (R+Delta_carrot)*np.sin(theta)
    new_heading = np.arctan2(yt-pos[1], xt-pos[0])
    new_velocity = min(K_carrot*(new_heading-current_heading)*current_velocity, cf.MAX_VELOCITY)
    return new_heading, new_velocity

def has_reached_waypoint2D(wi, pos, radius = .5):
    '''
    True if waypoint reached, i.e. if distance of pos from wi is less than radius metres.
    2D ONLY (horizontal distance with projection)
    '''
    return npla.norm(np.array(wi[:2])-np.array(pos[:2])) < radius
def has_reached_waypoint3D(wi, pos, radius = .5):
    '''
    True if waypoint reached, i.e. if distance of pos from wi is less than radius metres.
    3D distance
    '''
    return npla.norm(np.array(wi)-np.array(pos)) < radius



    




'''
'------------------------------------------------------------------------------------------------------------------------
'    Main
'------------------------------------------------------------------------------------------------------------------------
'''

def callback_TaskManager(data): #TODO subcribe to TC, SA, MC
    '''
    Task Manager: 
    - enable/disable functionalities with respect to miscellaneous cases.

    note: collision avoidance triggers itself, where the other functionalities wait to receive clearance to operate.
    '''
    global vehicle, agent

    # Decides which GNC algorithm should be used, based upon TC, SA, MC
    try:
        if not agent.enable_agent:
            # Disable all functionalities at the same time (first level of safety)
            agent.clear_pause = False
            agent.clear_ca = False
            agent.clear_wp = False
            agent.release = False
        else:
            if agent.clear_pause:
                # Disable all functionalities except pause when pause ordered
                agent.clear_ca = False
                agent.clear_wp = False
                agent.release = False
            else:
                if agent.clear_ca :
                    # Disable functionalities when collision avoidance is triggered
                    agent.clear_wp = False
                    agent.release = False                
                else:
                    agent.clear_wp = True
                    agent.release = True
        

            pass
    except rospy.ROSInterruptException:
        pass


def callback_wp(data):
    '''
    Waypoint following
    TODO: make sure command is meant for this agent
    #TODO : extend "==" to "almost equal", same for "!=" : heading & vel
    # TODO: velocity returned as a vector (dir = heading; norm = as defined) in carrot chasing above
    '''
    global vehicle, agent
    try:
        if agent.clear_wp and agent.action.type == "wp":
            # check if next waypoint reached & update waypoints
            iWP = agent.action.iWP

            # start with the appropriate altitude
            if iWP == 0:
                vehicle.simple_goto() # TODO use the wp loc in the drone's frame (known as NED, not WF)

            # Carrot chasing
            new_heading , new_groundspeed = calc_heading_speed(agent.action.wp[iWP-1],agent.action.wp[iWP],agent.pos, agent.heading, agent.groundspeed)
            # Update heading & vel in this order
            if vehicle.heading != new_heading:
                vehicle.heading = new_heading
            else:
                vehicle.groundspeed = new_groundspeed

            if iWP < len(agent.action.wp)-1 and has_reached_waypoint2D(agent.action.wp[iWP], agent.pos):
                agent.action.iWP += 1
                

            # debug
            if cf.debug_GNC_wp :
                # rospy.loginfo(agent.ned_from_start)
                pass

            rate.sleep()

            pass
    except rospy.ROSInterruptException:
        pass

def callback_VehicleState(data):
    '''
    Import vehicle state from pixhawk and publishes vehicle states to ros
    '''
    global vehicle, agent

    try:
        # Agent ID
        if agent.id != cf.AGENT_ID: agent.id = cf.AGENT_ID

        # Update battery level
        agent.battery = vehicle.battery.level

        # Update location
        agent.ned_from_start = vehicle.location.local_frame  # (north, east, down) from init loc
        agent.pos = NED2WF(agent.ned_from_start)

        # Update velocity
        agent.velocity = vehicle.velocity
        agent.groundspeed = npla.norm(np.array(agent.velocity[:2]))

        # Update attitude
        agent.ned_attitude = vehicle.attitude
        agent.attitude = agent.ned_attitude
        agent.heading = vehicle.heading

        # Publish updated state
        pub_agent_GNC2SA.publish(agent)

        # debug
        if cf.debug_GNC_VehicleState :
            rospy.loginfo(agent.ned_from_start)
            rospy.loginfo(agent.ned_attitude)
            rospy.loginfo(agent.battery)

        rate.sleep()

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
    rate = rospy.Rate(10)

    # [do nothing] add headers to some classes
    quickFix_header = Header()
    quickFix_header.stamp = rospy.Time.now()
    String.header = quickFix_header # adds a header in String to synchronize subscribing
    Float64.header = quickFix_header
    Agent.header = quickFix_header

    # Connect to vehicle (choose host (SITL/laptop/jetson...) in config file)
    vehicle = connect(cf.Pixhawk_USBpath, wait_ready=True, baud=115200)
    print('Connection successful')

    # Setup mode
    vehicle.mode = VehicleMode("GUIDED")
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(.5)

    # Arm vehicle
    vehicle.armed = True
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(.5)
    time.sleep(1)

    if True:
        # simple test sequence
        time.sleep(1)
        vehicle.simple_takeoff(15)
        time.sleep(5)
        vehicle.airspeed = 3
        point1 = LocationGlobalRelative(-35.36137565, 149.16472783, 20)
        vehicle.simple_goto(point1)
        time.sleep(30)


    # Collision avoidance
    request_clear_ca = False


    # Vehicle State setup
    agent = Agent()
    agent.pos_offset = Float32MultiArray(data = cf.POS_OFFSET)

    # wp following
    iWP = 1

    

    ## ----------------------- PUBLISHERS --------------------- [Code to run only once]
    # pub agent from GNC to SA
    pub_agent_GNC2SA = rospy.Publisher('agent_GNC2SA', Agent, queue_size=10)



    ## ----------------------- SUBSCRIBERS/CALLBACKS --------------------- [Code to run only once]

    
    # GNC TASK MANAGER
    ##################
    # ts_GNCTaskManager = message_filters.TimeSynchronizer([message_filters.Subscriber('SA', String),
    #                                        message_filters.Subscriber('TC', String)], 10)
    # ts_GNCTaskManager.registerCallback(callback_GNCTaskManager)
    rospy.Subscriber("time",Float64,callback_TaskManager)


    # VEHICLE STATE
    ###############
    # ts_VehicleState = message_filters.TimeSynchronizer(message_filters.Subscriber('clock/time', Float64),10)
    # ts_VehicleState.registerCallback(callback_VehicleState)
    rospy.Subscriber("time",Float64,callback_VehicleState)


    # WAYPOINT FOLLOWING
    ###############
    # ts_VehicleState = message_filters.TimeSynchronizer(message_filters.Subscriber('clock/time', Float64),10)
    # ts_VehicleState.registerCallback(callback_VehicleState)
    rospy.Subscriber("time",Float64,callback_wp)


    # COLISION AVOIDANCE
    #####################
    # ts_CollisionAvoidance = message_filters.TimeSynchronizer(message_filters.Subscriber('bbox_position2', Float64),10)
    # ts_CollisionAvoidance.registerCallback(callback_CollisionAvoidance)
    rospy.Subscriber("bbox_position2", multipositions, callback_CollisionAvoidance)


    # TEST
    #####################
    rospy.Subscriber("time",Float64, callback_Test)




    # Prevent undesired program ending
    rospy.spin()

    # Close vehicle object
    #vehicle.close()

    pass

