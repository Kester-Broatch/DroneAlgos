#!/usr/bin/env python3
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
'
'     Import libraries
'    
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
import navpy

# ROS & piloting
import rospy
from sensor_msgs.msg import *
from std_msgs.msg import *
import message_filters
from dronekit import *
try : 
    from test.msg import Action, Agent, position_detection, multipositions, testimage
except Exception as e:
    print("!Could not import some stuff\n",e)

# Perso
import config as cf
from CA_functions import *


# os.system('source ~/Desktop/SWARM/devel/setup.bash')


'''
'------------------------------------------------------------------------------------------------------------------------
'
'    Toolbox
'
'------------------------------------------------------------------------------------------------------------------------
'''

################################################################################
### Math
################################################################################

def LLA2NED(global_frame_Float32MultiArray, ned_origin = cf.WF_ORIGIN_LLA):
    global_frame = global_frame_Float32MultiArray.data
    newloc = Float32MultiArray()
    newloc.data = navpy.lla2ned(global_frame[0],global_frame[1],global_frame[2],
        ned_origin[0],ned_origin[1],ned_origin[2])
    return newloc

def NED2LLA(ned_frame_Float32MultiArray, ned_origin = cf.WF_ORIGIN_LLA):
    newloc = Float32MultiArray()
    newloc.data = navpy.ned2lla(ned_frame_Float32MultiArray.data, ned_origin[0],ned_origin[1],ned_origin[2])
    return newloc

def NED2WF(ned_frame_Float32MultiArray, wf_heading = cf.WF_HEADING):
    dcm = navpy.angle2dcm(wf_heading,0,0)
    newloc = Float32MultiArray()
    newloc.data = np.matmul(ned_frame_Float32MultiArray.data,dcm)
    return newloc

def WF2NED(wf_frame_Float32MultiArray, wf_heading = cf.WF_HEADING):
    dcm = navpy.angle2dcm(-wf_heading,0,0)
    newloc = Float32MultiArray()
    newloc.data = np.matmul(wf_frame_Float32MultiArray.data,dcm)
    return newloc

def LLA2WF(global_frame_Float32MultiArray):
    newloc = Float32MultiArray()
    newloc = NED2WF(LLA2NED(global_frame_Float32MultiArray))
    return newloc

def WF2LLA(wf_frame_Float32MultiArray):
    newloc = Float32MultiArray()
    newloc = NED2LLA(WF2NED(wf_frame_Float32MultiArray))
    return newloc
    
def spline(wi, wi1, wi2, deg = 2, delta = .05):
    curve = NURBS.Curve()
    curve.degree = deg
    curve.ctrlpts = [wi, wi1, wi2]
    curve.knotvector = [0, 0, 0, 1, 1, 1]
    curve.delta = delta
    return curve.evalpts

def unit_vector(vector):
    """ Returns the unit vector of the vector.  """
    return np.array(vector) / npla.norm(np.array(vector))

def angle_between(v1, v2):
    """ Returns the angle in radians between vectors 'v1' and 'v2' """
    return np.arccos(np.clip(np.dot(unit_vector(np.array(v1)), unit_vector(np.array(v2))), -1.0, 1.0))

def isEqual(u,v, eps_scalar = 1e-2, eps_angle = 5.):
    '''
    returns whether u and v are **ALMOST** equal.
    - eps: if u,v are scalars, then eps is the absolute difference
    - eps_angle: IN DEGREES, max angle between vectors (n-dim, n>1) for them to be equal.
    '''
    try:
        return abs( angle_between(u,v) ) < eps_angle *np.pi/180.
    except:
        return abs( u-v ) < eps_scalar
    return

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

def convert_velocity(heading, groundspeed):
    return groundspeed*np.cos(heading), groundspeed*np.sin(heading), 0.

################################################################################
### Drone functions & related
################################################################################

def change_altitude(new_alt):
    global vehicle
    chg_alt = LocationGlobalRelative( vehicle.location.global_relative_frame.lat , vehicle.location.global_relative_frame.lon ,new_alt)
    vehicle.simple_goto(chg_alt)
    return

def change_yaw(heading, relative=False):
    global vehicle
    if relative:
        is_relative=1 #yaw relative to direction of travel
    else:
        is_relative=0 #yaw is an absolute angle
    # create the CONDITION_YAW command using command_long_encode()
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
        0, #confirmation
        heading,    # param 1, yaw in degrees
        0,          # param 2, yaw speed deg/s
        1,          # param 3, direction -1 ccw, 1 cw
        is_relative, # param 4, relative offset 1, absolute angle 0
        0, 0, 0)    # param 5 ~ 7 not used
    # send command to vehicle
    vehicle.send_mavlink(msg)
    return

def change_ned_velocity(velocity_x, velocity_y, velocity_z):
    """
    Move vehicle in direction based on specified velocity vectors.
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, 0, 0, # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
    vehicle.send_mavlink(msg)
    return

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
################################################################################
### handle data types
################################################################################

def encode_Float32MultiArray(M_List):
    mat = Float32MultiArray()
    mat.layout.dim.append(MultiArrayDimension())
    mat.layout.dim.append(MultiArrayDimension())
    mat.layout.dim[0].label = "height"
    mat.layout.dim[1].label = "width"
    mat.layout.dim[0].size = len(M_List)
    mat.layout.dim[1].size = len(M_List[0])
    mat.layout.dim[0].stride = len(M_List)*len(M_List[0])
    mat.layout.dim[1].stride = len(M_List)
    mat.data = [0]*len(M_List)*len(M_List[0])
    # save a few dimensions:
    dstride1 = mat.layout.dim[1].stride
    for i in range(len(M_List)):
        for j in range(len(M_List[0])):
            mat.data[i + dstride1*j] = M_List[i][j]
    return mat

def decode_Float32MultiArray(M_Float32MultiArray):
    M_List = []
    dim0  = M_Float32MultiArray.layout.dim[0].size
    dim1 = M_Float32MultiArray.layout.dim[1].size
    M_List = [[0 for i in range(dim1)] for j in range(dim0)]
    dstride1 = M_Float32MultiArray.layout.dim[1].stride
    for i in range(dim0):
        for j in range(dim1):
            M_List[i][j] = M_Float32MultiArray.data[i + dstride1*j]
    return M_List

'''
'------------------------------------------------------------------------------------------------------------------------
'
'    Callback
'
'------------------------------------------------------------------------------------------------------------------------
'''

def callback_TaskManager(data):
    '''
    Task Manager: 
    - enable/disable functionalities with respect to miscellaneous cases.

    note: collision avoidance triggers itself, where the other functionalities wait to receive clearance to operate.
    '''
    global vehicle, agent

    # Decides which GNC algorithm should be used, based upon TC, SA, MC
    try:
        if agent.enable_agent is not None and not agent.enable_agent.data:
            # Disable all functionalities at the same time (first level of safety)
            agent.clear_ca = Bool(data=False)
            agent.clear_wp = Bool(data=False)
            agent.clear_release = Bool(data=False)
        else:
            if agent.clear_ca :
                # Disable functionalities when collision avoidance is triggered
                agent.clear_wp = Bool(data=False)
                agent.clear_release = Bool(data=False)                
            else:
                agent.clear_wp = Bool(data=True)
                agent.clear_release = Bool(data=True)
        pass
    except rospy.ROSInterruptException:
        pass

def callback_VehicleState(data_time):
    '''
    Import vehicle state from pixhawk and publishes vehicle states to ros
    TODO: check if groundspeed is read as it should be
    '''
    global vehicle, agent

    try:
        # Agent ID
        if agent.id != cf.AGENT_ID: agent.id = Int8(data=cf.AGENT_ID)

        # Update battery level
        agent.battery = Float32(data=vehicle.battery.level)

        # Update location
        n_e_d = vehicle.location.local_frame
        agent.ned_from_start = Float32MultiArray(data=[n_e_d.north, n_e_d.east, n_e_d.down ]) # (north, east, down) from init loc
        agent.pos = NED2WF(agent.ned_from_start)
        agent.alt = Float32(data=vehicle.location.global_relative_frame.alt)

        # Update attitude
        agent.ned_attitude = Float32MultiArray(data=[vehicle.attitude.pitch, vehicle.attitude.yaw, vehicle.attitude.roll])
        agent.attitude = agent.ned_attitude
        agent.heading = Float32(data=vehicle.attitude.yaw) # heading in rad

        # Update velocity
        agent.velocity = Float32MultiArray(data=vehicle.velocity)
        agent.groundspeed = Float32(vehicle.groundspeed) # npla.norm(np.array(agent.velocity[:2]))

        # Publish updated state
        pub_agent_GNC2SA.publish(agent)

        # debug
        if cf.debug_GNC_VehicleState :
            print("\n\nDebug VehicleState :")
            print("id : \t"), rospy.loginfo(agent.id.data)
            print("battery : \t"), rospy.loginfo(agent.battery.data)
            print("ned_from_start start : "), rospy.loginfo(agent.ned_from_start.data)
            print("pos : \t"), rospy.loginfo(agent.pos.data)
            print("alt : \t"), rospy.loginfo(agent.alt.data)
            print("ned_attitude : \t"), rospy.loginfo(agent.ned_attitude.data)
            print("heading : \t"), rospy.loginfo(agent.heading.data)
            print("velocity : \t"), rospy.loginfo(agent.velocity.data)
            print("groundspeed : \t"), rospy.loginfo(agent.groundspeed.data)

        rate.sleep()

        pass
    except rospy.ROSInterruptException:
        pass

def callback_VehicleState_update_MP(agent_MP2GNC):
    '''
    Import mission-related parameters (mission_state, actions, etc.)
    '''
    global agent
    try:
        # Update mission state
        # agent.mission_state = agent_MP2GNC.mission_state

        # Update action
        agent.action = agent_MP2GNC.action

        

        # Update pos_est (estimated position from SA via MP)
        # agent.pos_est = agent_MP2GNC.pos_est

        if cf.debug_GNC_VehicleState :
            print("wp (encoded) : \t"), rospy.loginfo(agent.action)
            print( "wp :", decode_Float32MultiArray( agent_MP2GNC.action.wp) )

        rate.sleep()
        pass
    except rospy.ROSInterruptException:
        pass


def callback_wp(data):
    '''
    Waypoint following
    TODO: make sure command is meant for this agent
    #TODO : extend "==" to "almost equal", same for "!=" : heading & vel
    '''
    global vehicle, agent
    try:
        if agent.clear_wp and agent.action.cmd == "wp":
            # check if next waypoint reached & update waypoints
            iWP = agent.action.iWP

            # reach Wi+1's altitude first
            if not isEqual( agent.alt , decode_Float32MultiArray(agent.action.wp)[iWP][2], eps_scalar=2e-1):
                change_altitude(abs(decode_Float32MultiArray(agent.action.wp)[iWP][2])) ######################################### careful about altitude NED & LocalGlobalWhatev
            # move towards Wi+1 then, horizontally
            else:
                # Carrot chasing
                new_heading , new_groundspeed = calc_heading_speed(decode_Float32MultiArray(agent.action.wp)[iWP-1],decode_Float32MultiArray(agent.action.wp)[iWP],agent.pos, agent.heading, agent.groundspeed)
                # Update heading & vel, in this order
                if not isEqual( vehicle.heading , new_heading , eps_scalar=5.*np.pi/180. ):
                    change_yaw(new_heading)
                else:
                    vehicle.groundspeed = new_groundspeed

            if iWP < len(decode_Float32MultiArray(agent.action.wp))-1 and has_reached_waypoint3D(decode_Float32MultiArray(agent.action.wp)[iWP], agent.ned_from_start):
                agent.action.iWP += 1
                
            # debug
            if cf.debug_GNC_wp :
                # rospy.loginfo(new_heading)
                # rospy.loginfo(new_groundspeed)
                pass

            rate.sleep()
            pass
    except rospy.ROSInterruptException:
        pass

def callback_CollisionAvoidance(data): 
    global vehicle

    # Implements collision avoidance 
    try:
        # collect SA info from multiple targets
        # for item in data.lists:
        #     mid_u = (item.right-item.left)/2+item.left
        #     mid_v = (item.bottom-item.top)/2+item.top
        #     dist_cm, vert_ang, horz_ang = getDistanceAngle(mid_u,mid_v,item.distance)[0]
        
        # TODO make this for multiple objects
        dist = 2
        vert_ang = 0.1
        horz_ang = 0.1

        



        # TODO Check boundaries and ground 

        # Attractive potential 



        

            
        pass
    except rospy.ROSInterruptException:
        pass



def callback_Test(data): 
    global vehicle

    try:


        pass
    except rospy.ROSInterruptException:
        pass


'''
'------------------------------------------------------------------------------------------------------------------------
'
'    Main
'
'------------------------------------------------------------------------------------------------------------------------
'''


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
    Action.header = quickFix_header

    # Connect to vehicle (choose host (SITL/laptop/jetson...) in config file)
    vehicle = connect(cf.Pixhawk_USBpath, wait_ready=True, baud=115200)
    print('Connection successful')

    # Setup mode
    vehicle.mode = VehicleMode("GUIDED")
    print(" Waiting for vehicle to initialise...")
    while not vehicle.is_armable:
        time.sleep(.5)
    print(" Initialised! ")

    # Arm vehicle
    vehicle.armed = True
    print(" Waiting for arming...")
    while not vehicle.armed:
        time.sleep(.5)
    time.sleep(1)
    print(" Armed! ")

    if True:
        # simple test sequence
        vehicle.simple_takeoff(15)
        time.sleep(5)
        # vehicle.airspeed = 3
        # point1 = LocationGlobalRelative(-35.36137565, 149.16472783, 20)
        # vehicle.simple_goto(point1)
        # time.sleep(30)


    # Collision avoidance
    request_clear_ca = False


    # Vehicle State setup
    agent = Agent()

    # agent.id = Int8()
    # agent.battery = Float32()
    # agent.pos_offset = Float32MultiArray()
    # agent.yaw0 = Float32()
    # agent.ned_from_start = Float32MultiArray() # (north, east, down) from init loc
    # agent.pos = Float32MultiArray()
    # agent.alt = Float32()
    # agent.ned_attitude = Float32MultiArray()
    # agent.attitude = Float32MultiArray()
    # agent.heading = Float32()
    # agent.velocity = Float32MultiArray()
    # agent.groundspeed = Float32()
    # agent.enable_agent = Bool()
    # agent.enable_ca = Bool()
    # agent.enable_wp = Bool()
    # agent.enable_release = Bool()
    # agent.enable_RTH = Bool()
    # agent.enable_LAND = Bool()
    # agent.enable_manual = Bool()
    # agent.clear_ca = Bool()
    # agent.clear_wp = Bool()
    # agent.clear_release = Bool()
    # agent.action = Action()
    # agent.action.cmd = String()
    # agent.action.wp = Float32MultiArray()
    # agent.action.iWP = Int32()
    # agent.action.request_release = Bool()
    # agent.action.state_release = Bool()

    # agent.pos_offset = Float32MultiArray(data = cf.POS_OFFSET)
    agent.yaw0 = Float32(data=vehicle.attitude.yaw)

    agent.clear_wp = True


    # wp following
    iWP = 0

    

    ## ----------------------- PUBLISHERS --------------------- [Code to run only once]
    # pub agent from GNC to SA
    pub_agent_GNC2SA = rospy.Publisher('agent_GNC2SA', Agent, queue_size=10)



    ## ----------------------- SUBSCRIBERS/CALLBACKS --------------------- [Code to run only once]

    
    # GNC TASK MANAGER
    ##################
    # ts_GNCTaskManager = message_filters.TimeSynchronizer([message_filters.Subscriber('SA', String),
    #                                        message_filters.Subscriber('TC', String)], 10)
    # ts_GNCTaskManager.registerCallback(callback_GNCTaskManager)
    # rospy.Subscriber("time",Float64,callback_TaskManager)


    # VEHICLE STATE _ import drone parameters into ROS network
    ###############
    # ts_VehicleState = message_filters.TimeSynchronizer(message_filters.Subscriber('clock/time', Float64),10)
    # ts_VehicleState.registerCallback(callback_VehicleState)
    rospy.Subscriber("time",Float64,callback_VehicleState)

    # VEHICLE STATE _ import mission/action from MP
    #####################
    rospy.Subscriber("agent_MP2GNC",Agent , callback_VehicleState_update_MP)


    # WAYPOINT FOLLOWING
    ###############
    # ts_VehicleState = message_filters.TimeSynchronizer(message_filters.Subscriber('clock/time', Float64),10)
    # ts_VehicleState.registerCallback(callback_VehicleState)
    rospy.Subscriber("time",Float64,callback_wp)


    # COLISION AVOIDANCE
    #####################
    # ts_CollisionAvoidance = message_filters.TimeSynchronizer(message_filters.Subscriber('bbox_position2', Float64),10)
    # ts_CollisionAvoidance.registerCallback(callback_CollisionAvoidance)
    # rospy.Subscriber("bbox_position2", multipositions, callback_CollisionAvoidance)
    # rospy.Subscriber("time",Float64,callback_CollisionAvoidance)


    # TEST
    #####################
    # rospy.Subscriber("time",Float64, callback_Test)




    # Prevent undesired program ending
    rospy.spin()

    # Close vehicle object
    #vehicle.close()

    pass

