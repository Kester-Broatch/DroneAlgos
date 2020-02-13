#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import numpy as np


# def fun(x,y,z,d,a1,a2):
#     pass

def getDistanceAngle(pixel_x, pixel_y, real_z):
    camera_fx = 383.599
    camera_fy = 383.599
    camera_cx = 320.583
    camera_cy = 238.327

    z = np.float(real_z)
    x = (pixel_x-camera_cx)*z/camera_fx
    y = (pixel_y-camera_cy)*z/camera_fy

    horizon_angle = math.atan2(x,z)
    vertical_angle = math.atan2(y,z)
    absolute_distance = math.sqrt(x*x+y*y+z*z)
    # print("x: ",x)
    # print("y: ",y)
    # print("z: ",z)
    # print("absolut_distance: ",absolute_distance)
    # print("horizon_angle: ",horizon_angle)
    # print("vertical_angle: ",vertical_angle)

    return absolute_distance, vertical_angle, horizon_angle
    #pass
