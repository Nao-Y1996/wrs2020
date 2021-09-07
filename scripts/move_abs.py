#!/usr/bin/python
# -*- coding: utf-8 -*-

import hsrb_interface
import rospy
import sys
from hsrb_interface import geometry
import json
import os
import tf
import csv
import sqlite3
import numpy as np
import random
from rospy.timer import sleep

DBNAME = 'HSRstate.db'

# Move timeout[s]
_MOVE_TIMEOUT=60.0
# Grasp force[N]
_GRASP_FORCE=0.2
# Preparation for using the robot functions
robot = hsrb_interface.Robot()
omni_base = robot.get('omni_base')
whole_body = robot.get('whole_body')
gripper = robot.get('gripper')
tts = robot.get('default_tts')
collision_world=robot.try_get('global_collision_world')
whole_body.collision_world = collision_world


# conn = sqlite3.connect(DBNAME)
# cur = conn.cursor()

def move_to_abs(x,y,z,radius, time_out):
    goal_position = np.array([x,y])
    is_success = False
    # omni_base.go_rel(0, 0, z, 15) # ゴール方向に体を向ける
    try:
        print("move_to_abs : moving")
        omni_base.go_abs(x, y, z, time_out)
        whole_body.move_to_go()
        is_success = True
        print("move_to_abs : succsess")
    except hsrb_interface.exceptions.MobileBaseError: #  Failed to reach goal ()
        position_now = np.array(omni_base.pose[0:2])
        if np.linalg.norm(goal_position-position_now)<=radius: # ゴールから半径radius内にいれば移動成功とする
            whole_body.move_to_go()
            is_success = True
            print("move_to_abs : succsess in radius")
        # -------------移動失敗のとき--------------
        else:
            x = position_now[0] + random.uniform(0.1, 0.3)
            y = position_now[1] + random.uniform(0.1, 0.3)
            is_success = False
    # -------------移動失敗のとき--------------
    except:
        position_now = np.array(omni_base.pose[0:2])
        x = position_now[0] + random.uniform(0.1, 0.3)
        y = position_now[1] + random.uniform(0.1, 0.3)
        is_success = False

    return is_success

if __name__ == "__main__":

    
    while not rospy.is_shutdown():

        
        move_to_abs(1.0, 0.0, 0.0, radius=0.03, time_out=30)
        move_to_abs(.0, 0.0, 0.0, radius=0.03, time_out=30)

