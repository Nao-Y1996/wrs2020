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
    sub_positions = goal_position + [[radius,radius],[-radius,radius],[radius,-radius],[-radius,-radius]]
    i = 0
    while True:
        # omni_base.go_rel(0, 0, z, 15) # ゴール方向に体を向ける
        try:
            print('f:move_to : 移動します')
            omni_base.go_abs(x, y, z, time_out)
            print('f:move_to : 成功')
            whole_body.move_to_neutral()
            break
        except hsrb_interface.exceptions.MobileBaseError: #  Failed to reach goal ()
            position_now = np.array(omni_base.pose[0:2])
            if np.linalg.norm(goal_position-position_now)<=radius: # ゴールから半径radius内にいれば移動成功とする
                print('f:move_to : 半径内で成功')
                whole_body.move_to_neutral()
                break
            # -------------移動失敗のとき--------------
            # radiusを半径とする円の外接四角形の四隅への移動を順番に試みる
            try:
                print('f:move_to : 失敗　予備位置へ移動して再度tryします  -->  予備位置ナンバー = ',i)
                omni_base.go_abs(sub_positions[i][0], sub_positions[i][1], z, 10.0)
            except:
                i += 1
                if i==4:
                    i=0

if __name__ == "__main__":

    
    while not rospy.is_shutdown():

        
        move_to_abs(1.0, 0.0, 0.0, radius=0.03, time_out=30)
        move_to_abs(.0, 0.0, 0.0, radius=0.03, time_out=30)

