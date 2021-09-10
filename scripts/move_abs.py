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


def move_to_abs(x,y,z,radius, time_out):
    goal_position = np.array([x,y])
    sub_positions = goal_position + [[radius,radius],[-radius,radius],[radius,-radius],[-radius,-radius]]
    i = 0
    while True:
        try:
            print("f-move_to : moving to goal")
            omni_base.go_abs(x, y, z, time_out)
            print("f-move_to : succsess")
            whole_body.move_to_neutral()
            break
        except hsrb_interface.exceptions.MobileBaseError: #  Failed to reach goal (障害物やtime_outのとき)
            position_now = np.array(omni_base.pose[0:2])
            if np.linalg.norm(goal_position-position_now)<=radius: # ゴールから半径radius内にいれば移動成功とする
                print("f-move_to : succsess in radius")
                whole_body.move_to_neutral()
                break
            
            try:
                # 目標地点を少しずらす（目標地点からradiusを半径とする円の外接四角形の四隅への移動を順番に試みる）
                # 目標点を動かして移動し直すことで、Failed to reach goal (障害物やtime_outのとき)でも、新たな軌道が生成され、動き直しが可能
                # time_outを数秒程度に短くすることで素早く動き直しができる（常に軌道生成するので計算コスト高い）
                print("f-move_to : moving to sub goal sub_No."+str(i+1))
                omni_base.go_abs(sub_positions[i][0], sub_positions[i][1], z, time_out)
            except:
                i += 1
                if i==4:
                    i=0
                # print("f-get_marker_position : 例外エラーの発生")
                # import traceback
                # traceback.print_exc()


# 半径内で移動成功
# 障害物とTimeOutの発生時はもう一度経路生成して、移動を継続
# 経路生成ができない時は無限ループにハマって止まる
def move_to_abs2(x,y,z,radius, time_out):
    goal_position = np.array([x,y])
    while True:
        try:
            print("f-move_to : moving to goal")
            omni_base.go_abs(x, y, z, time_out)
            print("f-move_to : succsess")
            whole_body.move_to_neutral()
            break
        except hsrb_interface.exceptions.MobileBaseError: #  Failed to reach goal (障害物やtime_outのとき)
            position_now = np.array(omni_base.pose[0:2])
            if np.linalg.norm(goal_position-position_now)<=radius: # ゴールから半径radius内にいれば移動成功とする
                print("f-move_to : succsess in radius")
                whole_body.move_to_neutral()
                break
        except:
            print("f-get_marker_position : 例外エラーの発生2")
            import traceback
            traceback.print_exc()

def move_to_abs3(x,y,z,radius, time_out):
    goal_position = np.array([x,y])
    # sub_positions = goal_position + [[radius,radius],[-radius,radius],[radius,-radius],[-radius,-radius]]
    i = 0
    while True:
        try:
            print("f-move_to : moving to goal")
            omni_base.go_abs(x, y, z, time_out)
            print("f-move_to : succsess")
            whole_body.move_to_neutral()
            break
        except hsrb_interface.exceptions.MobileBaseError: #  Failed to reach goal (障害物やtime_outのとき)
            position_now = np.array(omni_base.pose) # [x,y,z]
            if np.linalg.norm(goal_position-position_now[0:2])<=radius: # ゴールから半径radius内にいれば移動成功とする
                print("f-move_to : succsess in radius")
                whole_body.move_to_neutral()
                break
            
            try:
                # 目標地点を少しずらす（目標地点からradiusを半径とする円の外接四角形の四隅への移動を順番に試みる）
                # 目標点を動かして移動し直すことで、Failed to reach goal (障害物やtime_outのとき)でも、新たな軌道が生成され、動き直しが可能
                # time_outを数秒程度に短くすることで素早く動き直しができる（常に軌道生成するので計算コスト高い）
                print("f-move_to : moving to sub goal sub_No."+str(i+1))
                sub_positions = position_now[0:2] + [[radius,radius],[-radius,radius],[radius,-radius],[-radius,-radius]]
                omni_base.go_abs(sub_positions[i][0], sub_positions[i][1], position_now[2], time_out)
            except:
                i += 1
                if i==4:
                    i=0
                # print("f-get_marker_position : 例外エラーの発生")
                # import traceback
                # traceback.print_exc()

if __name__ == "__main__":


    # move_to_abs(0.0, 0.0, 0.0, radius=0.03, time_out=5)
    move_to_abs2(0.0, 0.0, 0.0, radius=0.03, time_out=5)
