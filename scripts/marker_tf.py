#!/usr/bin/python
# -*- coding: utf-8 -*-

import hsrb_interface
import rospy
from hsrb_interface import geometry
import json
import os
import numpy as np
import tf
import sqlite3
import sys
from std_msgs.msg import Float32MultiArray

from rospy import Time 


# DBNAME2 = "maker_position.db"

# # Move timeout[s]
# _MOVE_TIMEOUT=60.0
# # Grasp force[N]
# _GRASP_FORCE=0.2
# # Preparation for using the robot functions
# robot = hsrb_interface.Robot()
# omni_base = robot.get("omni_base")
# whole_body = robot.get("whole_body")
# gripper = robot.get("gripper")
# tts = robot.get("default_tts")
# collision_world=robot.try_get("global_collision_world")
# whole_body.collision_world = collision_world

rospy.init_node('marker_position_pub', anonymous=True)

listener = tf.TransformListener()

# pub = rospy.Publisher('ARmarker_position', Float32MultiArray, queue_size=10)
br = tf.TransformBroadcaster()
rate = rospy.Rate(10)

    


if __name__ == "__main__":

    markerNames = ['ar_marker/14','ar_marker/710','ar_marker/711','ar_marker/712']
    # tfs_length = 1

    # markers_translations = np.array([[0.0]*3]  * len(markerNames))
    stable_markers_translations = np.array([[0.0]*3] * len(markerNames))

    # markers_rotations = np.array([[0.0]*4] * len(markerNames))
    stable_markers_rotations = np.array([[0.0]*4] * len(markerNames))
    for j in range(len(markerNames)):
        print(j)
        stable_markers_rotations[j][3]=1.0

    while not rospy.is_shutdown():
        print('-----------------------------------------------------------')
        for i, marker in enumerate(markerNames):
            try:
                (trans,rot) = listener.lookupTransform("/map", marker, rospy.Time(0))
                stable_markers_translations[i] = trans
                stable_markers_rotations[i] = rot
                print(marker + ' exists.')
            except:
                # print(marker + ' does not exist.')
                try:
                    (trans,rot) = listener.lookupTransform("/map", 'stable_'+marker, rospy.Time(0))
                    stable_markers_translations[i] = trans
                    stable_markers_rotations[i] = rot
                    print('stable_'+marker + ' exists.')
                except :
                    # stable_markers_rotations[i][3]=1.0
                    # import traceback
                    # traceback.print_exc()
                    pass

        # map基準でTFをbroadcast
        for i, marker in enumerate(markerNames):
            #-------------------------↓　ありえない姿勢？(rotation)になるとbroadcastが止まってしまうという問題がある　↓-------------------------------
            # br.sendTransform(stable_markers_translations[i], stable_markers_rotations[i], Time.now(), 'stable_'+marker, '/map') 
            # br.sendTransform(stable_markers_translations[i], [0.46,-0.528, 0.53, -0.47], Time.now(), 'stable_'+marker, '/map')

            #-----------------------------------------------------------------------------------------------------------------------------
            try :
                # br.sendTransform(stable_markers_translations[i], [0.46,-0.528, 0.53, -0.47], Time.now(), 'stable_'+marker, '/map')
                br.sendTransform(stable_markers_translations[i],stable_markers_rotations[i], Time.now(), 'stable_'+marker, '/map')
            except tf.DENORMALIZED_QUATERNION:
                print('=='*20)
                pass
        rate.sleep()
        
            
