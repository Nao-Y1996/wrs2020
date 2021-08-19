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

class PickAndPlace():
    def __init__(self, conf):
        self.conf = conf

    def grasp(self, target):
        setting = self.conf[target]

        def hand_control(key, ref):
            x, y, z, ei, ej, ek = setting[key]
            goal = geometry.pose(x, y, z, ei, ej, ek)
            whole_body.move_end_effector_pose(goal, ref)

        if target != 'poster':
            # open hand
            gripper.command(1.2)
            # Move the hand to front of the target
            hand_control("target_to_hand", setting["target_position"])
            # Specify the force to grasp
            gripper.apply_force(setting["GRASP_FORCE"])
            # Move the hand up on end effector coordinate
            hand_control("hand_up", 'hand_palm_link')
            # Move the hand back on end effector coordinate
            hand_control("hand_back", 'hand_palm_link')
            # Transit to initial posture
            whole_body.move_to_neutral()

        if target == 'poster':
            # method for grasp object like bowl from above
            # open hand
            gripper.command(1.2)
            # Move the hand to front of the target
            hand_control("target_to_hand", target)
            # down the hand to to grasp
            hand_control("hand_down_to_grasp", 'hand_palm_link')
            # Specify the force to grasp
            gripper.apply_force(_GRASP_FORCE)
            # Move the hand up on end effector coordinate
            hand_control("hand_up", 'hand_palm_link')

            omni_base.go_rel(-0.3, 0.0, 0.0, 100.0)
            # whole_body.move_to_joint_positions({'arm_lift_joint': 0.0})

    def place(self, target, ar_marker):
        setting = self.conf[target]

        def hand_control(key, ref):
            x, y, z, ei, ej, ek = setting[key]
            goal = geometry.pose(x, y, z, ei, ej, ek)
            whole_body.move_end_effector_pose(goal, ref)

        if setting["grasp_type"] == 'front':
            # Move the hand to place position
            hand_control("tf_place", ar_marker)
            # Move the hand down on end effector coordinate
            hand_control("hand_down",  'hand_palm_link')
            # open hand
            gripper.command(1.2)
            # Move the hand up on end effector coordinate
            hand_control("hand_up_after_place", 'hand_palm_link')

            omni_base.go_rel(-0.5, 0.0, 0.0, 100.0)

            # Transit to initial posture
            whole_body.move_to_neutral()
        
        if setting["grasp_type"] == 'side':
            # method for place object like bowl
            # Move the hand to place position
            hand_control("tf_place", ar_marker)
            # Move the hand down on end effector coordinate
            hand_control("hand_down_to_place",  'hand_palm_link')
            # open hand
            gripper.command(1.2)
            # Move the hand up on end effector coordinate
            hand_control("hand_up_after_place", 'hand_palm_link')
            # Move the hand back on end effector coordinate
            hand_control("hand_back",  'hand_palm_link')
            # Transit to initial posture
            whole_body.move_to_neutral()

if __name__ == "__main__":


    # read json
    dir = os.path.dirname(__file__)
    f = open(dir + '/object_params.json', 'r')
    grasp_conf = json.load(f)

    pick_and_place = PickAndPlace(grasp_conf)
    while not rospy.is_shutdown():

        # 現在の移動(位置)状態を取得
        # position_state = rospy.get_param("/position_state")
        # with open('state.csv') as f:
        #     reader = csv.reader(f)
        #     l = [row for row in reader]
        # position_state = int(l[0][0])
        position_state = 0

        conn = sqlite3.connect(DBNAME)
        cur = conn.cursor()
        cur.execute('SELECT * FROM state_manager')
        print(cur.fetchall())
        cur.close()
        conn.close()


        # 現在の位置が、ポスター回収地点のとき
        if position_state == 1:
            target = 'Bottle'
            whole_body.move_to_neutral()
            pick_and_place.grasp(target)
            whole_body.move_to_neutral()


        
        # print(position_state)
        rospy.sleep(0.5)
        # get target name
        # target = 'Bottle'
        # target = "CaffeLatte"
        # target = "Protein"
        # target = "RiceBall"
        # target = "Sandwich"
        # target = "VegetableStick"
        

        # listener = tf.TransformListener()
        # now = rospy.Time.now()
        # (trans,rot) = listener.lookupTransform(grasp_conf[target]["base_position"], "odom", now)

        # collision_world.remove_all()
        # table = collision_world.add_box(x=0.7, y=0.5, z=0.69, pose=geometry.pose(x=-0.35, y=0.20, z=0.2), frame_id='ar_marker/711')
        # whole_body.collision_world = collision_world
        # omni_base.go_abs(0, 0, 0, 300.0)
        # whole_body.move_to_neutral()
        # omni_base.go_abs(1.4, 0.2, 1.54, 300.0)
        # whole_body.move_to_joint_positions({'arm_lift_joint': 0.2})
        
        
        # rospy.sleep(5.0)
        

        # grasp
        # pick_and_place.grasp(target)
        # rospy.sleep(3.0)

        # omni_base.go_abs(0, 0, 0, 300.0)
        # whole_body.move_to_neutral()

        # where to place (select teble)
        # ar_marker = "ar_marker/712"

        # Look at the table
        # if ar_marker == 'ar_marker/712':
        #     omni_base.go_abs(1.5, 0.1, -1.57, 300.0)
        
        # # place
        # num = 0.1
        # while True:
        #     try:
        #         pick_and_place.place(target, ar_marker)
        #         break
        #     except:
        #         print('置くのに失敗しました')
        #         omni_base.go_rel(num, 0, 0, 300.0)
        #         whole_body.move_to_neutral()
        #         rospy.sleep(2)
        #         num = -num

        # omni_base.go_abs(0, 0, 0, 300.0)
        # whole_body.move_to_neutral()




    """
    移動
    定位位置に移動してくる

    ARマーカーが見えるかどうか確認する
    ・見えないとき
        位置を調整する
    ・見えるとき
        ロボットの位置にTF1を生成
        TF1を参照にして対象のTFを固定してTF_targetを生成して固定
        
    掴み方
    対象の手前にグリッパーを持ってくる
    対象の位置にグリッパーを持ってくる

    """