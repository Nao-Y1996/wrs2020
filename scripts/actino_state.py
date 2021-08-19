#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
# Copyright (C) 2016 Toyota Motor Corporation
import actionlib
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, PoseStamped, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import rospy
import tf.transformations
import math
import tf
from geometry_msgs.msg import Vector3


""" cli.get_state() や GoalStatus.ACTIVE　で帰ってくる数字の意味
PENDING         = 0   # そのゴールはアクションサーバーによってまだ処理されていません。
ACTIVE          = 1   # ゴールは現在アクションサーバーによって処理されています。
PREEMPTED       = 2   # ゴールが実行を開始した後にキャンセル要求を受けたため,その後、実行を完了した（ターミナル状態）。
SUCCEEDED       = 3   # ゴールはアクションサーバーによって正常に達成された(ターミナル状態)
ABORTED         = 4   # Tアクションサーバーの実行中に何らかの障害でゴールが中止された(ターミナル状態)
REJECTED        = 5   # ゴールが達成できないか無効であるため、アクションサーバーがゴールを処理せずに拒否した(Terminal State)
PREEMPTING      = 6   # ゴールが実行開始後にキャンセル要求を受け、まだ実行が完了していない。
RECALLING       = 7   # ゴールが実行開始前にキャンセル要求を受け取ったが、アクションサーバーはまだゴールがキャンセルされたことを確認していない。
RECALLED        = 8   #  ゴールが実行開始前にキャンセル要求を受け取り、キャンセルに成功した(ターミナル状態)
LOST            = 9   #  アクションクライアントはゴールがLOSTであることを判断できる。これは、アクションサーブによって無線で送信されるべきではありません。
"""
    

def feedback_cb(feedback):
    """feedbackのcallback関数"""
    # cli.wait_for_result()で待っている間実行される
    # rospy.sleep(0.5)
    # print(feedback)
    position = feedback.base_position.pose.position
    orientation = feedback.base_position.pose.orientation
    x, y = position.x, position.y 
    e = tf.transformations.euler_from_quaternion((orientation.x, orientation.y, orientation.z, orientation.w))
    print(np.round([x, y, math.degrees(e[2])], 3))


def createGoal(goal_instance, pose_instance, x, y, yaw, frame):
    pose_instance.header.stamp = rospy.Time.now()
    pose_instance.header.frame_id = frame
    pose_instance.pose.position = Point(x, y, 0)
    quat = tf.transformations.quaternion_from_euler(0, 0, yaw)
    pose_instance.pose.orientation = Quaternion(*quat)
    goal_instance.target_pose = pose_instance

    return goal_instance
if __name__=="__main__":

    # 位置合わせたあとの、マップの初期位置
    # 0.07865259055059612, -0.5795284561852135, 0.04843300419215437
    # １、２
    # -1.208820077816414, 3.3593553113225045, -2.2326029218111536
    # ３、５
    # 1.0364684580131838, 3.7364411780851223, 0.7068780818689953
    # （レジの前）
    # 1.5782184826733499, 1.6871776823854516, -0.9366680768555469
    # ４
    # -0.6026532252761044, 0.3327925179753355, 3.0990727019566204


    rospy.init_node('test')

    # initialize action client
    cli = actionlib.SimpleActionClient('/move_base/move', MoveBaseAction)
    # wait for the action server to establish connection
    cli.wait_for_server()

    new_pose = PoseStamped()
    goal = MoveBaseGoal()
    goal = createGoal(goal, new_pose, 0.0, 0.0, 0.0,"map")
    cli.send_goal(goal, feedback_cb = feedback_cb)

    while cli.get_state() == GoalStatus.ACTIVE:

        action_state = cli.get_state()
        print(action_state)
        # print(GoalStatus.SUCCEEDED)
    # goal_status = _GOALSTATUS[GoalStatus.SUCCEEDED]
    

    # goal = createGoal(goal, new_pose, 0.0, 0.0, 0.0,"map")
    # cli.send_goal(goal, feedback_cb = feedback_cb)


    # while not rospy.is_shutdown():

        # goal = createGoal(goal, new_pose, -1.208820077816414, 3.3593553113225045, -2.2326029218111536,"map")
        # cli.send_goal(goal, feedback_cb = feedback_cb)
        # action_state = cli.get_state()
        # print(action_state)
        # cli.wait_for_result()# 自律移動が終了するまで待つ

        # rospy.sleep(1)

        # goal = createGoal(goal, new_pose, 1.0364684580131838, 3.7364411780851223, 0.7068780818689953,"map")
        # cli.send_goal(goal, feedback_cb = feedback_cb)
        # cli.wait_for_result()
        # rospy.sleep(1)

        # goal = createGoal(goal, new_pose, -0.6026532252761044, 0.3327925179753355, 3.0990727019566204,"map")
        # cli.send_goal(goal, feedback_cb = feedback_cb)
        # cli.wait_for_result()
        # rospy.sleep(1)

