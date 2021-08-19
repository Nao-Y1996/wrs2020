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

from rospy.timer import sleep
from playsound import playsound


# Preparation for using the robot functions
robot = hsrb_interface.Robot()
omni_base = robot.get("omni_base")
whole_body = robot.get("whole_body")
gripper = robot.get("gripper")
tts = robot.get("default_tts")
collision_world=robot.try_get("global_collision_world")
whole_body.collision_world = collision_world

listener = tf.TransformListener()


def move_to_abs(x,y,z,radius, time_out):
    goal_position = np.array([x,y])
    sub_positions = goal_position + [[radius,radius],[-radius,radius],[radius,-radius],[-radius,-radius]]
    i = 0
    while True:
        print(i)
        # omni_base.go_rel(0, 0, z, 15) # ゴール方向に体を向ける
        try:
            print("f-move_to : moving")
            omni_base.go_abs(x, y, z, time_out)
            print("f-move_to : succsess")
            whole_body.move_to_neutral()
            break
        except hsrb_interface.exceptions.MobileBaseError: #  Failed to reach goal ()
            position_now = np.array(omni_base.pose[0:2])
            if np.linalg.norm(goal_position-position_now)<=radius: # ゴールから半径radius内にいれば移動成功とする
                print("f-move_to : succsess in radius")
                whole_body.move_to_neutral()
                break
            # -------------移動失敗のとき--------------
            # radiusを半径とする円の外接四角形の四隅への移動を順番に試みる
            try:
                print("f-move_to : failed  moving to sub position and try again  -->  sub position No."+str(i))
                omni_base.go_abs(sub_positions[i][0], sub_positions[i][1], z, 15.0)
                print(i,i,i)
            except:
                i += 1
                if i==4:
                    i=0
                print('-----------------')
                print(i)
                print('-----------------')
                print("f-get_marker_position : 例外エラーの発生")
                import traceback
                traceback.print_exc()

def get_marker_position(markerName):
    num_trial = 4
    for i in range(num_trial):
        try:
            (trans,rot) = listener.lookupTransform(markerName, "/map", rospy.Time(0))
            print("マーカーを視認しました。")
            return (markerName, trans,rot)
            # マーカー位置を基準に位置補正
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            if i < num_trial-1:
                print("マーカーが見えません。姿勢を変えて再度tryします。")
                # position_now = np.array(omni_base.pose)
                x = -1*0.1 if i%2==0 else 0.1
                omni_base.go_rel(x, 0, 0, 10.0)
                whole_body.move_to_neutral()
                rospy.sleep(10.0)
            else:
                # markerName = 'stable_'+markerName
                print("姿勢を変えてもマーカーが見えません. map基準のマーカーを利用します")
                try:
                    (trans,rot) = listener.lookupTransform('stable_'+markerName, "/map", rospy.Time(0))
                    return ('stable_'+markerName, trans, rot)
                except:
                    print("map基準のマーカーが利用できません")
        except:
            print("f-get_marker_position : 例外エラーの発生")
            import traceback
            traceback.print_exc()
    print("どうしてもマーカーが見えません.")
    


if __name__ == "__main__":

    whole_body.move_to_go()
    # playsound("/home/kubotalab-hsr/catkin_ws/src/wrs2020/scripts/WRS2020.mp3")

    # 初期位置
    whole_body.move_to_neutral()
    # move_to_abs(0.07865259055059612, -0.5795284561852135, 0.04843300419215437, radius=0.1, time_out=30)

    # # --------------------ポスターを掴みに行く--------------------
    whole_body.move_to_go()
    move_to_abs(-1.208820077816414, 3.3593553113225045, -2.2326029218111536, radius=0.1, time_out=30)
    # ar_marker, target = "ar_marker/708", "Poster"
    # print("ターゲット："+target)
    whole_body.move_to_neutral()
    # rospy.sleep(10.0)
    # ar_marker,trans,rot = get_marker_position(ar_marker)


    # # --------------------ポスターを置きに行く--------------------
    # ar_marker = "ar_marker/709"
    whole_body.move_to_neutral()
    # rospy.sleep(10.0)
    # ar_marker, trans,rot = get_marker_position(ar_marker)
    whole_body.move_to_go()
    move_to_abs(1.960140937495598, 1.8912235350707476, -1.2097353363898349, radius=0.1, time_out=30)
    # whole_body.move_to_go()
    # move_to_abs(0.8, 0.0, 0.0, radius=0.1, time_out=30)



    # # --------------------商品を掴みに行く--------------------
    whole_body.move_to_go()
    move_to_abs(-0.4026532252761044, 0.3327925179753355, 3.0990727019566204, radius=0.1, time_out=30)
    # ar_marker, target = "ar_marker/710", "Bottle"
    # print("ターゲット："+target)
    # whole_body.move_to_neutral()
    # rospy.sleep(10.0)
    # ar_marker, trans,rot = get_marker_position(ar_marker)


    # # --------------------商品を置きに行く--------------------

    move_to_abs(1.960140937495598, 1.8912235350707476, -1.2097353363898349, radius=0.1, time_out=30)
    whole_body.move_to_go()
    # ar_marker = "ar_marker/711" # where to place (select teble)
    # whole_body.move_to_neutral()
    # rospy.sleep(10.0)
    # ar_marker, trans,rot = get_marker_position(ar_marker)
    

    # 待機
    move_to_abs(2.8143008155844016, 1.8117163633784512, 2.7913842123373542, radius=0.1, time_out=30)
    whole_body.move_to_go()


    # whole_body.move_to_go()
    # move_to_abs(0.0, 0.0, 0.0, radius=0.1, time_out=30)
    # whole_body.move_to_go()

# 目的地から０．２ｍ以内であれば到着したことにして次のステップへ

# 次のステップへの遷移にかんして：　「到着」のほかに「AR見えたら」も追加

# AR見えたら、AR基準で位置を修正する


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

        # 位置合わせたあとの、マップの初期位置
    # 0.07865259055059612, -0.5795284561852135, 0.04843300419215437
    # １、２
    # -1.208820077816414, 3.3593553113225045, -2.2326029218111536
    # ３、５ （レジの前）
    # 1.960140937495598, 1.8912235350707476, -1.2097353363898349(grasp)
    # 2.8143008155844016, 1.8117163633784512, 2.7913842123373542(待機)
    # ４
    # -0.6026532252761044, 0.3327925179753355, 3.0990727019566204