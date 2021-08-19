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

DBNAME2 = "maker_position.db"

# Move timeout[s]
_MOVE_TIMEOUT=60.0
# Grasp force[N]
_GRASP_FORCE=0.2
# Preparation for using the robot functions
robot = hsrb_interface.Robot()
omni_base = robot.get("omni_base")
whole_body = robot.get("whole_body")
gripper = robot.get("gripper")
tts = robot.get("default_tts")
collision_world=robot.try_get("global_collision_world")
whole_body.collision_world = collision_world

listener = tf.TransformListener()


# try:
#     os.remove(DBNAME2)
# except:
#     pass
# conn = sqlite3.connect(DBNAME2)
# try:
#     cur = conn.cursor()
#     cur.execute("CREATE TABLE maker_position(maker_name TEXT PRIMARY KEY, x REAL, y REAL, z REAL)")
#     cur.close()
#     conn.commit()
# except sqlite3.OperationalError:
#     import traceback
#     traceback.print_exc()
#     sys.exit("maker_positionテーブルの初期化に失敗しました") 
# conn.close()

class PickAndPlace():
    def __init__(self, conf):
        self.conf = conf

    def grasp(self, target, ar_marker):
        setting = self.conf[target]

        def hand_control(key, ref):
            x, y, z, ei, ej, ek = setting[key]
            goal = geometry.pose(x, y, z, ei, ej, ek)
            while True:
                try:
                    now = rospy.Time.now()
                    listener.waitForTransform(ref, "/map", now, rospy.Duration(3.0))
                    whole_body.move_end_effector_pose(goal, ref)
                    break
                # except FollowTrajectoryError:
                #     print("f-PickAndPlace-grasp-hand_controll : 例外エラーの発生 ↓")
                #     print('Received comm state PREEMPTING when in simple state DONE with SimpleActionClient in NS /hsrb/omni_base_controller/follow_joint_trajectory')
                except:
                    print("f-PickAndPlace-grasp-hand_controll : 例外エラーの発生")
                    import traceback
                    traceback.print_exc()

        if target != "Poster":
            # open hand
            gripper.command(1.2)
            # Move the hand to front of the target
            hand_control("target_to_hand", ar_marker)
            # Specify the force to grasp
            gripper.apply_force(setting["GRASP_FORCE"])
            # Move the hand up on end effector coordinate
            hand_control("hand_up", "hand_palm_link")
            # Move the hand back on end effector coordinate
            hand_control("hand_back", "hand_palm_link")
            # Transit to initial posture
            whole_body.move_to_neutral()

        if target == "Poster":
            whole_body.move_to_go()
            # open hand
            gripper.command(1.2)
            # Move the hand to left of the target
            hand_control("target_to_hand", ar_marker)
            # Right slide 
            hand_control("target_to_hand2", "hand_palm_link")
            # Specify the force to grasp
            gripper.apply_force(_GRASP_FORCE)
            # Move the hand up on end effector coordinate
            hand_control("hand_up", "hand_palm_link")
            # Move the hand back on end effector coordinate
            hand_control("hand_back", "hand_palm_link")
            # Transit to initial posture
            whole_body.move_to_go()

    def place(self, target, ar_marker):
        setting = self.conf[target]

        def hand_control(key, ref):
            x, y, z, ei, ej, ek = setting[key]
            goal = geometry.pose(x, y, z, ei, ej, ek)
            whole_body.move_end_effector_pose(goal, ref)

        if target != "Poster":
            # Move the hand to place position
            hand_control("tf_place", ar_marker)
            # Move the hand down on end effector coordinate
            hand_control("hand_down",  "hand_palm_link")
            # open hand
            gripper.command(1.2)
            # Move the hand up on end effector coordinate
            hand_control("hand_up_after_place", "hand_palm_link")
            omni_base.go_rel(-0.5, 0.0, 0.0, 100.0)
            # Transit to initial posture
            whole_body.move_to_neutral()
        
        if target == "Poster":
            whole_body.move_to_go()
            # Move the hand to place position
            hand_control("tf_place", ar_marker)
            # Move the hand down on end effector coordinate
            hand_control("hand_down",  "hand_palm_link")
            # open hand
            gripper.command(1.2)
            # Move the hand left 
            hand_control("hand_slide", "hand_palm_link")
            # Move the hand back on end effector coordinate
            hand_control("hand_back_after_place",  "hand_palm_link")
            # Transit to initial posture
            whole_body.move_to_go()

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

    # read json
    dir = os.path.dirname(__file__)
    f = open(dir + "/object_params.json", "r")
    grasp_conf = json.load(f)

    pick_and_place = PickAndPlace(grasp_conf)

    # get target name
    # target = "Bottle"
    # target = "CaffeLatte"
    # target = "Protein"
    # target = "RiceBall"
    # target = "Sandwich"
    # target = "VegetableStick"
    # while not rospy.is_shutdown():

    # --------------------ポスターを掴みに行く--------------------
    whole_body.move_to_go()
    move_to_abs(-1.208820077816414, 3.3593553113225045, -2.2326029218111536, radius=0.1, time_out=300)

    # grasp
    ar_marker, target = "ar_marker/712", "Poster"
    print("ターゲット："+target)
    whole_body.move_to_neutral()
    rospy.sleep(10.0)
    ar_marker,trans,rot = get_marker_position(ar_marker)
    pick_and_place.grasp(target, ar_marker)
    rospy.sleep(3.0)

    # --------------------ポスターを置きに行く--------------------
    whole_body.move_to_go()
    move_to_abs(1.0364684580131838, 3.7364411780851223, 0.7068780818689953, radius=0.1, time_out=300)
    # whole_body.move_to_go()
    # move_to_abs(0.8, 0.0, 0.0, radius=0.1, time_out=30)

    # # place
    ar_marker = "ar_marker/709"
    whole_body.move_to_neutral()
    rospy.sleep(10.0)
    ar_marker, trans,rot = get_marker_position(ar_marker)
    try:
        pick_and_place.place(target, ar_marker)
    except:
        print("置くのに失敗しました")
        import traceback
        traceback.print_exc()

    # --------------------商品を掴みに行く--------------------
    whole_body.move_to_go()
    move_to_abs(-0.6026532252761044, 0.3327925179753355, 3.0990727019566204, radius=0.1, time_out=300)

    # grasp
    ar_marker, target = "ar_marker/711", "Bottle"
    print("ターゲット："+target)
    whole_body.move_to_neutral()
    rospy.sleep(10.0)
    ar_marker, trans,rot = get_marker_position(ar_marker)
    pick_and_place.grasp(target, ar_marker)
    rospy.sleep(3.0)


    # --------------------商品を置きに行く--------------------
    whole_body.move_to_go()
    move_to_abs(1.0364684580131838, 3.7364411780851223, 0.7068780818689953, radius=0.1, time_out=30)
    whole_body.move_to_go()
    # move_to_abs(0.8, 0.0, 0.0, radius=0.1, time_out=30)

    # # place
    ar_marker = "ar_marker/708" # where to place (select teble)
    whole_body.move_to_neutral()
    rospy.sleep(10.0)
    ar_marker, trans,rot = get_marker_position(ar_marker)
    try:
        pick_and_place.place(target, ar_marker)
    except:
        print("置くのに失敗しました")
        import traceback
        traceback.print_exc()


    whole_body.move_to_go()
    move_to_abs(0.0, 0.0, 0.0, radius=0.1, time_out=30)
    whole_body.move_to_go()

# 目的地から０．２ｍ以内であれば到着したことにして次のステップへ

# 次のステップへの遷移にかんして：　「到着」のほかに「AR見えたら」も追加

# AR見えたら、AR基準で位置を修正する


    """
    移動
    定位位置に移動してくる

    ARマーカーに対して前方にいれば到着判定
    

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
    # ３、５
    # 1.0364684580131838, 3.7364411780851223, 0.7068780818689953
    # （レジの前）
    # 1.5782184826733499, 1.6871776823854516, -0.9366680768555469
    # ４
    # -0.6026532252761044, 0.3327925179753355, 3.0990727019566204