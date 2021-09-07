#!/usr/bin/python
# -*- coding: utf-8 -*-

import hsrb_interface
import rospy
from hsrb_interface import geometry
import json
import os
import numpy as np
import tf as TF
import sqlite3
import sys

from rospy.timer import sleep
from playsound import playsound
from tf2_ros import TransformException

DBNAME2 = "maker_position.db"

# Move timeout[s]
_MOVE_TIMEOUT=60.0
# Grasp force[N]
_GRASP_FORCE=0.2
# Preparation for using the robot functions
while True:
    try:
        robot = hsrb_interface.Robot()
        break
    except (hsrb_interface.exceptions.RobotConnectionError, rospy.exceptions.ROSInitException):
        print('起動中です')
        rospy.sleep(1)

omni_base = robot.get("omni_base")
whole_body = robot.get("whole_body")
gripper = robot.get("gripper")
tts = robot.get("default_tts")
collision_world=robot.try_get("global_collision_world")
whole_body.collision_world = collision_world

listener = TF.TransformListener()


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
                    listener.waitForTransform("/map", ref, now, rospy.Duration(3.0))
                    whole_body.move_end_effector_pose(goal, ref)
                    break
                # except FollowTrajectoryError:
                #     print("f-PickAndPlace-grasp-hand_controll : 例外エラーの発生 ↓")
                #     print('Received comm state PREEMPTING when in simple state DONE with SimpleActionClient in NS /hsrb/omni_base_controller/follow_joint_trajectory')
                except TransformException: # Lookup would require extrapolation into the future.
                    print('"f-PickAndPlace-grasp-hand_controll :　TransformExceptionです')
                    
                    pass
                except hsrb_interface.exceptions.MotionPlanningError:
                    print('armの起動生成ができません　goal in collision!')
                except:
                    print("f-PickAndPlace-grasp-hand_controll : 例外エラーの発生")
                    import traceback
                    traceback.print_exc()
                    break

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
            whole_body.move_to_go()

        if target == "Poster":
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
            whole_body.move_to_go()
        
        if target == "Poster":
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
            print("move_to_abs : moving")
            omni_base.go_abs(x, y, z, time_out)
            print("move_to_abs : succsess")
            whole_body.move_to_go()
            break
        except hsrb_interface.exceptions.MobileBaseError: #  Failed to reach goal ()
            position_now = np.array(omni_base.pose[0:2])
            if np.linalg.norm(goal_position-position_now)<=radius: # ゴールから半径radius内にいれば移動成功とする
                print("move_to_abs : succsess in radius")
                whole_body.move_to_go()
                break
            # -------------移動失敗のとき--------------
            # 目標地点からradiusを半径とする円の外接四角形の四隅への移動を順番に試みる
            try:
                print("move_to_abs : failed \n Moving to sub_position and trying again  -->  sub position No."+str(i))
                print("sub position : ", sub_positions[i][0], sub_positions[i][1], z)
                omni_base.go_abs(sub_positions[i][0], sub_positions[i][1], z, 15.0)
            except:
                print('failed : Moving to sub_position')
                pass
            # except:
            #     print("f-move_to_abs : 例外エラーの発生")
            #     import traceback
            #     traceback.print_exc()
            finally:
                i += 1
                if i==4:
                    i=0
            # 移動できないと無限ループになる

def Get_TF_from_map(tf):
    is_detecting = False
    trans,rot = None, None
    try:
        (trans,rot) = listener.lookupTransform("/map", tf, rospy.Time(0))
        is_detecting = True
    except (TF.LookupException, TF.ConnectivityException, TF.ExtrapolationException):
        print('Failed to get TF of '+ tf )
    except:
        print("function : check_visibleness_marker : 例外エラーの発生")
        import traceback
        traceback.print_exc()
    return (is_detecting, tf, trans,rot)

def ensure_detecting_ARmarker(tf):
    num_trial = 2
    for i in range(num_trial+1):
        is_detecting, tf, trans,rot = Get_TF_from_map(tf)

        if is_detecting:
            print('マーカーを視認しました')
            return (trans,rot)
        else:
            print(1)
            if i < num_trial:
                print("マーカーが見えません。姿勢を変えて再度tryします。")
                # position_now = np.array(omni_base.pose)
                x = -1*0.1 if i%2==0 else 0.1
                omni_base.go_rel(x, 0, 0, 10.0)
                whole_body.move_to_go()
                rospy.sleep(10.0)
            else:
                print("姿勢を変えてもマーカーが見えません.")
                return (None, None)
    


if __name__ == "__main__":

    # read json
    dir = os.path.dirname(__file__)
    f = open(dir + "/object_params.json", "r")
    grasp_conf = json.load(f)

    f = open(dir + "/position_params.json", "r")
    positions = json.load(f)

    whole_body.liner_weight = 70
    pick_and_place = PickAndPlace(grasp_conf)


    # --------------------ポスターを掴みに行く--------------------
    whole_body.move_to_go()
    move_to_abs(0.0, 0.0, 0.0, radius=0.1, time_out=30)
    # move_to_abs(positions['grasp_poster'][0], positions['grasp_poster'][1], positions['grasp_poster'][2], radius=0.1, time_out=300)


    # grasp
    whole_body.move_to_go()
    rospy.sleep(10.0)
    ar_marker, target = "memorized_ar_marker/14", "Poster"
    print("ターゲット："+target)
    trans,rot = ensure_detecting_ARmarker(ar_marker)

    print("場所を修正します")
    move_to_abs(trans[0]-0.9, trans[1]+0.4, 0.0, radius=0.1, time_out=30)
    whole_body.move_to_go()
    
    height = trans[2]
    if height >=0.6:
        arm_lift = height-0.6
        whole_body.move_to_joint_positions({'arm_lift_joint': arm_lift})
    
    if trans!=None:
        try:
            pick_and_place.grasp(target, ar_marker)
        except:
            print("摑むのに失敗しました")
            import traceback
            traceback.print_exc()
    else:
        print('マーカーが利用できません')
        tts.say('マーカーが見えません')
    
    rospy.sleep(3.0)

    # --------------------ポスターを置きに行く(レジ前)--------------------
    whole_body.move_to_go()
    move_to_abs(0.0, 0.0, 0.0, radius=0.1, time_out=30)
    # move_to_abs(positions['place_poster'][0], positions['place_poster'][1], positions['place_poster'][2], radius=0.1, time_out=300)


    # # place
    whole_body.move_to_go()
    rospy.sleep(10.0)
    ar_marker = "memorized_ar_marker/14"
    trans,rot = ensure_detecting_ARmarker(ar_marker)

    print("場所を修正します")
    move_to_abs(trans[0]-0.9, trans[1]+0.4, 0.0, radius=0.1, time_out=30)
    whole_body.move_to_go()

    height = trans[2]
    if height >=0.6:
        arm_lift = height-0.6
        whole_body.move_to_joint_positions({'arm_lift_joint': arm_lift})

    if trans!=None:
        try:
            pick_and_place.place(target, ar_marker)
        except:
            print("置くのに失敗しました")
            import traceback
            traceback.print_exc()
    else:
        print('マーカーが利用できません')
        tts.say('マーカーが見えません')
    

    # --------------------商品を掴みに行く--------------------
    whole_body.move_to_go()
    move_to_abs(0.0, 0.0, 0.0, radius=0.1, time_out=30)
    # move_to_abs(positions['grasp_bottle'][0], positions['grasp_bottle'][1], positions['grasp_bottle'][2], radius=0.1, time_out=300)


    # grasp
    whole_body.move_to_neutral()
    rospy.sleep(10.0)
    ar_marker, target = "memorized_ar_marker/710", "Bottle"
    print("ターゲット："+target)
    trans,rot = ensure_detecting_ARmarker(ar_marker)

    height = trans[2]
    if height >=0.6:
        arm_lift = height-0.6
        whole_body.move_to_joint_positions({'arm_lift_joint': arm_lift})
    
    if trans!=None:
        try:
            pick_and_place.grasp(target, ar_marker)
        except:
            print("摑むのに失敗しました")
            import traceback
            traceback.print_exc()
    else:
        print('マーカーが利用できません')
        tts.say('マーカーが見えません')
    rospy.sleep(3.0)

    
    # --------------------商品を置きに行く(レジ前)--------------------
    whole_body.move_to_go()
    move_to_abs(0.0, 0.0, 0.0, radius=0.1, time_out=30)
    # move_to_abs(positions['place_bottle'][0], positions['place_bottle'][1], positions['place_bottle'][2], radius=0.1, time_out=300)

    
    # # place
    whole_body.move_to_neutral()
    rospy.sleep(10.0)
    ar_marker = "memorized_ar_marker/710" # where to place (select teble)
    trans,rot = ensure_detecting_ARmarker(ar_marker)

    height = trans[2]
    if height >=0.6:
        arm_lift = height-0.6
        whole_body.move_to_joint_positions({'arm_lift_joint': arm_lift})

    if trans!=None:
        try:
            pick_and_place.place(target, ar_marker)
        except:
            print("置くのに失敗しました")
            import traceback
            traceback.print_exc()
    else:
        print('マーカーが利用できません')
        tts.say('マーカーが見えません')


    
    # -------------------------待機-------------------------
    whole_body.move_to_go()
    move_to_abs(0.0, 0.0, 0.0, radius=0.1, time_out=30)
    # playsound("/home/kubotalab-hsr/catkin_ws/src/wrs2020/scripts/WRS2020.mp3")
    whole_body.move_to_go()


# 次のステップへの遷移にかんして：　「到着」のほかに「AR見えたら」も追加

