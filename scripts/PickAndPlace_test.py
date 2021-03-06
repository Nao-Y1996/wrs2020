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

_DB_MOVE_STATE = "move_state.db"

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
# collision_world=robot.try_get("global_collision_world")
# whole_body.collision_world = collision_world

listener = TF.TransformListener()


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
                    print('"f-PickAndPlace-grasp-hand_controll :　TransformException')
                    import traceback
                    traceback.print_exc()
                    pass
                except hsrb_interface.exceptions.MotionPlanningError:
                    print('armの起動生成ができません　goal in collision!')
                    import traceback
                    traceback.print_exc()
                    break
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
                    import traceback
                    traceback.print_exc()
                    pass
                except hsrb_interface.exceptions.MotionPlanningError:
                    print('armの起動生成ができません　goal in collision!')
                    import traceback
                    traceback.print_exc()
                    break
                except:
                    print("f-PickAndPlace-grasp-hand_controll : 例外エラーの発生")
                    import traceback
                    traceback.print_exc()
                    break

        if target != "Poster":
            # Move the hand to place position
            hand_control("tf_place", ar_marker)
            # Move the hand down on end effector coordinate
            hand_control("hand_down",  "hand_palm_link")
            # open hand
            gripper.command(1.2)
            # Move the hand up on end effector coordinate
            hand_control("hand_up_after_place", "hand_palm_link")
            omni_base.go_rel(-0.5, 0.0, 0.0, 10.0)
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
            # -----------------------
            # else:
            #     omni_base.go_rel(x, y, z, time_out)
            # -----------------------
        except:
            print("move_to_abs : Unexpected Error! Retry Demonstration !")
            tts.say('リトライしてください')
            import traceback
            traceback.print_exc()

def Get_TF_from_map(tf):
    is_detecting = False
    trans,rot = None, None
    try:
        (trans,rot) = listener.lookupTransform("/map", tf, rospy.Time(0))
        is_detecting = True
    except (TF.LookupException, TF.ConnectivityException, TF.ExtrapolationException):
        print('Failed to get TF of '+ tf )
    except:
        print("Get_TF_from_map :  Unexpected Error! Retry Demonstration !")
        tts.say('リトライしてください')
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
                tts.say('リトライしてください')
                return (None, None)
    
# DBの初期化
# try:
#     os.remove(_DB_MOVE_STATE)
# except:
#     pass
# conn = sqlite3.connect(_DB_MOVE_STATE)
# try:
#     cur = conn.cursor()
#     cur.execute("CREATE TABLE move_state(maker_name TEXT PRIMARY KEY, x REAL, y REAL, z REAL)")
#     cur.close()
#     conn.commit()
# except sqlite3.OperationalError:
#     import traceback
#     traceback.print_exc()
#     sys.exit(_DB_MOVE_STATE, "の初期化に失敗しました") 
# conn.close()



if __name__ == "__main__":


    # read json
    dir = os.path.dirname(__file__)
    f = open(dir + "/object_params.json", "r")
    grasp_conf = json.load(f)

    f = open(dir + "/position_params.json", "r")
    positions = json.load(f)

    whole_body.liner_weight = 80
    pick_and_place = PickAndPlace(grasp_conf)


    x, y, z = omni_base.pose
    # --------------------ポスターを掴みに行く--------------------
    whole_body.move_to_go()
    move_to_abs(x, y, z, radius=0.1, time_out=3)


    # grasp
    whole_body.move_to_go()
    rospy.sleep(10.0)
    ar_marker, target = "memorized_ar_marker/709", "Poster"
    print("ターゲット："+target)
    trans,rot = ensure_detecting_ARmarker(ar_marker)

    print("場所を修正します")
    try:
        omni_base.go_pose(geometry.pose(ei=3.14, ej=-1.57), 10.0, ref_frame_id='grasp_position')
        print("場所を修正しました")
    except:
        pass
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
    move_to_abs(x, y, z, radius=0.1, time_out=3)

    # # place
    whole_body.move_to_go()
    rospy.sleep(10.0)
    ar_marker = "memorized_ar_marker/710"
    trans,rot = ensure_detecting_ARmarker(ar_marker)

    print("場所を修正します")
    try:
        omni_base.go_pose(geometry.pose(ei=3.14, ej=-1.57), 10.0, ref_frame_id='place_position')
        print("場所を修正しました")
    except:
        pass
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
    move_to_abs(x, y, z, radius=0.1, time_out=3)

    # grasp
    whole_body.move_to_neutral()
    rospy.sleep(10.0)
    ar_marker, target = "memorized_ar_marker/711", "Bottle"
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
    move_to_abs(x, y, z, radius=0.1, time_out=3)
    
    # # place
    whole_body.move_to_neutral()
    rospy.sleep(10.0)
    ar_marker = "memorized_ar_marker/712" # where to place (select teble)
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
    move_to_abs(x, y, z, radius=0.1, time_out=3)
    gripper.command(0.0)
    rospy.sleep(10)
    
    whole_body.move_to_joint_positions({'head_pan_joint': 0.2, 'head_tilt_joint': 0.3})
    playsound("/home/kubotalab-hsr/catkin_ws/src/wrs2020/scripts/WRS2020.mp3")
    whole_body.move_to_go()


# 次のステップへの遷移にかんして：　「到着」のほかに「AR見えたら」も追加

