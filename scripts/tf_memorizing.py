#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import tf as TF
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3


rospy.init_node('marker_position_pub', anonymous=True)
listener = TF.TransformListener()
br = TF.TransformBroadcaster()
rate = rospy.Rate(10)

def create_position_TF(marker, tf_name, x, z):
    try: # マーカーの左手前にTFを生成する
        trans, rot = listener.lookupTransform(marker, marker, rospy.Time(0))
        # 位置を修正
        trans[0], trans[1], trans[2] = trans[0]+x, trans[1], trans[2]+z
        # 向きを修正
        rot = TF.transformations.euler_from_quaternion((rot[0],rot[1],rot[2],rot[3]))
        rot = (rot[0], rot[1], rot[2]-1.57)
        rot = TF.transformations.quaternion_from_euler(rot[0],rot[1],rot[2])

        br.sendTransform(trans, rot, rospy.Time.now(), tf_name+'_ref',  marker)
    # except (TF.LookupException,TF.ConnectivityException):
    #     pass
    except :
        pass
        # print('======== 【Unexpected Error】 ========\n======== check the code! ========')
        # import traceback
        # traceback.print_exc()

    try:# マーカーの左手前にTFの真下にTFを生成
        trans, rot = listener.lookupTransform("/map", tf_name+'_ref', rospy.Time(0))
        # 位置を修正
        trans[2] = 0.0
        # 向きを修正
        br.sendTransform(trans, rot, rospy.Time.now(), tf_name,  '/map')
    # except (TF.LookupException,TF.ConnectivityException):
    #     pass
    except :
        pass
        # print('======== 【Unexpected Error】 ========\n======== check the code! ========')
        # import traceback
        # traceback.print_exc()

TFs = ['ar_marker/709','ar_marker/710','ar_marker/711','ar_marker/712','ar_marker/14'] # all TF that you want to memorize

memorized_tf_translations = np.array([[0.0]*3] * len(TFs))
memorized_tf_rotations = np.array([[0.0]*4] * len(TFs))
# Converting rotation [0.0, 0.0, 0.0, 0.0] into [0.0, 0.0, 0.0, 1.0]
# because ratation [0.0, 0.0, 0.0, 0.0] is not possible. 
for j in range(len(TFs)):
    memorized_tf_rotations[j][3]=1.0

if __name__ == "__main__":
    while not rospy.is_shutdown():
        print('-----------------------------------------------------------')
        for i, tf in enumerate(TFs):
            try:
                trans, rot = listener.lookupTransform("/map", tf, rospy.Time(0))
                memorized_tf_translations[i] = trans
                memorized_tf_rotations[i] = rot
                br.sendTransform(memorized_tf_translations[i], memorized_tf_rotations[i],rospy. Time.now(), 'memorized_'+tf, '/map')
                print(tf + ' exists.')
            except:
                try:
                    trans, rot = listener.lookupTransform("/map", 'memorized_'+tf, rospy.Time(0))
                    memorized_tf_translations[i] = trans
                    memorized_tf_rotations[i] = rot
                    br.sendTransform(memorized_tf_translations[i], memorized_tf_rotations[i], rospy.Time.now(), 'memorized_'+tf, '/map')
                except TF.LookupException:
                    pass
                except :
                    print('======== 【Unexpected Error】 ========\n======== check the code! ========')
                    import traceback
                    traceback.print_exc()

        #--------------------↓　以下のようにありえないrotationになるとbroadcastが止まってしまうのでコードを変更する際は注意すること　↓--------------------------
        # br.sendTransform(memorized_tf_translations[i], [0.0, 0.0, 0.0, 0.0], rospy.Time.now(), 'memorized_'+tf, '/map')
        #----------------------------------------------------------------------------------------------------------------------------------------
        
        # "memorized_ar_marker/7xx"が見えているときにそのマーカーの左手前にTFを生成する
        # 更に、生成したTFの真下にもTFを生成する（このTFにHSRをいどうさせる）
        # 生成した位置にHSRが移動してからのほうがポスターの把持や設置が成功しやすい
        # for tf in ["memorized_ar_marker/709", "memorized_ar_marker/710"]:
        create_position_TF('memorized_ar_marker/709', 'grasp_position',-0.4,-0.9)
        create_position_TF('memorized_ar_marker/710', 'place_position',-0.4,-0.9)
        create_position_TF('memorized_ar_marker/711', 'grasp_bottle_position',-0.0,-0.9)
        create_position_TF('memorized_ar_marker/712', 'place_bottle_position',-0.0,-0.9)
        rate.sleep()