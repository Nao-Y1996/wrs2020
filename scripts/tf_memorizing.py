#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import tf as TF


rospy.init_node('marker_position_pub', anonymous=True)
listener = TF.TransformListener()
br = TF.TransformBroadcaster()
rate = rospy.Rate(10)

    
TFs = ['ar_marker/14','ar_marker/710','ar_marker/711','ar_marker/712'] # all TF that you want to memorize

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
                # br.sendTransform(memorized_tf_translations[i], memorized_tf_rotations[i],rospy. Time.now(), 'memorized_'+tf, '/map') ここでbroadcastするほうがループ処理が早いがCtr+Cで止まらないので面倒
                print(tf + ' exists.')
            except:
                try:
                    trans, rot = listener.lookupTransform("/map", 'memorized_'+tf, rospy.Time(0))
                    memorized_tf_translations[i] = trans
                    memorized_tf_rotations[i] = rot
                    # br.sendTransform(memorized_tf_translations[i], memorized_tf_rotations[i], rospy.Time.now(), 'memorized_'+tf, '/map') ここでbroadcastするほうがループ処理が早いがCtr+Cで止まらないので面倒
                except TF.LookupException:
                    pass
                except :
                    print('======== 【Unexpected Error】 ========\n======== check the code! ========')
                    import traceback
                    traceback.print_exc()
        # map基準でTFをbroadcast
        for i, tf in enumerate(TFs):
            #-------------------------↓　ありえないrotationになるとbroadcastが止まってしまうのでコードを変更する際は注意すること　↓-------------------------------
            br.sendTransform(memorized_tf_translations[i], memorized_tf_rotations[i], rospy.Time.now(), 'memorized_'+tf, '/map') 
            # br.sendTransform(memorized_tf_translations[i], [0.46,-0.528, 0.53, -0.47], rospy.Time.now(), 'memorized_'+tf, '/map')
            #----------------------------------------------------------------------------------------------------------------------------------------
        rate.sleep()
        
            
