#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright (C) 2016 Toyota Motor Corporation
from geometry_msgs.msg import Point, PoseStamped, Quaternion
import rospy
import tf.transformations

rospy.init_node('test')

# initialize ROS publisher
pub = rospy.Publisher('goal', PoseStamped, queue_size=10)

# hsr_state = rospy.Subscriber('/hsrb/omni_base_controller/state', )

# wait to establish connection between the navigation interface
# move_base and navigation_log_recorder node
while pub.get_num_connections() < 2:
    rospy.sleep(0.1)

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

# input goal pose
goal_x = 0.3
goal_y = 0
goal_yaw = 0

# fill ROS message
goal = PoseStamped()
goal.header.stamp = rospy.Time.now()
goal.header.frame_id = "map"
goal.pose.position = Point(goal_x, goal_y, 0)
quat = tf.transformations.quaternion_from_euler(0, 0, goal_yaw)
goal.pose.orientation = Quaternion(*quat)

# publish ROS message
pub.publish(goal)