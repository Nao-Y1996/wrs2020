#!/usr/bin/env python
# -*- coding: utf-8 -*-

import matplotlib.pyplot as plt
import traceback
import cv2
import numpy as np
import os
from cv_bridge import CvBridge, CvBridgeError
import rospy
from sensor_msgs.msg import Image
import collections


import actionlib
import control_msgs.msg
import controller_manager_msgs.srv
import trajectory_msgs.msg


class getIm_from_HSR(object):

    def __init__(self):
        rgb = '/hsrb/head_rgbd_sensor/rgb/image_rect_color'
        self._bridge = CvBridge()
        self.rgb_image = None
        self.sub_count_rgb = 0
        self.sub_count_dep = 0
        # Subscribe color image data from HSR
        rospy.Subscriber(rgb, Image, self.callback)
        # Wait until connection
        rospy.wait_for_message(rgb, Image, timeout=15.0)

    def callback(self, data):
        try:
            self.rgb_image = self._bridge.imgmsg_to_cv2(data, "bgr8")
            self.sub_count_rgb += 1
        except CvBridgeError as cv_bridge_exception:
            rospy.logerr(cv_bridge_exception)


class face_detection():
    def __init__(self):
        self.faces = None
        self.face_areas = []
        self.face_centers = []

        self.cascade_path = os.path.dirname(
            __file__) + '/haarcascades/haarcascade_frontalface_alt2.xml'

    def detect(self, input_image):
        cascade = cv2.CascadeClassifier(self.cascade_path)
        self.faces = np.array(list(cascade.detectMultiScale(
            input_image, scaleFactor=1.9, minNeighbors=2)))

        self.face_areas = []
        self.face_centers = []
        for (x, y, w, h) in self.faces:
            face_center_x = x+w//2
            face_center_y = y+h//2
            self.face_areas.append(w * h)
            self.face_centers.append([face_center_x, face_center_y])
            cv2.rectangle(input_image, (x, y), (x+w, y+h),
                          color=(0, 0, 225), thickness=3)
            # cv2.circle(input_image, (face_center_x, face_center_y), 10, color=(255, 255, 255), thickness=1, lineType=cv2.LINE_8, shift=0)
            # print(self.face_areas, self.face_centers)

        return input_image


def skin_detect(img):

    def __create_mask_image(channel, th_max, th_min):
        ret, src = cv2.threshold(channel, th_max, 255, cv2.THRESH_TOZERO_INV)
        ret, dst = cv2.threshold(src, th_min, 255, cv2.THRESH_BINARY)
        return dst

    H_LOW_THRESHOLD = 0
    H_HIGH_THRESHOLD = 15
    S_LOW_THRESHOLD = 50
    S_HIGH_THRESHOLD = 255
    V_LOW_THRESHOLD = 50
    V_HIGH_THRESHOLD = 255
    # BGR -> HSV
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    # Break down channels
    try:
        H, S, V = cv2.split(hsv)
        # Create mask images(H, S, V)
        h_dst = __create_mask_image(H, H_HIGH_THRESHOLD, H_LOW_THRESHOLD)
        s_dst = __create_mask_image(S, S_HIGH_THRESHOLD, S_LOW_THRESHOLD)
        # v_dst = create_mask_image(V, V_HIGH_THRESHOLD, V_LOW_THRESHOLD)
        dst = cv2.bitwise_and(h_dst, s_dst)
    except ValueError:
        dst = None

    return dst




if __name__ == '__main__':
    rospy.init_node('face')

    # initialize action client
    cli=actionlib.SimpleActionClient(
        '/hsrb/head_trajectory_controller/follow_joint_trajectory',
        control_msgs.msg.FollowJointTrajectoryAction)
    # wait for the action server to establish connection
    cli.wait_for_server()
    # make sure the controller is running
    rospy.wait_for_service('/hsrb/controller_manager/list_controllers')
    list_controllers=rospy.ServiceProxy(
        '/hsrb/controller_manager/list_controllers',
        controller_manager_msgs.srv.ListControllers)
    running=False
    while running is False:
        rospy.sleep(0.1)
        for c in list_controllers().controller:
            if c.name == 'head_trajectory_controller' and c.state == 'running':
                running=True

    # face_tracking()
    HSR_image=getIm_from_HSR()
    height, width, _ = HSR_image.rgb_image.shape
    cx, cy = width//2, height//2
    
    FD=face_detection()
    spin_rate=rospy.Rate(100)

    # capture=cv2.VideoCapture(0)

    is_detcting_Human_Face=False
    tracking=False

    kp, ki, kd=0.3, 0.02, 0.5
    p_errors, errors = [0, 0], [0, 0]
    fcx_error_list,fcy_error_list = [0]*100, [0]*100
    pan, tilt=0.0, 0.0

    pid_check_range = 100
    fig, ax = plt.subplots(1, 2)
    fcx_list = [0]*pid_check_range
    fcx_lines, = ax[0].plot(list(range(pid_check_range)), fcx_list)
    fcy_list = [0]*pid_check_range
    fcy_lines, = ax[1].plot(list(range(pid_check_range)), fcy_list)
    fcx,fcy =0, 0 # 顔が検出されていないときのグラフ表示のために必要
    count = 0
    while not rospy.is_shutdown():
        #face detection
        detection_image = FD.detect(HSR_image.rgb_image)
        

        if len(FD.face_areas) != 0:
            # get index of neraest face
            i = np.argmax(np.array(FD.face_areas))
            #　make face-rectangle smaller
            w = int(FD.faces[i][2]*0.6)
            h = int(FD.faces[i][3]*0.6)
            fcx = FD.face_centers[i][0]
            fcy = FD.face_centers[i][1]
            # 顔範囲の切り出し,大きさの計算
            face_img = detection_image[fcy-h:fcy+h, fcx-w:fcx+w]
            face_skin_img = skin_detect(face_img)
            
            try:
                if int(np.average(face_skin_img)) >= 40:
                    is_detcting_Human_Face=True
                else:
                    is_detcting_Human_Face=False
            except:
                    is_detcting_Human_Face=False
        else:
            is_detcting_Human_Face=False
            # fcx_error_list,fcy_error_list = [0]*100, [0]*100
            
        if is_detcting_Human_Face:
            errors=[fcx - cx, fcy - cy]
            pan=kp * errors[0] + kd*(errors[0] - p_errors[0]) + ki * np.sum(fcx_error_list)
            tilt=kp * errors[1] + kd*(errors[1] - p_errors[1]) + ki * np.sum(fcy_error_list)
            pan=-pan/100
            # pan = np.clip(pan, -3.839,1.745)
            tilt=-tilt/100
            # tilt = np.clip(tilt, -1.570,0.523)

            # fill ROS message
            goal=control_msgs.msg.FollowJointTrajectoryGoal()
            traj=trajectory_msgs.msg.JointTrajectory()
            traj.joint_names=["head_pan_joint", "head_tilt_joint"]
            p=trajectory_msgs.msg.JointTrajectoryPoint()
            p.positions=[pan, tilt]
            p.velocities=[0, 0]
            p.time_from_start=rospy.Time(3)
            traj.points=[p]
            goal.trajectory=traj

            # send message to the action server
            cli.send_goal(goal)

            p_errors = errors
            fcx_error_list[count]=errors[0]
            fcy_error_list[count]=errors[1]


        else:
            p_errors, errors=[0, 0], [0, 0]
            pan, tilt=0.0, 0.0


        cv2.imshow("Face Detection", detection_image)
        # cv2.imshow("Face Detection", face_skin_img)

        # ------------------------PIDのチェック-------------------------
        # print(fcx, cx)
        # print(fcy, cy)
        # print('----------')
        fcx_list.append(fcx-cx)
        # list(collections.deque(fcx_list, pid_check_range)) 
        fcy_list.append(fcy-cy)
        # list(collections.deque(fcy_list, pid_check_range)) 
        try:
            fcxs = list(collections.deque(fcx_list, pid_check_range))
            fcx_lines.set_data(list(range(100)), fcxs)
            ax[0].set_xlim(0, 100)
            ax[0].set_ylim(-cx, cx)
            fcys = list(collections.deque(fcy_list, pid_check_range))
            fcy_lines.set_data(list(range(100)), fcys)
            ax[1].set_xlim(0, 100)
            ax[1].set_ylim(-cy, cy)
            plt.pause(0.1)
        except:
            pass
        # ------------------------------------------------------------
        cv2.waitKey(1)
        spin_rate.sleep()
        print(fcx_error_list)

        count += 1
        if count==100:
            count = 0
