#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright (C) 2016 Toyota Motor Corporation


# rospy.init_node('test')



# # make sure the controller is running
# rospy.wait_for_service('/hsrb/controller_manager/list_controllers')
# list_controllers = rospy.ServiceProxy(
#     '/hsrb/controller_manager/list_controllers',
#     controller_manager_msgs.srv.ListControllers)
# running = False
# while running is False:
#     rospy.sleep(0.1)
#     for c in list_controllers().controller:
#         if c.name == 'head_trajectory_controller' and c.state == 'running':
#             running = True

# # fill ROS message
# goal = control_msgs.msg.FollowJointTrajectoryGoal()
# traj = trajectory_msgs.msg.JointTrajectory()
# traj.joint_names = ["head_pan_joint", "head_tilt_joint"]
# p = trajectory_msgs.msg.JointTrajectoryPoint()
# p.positions = [0.5, 0.5]
# p.velocities = [0, 0]
# p.time_from_start = rospy.Time(3)
# traj.points = [p]
# goal.trajectory = traj

# # send message to the action server
# cli.send_goal(goal)

# # wait for the action server to complete the order
# # cli.wait_for_result()
# rospy.sleep(1)
# p.positions = [-0.5, -0.5]
# p.velocities = [0, 0]
# p.time_from_start = rospy.Time(3)
# traj.points = [p]
# goal.trajectory = traj
# cli.send_goal(goal)


import traceback
import cv2
import numpy as np
import os
from cv_bridge import CvBridge, CvBridgeError
import rospy
from sensor_msgs.msg import Image


import actionlib
import control_msgs.msg
import controller_manager_msgs.srv
import trajectory_msgs.msg



class getIm_from_HSR(object):

    def __init__(self):
        rgb = r'/hsrb/head_rgbd_sensor/rgb/image_rect_color'
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
            self.sub_count_rgb +=1
        except CvBridgeError as cv_bridge_exception:
            rospy.logerr(cv_bridge_exception)


class face_detection():
    def __init__(self):
        self.face_list = []
        self.face_areas = []
        self.face_centers = []

        self.cascade_path = dir = os.path.dirname(__file__) + '/haarcascades/haarcascade_frontalface_alt2.xml'

    def detect(self,input_image):
        cascade = cv2.CascadeClassifier(self.cascade_path)
        self.face_list = np.array(list(cascade.detectMultiScale(input_image,scaleFactor=1.9, minNeighbors=2)))

        self.face_areas = []
        self.face_centers = []
        for (x, y, w, h) in self.face_list:            
            face_center_x = x+w//2
            face_center_y = y+h//2
            self.face_areas.append(w * h)
            self.face_centers.append([face_center_x, face_center_y])
            cv2.rectangle(input_image, (x, y), (x+w, y+h), color=(0, 0, 225), thickness=3) 
            # cv2.circle(input_image, (face_center_x, face_center_y), 10, color=(255, 255, 255), thickness=1, lineType=cv2.LINE_8, shift=0)
        # print(self.face_areas, self.face_centers)

        return input_image

    def get_nearest_face_img(self, input_image):
        face_img = None
        detected_img = self.detect(input_image)
        try:
            #検出した顔の範囲をより小さい範囲に限定する
            id_nearest_face = np.argmax(np.array(self.face_areas))
            w= int(self.face_list[id_nearest_face][2]*0.6)
            h= int(self.face_list[id_nearest_face][3]*0.6)
            center_x= int(self.face_centers[id_nearest_face][0])
            center_y= int(self.face_centers[id_nearest_face][1])
            #顔範囲の切り出し,大きさの計算
            face_img = detected_img[center_y-h:center_y+h,center_x-w:center_x+w]
            return face_img ,center_x ,center_y
        except ValueError: # face_areaが空のリストである（顔が検出されない）とき
            return detected_img, None, None
        except:
            import traceback
            traceback.print_exc()  
            return detected_img, None, None
        

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


def face_tracking():
    sub_cycle = 100
    pan = 0.0
    tilt = 0.0
    pan_old = 0.0
    tilt_old = 0.0
    yaw = 0.0
    detection_count = 0


    try:
        HSR_image = getIm_from_HSR()
        FD = face_detection()
        spin_rate = rospy.Rate(sub_cycle)

	
        while not rospy.is_shutdown():

            while  not rospy.is_shutdown():
                face_im = FD.get_face_img(HSR_image.rgb_image)

                cv2.imshow("Face Detection", face_img)
                
                
                # if (Face.face_num==1):
                #     #検出した顔の範囲をより小さい範囲に限定する
                #     w= int(Face.face_list[0][2]*0.6)
                #     h= int(Face.face_list[0][3]*0.6)
                #     x= int(Face.face_x-w/2)
                #     y= int(Face.face_y-h/2)
                #     #顔範囲の切り出し,大きさの計算
                #     face_img = detection_image[y:y+h,x:x+w]
                #     face_skin = skin_detect(face_img)
                #     face_h, face_w= face_skin.shape[:2]
                #     face_area = face_h*face_w
                #     # cv2.imshow("Face_skin", face_skin)
                #     face_pixel = int(np.average(face_skin))
                # cv2.waitKey(1)

                #一人の人間の顔判定
                # if (Face.face_num==1)and(face_pixel>40):
                #     Human_Face = True
                #     detection_count += 1
                    
		#人間の顔であればDepth取得
                # if Human_Face==True:

		#顔追従
                # if (abs(Face.Vx)>20 or abs(Face.Vy)>20)and(Human_Face==True)and(detection_count>1):
                #     tracking=True
                # if (abs(Face.Vx)<20 and abs(Face.Vy)<20):
                #     tracking = False
                    
                

       
                
        
    except rospy.ROSException as wait_for_msg_exception:
        rospy.logerr(wait_for_msg_exception)

    cv2.destroyAllWindows()




if __name__ == '__main__':
    rospy.init_node('face')

    # initialize action client
    cli = actionlib.SimpleActionClient(
        '/hsrb/head_trajectory_controller/follow_joint_trajectory',
        control_msgs.msg.FollowJointTrajectoryAction)
    # wait for the action server to establish connection
    cli.wait_for_server()
    # make sure the controller is running
    rospy.wait_for_service('/hsrb/controller_manager/list_controllers')
    list_controllers = rospy.ServiceProxy(
        '/hsrb/controller_manager/list_controllers',
        controller_manager_msgs.srv.ListControllers)
    running = False
    while running is False:
        rospy.sleep(0.1)
        for c in list_controllers().controller:
            if c.name == 'head_trajectory_controller' and c.state == 'running':
                running = True

    # face_tracking()
    HSR_image = getIm_from_HSR()
    height, width , _ = HSR_image.rgb_image.shape
    cx, cy = width//2, height//2
    print(cx, cy)
    FD = face_detection()
    spin_rate = rospy.Rate(100)



    capture = cv2.VideoCapture(0)

    Human_Face = False
    tracking = False
    kp, kd = 0.3, 0.5

    while  not rospy.is_shutdown():
        if not Human_Face:

            p_errors, errors = [0,0], [0,0]
            pan, tilt = 0.0, 0.0

        ret, frame = capture.read()
        
        detection_img = FD.detect(HSR_image.rgb_image)
        # cv2.imshow("Face Detection", detection_img)

        Human_Face = False
        face_img, fcx, fcy = FD.get_nearest_face_img(HSR_image.rgb_image)

        face_skin_img = skin_detect(face_img)
        if face_skin_img is not None:
            face_pixel = int(np.average(face_skin_img))
            if(face_pixel>40):
                Human_Face = True
                tracking = True

        if tracking & (fcx is not None):
            errors = [fcx - cx, fcy - cy]
            pan  = kp * errors[0] + kd*(errors[0] - p_errors[0])
            tilt = kp * errors[1] + kd*(errors[1] - p_errors[1])
            pan = -pan/100
            tilt = -tilt/100
            print(pan, tilt)



            # fill ROS message
            goal = control_msgs.msg.FollowJointTrajectoryGoal()
            traj = trajectory_msgs.msg.JointTrajectory()
            traj.joint_names = ["head_pan_joint", "head_tilt_joint"]
            p = trajectory_msgs.msg.JointTrajectoryPoint()
            p.positions = [pan, tilt]
            p.velocities = [0, 0]
            p.time_from_start = rospy.Time(3)
            traj.points = [p]
            goal.trajectory = traj

            # send message to the action server
            cli.send_goal(goal)



        cv2.imshow("Face Detection", detection_img)
        
        # print(type(face_skin_img))
        cv2.waitKey(1)
        spin_rate.sleep()
        p_errors = errors
