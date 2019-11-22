#!/usr/bin/env python
# -*- coding: utf-8 -*

import rospy
import sys
import time
import math
import cv2
import PIL
import numpy as np
import actionlib
# ros msgs
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from darknet_ros_msgs.msg import BoundingBoxes
from object_recognizer.msg import ImageRange
# ros srvs
from object_recognizer.srv import RecognizeExistence
# action msgs
from manipulation.msg import *

class ObjectRecognizer:
    def __init__(self):
        #TopicSubscriber
        realsense_sub = rospy.Subscriber('/camera/color/image_raw',Image,self.ImageCB)
        bounding_box_sub  = rospy.Subscriber('/darknet_ros/bounding_boxes',BoundingBoxes,self.BoundingBoxCB)
        detector_sub = rospy.Subscriber('/object/xyz_centroid',Point,self.detectorCB)
        #TopicPublisher service化,モジュール化したい
        self.image_range_pub = rospy.Publisher('/object/image_range',ImageRange,queue_size=1)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel_mux/input/teleop',Twist,queue_size=1)
        #ServiceServer
        recog_service_server = rospy.Service('/object/recog',SetBool,self.recognizeObject)
        #ActionServer
        self.sas = actionlib.SimpleActionServer('/object/localize',
                                                ObjectRecognizerAction,
                                                execute_cb = self.localizeObject,
                                                auto_start = False)
        self.sas.register_preempt_callback(self.actionPreempt)
        
        self.bridge = CvBridge() # 仕様がわかり次第消す
        self.bbox = 'None'
        self.update_time = 0 # darknetからpublishされた時刻を保存
        self.update_flg = False # darknetからpublishされたかどうかの確認
        self.object_centroid = False
        self.preempt_flg = False

        self.sas.start()

    def ImageCB(self,img):
        self.full_image = img

    def BoundingBoxCB(self,bb):
        self.update_time = time.time()
        self.update_flg = True
        self.bbox = bb.boundingBoxes

    def detectorCB(self, res):
        self.object_centroid = res

    def recognizeObject(self, object_name='None'):
        if type(object_name) == type(String()):
            object_name = object_name.data
        bb = self.bbox
        object_list = []
        for i in range(len(bb)):
            object_list.append(bb[i].Class)
        object_existence = object_name in object_list
        return object_existence, target_list

    def actionPreempt(self):
        rospy.loginfo('preempt callback')
        self.sas.set_preempted(text = 'message for preempt')
        self.preempt_flg = True
        
    def localizeObject(self, goal):
        target = goal.recog_goal
        localize_feedback = ObjectRecognizerFeedback()
        localize_result = ObjectRecognizerResult()
        loop_flg = True
        while(loop_flg):
            object_existence, object_list = self.recognizeObject(target)
            range_flg = False
            # ここらへんをもう少し綺麗に書きたい
            if target == 'None' and not object_existence:# 適当に見えたものを掴むための処理
                list_num = 0
                range_flg = True
            elif object_existence:# 指定のものを掴むための処理
                list_num = object_list.index(target)
                range_flg = True
            if range_flg:
                object_image_range = ImageRange()
                object_image_range.top = bb[list_num].ymin
                object_image_range.bottom = bb[list_num].ymax
                object_image_range.left = bb[list_num].xmin
                object_image_range.right = bb[list_num].xmax
                rospy.sleep(0.2)
                self.image_range_pub.publish(object_image_range)
                while self.object_centroid == False and not rospy.is_shutdown():
                    pass
                object_coordinate = self.object_centroid
                self.object_centroid = False
                
                if not math.isnan(object_coordinate.x):# 物体が正面になるように回転する処理
                    object_coordinate.y += 0.08 # calibrate RealSenseCamera d435
                    object_angle = math.atan2(object_coordinate.y, object_coordinate.x)
                    if abs(object_angle) > 0.05:
                        rospy.loginfo('There is not object in front.')
                        cmd = Twist()
                        cmd.linear.x = 0
                        cmd.angular.z = obj_angle * 4.0 #要調整
                        if abs(cmd.angular.z) < 0.85:
                            cmd.angular.z = int(cmd.angular.z/abs(cmd.angular.z))*0.85
                        rospy.loginfo('cmd.angura.z : %s'%(obj_angle))
                        self.cmd_vel_pub.publish(cmd)
                        rospy.sleep(0.2)
                        # retry
                    else:
                        # success
                        loop_flg = False
                else:
                    range_flg = False
            if loop_flg:
                localize_feedback.recog_feedback = range_flg
                self.sas.publish_feedback(localize_feedback)
                range_flg = False
            if self.preempt_flg:
                self.preempt_flg = False
                break
        else:
            rospy.loginfo('Succeeded')
            localize_result.recog_result = self.object_centroid
            self.sas.set_succeeded(localize_result)

    def initializeObject(self):
        rate = rospy.Rate(3.0)
        while not rospy.is_shutdown():
            if time.time() - self.update_time > 1.5 and self.update_flg:
                self.bbox = 'None'
                self.update_flg = False
                rospy.loginfo('initialize') # test
            rate.sleep()
            
            
if __name__ == '__main__':
    rospy.init_node('object_recognizer')
    obj_recog = ObjectRecognizer()
    obj_recog.initializeObject()
    rospy.spin()
