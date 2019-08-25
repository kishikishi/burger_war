#!/usr/bin/env python
# -*- coding: utf-8 -*-
import random
from decimal import (Decimal, ROUND_DOWN)
import sys
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
from move_base_msgs.msg import MoveBaseActionResult
from tf.transformations import quaternion_from_euler
import math
import numpy as np
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from nav_msgs.msg import Odometry

state_x = -1
count = 0

DISTINATION = [[-1, 0, 0], [-0.5, 0, 0]]
DISTINATION2 = [[0, 1.5, 315], [1.5, 0, 270], [0, -1.5, 225], [-1.5, 0, 135]]
DISTINATION3 = [[-0.7,0.7, 45], [0, 1.5, 315], [0.7, 0.7, 315], [1.5, 0, 225], [0.7, -0.7, 225],[0, -1.5, 225],[-0.7,0.7, 135], [-1.5, 0, 315]]
DISTINATION4 = [[-0.8, 0.15, 50], [0, 0.5, 300], [-0.8, -0.15, 310], [0, -0.5, 60]]
DISTINATION_NOW = 0
DISTINATION_NUM = 2
DISTINATION_NUM2 = 4
DISTINATION_NUM3 = 8
DISTINATION_NUM4 = 4
FLG_GOAL = 0
FLG_STUCK = 0

mode = 1  # 0:wall running, 1:color attack

x_buf = 0
y_buf = 0
cnt = 0

class OnigiriRun(object):

    def __init__(self):
        # self.scan = rospy.Subscriber(
        #     '/red_bot/scan', LaserScan, self.LaserScanCallback, queue_size=1)
        # RESPECT @hotic06 ロボット名の取得の仕方
        robot_name=rospy.get_param('~robot_name')
        self.name = robot_name
        print(self.name)
        self.result = rospy.Subscriber(
            '/'+self.name  + '/move_base/result', MoveBaseActionResult, self.goalcallback, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.slam = rospy.Publisher(
            '/'+self.name + '/move_base_simple/goal', PoseStamped, queue_size=1)
        
        # velocity publisher
        self.vel_pub = rospy.Publisher('cmd_vel', Twist,queue_size=1)

        # odometry subscriber
        self.odom_sub = rospy.Subscriber("/" + self.name + "/odom",Odometry,self.odom_callback)

        header = {'stamp': rospy.Time.now(), 'frame_id': self.name +"/map"}
        pose = {'position': {'x': 1.0, 'y': 1.0,
                             'z': 0.0}, 'orientation': {'w': 1.0}}

    def togoal(self, x, y, radians):
        '''
        概要
        目的地を指定して移動する

    　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　引数
        x : 上下方向の座標 (-1<x<1)
        y : 左右方向の座標 (-1<y<1)
        radians : 目標姿勢。0度が上。半時計回りに増加する方向。 
        '''
        goal = PoseStamped()
        goal.header.frame_id = self.name + "/map" # 世界座標系で指定する
        # goal.header.frame_id = "red_bot/map" # 世界座標系で指定する
        goal.header.stamp = rospy.Time.now()  # タイムスタンプは今の時間
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = 0.0
        q = quaternion_from_euler(0, 0, math.radians(radians))
        goal.pose.orientation = Quaternion(*q)
        self.slam.publish(goal)

    def goalcallback(self, data):
        if(data.status.status == 3):
            global FLG_GOAL
            FLG_GOAL = 1

    # def LaserScanCallback(self, data):
    #     vel = Twist()
    #     vel.linear.x = 0.1
    #     self.range = data.ranges[0]
    #     if self.range > 0.5:
    #         vel.angular.z = 0.5+(self.range-0.1)*2
    #     elif self.range < 0.5:
    #         vel.linear.x = -0.1
    #         vel.angular.z = -0.5+(self.range-0.1)*2
    #     else:
    #         vel.angular.z = 0
    #     print(self.range)
    #     self.cmd_vel_pub.publish(vel)

    def checkImage(self):
        '''
        mode=1->color attack. 
        mode=0 ->wall running.
        '''
        global state_x
        global mode
        if state_x == -1:
            mode = 0
        else:
            mode = 1

    def calcTwist(self):
        global state_x
        global mode
        mode = 1
        if state_x >= 360:
            # print("right")
            x = 0
            th = -0.2
        elif state_x <= 280 and state_x >= 0:
            # print("left")
            x = 0
            th = 0.2
        elif 260 < state_x and state_x <360: # 対象が範囲内
            # print("center")
            x = 0.2
            th = 0
        elif state_x == -1:  # 対象が範囲内にないとき
            return

        twist = Twist()
        twist.linear.x = x; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th
        return twist


    def strategy(self):
        r = rospy.Rate(1) # change speed 1fps

        target_speed = 0
        target_turn = 0
        control_speed = 0
        control_turn = 0

        twist = self.calcTwist()
        # print(twist)
        self.vel_pub.publish(twist)

        r.sleep() 

    def odom_callback(self,odom):
        global x_buf
        global y_buf
        global cnt
        global FLG_STUCK

        # 小数点第４位以下を切り捨て
        pose_x = Decimal(odom.pose.pose.position.x).quantize(Decimal('0.001'), rounding=ROUND_DOWN)
        pose_y = Decimal(odom.pose.pose.position.y).quantize(Decimal('0.001'), rounding=ROUND_DOWN)

        if x_buf == pose_x and y_buf == pose_y:
            cnt += 1
        else:
            x_buf = pose_x
            y_buf = pose_y
            cnt = 0

        if cnt > 30:
            FLG_STUCK = 1

    # スタック脱出
    def collisionDetect(self):
        r = rospy.Rate(1) # change speed 1fps

        target_speed = 0
        target_turn = 0
        control_speed = 0
        control_turn = 0

        for i in range(2):
            value = random.randint(1,1000)
            if value < 250:
                x = -0.2
                th = 1
            elif value < 500:
                x = 0.2
                th = 1
            elif value < 750:
                x = -0.2
                th = 1
            elif value < 1000:
                x = 0.2
                th = -1
            else:
                x = 0
                th = 0
            
            twist = Twist()
            twist.linear.x = x; twist.linear.y = 0; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th
            self.vel_pub.publish(twist)
            r.sleep() 
        #self.vel_pub.publish(Twist())

class image_converter:
    def __init__(self):
        robot_name=rospy.get_param('~robot_name')
        self.name = robot_name
        self.image_pub = rospy.Publisher("image_topic", Image, queue_size=1)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/"+ self.name+"/image_raw",Image,self.callback)


    def callback(self,data):
        global state_x
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        cv2.imwrite("/home/lsic/デスクトップ/dst/1-test.png",cv_image)

        # RGB表色系からHSV表色系に変換                                                           
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        cv2.imwrite("/home/lsic/デスクトップ/dst/2-hsv.png",hsv_image)

        # 赤                                                 
        # color_min = np.array([150,100,150])
        # color_max = np.array([180,255,255])

        # 青
        # color_min = np.array([110, 50, 50])
        # color_max = np.array([130, 255, 255])

        # 緑
        color_min = np.array([30, 100, 200])
        color_max = np.array([60, 255, 250])
        
        #マスクの画像
        mask = cv2.inRange(hsv_image, color_min, color_max)
        res = cv2.bitwise_and(cv_image, cv_image, mask=mask)
        cv2.imwrite("/home/lsic/デスクトップ/dst/3-blue-mask.png",res)
 
        # グレースケール化（色抽出）
        img = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)

        mu = cv2.moments(img, False)
        if mu["m00"] != 0:
            state_x,y= int(mu["m10"]/mu["m00"]) , int(mu["m01"]/mu["m00"])
        else:
            state_x,y = -1,-1

        # print(state_x,y)

if __name__ == '__main__':
    rospy.init_node('onigiri_run')
    image   = image_converter()

    a = OnigiriRun()
    while not rospy.is_shutdown():
        a.checkImage()

        if(FLG_STUCK):
            print("STUCK!")
            FLG_STUCK = 0
            a.collisionDetect()
        elif mode:
            a.strategy()
        else:    
            a.togoal(*DISTINATION4[DISTINATION_NOW])
            # print(FLG_GOAL)
            if(FLG_GOAL):
                FLG_GOAL = 0
                DISTINATION_NOW += 1
                print(DISTINATION_NOW)
                if(DISTINATION_NOW == DISTINATION_NUM4):
                    DISTINATION_NOW = 0
    rospy.spin()
