#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys
import rospy
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
from move_base_msgs.msg import MoveBaseActionResult
from actionlib_msgs.msg import GoalID
from move_base_msgs.msg import MoveBaseActionFeedback
from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import math
import numpy as np
import cv2
import sys
import select
import termios
import tty
from robotStatus import *


class Trutlerun(object):

    def __init__(self, image):
        self.odom = rospy.Subscriber(
            'odom', Odometry, self.odomCallback, queue_size=1)
        self.scan = rospy.Subscriber(
            'scan', LaserScan, self.laserScanCallback, queue_size=3)
        self.result = rospy.Subscriber(
            'move_base/result', MoveBaseActionResult, self.goalCallback, queue_size=1)
        # self.result = rospy.Subscriber(
        #     'war_state', String, self.warstatecallback, queue_size=1)
        self.cmd_vel_pub = rospy.Subscriber(
            'cmd_vel', Twist, self.speedCallback, queue_size=1)

        self.slam = rospy.Publisher(
            'move_base_simple/goal', PoseStamped, queue_size=3)
        self.cancel = rospy.Publisher(
            'move_base/cancel', GoalID, queue_size=1)
        self.vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=3)
        self.image = image
        self.cflg_odom = False
        self.cflg_laser = False
        self.cflg_goal = False
        self.imgcount = 0

    def warStateCallback(self, data):
        print(data)

    def odomCallback(self, data):
        STATUS.d_odm = data
        self.cflg_odom = True

    def speedCallback(self, data):
        STATUS.twist[0] = data.linear.x
        STATUS.twist[1] = data.angular.z
        print(STATUS.twist)

    def goalCallback(self, data):
        if(data.status.status == 3 and STATUS.flg_timer == 0):
            self.cflg_goal = 1

    def laserScanCallback(self, data):
        self.cflg_laser = True
        STATUS.d_range = self.forspline(np.array(data.ranges))

    def updateStatus(self):
        if(self.cflg_odom):
            self.cflg_odom = False
            STATUS.position[1] = STATUS.position[0]
            STATUS.position[0] = np.array(
                [STATUS.d_odm.pose.pose.position.x, STATUS.d_odm.pose.pose.position.z])
            #vel = STATUS.position[0]-STATUS.position[1]
            #STATUS.vel = np.linalg.norm(vel, ord=2)*30
            STATUS.orient = np.rad2deg(euler_from_quaternion(
                (STATUS.d_odm.pose.pose.orientation.x, STATUS.d_odm.pose.pose.orientation.y, STATUS.d_odm.pose.pose.orientation.z, STATUS.d_odm.pose.pose.orientation.w))[2])
            if (STATUS.orient < 0):
                STATUS.orient = 360+STATUS.orient
            

        if(self.cflg_laser):
            self.cflg_laser = False
            # update direction
            if(STATUS.mode == M.NAVI):
                STATUS.flg_direction = STATUS.twist[0] >= 0
            STATUS.flg_issafe = self.checkSafeRoute()

            if(STATUS.flg_issafe):
                if(STATUS.flg_strotate):
                    print('stop0')
                    STATUS.flg_strotate = False
                    STATUS.setModeInTime(M.STOP,STATUS.nextmode,0.1)
            else:
                if(STATUS.mode == M.NAVI and STATUS.twist[0]<0.05):
                    STATUS.setMode(M.STOP)
                    self.inNaviAvoid(M.NAVI)
                if(STATUS.mode == M.STRAIGHT):
                    print('stop1')
                    STATUS.setMode(M.STOP)
                    self.avoidobstacle(M.STRAIGHT)
                if(STATUS.mode == M.ATTACK):
                    STATUS.setMode(M.STOP)                    
                    self.avoidobstacle(M.STRAIGHT)                    
                if(STATUS.mode == M.ESCAPE):
                    print('stop2')
                    STATUS.setMode(M.STOP)
                    rospy.Timer(rospy.Duration(5),self.setStatusStraight, oneshot=True)
                    self.avoidobstacle(M.ESCAPE)                

        if(STATUS.mode == M.ROTATE):
            if(STATUS.twist[1]==0):
                if(STATUS.flg_rotate==1):
                    self.pubTwist(0,0.8)
                if(STATUS.flg_rotate==-1):
                    self.pubTwist(0,-0.8)
            if(STATUS.flg_rotate != 0):
                myrobot.checkrotation()

        if(self.cflg_goal):
            self.cflg_goal = False
            STATUS.distination_now += 1
            print("Distination: " + str(STATUS.distination_now))
            if(STATUS.distination_now == 4):
                STATUS.distination_now = 0
            self.toGoal(*STATUS.basepoint[STATUS.distination_now])
            STATUS.flg_timer = True
            # rospy.Timer(rospy.Duration(10),
            #             myrobot.changeDestination, oneshot=True)
            rospy.Timer(rospy.Duration(
                1), self.resetTimer, oneshot=True)

        if(STATUS.flg_image):
            STATUS.flg_image = 0
            r,g = image.checkimage()
            if(STATUS.flg_target > 0 and STATUS.mode!=M.ATTACK):
                STATUS.flg_swing=False            
                STATUS.flg_target = 0
                STATUS.setMode(M.ATTACK)
            elif(STATUS.mode == M.ATTACK):
                if(STATUS.flg_target==2):
                    STATUS.flg_target = -1
                    STATUS.setModeInTime(M.ESCAPE,M.STRAIGHT,10)
                    STATUS.setBusy(5)
                if(STATUS.flg_target==1):
                    STATUS.flg_target = -1
                    z=(g-320)/160
                    self.pubTwist(STATUS.speed,-z)
                if(STATUS.flg_target==0):
                    STATUS.setModeInTime(M.ESCAPE,M.STRAIGHT,10)
                    STATUS.setBusy(5)

            if(STATUS.mode == M.SEARCH):
                if(STATUS.flg_enemy):
                    z=(r-320)/160
                    self.pubTwist(0,-r)
                else:
                    STATUS.setModeInTime(M.ESCAPE,M.STRAIGHT,5)
    
    def Swing(self,event):
        print('testttttttttttttttttttttttttt')
        if (STATUS.mode==M.STRAIGHT):
            target = 90 if STATUS.d_range[90]>STATUS.d_range[270] else -90
            self.rotation(target,M.STRAIGHT)
            #STATUS.flg_swing=True

    def toGoal(self, x, y, radians):
        '''
        概要
        目的地を指定して移動する
        x : 上下方向の座標 (-1<x<1)
        y : 左右方向の座標 (-1<y<1)
        radians : 目標姿勢。0度が上。半時計回りに増加する方向。 
        '''
        goal = PoseStamped()
        goal.header.frame_id = "map"        # 世界座標系で指定する
        goal.header.stamp = rospy.Time.now()  # タイムスタンプは今の時間
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = 0.0
        q = quaternion_from_euler(0, 0, math.radians(radians))
        goal.pose.orientation = Quaternion(*q)
        print('呼び出し')
        self.slam.publish(goal)

    def checkSafeRoute(self, distance=0.22, width=0.14):
        th = np.linalg.norm([distance, width], ord=2)
        angle = int(np.rad2deg(np.arctan2(width, distance)))
        if(STATUS.flg_direction):
            checkresult = (np.all(STATUS.d_range[0:angle] > th)
                           and np.all(STATUS.d_range[-angle:360] > th))
        else:
            checkresult = np.all(STATUS.d_range[180-angle:180+angle] > th)
        return checkresult

    def rotation(self, angle, mode):
        if(STATUS.flg_isbusy):
            return
        STATUS.setMode(M.ROTATE)
        STATUS.targetorient = STATUS.orient+angle
        STATUS.nextmode = mode
        if(STATUS.targetorient > 359):
            STATUS.targetorient = STATUS.targetorient-360
            STATUS.flg_targetorient = 1

        if (STATUS.targetorient < 0):
            STATUS.targetorient = 360+STATUS.targetorient
            STATUS.flg_targetorient = 1

        if(angle > 0):
            if (STATUS.targetorient > 355):
                STATUS.targetorient = 355
            z = 0.8
            STATUS.flg_rotate = 1

        if(angle < 0):
            if (STATUS.targetorient < 5):
                STATUS.targetorient = 5
            z = - 0.8
            STATUS.flg_rotate = -1
        self.pubTwist(0, z)

    def checkrotation(self):
        if(STATUS.flg_rotate == 1):
            if (STATUS.flg_targetorient):
                if (abs(STATUS.orient-STATUS.targetorient) < 5):
                    STATUS.flg_targetorient = 0
            elif(STATUS.orient > STATUS.targetorient):
                STATUS.setMode(M.STOP)
                if(STATUS.flg_direction==False):
                    target=180 if STATUS.twist[0]<0 else -180
                    self.rotation(target,STATUS.nextmode)
                    return
                if(STATUS.flg_swing):
                    STATUS.flg_swing=False
                    self.rotation(-90,STATUS.nextmode)
                    return
                print('stop3')
                STATUS.flg_rotate = 0
                STATUS.setMode(STATUS.nextmode)
                STATUS.nextmode=M.NONE
                #STATUS.setBusy(0.5)
        if(STATUS.flg_rotate == -1):
            if (STATUS.flg_targetorient):
                if (abs(STATUS.orient-STATUS.targetorient) < 5):
                    STATUS.flg_targetorient = 0
            elif(STATUS.orient < STATUS.targetorient):
                STATUS.setMode(M.STOP)
                if(STATUS.flg_direction==False):
                    target=180 if STATUS.twist[0]<0 else -180
                    self.rotation(target,STATUS.nextmode)
                    return
                
                print('stop4')
                STATUS.flg_rotate = 0
                STATUS.setMode(STATUS.nextmode)
                STATUS.nextmode=M.NONE
                #STATUS.setBusy(0.5)
    
    def inNaviAvoid(self, nextmode):
        target = 180 if STATUS.twist[1]>0 else -180
        STATUS.flg_strotate = True
        self.rotation(target, nextmode)
    
    def avoidobstacle(self, nextmode):
        emp = 27
        rg = 80
        STATUS.d_range[0:emp] = STATUS.d_range[0:emp]*10
        STATUS.d_range[-emp:360] = STATUS.d_range[-emp:360]*10
        STATUS.d_range[180-emp:180+emp] = STATUS.d_range[180-emp:180+emp]*10
        if(STATUS.flg_direction):
            target = 180 if np.mean(STATUS.d_range[0:rg]) > np.mean(
                STATUS.d_range[-rg:360]) else -180
        else:
            target = -180 if (np.mean(STATUS.d_range[180-rg:180]) >
                             np.mean(STATUS.d_range[180:180+rg])) else 180
        STATUS.flg_strotate = True
        if(STATUS.mode==M.ESCAPE):
            target*=0.3
        self.rotation(target, nextmode)

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


        # ----------------- util funcs --------------#
    
    def forspline(self,data):
        minix=np.where(data>0)[0][0]
        l=len(data)-minix
        for i in range(l):
            if (data[i+minix]==0):
                data[i+minix]=data[i+minix-1]
        for i in range(minix):
            data[i]=data[-1]  
        return data

    def resetTimer(self, event):
        STATUS.flg_timer = False

    def changeDestination(self, event):
        if(STATUS.flg_timer == False):
            self.cflg_goal = 1
    def setStatusStraight(self,event):
        STATUS.setMode(M.STRAIGHT)

    def pubTwist(self, x, z):
        vel = Twist()
        vel.linear.x = x
        vel.linear.y = 0
        vel.linear.z = 0
        vel.angular.x = 0
        vel.angular.y = 0
        vel.angular.z = z
        self.vel_pub.publish(vel)

    def stop(self):
        self.pubTwist(0, 0)
    





class image_converter:
    def __init__(self):
        self.image_pub = rospy.Publisher("image_topic", Image, queue_size=1)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(
            "image_raw", Image, self.callback)
        self.image = 0

    def callback(self, data):
        self.image = data
        STATUS.flg_image = True

    def checkimage(self):
        ng=0
        rg=0
        STATUS.flg_enemy=False

        try:
            cv_image = self.bridge.imgmsg_to_cv2(self.image, "bgr8")
        except CvBridgeError as e:
            print(e)
        # RGB表色系からHSV表色系に変換
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        # 赤
#         color_min = np.array([0, 50, 50])
#         color_max = np.array([10, 255, 255])
        color_min = np.array([150,100,150])
        color_max = np.array([180,255,255])

        # マスクの画像
        mask = cv2.inRange(hsv_image, color_min, color_max)
        res = cv2.bitwise_and(cv_image, cv_image, mask=mask)
        # グレースケール化（色抽出）
        img = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)
        mu = cv2.moments(img, False)
        cv2.imwrite("/home/nryk/Desktop/dst/2-hsv.png", img)
        if(len(np.where(img>0)[0])>100):
            STATUS.flg_enemy=True

        if mu["m00"] != 0:
            state_rx, ry = int(mu["m10"]/mu["m00"]), int(mu["m01"]/mu["m00"])
        else:
            state_rx, ry = -1, -1
            
        # 青
        # color_min = np.array([110, 50, 50])
        # color_max = np.array([130, 255, 255])
        # 緑
        color_min = np.array([40, 50, 50])
        color_max = np.array([60, 255, 250])
        # マスクの画像
        mask = cv2.inRange(hsv_image, color_min, color_max)
        res = cv2.bitwise_and(cv_image, cv_image, mask=mask)
        # グレースケール化（色抽出）
        img = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)
        mu = cv2.moments(img, False)
        if mu["m00"] != 0:
            state_gx, gy = int(mu["m10"]/mu["m00"]), int(mu["m01"]/mu["m00"])
            # print('green')
            ng=len(np.where(img>0)[0])
        else:
            state_gx, gy = -1, -1
    
        if(ng>3000):
            STATUS.flg_target = 1
        if(ng>30000):
            STATUS.flg_target = 2

        
        # z = (state_x-320)/160
        
        return state_rx,state_gx


if __name__ == '__main__':
    rospy.init_node('onigiri_run')
    image = image_converter()
    myrobot = Trutlerun(image)
    STATUS.robot=myrobot
    rospy.timer.sleep(2)
    r=rospy.Rate(100)
    STATUS.setMode(M.STRAIGHT)
    #myrobot.rotation(45, M.STRAIGHT)
    #STATUS.setModeInTime(M.STRAIGHT,M.ESCAPE,10)
    rospy.Timer(rospy.Duration(10),myrobot.Swing)

    while not rospy.is_shutdown():
        myrobot.updateStatus()
        r.sleep()
