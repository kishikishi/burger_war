#!/usr/bin/env python
# -*- coding: utf-8 -*-
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


DISTINATION = [[-1, 0, 0], [-0.5, 0, 0]]
DISTINATION2 = [[0, 1, 0], [1, 0, 0], [0, -1, 0], [-1, 0, 0]]
DISTINATION_NOW = 0
DISTINATION_NUM = 2
DISTINATION_NUM2 = 4
FLG_GOAL = 0


class OnigiriRun(object):

    def __init__(self):
        # self.scan = rospy.Subscriber(
        #     '/red_bot/scan', LaserScan, self.LaserScanCallback, queue_size=1)
        self.result = rospy.Subscriber(
            '/red_bot/move_base/result', MoveBaseActionResult, self.goalcallback, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.slam = rospy.Publisher(
            '/red_bot/move_base_simple/goal', PoseStamped, queue_size=1)
        header = {'stamp': rospy.Time.now(), 'frame_id': "red_bot/map"}
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
        goal.header.frame_id = "red_bot/map"        # 世界座標系で指定する
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


if __name__ == '__main__':
    rospy.init_node('onigiri_run')
    a = OnigiriRun()
    while(1):
        a.togoal(*DISTINATION2[DISTINATION_NOW])
        print(FLG_GOAL)
        if(FLG_GOAL):
            FLG_GOAL = 0
            DISTINATION_NOW += 1
            print(DISTINATION_NOW)
            if(DISTINATION_NOW == DISTINATION_NUM2):
                DISTINATION_NOW = 0

    rospy.spin()
