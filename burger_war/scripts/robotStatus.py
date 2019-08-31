#!/usr/bin/env python
# -*- coding: utf-8 -*-
from enum import Enum
import rospy
import numpy as np

class M(Enum):
    NAVI = 1
    STOP = 2
    SEARCH = 3
    ROTATE = 4
    ATTACK = 5
    ESCAPE = 6
    STRAIGHT = 7
    NONE = 99


class STATUS():
    robot=0
    position = [0, 0]  # recent odom ([x,z])
    orient = 90
    vel = 0  # recent velocity (m/s)
    twist = [0, 0]  # recent twist ([x,z])
    targetorient = 0
    mode = 0
    basepoint = [[-1.1, 0, 0], [0, 1.1, 270], [1.1, 0, 180], [0, -1.1, 90]]
    # narrowpoint =  [[-0.7, 0.7, 45], [0, 1, 315], [0.7, 0.7, 315], [1, 0,
    #                                                             225], [0.7, -0.7, 225], [0, -1, 225], [-0.7, -0.7, 135], [-1, 0, 315]]

    narrowpoint=[[-0.7,0.7, 45], [0, 1.5, 315], [0.7, 0.7, 315], [1.5, 0, 225], [0.7, -0.7, 225],[0, -1.5, 225],[-0.7,0.7, 135], [-1.5, 0, 315]]
    distination_now = 1
    distination_num = 4
    nextmode = 0
    futuremode = 0
    flg_targetorient = False
    flg_isbusy = False
    flg_issafe = False
    flg_rotate = False
    flg_image = False
    flg_strotate = False
    flg_timer = False
    flg_target = 0
    flg_enemy = 0
    flg_direction = True
    flg_swing =False
    d_odm=0
    d_range= 0
    def __init__(self):
        pass
    speed=0.
    @staticmethod
    def setMode(mode):
        if mode == M.STOP:
            STATUS.flg_isbusy = False
        if(not STATUS.flg_isbusy):
            STATUS.mode = mode
            print('status -> ' + str(mode))
            if(STATUS.mode == M.NAVI):
                STATUS.robot.toGoal(*STATUS.basepoint[STATUS.distination_now])
            if(STATUS.mode == M.STOP):
                STATUS.robot.cancel.publish()
                STATUS.robot.stop()
            if(STATUS.mode == M.SEARCH):
                STATUS.robot.cancel.publish()
                STATUS.robot.stop()
            if(STATUS.mode == M.ATTACK):
                STATUS.robot.cancel.publish()
                STATUS.flg_direction = True
            if(STATUS.mode == M.STRAIGHT):
                STATUS.robot.cancel.publish()
                STATUS.robot.pubTwist(STATUS.speed,0)
                STATUS.flg_direction = True
                pass
            if(STATUS.mode == M.ESCAPE):
                STATUS.robot.cancel.publish()
                STATUS.robot.pubTwist(-STATUS.speed,0)
                STATUS.flg_direction = False
                pass
            
    
    @staticmethod
    def setModeInTime(mode,futuremode,time):
        if STATUS.nextmode!=M.ROTATE:
            STATUS.setMode(mode)
        else:
            STATUS.nextmode=mode
        STATUS.futuremode=futuremode
        STATUS.sleep(time,STATUS.setNextMode)

    @staticmethod
    def sleep(time, callback, oneshot=True):
        rospy.Timer(rospy.Duration(time), callback, oneshot)

    @staticmethod
    def setBusy(time):
        STATUS.flg_isbusy = True
        STATUS.sleep(time, STATUS.resetStatusFlg)

    @staticmethod
    def resetStatusFlg(data):
        STATUS.flg_isbusy = False
    
    @staticmethod
    def setNextMode(data):
        if STATUS.nextmode!=M.ROTATE:
            STATUS.setMode(STATUS.futuremode)
        else:
            STATUS.nextmode=STATUS.futuremode
            print(STATUS.nextmode)
