#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2022, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

"""
# Notice
#   1. Changes to this file on Studio will not be preserved
#   2. The next conversion will overwrite the file with the same name
# 
# xArm-Python-SDK: https://github.com/xArm-Developer/xArm-Python-SDK
#   1. git clone git@github.com:xArm-Developer/xArm-Python-SDK.git
#   2. cd xArm-Python-SDK
#   3. python setup.py install
"""
import sys
import math
import time
import queue
import datetime
import random
import traceback
import threading
from xarm import version
from xarm.wrapper import XArmAPI


class RobotMain(object):
    """Robot Main Class"""
    def __init__(self, robot, **kwargs):
        self.alive = True
        self._arm = robot
        self._tcp_speed = 100
        self._tcp_acc = 2000
        self._angle_speed = 20
        self._angle_acc = 500
        self._variables = {}
        self._robot_init()

    # Robot init
    def _robot_init(self):
        self._arm.clean_warn()
        self._arm.clean_error()
        self._arm.motion_enable(True)
        self._arm.set_mode(0)
        self._arm.set_state(0)
        time.sleep(1)
        self._arm.register_error_warn_changed_callback(self._error_warn_changed_callback)
        self._arm.register_state_changed_callback(self._state_changed_callback)
        if hasattr(self._arm, 'register_count_changed_callback'):
            self._arm.register_count_changed_callback(self._count_changed_callback)

    # Register error/warn changed callback
    def _error_warn_changed_callback(self, data):
        if data and data['error_code'] != 0:
            self.alive = False
            self.pprint('err={}, quit'.format(data['error_code']))
            self._arm.release_error_warn_changed_callback(self._error_warn_changed_callback)

    # Register state changed callback
    def _state_changed_callback(self, data):
        if data and data['state'] == 4:
            self.alive = False
            self.pprint('state=4, quit')
            self._arm.release_state_changed_callback(self._state_changed_callback)

    # Register count changed callback
    def _count_changed_callback(self, data):
        if self.is_alive:
            self.pprint('counter val: {}'.format(data['count']))

    def _check_code(self, code, label):
        if not self.is_alive or code != 0:
            self.alive = False
            ret1 = self._arm.get_state()
            ret2 = self._arm.get_err_warn_code()
            self.pprint('{}, code={}, connected={}, state={}, error={}, ret1={}. ret2={}'.format(label, code, self._arm.connected, self._arm.state, self._arm.error_code, ret1, ret2))
        return self.is_alive

    @staticmethod
    def pprint(*args, **kwargs):
        try:
            stack_tuple = traceback.extract_stack(limit=2)[0]
            print('[{}][{}] {}'.format(time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(time.time())), stack_tuple[1], ' '.join(map(str, args))))
        except:
            print(*args, **kwargs)

    @property
    def is_alive(self):
        if self.alive and self._arm.connected and self._arm.error_code == 0:
            if self._arm.state == 5:
                cnt = 0
                while self._arm.state == 5 and cnt < 5:
                    cnt += 1
                    time.sleep(0.1)
            return self._arm.state < 4
        else:
            return False

    def draw_point(self, point, angle):
        if point == '1':
            lowX, lowY = -40, 180
        elif point == '2':
            lowX, lowY = 30, 180
        elif point == '3':
            lowX, lowY = 30, 390
        elif point == '4':
            lowX, lowY = -40, 390
        elif point == '5':
            lowX, lowY = -5, 295
        elif point == 'done':
            exit()
        else:
            print("Please enter 1 - 5")
            return
        highX, highY = lowX + 20, lowY + 20

        code = self._arm.set_position(*[lowX, lowY, 222.3, 180.0, 0.0, angle], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1.0, wait=True)
        if not self._check_code(code, 'set_position'):
            return
        input("Press Enter for next corner")
        code = self._arm.set_position(*[highX, lowY, 222.3, 180.0, 0.0, angle], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1.0, wait=True)
        if not self._check_code(code, 'set_position'):
            return
        input("Press Enter for next corner")
        code = self._arm.set_position(*[highX, highY, 222.3, 180.0, 0.0, angle], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1.0, wait=True)
        if not self._check_code(code, 'set_position'):
            return
        input("Press Enter for next corner")
        code = self._arm.set_position(*[lowX, highY, 222.3, 180.0, 0.0, angle], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1.0, wait=True)
        if not self._check_code(code, 'set_position'):
            return
        input("Press Enter for next corner")
        code = self._arm.set_position(*[lowX, lowY, 222.3, 180.0, 0.0, angle], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1.0, wait=True)
        if not self._check_code(code, 'set_position'):
            return
        print(f"Point {point} done")

    # Robot Main Run
    def run(self):
        try:
            self._tcp_speed = 83
            self._tcp_acc = 1000
            self._angle_speed = 30
            self._angle_acc = 200

            gripper_angle = 43.5
                #-91.5, -46.5, -1.5, 43.5, 88.5

            # INITIAL POSITION
            code = self._arm.set_position(*[-12.1, 181.6, 222.3, 180.0, 0, gripper_angle], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
            code = self._arm.set_gripper_position(85, wait=True, speed=5000, auto_enable=True)
            if not self._check_code(code, 'set_gripper_position'):
                return
            
            print("===At initial position===")

            while True:
                point = input("Point 1, 2, 3, 4, 5, or \'done\':\n")
                self.draw_point(point, gripper_angle)
            
        except Exception as e:
            self.pprint('MainException: {}'.format(e))
        self.alive = False
        self._arm.release_error_warn_changed_callback(self._error_warn_changed_callback)
        self._arm.release_state_changed_callback(self._state_changed_callback)
        if hasattr(self._arm, 'release_count_changed_callback'):
            self._arm.release_count_changed_callback(self._count_changed_callback)


if __name__ == '__main__':
    RobotMain.pprint('xArm-Python-SDK Version:{}'.format(version.__version__))
    arm = XArmAPI('192.168.1.207', baud_checkset=False)
    robot_main = RobotMain(arm)
    robot_main.run()
