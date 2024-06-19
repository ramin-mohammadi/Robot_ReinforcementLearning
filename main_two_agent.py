# from xarm.wrapper import XArmAPI
# arm = XArmAPI('192.168.1.207')

# code = XArmAPI.set_servo_angle(angle=[93.8, -58.8, 5.2, 53.6, 185.3],  wait=True, radius=-1.0)


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

from env_package import obj_det_xyz_angle
import numpy as np


class RobotBigGripper(object):
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

    # Robot Main Run
    def run(self):
        try:
            self._tcp_speed = 71
            self._tcp_acc = 1000
            self._angle_speed = 30
            self._angle_acc = 200
            code = self._arm.set_tcp_load(0.82, [0, 0, 48])
            
            print("test")
            print(self._arm.get_position()[1][0:3]) # how to get robot's xyz
            print(self._arm.get_position()[1]) # [x,y,z,roll,pitch,yaw]
            print(self._arm.get_servo_angle()[1]) # pay attention to only 5 first values bc xarm5 only has 5 joints

            # return
        
        
            if not self._check_code(code, 'set_tcp_load'):
                return
            code = self._arm.set_gripper_position(50, wait=True, speed=2500, auto_enable=True)
            if not self._check_code(code, 'set_gripper_position'):
                return
            code = self._arm.set_servo_angle(angle=[93.8, -58.8, 5.2, 53.6, 185.3], speed=self._angle_speed, mvacc=self._angle_acc, wait=True, radius=-1.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
            code = self._arm.set_servo_angle(angle=[89.0, -32.4, -17.4, 49.8, 180.5], speed=self._angle_speed, mvacc=self._angle_acc, wait=False, radius=-1.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
            code = self._arm.set_gripper_position(376, wait=True, speed=2500, auto_enable=True)
            if not self._check_code(code, 'set_gripper_position'):
                return
            code = self._arm.set_position(*[1.9, 281.3, 142.9, 180.0, 0.0, -91.5], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
            code = self._arm.set_pause_time(1)
            if not self._check_code(code, 'set_pause_time'):
                return
            code = self._arm.set_gripper_position(285, wait=True, speed=5000, auto_enable=True)
            if not self._check_code(code, 'set_gripper_position'):
                return
            code = self._arm.set_pause_time(1)
            if not self._check_code(code, 'set_pause_time'):
                return
            code = self._arm.set_position(*[1.9, 281.3, 254.7, 180.0, 0.0, -91.5], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
            code = self._arm.set_position(*[211.3, 112.4, 238.1, 180.0, 0.0, -91.5], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
            code = self._arm.set_position(*[214.3, 112.4, 217.4, 180.0, 0.0, -91.5], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
            code = self._arm.set_pause_time(1)
            if not self._check_code(code, 'set_pause_time'):
                return
            code = self._arm.set_gripper_position(448, wait=True, speed=2500, auto_enable=True)
            if not self._check_code(code, 'set_gripper_position'):
                return
            code = self._arm.set_position(*[213.3, 112.4, 281.2, 180.0, 0.0, -91.5], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
            code = self._arm.set_position(*[-0.5, 352.4, 250.2, 180.0, 0.0, -91.5], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
            code = self._arm.set_position(*[-0.5, 352.4, 141.2, 180.0, 0.0, -91.5], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
            code = self._arm.set_pause_time(1)
            if not self._check_code(code, 'set_pause_time'):
                return
            code = self._arm.set_gripper_position(285, wait=True, speed=5000, auto_enable=True)
            if not self._check_code(code, 'set_gripper_position'):
                return
            code = self._arm.set_pause_time(1)
            if not self._check_code(code, 'set_pause_time'):
                return
            code = self._arm.set_position(*[-0.5, 352.4, 238.5, 180.0, 0.0, -91.5], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
            code = self._arm.set_position(*[214.8, 112.6, 249.3, 180.0, 0.0, -91.5], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
            code = self._arm.set_position(*[215.8, 112.6, 236.3, 180.0, 0.0, -91.5], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
            code = self._arm.set_pause_time(1)
            if not self._check_code(code, 'set_pause_time'):
                return
            code = self._arm.set_gripper_position(459, wait=True, speed=5000, auto_enable=True)
            if not self._check_code(code, 'set_gripper_position'):
                return
            code = self._arm.set_position(*[214.8, 112.6, 265.4, 180.0, 0.0, -91.5], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
            code = self._arm.set_gripper_position(44, wait=True, speed=5000, auto_enable=True)
            if not self._check_code(code, 'set_gripper_position'):
                return
            code = self._arm.set_pause_time(1)
            if not self._check_code(code, 'set_pause_time'):
                return
            code = self._arm.set_position(*[205.0, 112.6, 252.0, 180.0, 0.0, -91.5], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
            code = self._arm.set_position(*[205.0, 112.6, 256.0, 180.0, 0.0, -91.5], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
            code = self._arm.set_position(*[219.0, 112.6, 257.0, 180.0, 0.0, -91.5], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
            code = self._arm.set_pause_time(1)
            if not self._check_code(code, 'set_pause_time'):
                return
            code = self._arm.set_position(*[219.0, 112.6, 251.0, 180.0, 0.0, -91.5], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
            code = self._arm.set_position(*[219.0, 112.6, 258.0, 180.0, 0.0, -91.5], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
            code = self._arm.set_pause_time(1)
            if not self._check_code(code, 'set_pause_time'):
                return
            code = self._arm.set_servo_angle(angle=[28.1, -38.6, -9.9, 48.5, 31.4], speed=self._angle_speed, mvacc=self._angle_acc, wait=True, radius=-1.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
            code = self._arm.set_pause_time(1)
            if not self._check_code(code, 'set_pause_time'):
                return
            code = self._arm.set_position(*[219.0, 117.2, 252.0, 180.0, 0.0, -3.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
            code = self._arm.set_pause_time(1)
            if not self._check_code(code, 'set_pause_time'):
                return
            code = self._arm.set_position(*[219.0, 117.2, 271.2, 180.0, 0.0, -3.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
            code = self._arm.set_position(*[211.0, 113.6, 268.7, 180.0, 0.0, -91.5], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1.0, wait=False)
            if not self._check_code(code, 'set_position'):
                return
            code = self._arm.set_servo_angle(angle=[93.8, -58.8, 5.2, 53.6, 185.3], speed=self._angle_speed, mvacc=self._angle_acc, wait=True, radius=-1.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
        except Exception as e:
            self.pprint('MainException: {}'.format(e))
        self.alive = False
        self._arm.release_error_warn_changed_callback(self._error_warn_changed_callback)
        self._arm.release_state_changed_callback(self._state_changed_callback)
        if hasattr(self._arm, 'release_count_changed_callback'):
            self._arm.release_count_changed_callback(self._count_changed_callback)

    def move_initial(self):
        try:
            self._tcp_speed = 71
            self._tcp_acc = 1000
            self._angle_speed = 30
            self._angle_acc = 200
            code = self._arm.set_tcp_load(0.82, [0, 0, 48])
            if not self._check_code(code, 'set_tcp_load'):
                return
            code = self._arm.set_position(*[-12.1, 181.6, 222.3, 180.0, 0.0, -91.5], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
                
            code = self._arm.set_servo_angle(angle=[93.8, -58.8, 5.2, 53.6, 185.3], speed=self._angle_speed, mvacc=self._angle_acc, wait=True, radius=-1.0)
            if not self._check_code(code, 'set_servo_angle'):
                return    
            
        except Exception as e:
            self.pprint('MainException: {}'.format(e))
        # self.alive = False
        # self._arm.release_error_warn_changed_callback(self._error_warn_changed_callback)
        # self._arm.release_state_changed_callback(self._state_changed_callback)
        # if hasattr(self._arm, 'release_count_changed_callback'):
        #     self._arm.release_count_changed_callback(self._count_changed_callback)
        
    def open_gripper(self):
        try:
            code = self._arm.set_gripper_position(850, wait=True, speed=2500, auto_enable=True)
            if not self._check_code(code, 'set_gripper_position'):
                return
        
            code = self._arm.set_pause_time(1)
            if not self._check_code(code, 'set_pause_time'):
                return
        
        except Exception as e:
            self.pprint('MainException: {}'.format(e))
            
    def close_gripper(self):
        try:
            code = self._arm.set_gripper_position(297, wait=True, speed=2500, auto_enable=True)
            if not self._check_code(code, 'set_gripper_position'):
                return
            
            code = self._arm.set_pause_time(1)
            if not self._check_code(code, 'set_pause_time'):
                return
        
        except Exception as e:
            self.pprint('MainException: {}'.format(e))
        
    # big gripper will place object on conveyor belt
    def two_agent_first_task(self, _target_position, _destination_position, yaw_rotation):
        try:
            self._tcp_speed = 71
            self._tcp_acc = 1000
            self._angle_speed = 30
            self._angle_acc = 200
            code = self._arm.set_tcp_load(0.82, [0, 0, 48])
            if not self._check_code(code, 'set_tcp_load'):
                return
            
            # move to block
            code = self._arm.set_position(*[_target_position[0], _target_position[1], _target_position[2], 180.0, 0.0, yaw_rotation], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
            
            code = self._arm.set_pause_time(1)
            if not self._check_code(code, 'set_pause_time'):
                return
            
            self.close_gripper()
            
            # move block to conveyor belt ( ideally should be green platform)
            code = self._arm.set_position(*[_destination_position[0], _destination_position[1], _destination_position[2], 180.0, 0.0, -91.5], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
            
            self.open_gripper()
            
            
        

        except Exception as e:
            self.pprint('MainException: {}'.format(e))
            # self.move_initial()
            
    
    
    
def get_xy_rotation_block():
    red_block, blue_block = obj_det_xyz_angle.get_pos_angle()
    if red_block.is_detected:
        return np.array([red_block.x, red_block.y, red_block.rotation])
    elif blue_block.is_detected:
        return np.array([blue_block.x, blue_block.y, blue_block.rotation])
    else:
        print("\nERROR: Object not detected\n")
        exit(-1)
        
        
        
    
        
        
        
class RobotSuction(object):
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
        
    def two_agent_second_task(self):
        try:
            code = self._arm.set_suction_cup(False, wait=False, delay_sec=0)
            if not self._check_code(code, 'set_suction_cup'):
                return
            self._tcp_speed = 50
            self._tcp_acc = 2000
            self._angle_speed = 10
            self._angle_acc = 500
            code = self._arm.set_position(*[305.8, -6.1, 179.5, 174.7, -4.5, -35.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
            if not self._check_code(code, 'set_position'):
                return
            code = self._arm.set_position(*[301.3, 192.4, 44.7, 174.7, -4.5, -35.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
            if not self._check_code(code, 'set_position'):
                return
            code = self._arm.set_servo_angle(angle=[8.7, 11.7, 11.7, 83.7, -5.0, -47.8], speed=self._angle_speed, mvacc=self._angle_acc, wait=False, radius=0.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
            code = self._arm.set_position(*[140.0, -60.0, 200.0, 174.7, -4.5, -35.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
            if not self._check_code(code, 'set_position'):
                return
            code = self._arm.set_position(*[140.0, -60.0, 50.0, 174.7, -4.5, -35.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
            if not self._check_code(code, 'set_position'):
                return
            code = self._arm.set_pause_time(1)
            if not self._check_code(code, 'set_pause_time'):
                return
            code = self._arm.set_suction_cup(True, wait=True, delay_sec=0)
            if not self._check_code(code, 'set_suction_cup'):
                return
            if self._arm.arm.check_air_pump_state(1, timeout=10.0):
                code = self._arm.set_pause_time(1)
                if not self._check_code(code, 'set_pause_time'):
                    return
                code = self._arm.set_position(*[140.0, -60.0, 200.0, 174.7, -4.5, -35.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
                if not self._check_code(code, 'set_position'):
                    return
                code = self._arm.set_position(*[408.6, -60.0, 200.0, 174.7, -4.5, -35.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
                if not self._check_code(code, 'set_position'):
                    return
                code = self._arm.set_position(*[408.6, -60.0, -10.0, 174.7, -4.5, -35.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
                if not self._check_code(code, 'set_position'):
                    return
                code = self._arm.set_pause_time(1)
                if not self._check_code(code, 'set_pause_time'):
                    return
                code = self._arm.set_suction_cup(False, wait=False, delay_sec=0)
                if not self._check_code(code, 'set_suction_cup'):
                    return
                code = self._arm.set_pause_time(1)
                if not self._check_code(code, 'set_pause_time'):
                    return
                code = self._arm.set_position(*[408.6, -60.0, 200.0, 174.7, -4.5, -35.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
                if not self._check_code(code, 'set_position'):
                    return
                code = self._arm.set_position(*[305.8, -6.1, 179.5, 174.7, -4.5, -35.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
                if not self._check_code(code, 'set_position'):
                    return
                code = self._arm.set_position(*[301.3, 192.4, 44.7, 174.7, -4.5, -35.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
                if not self._check_code(code, 'set_position'):
                    return
        except Exception as e:
            self.pprint('MainException: {}'.format(e))
                
           

if __name__ == '__main__':
    RobotBigGripper.pprint('xArm-Python-SDK Version:{}'.format(version.__version__))
    arm_one = XArmAPI('192.168.1.207', baud_checkset=False)
    # robot_main = RobotMain(arm)
    # robot_main.move_initial()
    # robot_main.run()
    first_agent = RobotBigGripper(arm_one)
    first_agent.move_initial()
    first_agent.open_gripper()
    
    yaw_rotation = -91.5
    _target_position = np.array([1.9,281.3,144])
    _destination_position = np.array([211, 113.6, 268.7])
    
    # connect to camera that performs object detection (detects red or blue) to get x,y, and yaw degree rotation
    camera_info = get_xy_rotation_block() # returns [x,y, yaw degree rotation]
    _target_position[0:2] = camera_info[0:2] # x y
    yaw_rotation = camera_info[2]
    first_agent.two_agent_first_task(_target_position, _destination_position, yaw_rotation) # move red block
    
    camera_info = get_xy_rotation_block() # returns [x,y, yaw degree rotation]
    _target_position[0:2] = camera_info[0:2] # x y
    yaw_rotation = camera_info[2]
    first_agent.two_agent_first_task(_target_position, _destination_position, yaw_rotation) # move blue block
    
    
    
    arm_two = XArmAPI('192.168.1.165', baud_checkset=False)
    second_agent = RobotSuction(arm_two)
    second_agent.two_agent_second_task()