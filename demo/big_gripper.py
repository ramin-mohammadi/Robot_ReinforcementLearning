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

import numpy as np


class BigGripper(object):
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
        
        
        

    def assemble(self, xy_blocks: np.ndarray, rotation_blocks: np.ndarray, n_blocks: int, height_block: float):
        try:
            self._tcp_speed = 71
            self._tcp_acc = 1000
            self._angle_speed = 30
            self._angle_acc = 200
            code = self._arm.set_tcp_load(0.82, [0, 0, 48])
            if not self._check_code(code, 'set_tcp_load'):
                return
            
            # move to initial position
            code = self._arm.set_position(*[-12.1, 181.6, 222.3, 180.0, 0.0, -91.5], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
            
            """
            Loop:
            - open gripper
            - move to block (x y given by camera)
            - close gripper
            - move block above where it is being put down (initial height + ith block * height_block)
            - move block down onto the below platform or block (initial height + ith block * height_block)
            - open gripper
            - move arm above block
            - close gripper
            - push gripper down onto block
            - make arm go above block again
            - move to initial position
            """
            
            # destination_x = 211.3
            # destination_y = 108 

            destination_x = 206.5
            destination_y = -77
            
            for i in range(n_blocks):
                # open gripper
                code = self._arm.set_gripper_position(850, wait=True, speed=5000, auto_enable=True)
                if not self._check_code(code, 'set_gripper_position'):
                    return
                
                # avoid collision
                code = self._arm.set_position(175, 150, 300, 180.0, 0.0, -91.5, speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1.0, wait=True)
                if not self._check_code(code, 'set_position'):
                    return
                
                # move to block (x y given by camera)
                code = self._arm.set_position(*[xy_blocks[i][0], xy_blocks[i][1], 210, 180.0, 0.0, rotation_blocks[i]], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1.0, wait=True)
                if not self._check_code(code, 'set_position'):
                    return

                # close gripper
                code = self._arm.set_gripper_position(296, wait=True, speed=5000, auto_enable=True)
                if not self._check_code(code, 'set_gripper_position'):
                    return
                
                # move to position to avoid collision
                code = self._arm.set_position(*[xy_blocks[i][0], xy_blocks[i][1], 250, 180.0, 0.0, rotation_blocks[i]], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1.0, wait=True)
                if not self._check_code(code, 'set_position'):
                    return
                
                # avoid collision
                code = self._arm.set_position(*[175, 0, 300, 180.0, 0.0, rotation_blocks[i]], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1.0, wait=True)
                if not self._check_code(code, 'set_position'):
                    return
                
                # move block above where it is being put down (initial height + ith block * height_block)
                initial_height_above = 290.2
                code = self._arm.set_position(*[destination_x, destination_y, initial_height_above + i * height_block, 180.0, 0.0, -91.5], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1.0, wait=True)
                if not self._check_code(code, 'set_position'):
                    return
            
                # move block down onto the below platform or block
                initial_height_down = 231
                code = self._arm.set_position(*[destination_x, destination_y, initial_height_down + i * height_block, 180.0, 0.0, -91.5], speed=40, mvacc=self._tcp_acc, radius=-1.0, wait=True)
                if not self._check_code(code, 'set_position'):
                    return
                
                # open gripper
                code = self._arm.set_gripper_position(850, wait=True, speed=1000, auto_enable=True)
                if not self._check_code(code, 'set_gripper_position'):
                    return
                
                # move arm above block
                code = self._arm.set_position(*[destination_x, destination_y, initial_height_above + i * height_block, 180.0, 0.0, -1.5], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1.0, wait=True)
                if not self._check_code(code, 'set_position'):
                    return
                
                # close gripper
                code = self._arm.set_gripper_position(-10, wait=True, speed=5000, auto_enable=True)
                if not self._check_code(code, 'set_gripper_position'):
                    return
                
                # push gripper down onto block                
                z_down = 42.5  # mm to move arm down by to push block down, z = 246.7 for first block -> 290.2 - 246.7 = 43.5
                code = self._arm.set_position(*[destination_x, destination_y, initial_height_above + (i * height_block) - z_down, 180.0, 0.0, -1.5], speed=20, mvacc=self._tcp_acc, radius=-1.0, wait=True)
                if not self._check_code(code, 'set_position'):
                    return
                
                # arm up to set up second push
                code = self._arm.set_position(*[destination_x, destination_y, initial_height_above + (i * height_block) - z_down + 20, 180.0, 0.0, -1.5], speed=20, mvacc=self._tcp_acc, radius=-1.0, wait=True)
                if not self._check_code(code, 'set_position'):
                    return
                
                # second push
                code = self._arm.set_position(*[destination_x, destination_y, initial_height_above + (i * height_block) - z_down, 180.0, 0.0, -1.5], speed=20, mvacc=self._tcp_acc, radius=-1.0, wait=True)
                if not self._check_code(code, 'set_position'):
                    return
                
                # make arm go above block again
                code = self._arm.set_position(*[destination_x, destination_y, initial_height_above + i * height_block, 180.0, 0.0, -1.5], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1.0, wait=True)
                if not self._check_code(code, 'set_position'):
                    return
                
                # avoid collision
                # code = self._arm.set_position(*[175, 150, 250, 180.0, 0.0, rotation_blocks[i]], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1.0, wait=True)
                # if not self._check_code(code, 'set_position'):
                #     return
                
                # move to initial position
                # code = self._arm.set_position(*[-12.1, 181.6, 222.3, 180.0, 0.0, -91.5], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1.0, wait=True)
                # if not self._check_code(code, 'set_position'):
                #     return
        
        
            # open gripper
            code = self._arm.set_gripper_position(850, wait=True, speed=5000, auto_enable=True)
            if not self._check_code(code, 'set_gripper_position'):
                return
                
            
        except Exception as e:
            self.pprint('MainException: {}'.format(e))
        # self.alive = False
        # self._arm.release_error_warn_changed_callback(self._error_warn_changed_callback)
        # self._arm.release_state_changed_callback(self._state_changed_callback)
        # if hasattr(self._arm, 'release_count_changed_callback'):
        #     self._arm.release_count_changed_callback(self._count_changed_callback)
     
     
     
     
        
    def disassemble(self, xy_blocks: np.ndarray, n_blocks: int, height_block: float):
        try:
            self._tcp_speed = 71
            self._tcp_acc = 1000
            self._angle_speed = 30
            self._angle_acc = 200
            code = self._arm.set_tcp_load(0.82, [0, 0, 48])
            if not self._check_code(code, 'set_tcp_load'):
                return
            
            # move to initial position
            code = self._arm.set_position(*[-12.1, 181.6, 222.3, 180.0, 0.0, -91.5], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
            
            
            
            """
            Loop:
            - open gripper
            - rotate yaw of gripper (so can pick up block at correct angle)
            - move arm above block
            - move arm down to block 
            - close gripper
            - move arm above again
            - move block to a new location (for now back to the block's starting spot)
            - open gripper
            - move to initial position
            """
            destination_x = 201.3
            destination_y = 106.6 
            
            for i in reversed(range(n_blocks)):
                # open gripper
                code = self._arm.set_gripper_position(850, wait=True, speed=5000, auto_enable=True)
                if not self._check_code(code, 'set_gripper_position'):
                    return
                
                # rotate yaw of gripper
                code = self._arm.set_position(*[-12.1, 181.6, 222.3, 180.0, 0.0, 0.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1.0, wait=True)
                if not self._check_code(code, 'set_position'):
                    return
                
                # move arm above block
                initial_height_above = 290.2
                code = self._arm.set_position(*[destination_x, destination_y, initial_height_above + i * height_block, 180.0, 0.0, -91.5], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1.0, wait=True)
                if not self._check_code(code, 'set_position'):
                    return
                
                # move arm down to block 
                initial_height_down = 231
                code = self._arm.set_position(*[destination_x, destination_y, initial_height_down + i * height_block, 180.0, 0.0, -91.5], speed=40, mvacc=self._tcp_acc, radius=-1.0, wait=True)
                if not self._check_code(code, 'set_position'):
                    return

                # close gripper
                code = self._arm.set_gripper_position(260, wait=True, speed=1000, auto_enable=True)
                if not self._check_code(code, 'set_gripper_position'):
                    return
                
                # move arm above again
                code = self._arm.set_position(*[destination_x, destination_y, initial_height_above + i * height_block, 180.0, 0.0, -91.5], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1.0, wait=True)
                if not self._check_code(code, 'set_position'):
                    return
                
                # move block to a new location
                code = self._arm.set_position(*[xy_blocks[i][0], xy_blocks[i][1], 155.3, 180.0, 0.0, -91.5], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1.0, wait=True)
                if not self._check_code(code, 'set_position'):
                    return
                
                # open gripper
                code = self._arm.set_gripper_position(850, wait=True, speed=1000, auto_enable=True)
                if not self._check_code(code, 'set_gripper_position'):
                    return
                
                 # move to initial position
                code = self._arm.set_position(*[-12.1, 181.6, 222.3, 180.0, 0.0, -91.5], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1.0, wait=True)
                if not self._check_code(code, 'set_position'):
                    return
           
           
        except Exception as e:
            self.pprint('MainException: {}'.format(e))
        self.alive = False
        self._arm.release_error_warn_changed_callback(self._error_warn_changed_callback)
        self._arm.release_state_changed_callback(self._state_changed_callback)
        if hasattr(self._arm, 'release_count_changed_callback'):
            self._arm.release_count_changed_callback(self._count_changed_callback)
            
    def disassemble_ith(self, i: int, xy_blocks: np.ndarray, n_blocks: int, height_block: float):
        try:
            self._tcp_speed = 71
            self._tcp_acc = 1000
            self._angle_speed = 30
            self._angle_acc = 200
            code = self._arm.set_tcp_load(0.82, [0, 0, 48])
            if not self._check_code(code, 'set_tcp_load'):
                return
            
            # move to initial position
            code = self._arm.set_position(*[-12.1, 181.6, 222.3, 180.0, 0.0, -91.5], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
            
            
            
            """
            Loop:
            - open gripper
            - rotate yaw of gripper (so can pick up block at correct angle)
            - move arm above block
            - move arm down to block 
            - close gripper
            - move arm above again
            - move block to a new location (for now back to the block's starting spot)
            - open gripper
            - move to initial position
            """
            destination_x = 215.5 # 201.3 for the bottom block
            if i == 0:
                destination_x = 201.3
            destination_y = 106.6 
            
            
            # open gripper
            code = self._arm.set_gripper_position(850, wait=True, speed=5000, auto_enable=True)
            if not self._check_code(code, 'set_gripper_position'):
                return
            
            # rotate yaw of gripper
            code = self._arm.set_position(*[-12.1, 181.6, 222.3, 180.0, 0.0, 0.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
            
            # move arm above block
            initial_height_above = 290.2
            code = self._arm.set_position(*[destination_x, destination_y, initial_height_above + i * height_block, 180.0, 0.0, 0.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
            
            # move arm down to block 
            initial_height_down = 231 # 231
            code = self._arm.set_position(*[destination_x, destination_y, initial_height_down + i * height_block, 180.0, 0.0, 0.0], speed=40, mvacc=self._tcp_acc, radius=-1.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return

            # close gripper
            code = self._arm.set_gripper_position(250, wait=True, speed=1000, auto_enable=True) # 260
            if not self._check_code(code, 'set_gripper_position'):
                return
            
            # move arm above again
            code = self._arm.set_position(*[destination_x, destination_y, initial_height_above + i * height_block, 180.0, 0.0, 0.0], speed=30, mvacc=self._tcp_acc, radius=-1.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
            
            # move block to a new location
            code = self._arm.set_position(*[xy_blocks[i][0], xy_blocks[i][1], 175, 180.0, 0.0, -91.5], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
            
            # open gripper
            code = self._arm.set_gripper_position(850, wait=True, speed=1000, auto_enable=True)
            if not self._check_code(code, 'set_gripper_position'):
                return
            
            # move to initial position
            code = self._arm.set_position(*[-12.1, 181.6, 222.3, 180.0, 0.0, -91.5], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
        
        
        except Exception as e:
            self.pprint('MainException: {}'.format(e))


if __name__ == '__main__':
    BigGripper.pprint('xArm-Python-SDK Version:{}'.format(version.__version__))
    arm = XArmAPI('192.168.1.207', baud_checkset=False)
    robot_main = BigGripper(arm)
    
    height_block = 19.5
    xy_blocks = [[1.4, 230], [1.4, 230], [1.4, 230], [1.4, 230]]
    # robot_main.assemble(xy_blocks=xy_blocks, n_blocks=4, height_block=height_block)
    robot_main.disassemble(xy_blocks=xy_blocks, n_blocks=4, height_block=height_block)
