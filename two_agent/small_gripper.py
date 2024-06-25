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

    
    def disassemble(self, n_blocks, height_block):
        try:
            self._tcp_speed = 71
            self._tcp_acc = 1000
            self._angle_speed = 30
            self._angle_acc = 200
            code = self._arm.set_tcp_load(0.277, [0, 0, 30])
            if not self._check_code(code, 'set_tcp_load'):
                return
       
            # move to initial position
            code = self._arm.set_position(*[-26.0, 167.6, -54.2, -70.9, 89.2, -160.6], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
            if not self._check_code(code, 'set_position'):
                return
            
            
            lowest_z = -58.5 # z for bottom block
            
            """
            Loop:
            - open gripper
            - move to ith-1 block's z
            - close gripper (if on bottom block so i=0 then dont close gripper)
            """
            
            for i in reversed(range(n_blocks - 1)): # - 1 b/c dont want to grab the top block
                # open gripper
                code = self._arm.open_lite6_gripper()
                time.sleep(0.5)
                self._arm.stop_lite6_gripper()
                if not self._check_code(code, 'open_lite6_gripper'):
                    return
                
                # move to ith-1 block's z
                code = self._arm.set_position(*[-26.0, 33.0, lowest_z + i * height_block, -70.9, 89.2, -160.6], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
                if not self._check_code(code, 'set_position'):
                    return
                
                # close gripper
                if i != 0: # keep gripper open for last block
                    code = self._arm.close_lite6_gripper()
                    if not self._check_code(code, 'close_lite6_gripper'):
                        return
                time.sleep(3)

                
            
            # move back to initial position
            code = self._arm.set_position(*[-26.0, 167.6, -54.2, -70.9, 89.2, -160.6], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
            if not self._check_code(code, 'set_position'):
                return
        except Exception as e:
            self.pprint('MainException: {}'.format(e))
        self.alive = False
        self._arm.release_error_warn_changed_callback(self._error_warn_changed_callback)
        self._arm.release_state_changed_callback(self._state_changed_callback)
        if hasattr(self._arm, 'release_count_changed_callback'):
            self._arm.release_count_changed_callback(self._count_changed_callback)


# if __name__ == '__main__':
#     RobotMain.pprint('xArm-Python-SDK Version:{}'.format(version.__version__))
#     arm = XArmAPI('192.168.1.176', baud_checkset=False)
#     robot_main = RobotMain(arm)
#     # robot_main.run()
    
#     height_block = 19.5    # -58.5 -> -39  (mm)
#     robot_main.disassemble(n_blocks=4, height_block=height_block)
