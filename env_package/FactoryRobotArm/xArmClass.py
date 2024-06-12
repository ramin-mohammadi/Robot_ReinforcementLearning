
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

    # Robot Main Run
    def run(self):
        try:
            self._tcp_speed = 71
            self._tcp_acc = 1000
            self._angle_speed = 30
            self._angle_acc = 200
            code = self._arm.set_tcp_load(0.82, [0, 0, 48])
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


# if __name__ == '__main__':
#     RobotMain.pprint('xArm-Python-SDK Version:{}'.format(version.__version__))
#     arm = XArmAPI('192.168.1.207', baud_checkset=False)
#     robot_main = RobotMain(arm)
#     robot_main.run()

    def test(self, target_pos):
        try:
            self._tcp_speed = 71
            self._tcp_acc = 1000
            self._angle_speed = 30
            self._angle_acc = 200
            code = self._arm.set_tcp_load(0.82, [0, 0, 48])
            if not self._check_code(code, 'set_tcp_load'):
                return
            
            code = self._arm.set_position(*[target_pos[0], target_pos[1], target_pos[2], 180.0, 0.0, -91.5], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return

        except Exception as e:
            self.pprint('MainException: {}'.format(e))
            
    def move_to(self, pos):
        try:
            self._tcp_speed = 71
            self._tcp_acc = 1000
            self._angle_speed = 30
            self._angle_acc = 200
            code = self._arm.set_tcp_load(0.82, [0, 0, 48])
            if not self._check_code(code, 'set_tcp_load'):
                return
            
            code = self._arm.set_position(*[pos[0], pos[1], pos[2], 180.0, 0.0, -91.5], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
            
            code = self._arm.set_pause_time(1)
            if not self._check_code(code, 'set_pause_time'):
                return

        except Exception as e:
            self.pprint('MainException: {}'.format(e))
            # self.move_initial()
            
    def open_gripper(self):
        try:
            code = self._arm.set_gripper_position(450, wait=True, speed=2500, auto_enable=True)
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
            
        # self.alive = False
        # self._arm.release_error_warn_changed_callback(self._error_warn_changed_callback)
        # self._arm.release_state_changed_callback(self._state_changed_callback)
        # if hasattr(self._arm, 'release_count_changed_callback'):
        #     self._arm.release_count_changed_callback(self._count_changed_callback)
            
    def move_initial(self):
        try:
            self._tcp_speed = 71
            self._tcp_acc = 1000
            self._angle_speed = 30
            self._angle_acc = 200
            code = self._arm.set_tcp_load(0.82, [0, 0, 48])
            if not self._check_code(code, 'set_tcp_load'):
                return
            
            # xyz roll pitch yaw
            code = self._arm.set_position(*[-12.1, 181.6, 222.3, 180.0, 0.0, -91.5], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
            
            # joints
            code = self._arm.set_servo_angle(angle=[93.8, -58.8, 5.2, 53.6, 185.3], speed=self._angle_speed, mvacc=self._angle_acc, wait=True, radius=-1.0)
            if not self._check_code(code, 'set_servo_angle'):
                return

        except Exception as e:
            self.pprint('MainException: {}'.format(e))
            
        # IMPORTANT: after below code block , arm will no longer be able to move
        # self.alive = False
        # self._arm.release_error_warn_changed_callback(self._error_warn_changed_callback)
        # self._arm.release_state_changed_callback(self._state_changed_callback)
        # if hasattr(self._arm, 'release_count_changed_callback'):
        #     self._arm.release_count_changed_callback(self._count_changed_callback)

        
        
