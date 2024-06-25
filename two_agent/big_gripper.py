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
    def assemble(self):
        try:
            self._tcp_speed = 71
            self._tcp_acc = 1000
            self._angle_speed = 30
            self._angle_acc = 200
            code = self._arm.set_tcp_load(0.82, [0, 0, 48])
            if not self._check_code(code, 'set_tcp_load'):
                return
            code = self._arm.set_gripper_position(850, wait=True, speed=5000, auto_enable=True)
            if not self._check_code(code, 'set_gripper_position'):
                return
            code = self._arm.set_position(*[1.4, 230.0, 148.3, 180.0, 0.0, -91.5], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
            code = self._arm.set_gripper_position(296, wait=True, speed=5000, auto_enable=True)
            if not self._check_code(code, 'set_gripper_position'):
                return
            code = self._arm.set_position(*[211.3, 104.1, 450.0, 180.0, 0.0, -91.5], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
            code = self._arm.set_position(*[211.3, 104.1, 232.0, 180.0, 0.0, -91.5], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
            code = self._arm.set_gripper_position(850, wait=True, speed=1000, auto_enable=True)
            if not self._check_code(code, 'set_gripper_position'):
                return
            code = self._arm.set_position(*[211.3, 104.1, 258.1, 180.0, 0.0, -91.5], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
            code = self._arm.set_gripper_position(100, wait=True, speed=5000, auto_enable=True)
            if not self._check_code(code, 'set_gripper_position'):
                return
            code = self._arm.set_position(*[211.3, 108.2, 244.5, 180.0, 0.0, -91.5], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
            code = self._arm.set_position(*[211.3, 108.2, 335.5, 180.0, 0.0, -91.5], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
            code = self._arm.set_position(*[-12.1, 181.6, 222.3, 180.0, 0.0, -91.5], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
            code = self._arm.set_gripper_position(850, wait=True, speed=5000, auto_enable=True)
            if not self._check_code(code, 'set_gripper_position'):
                return
            code = self._arm.set_position(*[1.4, 300.5, 148.3, 180.0, 0.0, -91.5], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
            code = self._arm.set_gripper_position(296, wait=True, speed=4000, auto_enable=True)
            if not self._check_code(code, 'set_gripper_position'):
                return
            code = self._arm.set_position(*[211.3, 104.1, 450.0, 180.0, 0.0, -91.5], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
            code = self._arm.set_position(*[213.3, 106.1, 252.2, 180.0, 0.0, -91.5], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
            code = self._arm.set_gripper_position(850, wait=True, speed=1000, auto_enable=True)
            if not self._check_code(code, 'set_gripper_position'):
                return
            code = self._arm.set_position(*[213.3, 106.1, 275.9, 180.0, 0.0, -91.5], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
            code = self._arm.set_gripper_position(100, wait=True, speed=5000, auto_enable=True)
            if not self._check_code(code, 'set_gripper_position'):
                return
            code = self._arm.set_position(*[213.3, 106.1, 265.0, 180.0, 0.0, -91.5], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
            code = self._arm.set_position(*[210.9, 101.1, 335.5, 180.0, 0.0, -91.5], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
            code = self._arm.set_position(*[-12.1, 181.6, 222.3, 180.0, 0.0, -91.5], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
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
        
    def disassemble(self):
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
            code = self._arm.set_gripper_position(850, wait=True, speed=5000, auto_enable=True)
            if not self._check_code(code, 'set_gripper_position'):
                return
            code = self._arm.set_position(*[-12.1, 181.6, 222.3, 180.0, 0.0, 0.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
            code = self._arm.set_position(*[215.5, 106.6, 297.5, 180.0, 0.0, 0.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
            code = self._arm.set_position(*[215.5, 106.6, 248.0, 180.0, 0.0, 0.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
            code = self._arm.set_gripper_position(260, wait=True, speed=1000, auto_enable=True)
            if not self._check_code(code, 'set_gripper_position'):
                return
            code = self._arm.set_position(*[215.5, 106.6, 328.9, 180.0, 0.0, 0.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
            code = self._arm.set_position(*[1.4, 300.5, 148.3, 180.0, 0.0, -91.5], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
            code = self._arm.set_gripper_position(840, wait=True, speed=5000, auto_enable=True)
            if not self._check_code(code, 'set_gripper_position'):
                return
            code = self._arm.set_position(*[-12.1, 181.6, 222.3, 180.0, 0.0, -91.5], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
            code = self._arm.set_position(*[-12.1, 181.6, 222.3, 180.0, 0.0, 0.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
            code = self._arm.set_position(*[215.5, 106.6, 297.5, 180.0, 0.0, 0.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
            code = self._arm.set_position(*[201.3, 106.6, 230.4, 180.0, 0.0, 0.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
            code = self._arm.set_gripper_position(275, wait=True, speed=1000, auto_enable=True)
            if not self._check_code(code, 'set_gripper_position'):
                return
            code = self._arm.set_position(*[201.3, 106.6, 341.0, 180.0, 0.0, 0.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
            code = self._arm.set_position(*[1.4, 230.0, 148.3, 180.0, 0.0, -91.5], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
            code = self._arm.set_gripper_position(850, wait=True, speed=5000, auto_enable=True)
            if not self._check_code(code, 'set_gripper_position'):
                return
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


# if __name__ == '__main__':
#     RobotMain.pprint('xArm-Python-SDK Version:{}'.format(version.__version__))
#     arm = XArmAPI('192.168.1.207', baud_checkset=False)
#     robot_main = RobotMain(arm)
#     robot_main.run()
