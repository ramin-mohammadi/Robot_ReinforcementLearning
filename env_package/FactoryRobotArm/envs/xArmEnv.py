#import gym
# NOTE: gym seems to be old and gymnasium is new maintained environment package
import numpy as np
import gymnasium as gym
from gymnasium import spaces
import math
import time

import plotly.express as px
import plotly.graph_objects as go


from FactoryRobotArm import xArmClass


# FIRST GOAL get robot arm to go to detected location from camera, then can enhance to move object to a locaiton after picking it up and other more complicated tasks

class xArmEnv(gym.Env): 
    metadata = {"render_modes": ["human", "rgb_array"], "render_fps": 4}
    
    def __init__(self, render_mode=None):
        """
        Must define self.observation_space ad self.action_space here
        """
        
        # xArm object
        xArmClass.RobotMain.pprint('xArm-Python-SDK Version:{}'.format(xArmClass.version.__version__))
        self.arm = xArmClass.XArmAPI('192.168.1.207', baud_checkset=False)
        self.robot_main = xArmClass.RobotMain(self.arm)
        
        
        
        self.num_steps = 0
        # Bound: grid workspace (3D plane xyz) of conveyor belt??? -> possibly not true because the location of the robots are not in the conveyor belt grid
        # solution is to assume from the vertex (0,0,0) that me and Brian chose, the bounds is the entire robot working field? Or each robot will get a unique bound being how far they can reach in all directions
        # units is millimeters
        
        # THERE ARE DIFFERENT BOUNDARY VALUES for the smaller and bigger arms (LOOK at their manuals appendix which are the last few pages)
        
        # AGENT: xArm 5 (big arm)
        self.agent_x_low = -700
        self.agent_x_high = 700
        self.agent_y_low = -700
        self.agent_y_high = 700
        self.agent_z_low = 0
        self.agent_z_high = 950
        
        self.agent_roll_pitch_yaw_low = -180
        self.agent_roll_pitch_yaw_high = 180
        
        self.agent_joint1_low = -360
        self.agent_joint1_high = 360
        self.agent_joint2_low = -118
        self.agent_joint2_high = 120
        self.agent_joint3_low = -225
        self.agent_joint3_high = 11
        self.agent_joint4_low = -97
        self.agent_joint4_high = 180
        self.agent_joint5_low = -360
        self.agent_joint5_high = 360
        
        # TARGET: object on conveyor belt
        # self.target_x_low = -300
        # self.target_x_high = 300
        # self.target_y_low = -400
        # self.target_y_high = 400        
        # self.target_z_low = 0
        # self.target_z_high = 30
        
        # SIMPLE EXAMPLE:
        self.target_x_low = -3
        self.target_x_high = 3
        self.target_y_low = -3
        self.target_y_high = 3
        self.target_z_low = -3
        self.target_z_high = 3
        

        
        # robot arm is the agent, target is the object on the conveyor belt
        # i have specified the range of possible values for xyz THEN the shape is an array of 3 elements for xyz  [-2,200,11]
        # For the agent it is [x,y,z, Roll, Pitch, Yaw, J1,J2,J3,J4,J5] -> smaller arms have 6 joints so IMPORTANT: EACH arm will have their own custom "environment"
        # target will just have [x,y,z]
        self.observation_space = spaces.Dict( # later may have to account for observing if other arms are moving???
            {
                "agent" : spaces.Box(low=np.array([self.agent_x_low, self.agent_y_low, self.agent_z_low, self.agent_roll_pitch_yaw_low, self.agent_roll_pitch_yaw_low, self.agent_roll_pitch_yaw_low, self.agent_joint1_low, self.agent_joint2_low, self.agent_joint3_low, self.agent_joint4_low, self.agent_joint5_low]), 
                high=np.array([self.agent_x_high, self.agent_y_high, self.agent_z_high, self.agent_roll_pitch_yaw_high, self.agent_roll_pitch_yaw_high, self.agent_roll_pitch_yaw_high,self.agent_joint1_high, self.agent_joint2_high, self.agent_joint3_high, self.agent_joint4_high, self.agent_joint5_high]), 
                                     shape=(11,), dtype=np.float32), 
                # the z axis is needed for the target to help guide the agent's z position to the target's -> But PROBLEM here bc z pos is the attachment point of the gripper, not the tip of the gripper
                "target" : spaces.Box(low=np.array([self.target_x_low, self.target_y_low, self.target_z_low]), 
                                      high=np.array([self.target_x_high, self.target_y_high, self.target_z_high]), 
                                      shape=(3,), dtype=np.float32),
            }
        )
        
        """
        Actions: only do one at a time so for a linear motion command, only change one value at a time
        - move right
        - move left
        - move up
        - move down
        - rotate clockwise roll
        - rotate clockwise pitch
        - rotate clockwise yaw
        - rotate counter clockwise roll
        - rotate counter clockwise pitch
        - rotate counter clockwise yaw
        - rotate clockwise joint1
        - rotate clockwise joint2
        - rotate clockwise joint3
        - rotate clockwise joint4
        - rotate clockwise joint5 
        - rotate counter clockwise joint1
        - rotate counter clockwise joint2
        - rotate counter clockwise joint3
        - rotate counter clockwise joint4
        - rotate counter clockwise joint5 
        
        ADD LATER:
        - open gripper
        - close gripper
        - sleep?
        
        PROBLEM: determining how many units to move by (dijkstras? shortest path?) -> MAYBE just provide multiple 
        intervals it could possibly be moved at a time so 1,5,10,20,50,100mm
        then for degrees 1, 5, 10, 20, 60, 90, 180
        """
        # MOVE_X_POS_1 = 0
        # MOVE_X_POS_5 = 1
        # MOVE_X_POS_10 = 2
        # MOVE_X_POS_20 = 3
        # MOVE_X_POS_50 = 4
        # MOVE_X_POS_100 = 5

        # MOVE_X_NEG_1 = 6
        # MOVE_X_NEG_5 = 7
        # MOVE_X_NEG_10 = 8
        # MOVE_X_NEG_20 = 9
        # MOVE_X_NEG_50 = 10
        # MOVE_X_NEG_100 = 11
        
        # MOVE_Y_POS_1 = 12
        # MOVE_Y_POS_5 = 13
        # MOVE_Y_POS_10 = 14
        # MOVE_Y_POS_20 = 15
        # MOVE_Y_POS_50 = 16
        # MOVE_Y_POS_100 = 17

        # MOVE_Y_NEG_1 = 18
        # MOVE_Y_NEG_5 = 19
        # MOVE_Y_NEG_10 = 20
        # MOVE_Y_NEG_20 = 21
        # MOVE_Y_NEG_50 = 22
        # MOVE_Y_NEG_100 = 23

        # ROTATE_CLOCKWISE_ROLL_1 = 24
        # ROTATE_CLOCKWISE_ROLL_5 = 25
        # ROTATE_CLOCKWISE_ROLL_10 = 26
        # ROTATE_CLOCKWISE_ROLL_20 = 27
        # ROTATE_CLOCKWISE_ROLL_60 = 28
        # ROTATE_CLOCKWISE_ROLL_90 = 29
        # ROTATE_CLOCKWISE_ROLL_180 = 30
        
        # ROTATE_COUNTERCLOCKWISE_ROLL_1 = 31
        # ROTATE_COUNTERCLOCKWISE_ROLL_5 = 32
        # ROTATE_COUNTERCLOCKWISE_ROLL_10 = 33
        # ROTATE_COUNTERCLOCKWISE_ROLL_20 = 34
        # ROTATE_COUNTERCLOCKWISE_ROLL_60 = 35
        # ROTATE_COUNTERCLOCKWISE_ROLL_90 = 36
        # ROTATE_COUNTERCLOCKWISE_ROLL_180 = 37
        
        # ROTATE_CLOCKWISE_PITCH_1 = 38
        # ROTATE_CLOCKWISE_PITCH_5 = 39
        # ROTATE_CLOCKWISE_PITCH_10 = 40
        # ROTATE_CLOCKWISE_PITCH_20 = 41
        # ROTATE_CLOCKWISE_PITCH_60 = 42
        # ROTATE_CLOCKWISE_PITCH_90 = 43
        # ROTATE_CLOCKWISE_PITCH_180 = 44

        # ROTATE_COUNTERCLOCKWISE_PITCH_1 = 45
        # ROTATE_COUNTERCLOCKWISE_PITCH_5 = 46
        # ROTATE_COUNTERCLOCKWISE_PITCH_10 = 47
        # ROTATE_COUNTERCLOCKWISE_PITCH_20 = 48
        # ROTATE_COUNTERCLOCKWISE_PITCH_60 = 49
        # ROTATE_COUNTERCLOCKWISE_PITCH_90 = 50
        # ROTATE_COUNTERCLOCKWISE_PITCH_180 = 51
        
        # ROTATE_CLOCKWISE_YAW_1 = 52
        # ROTATE_CLOCKWISE_YAW_5 = 53
        # ROTATE_CLOCKWISE_YAW_10 = 54
        # ROTATE_CLOCKWISE_YAW_20 = 55
        # ROTATE_CLOCKWISE_YAW_60 = 56
        # ROTATE_CLOCKWISE_YAW_90 = 57
        # ROTATE_CLOCKWISE_YAW_180 = 58

        # ROTATE_COUNTERCLOCKWISE_YAW_1 = 59
        # ROTATE_COUNTERCLOCKWISE_YAW_5 = 60
        # ROTATE_COUNTERCLOCKWISE_YAW_10 = 61
        # ROTATE_COUNTERCLOCKWISE_YAW_20 = 62
        # ROTATE_COUNTERCLOCKWISE_YAW_60 = 63
        # ROTATE_COUNTERCLOCKWISE_YAW_90 = 64
        # ROTATE_COUNTERCLOCKWISE_YAW_180 = 65
        
        # ROTATE_CLOCKWISE_J1_1 = 66
        # ROTATE_CLOCKWISE_J1_5 = 67
        # ROTATE_CLOCKWISE_J1_10 = 68
        # ROTATE_CLOCKWISE_J1_20 = 69
        # ROTATE_CLOCKWISE_J1_60 = 70
        # ROTATE_CLOCKWISE_J1_90 = 71
        # ROTATE_CLOCKWISE_J1_180 = 72
        
        # ROTATE_COUNTERCLOCKWISE_J1_1 = 73
        # ROTATE_COUNTERCLOCKWISE_J1_5 = 74
        # ROTATE_COUNTERCLOCKWISE_J1_10 = 75
        # ROTATE_COUNTERCLOCKWISE_J1_20 = 76
        # ROTATE_COUNTERCLOCKWISE_J1_60 = 77
        # ROTATE_COUNTERCLOCKWISE_J1_90 = 78
        # ROTATE_COUNTERCLOCKWISE_J1_180 = 79

        # ROTATE_CLOCKWISE_J2_1 = 80
        # ROTATE_CLOCKWISE_J2_5 = 81
        # ROTATE_CLOCKWISE_J2_10 = 82
        # ROTATE_CLOCKWISE_J2_20 = 83
        # ROTATE_CLOCKWISE_J2_60 = 84
        # ROTATE_CLOCKWISE_J2_90 = 85
        # ROTATE_CLOCKWISE_J2_180 = 86
        
        # ROTATE_COUNTERCLOCKWISE_J2_1 = 87
        # ROTATE_COUNTERCLOCKWISE_J2_5 = 88
        # ROTATE_COUNTERCLOCKWISE_J2_10 = 89
        # ROTATE_COUNTERCLOCKWISE_J2_20 = 90
        # ROTATE_COUNTERCLOCKWISE_J2_60 = 91
        # ROTATE_COUNTERCLOCKWISE_J2_90 = 92
        # ROTATE_COUNTERCLOCKWISE_J2_180 = 93
        
        # ROTATE_CLOCKWISE_J3_1 = 94
        # ROTATE_CLOCKWISE_J3_5 = 95
        # ROTATE_CLOCKWISE_J3_10 = 96
        # ROTATE_CLOCKWISE_J3_20 = 97
        # ROTATE_CLOCKWISE_J3_60 = 98
        # ROTATE_CLOCKWISE_J3_90 = 99
        # ROTATE_CLOCKWISE_J3_180 = 100
        
        # ROTATE_COUNTERCLOCKWISE_J3_1 = 101
        # ROTATE_COUNTERCLOCKWISE_J3_5 = 102
        # ROTATE_COUNTERCLOCKWISE_J3_10 = 103
        # ROTATE_COUNTERCLOCKWISE_J3_20 = 104
        # ROTATE_COUNTERCLOCKWISE_J3_60 = 105
        # ROTATE_COUNTERCLOCKWISE_J3_90 = 106
        # ROTATE_COUNTERCLOCKWISE_J3_180 = 107
        
        # ROTATE_CLOCKWISE_J4_1 = 108
        # ROTATE_CLOCKWISE_J4_5 = 109
        # ROTATE_CLOCKWISE_J4_10 = 110
        # ROTATE_CLOCKWISE_J4_20 = 111
        # ROTATE_CLOCKWISE_J4_60 = 112
        # ROTATE_CLOCKWISE_J4_90 = 113
        # ROTATE_CLOCKWISE_J4_180 = 114
        
        # ROTATE_COUNTERCLOCKWISE_J4_1 = 115
        # ROTATE_COUNTERCLOCKWISE_J4_5 = 116
        # ROTATE_COUNTERCLOCKWISE_J4_10 = 117
        # ROTATE_COUNTERCLOCKWISE_J4_20 = 118
        # ROTATE_COUNTERCLOCKWISE_J4_60 = 119
        # ROTATE_COUNTERCLOCKWISE_J4_90 = 120
        # ROTATE_COUNTERCLOCKWISE_J4_180 = 121
        
        # ROTATE_CLOCKWISE_J5_1 = 122
        # ROTATE_CLOCKWISE_J5_5 = 123
        # ROTATE_CLOCKWISE_J5_10 = 124
        # ROTATE_CLOCKWISE_J5_20 = 125
        # ROTATE_CLOCKWISE_J5_60 = 126
        # ROTATE_CLOCKWISE_J5_90 = 127
        # ROTATE_CLOCKWISE_J5_180 = 128
        
        # ROTATE_COUNTERCLOCKWISE_J5_1 = 129
        # ROTATE_COUNTERCLOCKWISE_J5_5 = 130
        # ROTATE_COUNTERCLOCKWISE_J5_10 = 131
        # ROTATE_COUNTERCLOCKWISE_J5_20 = 132
        # ROTATE_COUNTERCLOCKWISE_J5_60 = 133
        # ROTATE_COUNTERCLOCKWISE_J5_90 = 134
        # ROTATE_COUNTERCLOCKWISE_J5_180 = 135
        
        # MOVE_Z_POS_1 = 136
        # MOVE_Z_POS_5 = 137
        # MOVE_Z_POS_10 = 138
        # MOVE_Z_POS_20 = 139
        # MOVE_Z_POS_50 = 140
        # MOVE_Z_POS_100 = 141

        # MOVE_Z_NEG_1 = 142
        # MOVE_Z_NEG_5 = 143
        # MOVE_Z_NEG_10 = 144
        # MOVE_Z_NEG_20 = 145
        # MOVE_Z_NEG_50 = 146
        # MOVE_Z_NEG_100 = 147
        
        # MOVE_X_POS_DEC_1 = 148 # move x+ direction by 0.1 mm
        # MOVE_X_NEG_DEC_1 = 149
        # MOVE_Y_POS_DEC_1 = 150
        # MOVE_Y_NEG_DEC_1 = 151
        # MOVE_Z_POS_DEC_1 = 152
        # MOVE_Z_NEG_DEC_1 = 153
        
        # actions: [x,y,z, roll,pitch,yaw, J1,J2,J3,J4,J5]
        # self._actions_dict = {
        #     0: np.array([1,0,0, 0,0,0, 0,0,0,0,0]),
        #     1: np.array([5,0,0, 0,0,0, 0,0,0,0,0]),
        #     2: np.array([10,0,0, 0,0,0, 0,0,0,0,0]),
        #     3: np.array([20,0,0, 0,0,0, 0,0,0,0,0]),
        #     4: np.array([50,0,0, 0,0,0, 0,0,0,0,0]),
        #     5: np.array([100,0,0, 0,0,0, 0,0,0,0,0]),
            
        #     6: np.array([-1,0,0, 0,0,0, 0,0,0,0,0]),
        #     7: np.array([-5,0,0, 0,0,0, 0,0,0,0,0]),
        #     8: np.array([-10,0,0, 0,0,0, 0,0,0,0,0]),
        #     9: np.array([-20,0,0, 0,0,0, 0,0,0,0,0]),
        #     10: np.array([-50,0,0, 0,0,0, 0,0,0,0,0]),
        #     11: np.array([-100,0,0, 0,0,0, 0,0,0,0,0]),
            
        #     12: np.array([0,1,0, 0,0,0, 0,0,0,0,0]),
        #     13: np.array([0,5,0, 0,0,0, 0,0,0,0,0]),
        #     14: np.array([0,10,0, 0,0,0, 0,0,0,0,0]),
        #     15: np.array([0,20,0, 0,0,0, 0,0,0,0,0]),
        #     16: np.array([0,50,0, 0,0,0, 0,0,0,0,0]),
        #     17: np.array([0,100,0, 0,0,0, 0,0,0,0,0]),
            
        #     18: np.array([0,-1,0, 0,0,0, 0,0,0,0,0]),
        #     19: np.array([0,-5,0, 0,0,0, 0,0,0,0,0]),
        #     20: np.array([0,-10,0, 0,0,0, 0,0,0,0,0]),
        #     21: np.array([0,-20,0, 0,0,0, 0,0,0,0,0]),
        #     22: np.array([0,-50,0, 0,0,0, 0,0,0,0,0]),
        #     23: np.array([0,-100,0, 0,0,0, 0,0,0,0,0]),

        #     24: np.array([0,0,1, 0,0,0, 0,0,0,0,0]),
        #     25: np.array([0,0,5, 0,0,0, 0,0,0,0,0]),
        #     26: np.array([0,0,10, 0,0,0, 0,0,0,0,0]),
        #     27: np.array([0,0,20, 0,0,0, 0,0,0,0,0]),
        #     28: np.array([0,0,50, 0,0,0, 0,0,0,0,0]),
        #     29: np.array([0,0,100, 0,0,0, 0,0,0,0,0]),
            
        #     30: np.array([0,0,-1, 0,0,0, 0,0,0,0,0]),
        #     31: np.array([0,0,-5, 0,0,0, 0,0,0,0,0]),
        #     32: np.array([0,0,-10, 0,0,0, 0,0,0,0,0]),
        #     33: np.array([0,0,-20, 0,0,0, 0,0,0,0,0]),
        #     34: np.array([0,0,-50, 0,0,0, 0,0,0,0,0]),
        #     35: np.array([0,0,-100, 0,0,0, 0,0,0,0,0]),

        #     36: np.array([0,0,0, -1,0,0, 0,0,0,0,0]), # clockwise rotation is negative and otherwise for counter clockwise (positive)
        #     37: np.array([0,0,0, -5,0,0, 0,0,0,0,0]),
        #     38: np.array([0,0,0, -10,0,0, 0,0,0,0,0]),
        #     39: np.array([0,0,0, -20,0,0, 0,0,0,0,0]),
        #     40: np.array([0,0,0, -60,0,0, 0,0,0,0,0]),
        #     41: np.array([0,0,0, -90,0,0, 0,0,0,0,0]),
        #     42: np.array([0,0,0, -180,0,0, 0,0,0,0,0]),
            
        #     43: np.array([0,0,0, 1,0,0, 0,0,0,0,0]),
        #     44: np.array([0,0,0, 5,0,0, 0,0,0,0,0]),
        #     45: np.array([0,0,0, 10,0,0, 0,0,0,0,0]),
        #     46: np.array([0,0,0, 20,0,0, 0,0,0,0,0]),
        #     47: np.array([0,0,0, 60,0,0, 0,0,0,0,0]),
        #     48: np.array([0,0,0, 90,0,0, 0,0,0,0,0]),
        #     49: np.array([0,0,0, 180,0,0, 0,0,0,0,0]),
            
        #     50: np.array([0,0,0, 0,-1,0, 0,0,0,0,0]), 
        #     51: np.array([0,0,0, 0,-5,0, 0,0,0,0,0]),
        #     52: np.array([0,0,0, 0,-10,0, 0,0,0,0,0]),
        #     53: np.array([0,0,0, 0,-20,0, 0,0,0,0,0]),
        #     54: np.array([0,0,0, 0,-60,0, 0,0,0,0,0]),
        #     55: np.array([0,0,0, 0,-90,0, 0,0,0,0,0]),
        #     56: np.array([0,0,0, 0,-180,0, 0,0,0,0,0]),
            
        #     57: np.array([0,0,0, 0,1,0, 0,0,0,0,0]),
        #     58: np.array([0,0,0, 0,5,0, 0,0,0,0,0]),
        #     59: np.array([0,0,0, 0,10,0, 0,0,0,0,0]),
        #     60: np.array([0,0,0, 0,20,0, 0,0,0,0,0]),
        #     61: np.array([0,0,0, 0,60,0, 0,0,0,0,0]),
        #     62: np.array([0,0,0, 0,90,0, 0,0,0,0,0]),
        #     63: np.array([0,0,0, 0,180,0, 0,0,0,0,0]),
            
        #     64: np.array([0,0,0, 0,0,-1, 0,0,0,0,0]), 
        #     65: np.array([0,0,0, 0,0,-5, 0,0,0,0,0]),
        #     66: np.array([0,0,0, 0,0,-10, 0,0,0,0,0]),
        #     67: np.array([0,0,0, 0,0,-20, 0,0,0,0,0]),
        #     68: np.array([0,0,0, 0,0,-60, 0,0,0,0,0]),
        #     69: np.array([0,0,0, 0,0,-90, 0,0,0,0,0]),
        #     70: np.array([0,0,0, 0,0,-180, 0,0,0,0,0]),
            
        #     71: np.array([0,0,0, 0,0,1, 0,0,0,0,0]),
        #     72: np.array([0,0,0, 0,0,5, 0,0,0,0,0]),
        #     73: np.array([0,0,0, 0,0,10, 0,0,0,0,0]),
        #     74: np.array([0,0,0, 0,0,20, 0,0,0,0,0]),
        #     75: np.array([0,0,0, 0,0,60, 0,0,0,0,0]),
        #     76: np.array([0,0,0, 0,0,90, 0,0,0,0,0]),
        #     77: np.array([0,0,0, 0,0,180, 0,0,0,0,0]),

        #     78: np.array([0,0,0, 0,0,0, -1,0,0,0,0]), 
        #     79: np.array([0,0,0, 0,0,0, -5,0,0,0,0]),
        #     80: np.array([0,0,0, 0,0,0, -10,0,0,0,0]),
        #     81: np.array([0,0,0, 0,0,0, -20,0,0,0,0]),
        #     82: np.array([0,0,0, 0,0,0, -60,0,0,0,0]),
        #     83: np.array([0,0,0, 0,0,0, -90,0,0,0,0]),
        #     84: np.array([0,0,0, 0,0,0, -180,0,0,0,0]),
            
        #     85: np.array([0,0,0, 0,0,0, 1,0,0,0,0]),
        #     86: np.array([0,0,0, 0,0,0, 5,0,0,0,0]),
        #     87: np.array([0,0,0, 0,0,0, 10,0,0,0,0]),
        #     88: np.array([0,0,0, 0,0,0, 20,0,0,0,0]),
        #     89: np.array([0,0,0, 0,0,0, 60,0,0,0,0]),
        #     90: np.array([0,0,0, 0,0,0, 90,0,0,0,0]),
        #     91: np.array([0,0,0, 0,0,0, 180,0,0,0,0]),
            
        #     92: np.array([0,0,0, 0,0,0, 0,-1,0,0,0]), 
        #     93: np.array([0,0,0, 0,0,0, 0,-5,0,0,0]),
        #     94: np.array([0,0,0, 0,0,0, 0,-10,0,0,0]),
        #     95: np.array([0,0,0, 0,0,0, 0,-20,0,0,0]),
        #     96: np.array([0,0,0, 0,0,0, 0,-60,0,0,0]),
        #     97: np.array([0,0,0, 0,0,0, 0,-90,0,0,0]),
        #     98: np.array([0,0,0, 0,0,0, 0,-180,0,0,0]),
            
        #     99: np.array([0,0,0, 0,0,0, 0,1,0,0,0]),
        #     100: np.array([0,0,0, 0,0,0, 0,5,0,0,0]),
        #     101: np.array([0,0,0, 0,0,0, 0,10,0,0,0]),
        #     102: np.array([0,0,0, 0,0,0, 0,20,0,0,0]),
        #     103: np.array([0,0,0, 0,0,0, 0,60,0,0,0]),
        #     104: np.array([0,0,0, 0,0,0, 0,90,0,0,0]),
        #     105: np.array([0,0,0, 0,0,0, 0,180,0,0,0]),
            
        #     106: np.array([0,0,0, 0,0,0, 0,0,-1,0,0]), 
        #     107: np.array([0,0,0, 0,0,0, 0,0,-5,0,0]),
        #     108: np.array([0,0,0, 0,0,0, 0,0,-10,0,0]),
        #     109: np.array([0,0,0, 0,0,0, 0,0,-20,0,0]),
        #     110: np.array([0,0,0, 0,0,0, 0,0,-60,0,0]),
        #     111: np.array([0,0,0, 0,0,0, 0,0,-90,0,0]),
        #     112: np.array([0,0,0, 0,0,0, 0,0,-180,0,0]),
            
        #     113: np.array([0,0,0, 0,0,0, 0,0,1,0,0]),
        #     114: np.array([0,0,0, 0,0,0, 0,0,5,0,0]),
        #     115: np.array([0,0,0, 0,0,0, 0,0,10,0,0]),
        #     116: np.array([0,0,0, 0,0,0, 0,0,20,0,0]),
        #     117: np.array([0,0,0, 0,0,0, 0,0,60,0,0]),
        #     118: np.array([0,0,0, 0,0,0, 0,0,90,0,0]),
        #     119: np.array([0,0,0, 0,0,0, 0,0,180,0,0]),
            
        #     120: np.array([0,0,0, 0,0,0, 0,0,0,-1,0]), 
        #     121: np.array([0,0,0, 0,0,0, 0,0,0,-5,0]),
        #     122: np.array([0,0,0, 0,0,0, 0,0,0,-10,0]),
        #     123: np.array([0,0,0, 0,0,0, 0,0,0,-20,0]),
        #     124: np.array([0,0,0, 0,0,0, 0,0,0,-60,0]),
        #     125: np.array([0,0,0, 0,0,0, 0,0,0,-90,0]),
        #     126: np.array([0,0,0, 0,0,0, 0,0,0,-180,0]),
            
        #     127: np.array([0,0,0, 0,0,0, 0,0,0,1,0]),
        #     128: np.array([0,0,0, 0,0,0, 0,0,0,5,0]),
        #     129: np.array([0,0,0, 0,0,0, 0,0,0,10,0]),
        #     130: np.array([0,0,0, 0,0,0, 0,0,0,20,0]),
        #     131: np.array([0,0,0, 0,0,0, 0,0,0,60,0]),
        #     132: np.array([0,0,0, 0,0,0, 0,0,0,90,0]),
        #     133: np.array([0,0,0, 0,0,0, 0,0,0,180,0]),
            
        #     134: np.array([0,0,0, 0,0,0, 0,0,0,0,-1]), 
        #     135: np.array([0,0,0, 0,0,0, 0,0,0,0,-5]),
        #     136: np.array([0,0,0, 0,0,0, 0,0,0,0,-10]),
        #     137: np.array([0,0,0, 0,0,0, 0,0,0,0,-20]),
        #     138: np.array([0,0,0, 0,0,0, 0,0,0,0,-60]),
        #     139: np.array([0,0,0, 0,0,0, 0,0,0,0,-90]),
        #     140: np.array([0,0,0, 0,0,0, 0,0,0,0,-180]),
            
        #     141: np.array([0,0,0, 0,0,0, 0,0,0,0,1]),
        #     142: np.array([0,0,0, 0,0,0, 0,0,0,0,5]),
        #     143: np.array([0,0,0, 0,0,0, 0,0,0,0,10]),
        #     144: np.array([0,0,0, 0,0,0, 0,0,0,0,20]),
        #     145: np.array([0,0,0, 0,0,0, 0,0,0,0,60]),
        #     146: np.array([0,0,0, 0,0,0, 0,0,0,0,90]),
        #     147: np.array([0,0,0, 0,0,0, 0,0,0,0,180]),
            
        #     148: np.array([0.1,0,0, 0,0,0, 0,0,0,0,0]),
        #     149: np.array([-0.1,0,0, 0,0,0, 0,0,0,0,0]),
        #     150: np.array([0,0.1,0, 0,0,0, 0,0,0,0,0]),
        #     151: np.array([0,-0.1,0, 0,0,0, 0,0,0,0,0]),
        #     152: np.array([0,0,0.1, 0,0,0, 0,0,0,0,0]),
        #     153: np.array([0,0,-0.1, 0,0,0, 0,0,0,0,0]),
        # }
                
        # self.action_space = spaces.Discrete(154)
        
        # self.action_space = spaces.Discrete(42)
        # self._actions_dict = {
        #     0: np.array([1,0,0, 0,0,0, 0,0,0,0,0]),
        #     1: np.array([5,0,0, 0,0,0, 0,0,0,0,0]),
        #     2: np.array([10,0,0, 0,0,0, 0,0,0,0,0]),
        #     3: np.array([20,0,0, 0,0,0, 0,0,0,0,0]),
        #     4: np.array([50,0,0, 0,0,0, 0,0,0,0,0]),
        #     5: np.array([100,0,0, 0,0,0, 0,0,0,0,0]),
            
        #     6: np.array([-1,0,0, 0,0,0, 0,0,0,0,0]),
        #     7: np.array([-5,0,0, 0,0,0, 0,0,0,0,0]),
        #     8: np.array([-10,0,0, 0,0,0, 0,0,0,0,0]),
        #     9: np.array([-20,0,0, 0,0,0, 0,0,0,0,0]),
        #     10: np.array([-50,0,0, 0,0,0, 0,0,0,0,0]),
        #     11: np.array([-100,0,0, 0,0,0, 0,0,0,0,0]),
            
        #     12: np.array([0,1,0, 0,0,0, 0,0,0,0,0]),
        #     13: np.array([0,5,0, 0,0,0, 0,0,0,0,0]),
        #     14: np.array([0,10,0, 0,0,0, 0,0,0,0,0]),
        #     15: np.array([0,20,0, 0,0,0, 0,0,0,0,0]),
        #     16: np.array([0,50,0, 0,0,0, 0,0,0,0,0]),
        #     17: np.array([0,100,0, 0,0,0, 0,0,0,0,0]),
            
        #     18: np.array([0,-1,0, 0,0,0, 0,0,0,0,0]),
        #     19: np.array([0,-5,0, 0,0,0, 0,0,0,0,0]),
        #     20: np.array([0,-10,0, 0,0,0, 0,0,0,0,0]),
        #     21: np.array([0,-20,0, 0,0,0, 0,0,0,0,0]),
        #     22: np.array([0,-50,0, 0,0,0, 0,0,0,0,0]),
        #     23: np.array([0,-100,0, 0,0,0, 0,0,0,0,0]),

        #     24: np.array([0,0,1, 0,0,0, 0,0,0,0,0]),
        #     25: np.array([0,0,5, 0,0,0, 0,0,0,0,0]),
        #     26: np.array([0,0,10, 0,0,0, 0,0,0,0,0]),
        #     27: np.array([0,0,20, 0,0,0, 0,0,0,0,0]),
        #     28: np.array([0,0,50, 0,0,0, 0,0,0,0,0]),
        #     29: np.array([0,0,100, 0,0,0, 0,0,0,0,0]),
            
        #     30: np.array([0,0,-1, 0,0,0, 0,0,0,0,0]),
        #     31: np.array([0,0,-5, 0,0,0, 0,0,0,0,0]),
        #     32: np.array([0,0,-10, 0,0,0, 0,0,0,0,0]),
        #     33: np.array([0,0,-20, 0,0,0, 0,0,0,0,0]),
        #     34: np.array([0,0,-50, 0,0,0, 0,0,0,0,0]),
        #     35: np.array([0,0,-100, 0,0,0, 0,0,0,0,0]),
            
        #     36: np.array([0.1,0,0, 0,0,0, 0,0,0,0,0]),
        #     37: np.array([-0.1,0,0, 0,0,0, 0,0,0,0,0]),
        #     38: np.array([0,0.1,0, 0,0,0, 0,0,0,0,0]),
        #     39: np.array([0,-0.1,0, 0,0,0, 0,0,0,0,0]),
        #     40: np.array([0,0,0.1, 0,0,0, 0,0,0,0,0]),
        #     41: np.array([0,0,-0.1, 0,0,0, 0,0,0,0,0]),
        # }
        
        
        # SIMPLE EXAMPLE:
        # self._actions_dict = {
        #     0: np.array([1,0,0, 0,0,0, 0,0,0,0,0]),
        #     1: np.array([0,1,0, 0,0,0, 0,0,0,0,0]),
        #     2: np.array([0,0,1, 0,0,0, 0,0,0,0,0]),
        #     3: np.array([-1,0,0, 0,0,0, 0,0,0,0,0]),
        #     4: np.array([0,-1,0, 0,0,0, 0,0,0,0,0]),
        #     5: np.array([0,0,-1, 0,0,0, 0,0,0,0,0]),
        # }
        # self.action_space = spaces.Discrete(6)
        
        # Displacement:
        # self.reset()
        

        # self.action_space = spaces.Discrete(3)
        
        
        # xArm
        self.action_space = spaces.Discrete(1)
        
        

        
        assert render_mode is None or render_mode in self.metadata["render_modes"]
        self.render_mode = render_mode

        """
        If human-rendering is used, `self.window` will be a reference
        to the window that we draw to. `self.clock` will be a clock that is used
        to ensure that the environment is rendered at the correct framerate in
        human-mode. They will remain `None` until human-mode is used for the
        first time.
        """
        self.window = None
        self.clock = None
        
        
    def _get_obs(self):
        return {"agent": self._agent_position, "target": self._target_position}
    
    # distance between 2 points in 3D space (agent's and target's xyz positions)
    def _get_info(self):
        # return {"distance": np.linalg.norm(self._agent_position[0:3] - self._target_position)}
        return {"distance": math.sqrt(math.pow(self._agent_position[0] - self._target_position[0], 2) + math.pow(self._agent_position[1] - self._target_position[1], 2) + math.pow(self._agent_position[2] - self._target_position[2], 2)* 1.0),
                "num_steps": self.num_steps,
                "agent_pos": self._agent_position,
                "target_pos": self._target_position}
    
    def _get_x_displacement(self):
        return -(self._agent_position[0] - self._target_position[0]) # take negative because want agent to move in the direction of the displacement
    def _get_y_displacement(self):
        return -(self._agent_position[1] - self._target_position[1])
    def _get_z_displacement(self):
        return -(self._agent_position[2] - self._target_position[2])
    
    def get_random_target_pos(self):
        return np.array([self.np_random.integers(low=self.target_x_low, high=self.target_x_high), self.np_random.integers(low=self.target_y_low, high=self.target_y_high), self.np_random.integers(low=self.target_z_low, high=self.target_z_high) ])
    
    def update_agent_position(self):
        self._agent_position[0:6] = self.robot_main._arm.get_position()[1] # [x,y,z,roll,pitch,yaw]
        self._agent_position[6:11] = self.robot_main._arm.get_servo_angle()[1][0:5] # 5 joint angles
        
    
    """
    Agent's initial position , joints and rotation of the robot must be static -> not random values (for now assume initial position, would be better to start from zero position but cant do that bc not enough space for robot to go back to zero position).
    
    Target's position will be random 
    """
    def reset(self, seed=None):
        """
        Returns the observation of the initial state
        Reset the environment to initial state so that a new episode (independent of previous ones) may start
        """
        super().reset(seed=seed)
        
        # xArm: Agent's initial values will be static based off of values at initial position set on the arm
        # self._agent_position = np.array([-12.1,181.6,222.3, 180,0,-91.5, 93.8,-58.8,5.2,53.6,185.3]) # [x,y,z, roll,pitch,yaw, J1,J2,J3,J4,J5]
        
        
        # SIMPLE EXAMPLE:
        # self._agent_position = np.array([2,-1,5, -99999,0,-91.5, 93.8,-58.8,5.2,53.6,185.3]) # [x,y,z, roll,pitch,yaw, J1,J2,J3,J4,J5]
        
        # move robot to initial position
        self._agent_position = np.array([0,0,0,0,0,0,0,0,0,0,0]) # dummy values just to initialize _agent_position
        self.robot_main.move_initial()
        # self.update_agent_position()
        self._agent_position[0:6] = self.robot_main._arm.get_position()[1] # [x,y,z,roll,pitch,yaw]
        self._agent_position[6:11] = self.robot_main._arm.get_servo_angle()[1][0:5] # 5 joint angles
        

        
        # ADD LATER: speed and gripper clamping width

        # self._target_position = self.get_random_target_pos()
        # # We will sample the target's location randomly until it does not coincide with the agent's location
        # while np.array_equal(self._target_position, self._agent_position[0:3]):
        #     self._target_position = self.get_random_target_pos()
        
        
        # xArm
        self._target_position = np.array([-15,200,200])

            

        observation = self._get_obs()
        info = self._get_info()

        if self.render_mode == "human":
            self._render_frame()

        return observation, info
        
    def step(self, action):
        """
        Returns: Given current obs and action, returns the next observation, the reward, done and optionally additional info
        """
        self.num_steps += 1
        
        # action dictionary has to be defined here bc upon every step/action, the displacement changes so dictionary action must be reinitialized
        # self._actions_dict = {
        #     0: np.array([self._get_x_displacement(),0,0, 0,0,0, 0,0,0,0,0]), # x displacement
        #     1: np.array([0,self._get_y_displacement(),0, 0,0,0, 0,0,0,0,0]), # y displacement
        #     2: np.array([0,0,self._get_z_displacement(), 0,0,0, 0,0,0,0,0]), # z displacement
        # }
        
        #xArm
        self._actions_dict = {
            0: self.robot_main.test(self._target_position)
        }
        #update agent position by getting robot's current coordinates after move
        # self.update_agent_position()
        self._agent_position[0:6] = self.robot_main._arm.get_position()[1] # [x,y,z,roll,pitch,yaw]
        self._agent_position[6:11] = self.robot_main._arm.get_servo_angle()[1][0:5] # 5 joint angles
        print(self._agent_position)
        
        #IMPORTANT: state transition conditional branch could be implemented here -> NO bc then an action may get a reward that was from doing a different action
        
        # # get move array based on action index
        # move = self._actions_dict[action]
        
        
        # self._agent_position += move
        
        # Avoid out of bounds by truncating to upper/lower limit
        if self._agent_position[0] < self.agent_x_low:
            self._agent_position[0] = self.agent_x_low
        elif self._agent_position[0] > self.agent_x_high:
            self._agent_position[0] = self.agent_x_high
        elif self._agent_position[1] < self.agent_y_low:
            self._agent_position[1] = self.agent_y_low
        elif self._agent_position[1] > self.agent_y_high:
            self._agent_position[1] = self.agent_y_high
        elif self._agent_position[2] < self.agent_z_low:
            self._agent_position[2] = self.agent_z_low
        elif self._agent_position[2] > self.agent_z_high:
            self._agent_position[2] = self.agent_z_high
            
        elif self._agent_position[3] < self.agent_roll_pitch_yaw_low:
            self._agent_position[3] = self.agent_roll_pitch_yaw_low
        elif self._agent_position[3] > self.agent_roll_pitch_yaw_high:
            self._agent_position[3] = self.agent_roll_pitch_yaw_high
        elif self._agent_position[4] < self.agent_roll_pitch_yaw_low:
            self._agent_position[4] = self.agent_roll_pitch_yaw_low
        elif self._agent_position[4] > self.agent_roll_pitch_yaw_high:
            self._agent_position[4] = self.agent_roll_pitch_yaw_high
        elif self._agent_position[5] < self.agent_roll_pitch_yaw_low:
            self._agent_position[5] = self.agent_roll_pitch_yaw_low
        elif self._agent_position[5] > self.agent_roll_pitch_yaw_high:
            self._agent_position[5] = self.agent_roll_pitch_yaw_high
            
        elif self._agent_position[6] < self.agent_joint1_low:
            self._agent_position[6] = self.agent_joint1_low
        elif self._agent_position[6] > self.agent_joint1_high:
            self._agent_position[6] = self.agent_joint1_high
            
        elif self._agent_position[7] < self.agent_joint2_low:
            self._agent_position[7] = self.agent_joint2_low
        elif self._agent_position[7] > self.agent_joint2_high:
            self._agent_position[7] = self.agent_joint2_high
            
        elif self._agent_position[8] < self.agent_joint3_low:
            self._agent_position[8] = self.agent_joint3_low
        elif self._agent_position[8] > self.agent_joint3_high:
            self._agent_position[8] = self.agent_joint3_high
            
        elif self._agent_position[9] < self.agent_joint4_low:
            self._agent_position[9] = self.agent_joint4_low
        elif self._agent_position[9] > self.agent_joint4_high:
            self._agent_position[9] = self.agent_joint4_high
            
        elif self._agent_position[10] < self.agent_joint5_low:
            self._agent_position[10] = self.agent_joint5_low
        elif self._agent_position[10] > self.agent_joint5_high:
            self._agent_position[10] = self.agent_joint5_high
        
        
        # IMPORTANT: I NEED to account that moving joints will also move the robot's xyz 
        # -> set_servo_angle() is called for the joint motion command, maybe afterward, call get_position() ? -> xArm python SDK

        
        # maybe here, if action is within a certain range, perform the needed robot command
        # ex: 0 < action < 78 would be passing the first 6 values in self._agent_position to the code to perform a linear motion command for robot
        # > 78 would be joint motion code to perform  
        
        # An episode is done iff the agent has reached the target
        # CHANGE LATER: account if chosen movement resulted in robot error getting stuck, going out of bounds, colliding, speed too fast? etc
        z_offset = 0 # offset z bc the z of the agent should be more than the target since agent's z is not at tip of gripper
        terminated = np.array_equal(self._agent_position[0:3] , self._target_position + [0,0,z_offset])  
        # reward = 1 if terminated else 0  # Binary sparse rewards (FOR NOW, CHANGE LATER and make a CUSTOM REWARD SYSTEM)
                
        info = self._get_info()

        if terminated:
            reward = 10
        # elif info["distance"] < self._agent_position[3]: # give reward if the move chosen made the agent closer to target than previously
        #     reward = 0  

        else: reward = 0     
        # store euclidean distance in index 3
        # self._agent_position[3] = info["distance"]
        
        if reward > 1:
            print("AGENT REACHED TARGET!!!")
            # exit()
        
        observation = self._get_obs()

        if self.render_mode == "human":
            self._render_frame()


        # self._actions_dict = {
        #     0: np.array([self._get_x_displacement(),0,0, 0,0,0, 0,0,0,0,0]), # x displacement
        #     1: np.array([0,self._get_y_displacement(),0, 0,0,0, 0,0,0,0,0]), # y displacement
        #     2: np.array([0,0,self._get_z_displacement(), 0,0,0, 0,0,0,0,0]), # z displacement
        # }

        return observation, reward, terminated, False, info # WHAT does the False represent
    
    
    # pybullet?
    # gymnasium-robotics??
    # IDEALLY have 3d simulation of our robot, conveyor belt and object
    def render(self):
        fig = go.Figure(data=[
            go.Scatter3d(
                x=self._agent_position[0], y=self._agent_position[1], z=self._agent_position[2],
                mode='markers',
                text="AGENT", #label coordinate point
                name="Agent Coordinate",
                marker = dict(
                    size=6,
                    opacity=.5,
                    colorscale=px.colors.sequential.Bluyl)
            ),
            go.Scatter3d(
                x=self._target_position[0], y=self._target_position[1], z=self._target_position[2],
                mode='markers', 
                text="TARGET",
                name="Target Coordinate",
                marker = dict(
                    size=6,
                    opacity=.5,
                    colorscale=px.colors.sequential.Hot)
            )     
        ])
        
        fig.update_layout(scene = dict(
                        xaxis_title='x',
                        yaxis_title='y',
                        zaxis_title='z'),
                        )
        
        fig.show()