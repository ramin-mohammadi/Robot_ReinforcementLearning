#import gym
# NOTE: gym seems to be old and gymnasium is new maintained environment package
import numpy as np
import gymnasium as gym
from gymnasium import spaces

# FIRST GOAL get robot arm to go to detected location from camera, then can enhance to move object to a locaiton after picking it up and other more complicated tasks

class CustomEnv(gym.Env): 
    metadata = {"render_modes": ["human", "rgb_array"], "render_fps": 4}
    
    def __init__(self, render_mode=None):
        """
        Must define self.observation_space ad self.action_space here
        """
        # Bound: grid workspace (3D plane xyz) of conveyor belt??? -> possibly not true because the location of the robots are not in the conveyor belt grid
        # solution is to assume from the vertex (0,0,0) that me and Brian chose, the bounds is the entire robot working field? Or each robot will get a unique bound being how far they can reach in all directions
        # units is millimeters
        
        # THERE ARE DIFFERENT BOUNDARY VALUES for the smaller and bigger arms (LOOK at their manuals appendix which are the last few pages)
        
        # xArm 5 (big arm)
        self.x_low = -700
        self.x_high = 700
        self.y_low = -700
        self.y_high = 700
        self.z_low = 0
        self.z_high = 950
        
        self.roll_pitch_yaw_low = -180
        self.roll_pitch_yaw_high = 180
        
        self.joint1_low = -360
        self.joint1_high = 360
        self.joint2_low = -118
        self.joint2_high = 120
        self.joint3_low = -225
        self.joint3_high = 11
        self.joint4_low = -97
        self.joint4_high = 180
        self.joint5_low = -360
        self.joint5_high = 360
        
        # object on conveyor belt
        self.target_x_low = -300
        self.target_x_high = 300
        self.target_y_low = -400
        self.target_y_high = 400

        
        # robot arm is the agent, target is the object on the conveyor belt
        # i have specified the range of possible values for xyz THEN the shape is an array of 3 elements for xyz  [-2,200,11]
        # For the agent it is [x,y,z, Roll, Pitch, Yaw, J1,J2,J3,J4,J5] -> smaller arms have 6 joints so IMPORTANT: EACH arm will have their own custom "environment"
        # target will just have [x,y,z]
        self.observation_space = spaces.Dict( # later may have to account for observing if other arms are moving???
            {
                "agent" : spaces.Box(low=np.array([self.x_low, self.y_low, self.z_low, self.roll_pitch_yaw_low, self.roll_pitch_yaw_low, self.roll_pitch_yaw_low, self.joint1_low, self.joint2_low, self.joint3_low, self.joint4_low, self.joint5_low]), 
                high=np.array([self.x_high, self.y_high, self.z_high, self.roll_pitch_yaw_high, self.roll_pitch_yaw_high, self.joint1_high, self.joint2_high, self.joint3_high, self.joint4_high, self.joint5_high]), 
                                     shape=(11,), dtype=np.float32), # independent bound for each dimension
                # the z axis may be useless for the object 
                "target" : spaces.Box(low=np.array([self.target_x_low, self.target_y_low]), high=np.array([self.target_x_low, self.target_y_high]), 
                                      shape=(2,), dtype=np.float32),
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
        
        PROBLEM: determining how many units to move by (dijkstras? shortest path?) -> MAYBE just provide multiple 
        intervals it could possibly be moved at a time so 1,5,10,20,50,100mm
        then for degrees 1, 5, 10, 20, 60, 90, 180
        """
        MOVE_RIGHT_1 = 0
        MOVE_RIGHT_5 = 1
        MOVE_RIGHT_10 = 2
        MOVE_RIGHT_20 = 3
        MOVE_RIGHT_50 = 4
        MOVE_RIGHT_100 = 5

        MOVE_LEFT_1 = 6
        MOVE_LEFT_5 = 7
        MOVE_LEFT_10 = 8
        MOVE_LEFT_20 = 9
        MOVE_LEFT_50 = 10
        MOVE_LEFT_100 = 11
        
        MOVE_DOWN_1 = 12
        MOVE_DOWN_5 = 13
        MOVE_DOWN_10 = 14
        MOVE_DOWN_20 = 15
        MOVE_DOWN_50 = 16
        MOVE_DOWN_100 = 17

        MOVE_UP_1 = 18
        MOVE_UP_5 = 19
        MOVE_UP_10 = 20
        MOVE_UP_20 = 21
        MOVE_UP_50 = 22
        MOVE_UP_100 = 23

        ROTATE_CLOCKWISE_ROLL_1 = 24
        ROTATE_CLOCKWISE_ROLL_5 = 25
        ROTATE_CLOCKWISE_ROLL_10 = 26
        ROTATE_CLOCKWISE_ROLL_20 = 27
        ROTATE_CLOCKWISE_ROLL_60 = 28
        ROTATE_CLOCKWISE_ROLL_90 = 29
        ROTATE_CLOCKWISE_ROLL_180 = 30
        
        ROTATE_COUNTERCLOCKWISE_ROLL_1 = 31
        ROTATE_COUNTERCLOCKWISE_ROLL_5 = 32
        ROTATE_COUNTERCLOCKWISE_ROLL_10 = 33
        ROTATE_COUNTERCLOCKWISE_ROLL_20 = 34
        ROTATE_COUNTERCLOCKWISE_ROLL_60 = 35
        ROTATE_COUNTERCLOCKWISE_ROLL_90 = 36
        ROTATE_COUNTERCLOCKWISE_ROLL_180 = 37
        
        ROTATE_CLOCKWISE_PITCH_1 = 38
        ROTATE_CLOCKWISE_PITCH_5 = 39
        ROTATE_CLOCKWISE_PITCH_10 = 40
        ROTATE_CLOCKWISE_PITCH_20 = 41
        ROTATE_CLOCKWISE_PITCH_60 = 42
        ROTATE_CLOCKWISE_PITCH_90 = 43
        ROTATE_CLOCKWISE_PITCH_180 = 44

        ROTATE_COUNTERCLOCKWISE_PITCH_1 = 45
        ROTATE_COUNTERCLOCKWISE_PITCH_5 = 46
        ROTATE_COUNTERCLOCKWISE_PITCH_10 = 47
        ROTATE_COUNTERCLOCKWISE_PITCH_20 = 48
        ROTATE_COUNTERCLOCKWISE_PITCH_60 = 49
        ROTATE_COUNTERCLOCKWISE_PITCH_90 = 50
        ROTATE_COUNTERCLOCKWISE_PITCH_180 = 51
        
        ROTATE_CLOCKWISE_YAW_1 = 52
        ROTATE_CLOCKWISE_YAW_5 = 53
        ROTATE_CLOCKWISE_YAW_10 = 54
        ROTATE_CLOCKWISE_YAW_20 = 55
        ROTATE_CLOCKWISE_YAW_60 = 56
        ROTATE_CLOCKWISE_YAW_90 = 57
        ROTATE_CLOCKWISE_YAW_180 = 58

        ROTATE_COUNTERCLOCKWISE_YAW_1 = 59
        ROTATE_COUNTERCLOCKWISE_YAW_5 = 60
        ROTATE_COUNTERCLOCKWISE_YAW_10 = 61
        ROTATE_COUNTERCLOCKWISE_YAW_20 = 62
        ROTATE_COUNTERCLOCKWISE_YAW_60 = 63
        ROTATE_COUNTERCLOCKWISE_YAW_90 = 64
        ROTATE_COUNTERCLOCKWISE_YAW_180 = 65
        
        ROTATE_CLOCKWISE_J1_1 = 66
        ROTATE_CLOCKWISE_J1_5 = 67
        ROTATE_CLOCKWISE_J1_10 = 68
        ROTATE_CLOCKWISE_J1_20 = 69
        ROTATE_CLOCKWISE_J1_60 = 70
        ROTATE_CLOCKWISE_J1_90 = 71
        ROTATE_CLOCKWISE_J1_180 = 72
        
        ROTATE_COUNTERCLOCKWISE_J1_1 = 73
        ROTATE_COUNTERCLOCKWISE_J1_5 = 74
        ROTATE_COUNTERCLOCKWISE_J1_10 = 75
        ROTATE_COUNTERCLOCKWISE_J1_20 = 76
        ROTATE_COUNTERCLOCKWISE_J1_60 = 77
        ROTATE_COUNTERCLOCKWISE_J1_90 = 78
        ROTATE_COUNTERCLOCKWISE_J1_180 = 79

        ROTATE_CLOCKWISE_J2_1 = 80
        ROTATE_CLOCKWISE_J2_5 = 81
        ROTATE_CLOCKWISE_J2_10 = 82
        ROTATE_CLOCKWISE_J2_20 = 83
        ROTATE_CLOCKWISE_J2_60 = 84
        ROTATE_CLOCKWISE_J2_90 = 85
        ROTATE_CLOCKWISE_J2_180 = 86
        
        ROTATE_COUNTERCLOCKWISE_J2_1 = 87
        ROTATE_COUNTERCLOCKWISE_J2_5 = 88
        ROTATE_COUNTERCLOCKWISE_J2_10 = 89
        ROTATE_COUNTERCLOCKWISE_J2_20 = 90
        ROTATE_COUNTERCLOCKWISE_J2_60 = 91
        ROTATE_COUNTERCLOCKWISE_J2_90 = 92
        ROTATE_COUNTERCLOCKWISE_J2_180 = 93
        
        ROTATE_CLOCKWISE_J3_1 = 94
        ROTATE_CLOCKWISE_J3_5 = 95
        ROTATE_CLOCKWISE_J3_10 = 96
        ROTATE_CLOCKWISE_J3_20 = 97
        ROTATE_CLOCKWISE_J3_60 = 98
        ROTATE_CLOCKWISE_J3_90 = 99
        ROTATE_CLOCKWISE_J3_180 = 100
        
        ROTATE_COUNTERCLOCKWISE_J3_1 = 101
        ROTATE_COUNTERCLOCKWISE_J3_5 = 102
        ROTATE_COUNTERCLOCKWISE_J3_10 = 103
        ROTATE_COUNTERCLOCKWISE_J3_20 = 104
        ROTATE_COUNTERCLOCKWISE_J3_60 = 105
        ROTATE_COUNTERCLOCKWISE_J3_90 = 106
        ROTATE_COUNTERCLOCKWISE_J3_180 = 107
        
        ROTATE_CLOCKWISE_J4_1 = 108
        ROTATE_CLOCKWISE_J4_5 = 109
        ROTATE_CLOCKWISE_J4_10 = 110
        ROTATE_CLOCKWISE_J4_20 = 111
        ROTATE_CLOCKWISE_J4_60 = 112
        ROTATE_CLOCKWISE_J4_90 = 113
        ROTATE_CLOCKWISE_J4_180 = 114
        
        ROTATE_COUNTERCLOCKWISE_J4_1 = 115
        ROTATE_COUNTERCLOCKWISE_J4_5 = 116
        ROTATE_COUNTERCLOCKWISE_J4_10 = 117
        ROTATE_COUNTERCLOCKWISE_J4_20 = 118
        ROTATE_COUNTERCLOCKWISE_J4_60 = 119
        ROTATE_COUNTERCLOCKWISE_J4_90 = 120
        ROTATE_COUNTERCLOCKWISE_J4_180 = 121
        
        ROTATE_CLOCKWISE_J5_1 = 122
        ROTATE_CLOCKWISE_J5_5 = 123
        ROTATE_CLOCKWISE_J5_10 = 124
        ROTATE_CLOCKWISE_J5_20 = 125
        ROTATE_CLOCKWISE_J5_60 = 126
        ROTATE_CLOCKWISE_J5_90 = 127
        ROTATE_CLOCKWISE_J5_180 = 128
        
        ROTATE_COUNTERCLOCKWISE_J5_1 = 129
        ROTATE_COUNTERCLOCKWISE_J5_5 = 130
        ROTATE_COUNTERCLOCKWISE_J5_10 = 131
        ROTATE_COUNTERCLOCKWISE_J5_20 = 132
        ROTATE_COUNTERCLOCKWISE_J5_60 = 133
        ROTATE_COUNTERCLOCKWISE_J5_90 = 134
        ROTATE_COUNTERCLOCKWISE_J5_180 = 135
                
        self.action_space = spaces.Discrete(136)
        
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
        return {"agent": self._agent_location, "target": self._target_location}
    
    def _get_info(self):
        return {"distance": np.linalg.norm(self._agent_location - self._target_location)}
    
        
    def reset(self):
        """
        Returns the observation of the initial state
        Reset tge ebvuronment to initial state so that a new episode (independent of previous ones) may start
        ""
        
        