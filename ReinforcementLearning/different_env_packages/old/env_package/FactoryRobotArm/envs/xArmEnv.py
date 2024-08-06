#import gym
# NOTE: gym seems to be old and gymnasium is new maintained environment package
import numpy as np
import gymnasium as gym
from gymnasium import spaces
import math
import time

import plotly.express as px
import plotly.graph_objects as go

# directory to these local files is relative to the python package being the folder env_package
from FactoryRobotArm import xArmClass
# import obj_det_xyz_angle


# IMPORTANT: POSSIBLY CAN IMPLEMENT CONTROL FREQUENCY WITH time.sleep() at the end of a step()

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
        
        self.agent_x_vel_low = -10
        self.agent_x_vel_high = 10
        self.agent_y_vel_low = -10
        self.agent_y_vel_high = 10
        self.agent_z_vel_low = -10
        self.agent_z_vel_high = 10
        
        self.agent_x_low = -85
        self.agent_x_high = 270
        self.agent_y_low = 185
        self.agent_y_high = 365
        self.agent_z_low = 143
        self.agent_z_high = 290
        
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
        self.target_x_low = -85 #-300
        self.target_x_high = 270 #300
        self.target_y_low = 185 #-400
        self.target_y_high = 365 # 400  
        self.target_z_low = 143 # 0
        self.target_z_high = 290 # 400        
        
        # SIMPLE EXAMPLE:
        # self.target_x_low = -3
        # self.target_x_high = 3
        # self.target_y_low = -3
        # self.target_y_high = 3
        # self.target_z_low = -3
        # self.target_z_high = 3
        

        
        # robot arm is the agent, target is the object on the conveyor belt
        # i have specified the range of possible values for xyz THEN the shape is an array of 3 elements for xyz  [-2,200,11]
        # For the agent it is [x,y,z, Roll, Pitch, Yaw, J1,J2,J3,J4,J5] -> smaller arms have 6 joints so IMPORTANT: EACH arm will have their own custom "environment"
        # target will just have [x,y,z]
        
        
        
        
        # (agent x, agent y, agent z, target x, target y, target z, state_gripper -> 1 if open 0 if closed,  5 most recent historical actions)
        # STATES should also be placed in the observation space for model to learn from
        self.observation_space = spaces.Dict( # later may have to account for observing if other arms are moving???
            {
                "agent" : spaces.Box(low=np.array([self.agent_x_low, self.agent_y_low, self.agent_z_low, self.agent_roll_pitch_yaw_low, self.agent_roll_pitch_yaw_low, self.agent_roll_pitch_yaw_low, self.agent_joint1_low, self.agent_joint2_low, self.agent_joint3_low, self.agent_joint4_low, self.agent_joint5_low, self.agent_x_vel_low, self.agent_y_vel_low, self.agent_z_vel_low]), 
                                     high=np.array([self.agent_x_high, self.agent_y_high, self.agent_z_high, self.agent_roll_pitch_yaw_high, self.agent_roll_pitch_yaw_high, self.agent_roll_pitch_yaw_high,self.agent_joint1_high, self.agent_joint2_high, self.agent_joint3_high, self.agent_joint4_high, self.agent_joint5_high, self.agent_x_vel_high, self.agent_y_vel_high, self.agent_z_vel_high]), 
                                     shape=(14,), dtype=np.float32), 
                # the z axis is needed for the target to help guide the agent's z position to the target's -> But PROBLEM here bc z pos is the attachment point of the gripper, not the tip of the gripper -> assume z is static (figure out beforehand)
                "target" : spaces.Box(low=np.array([self.target_x_low, self.target_y_low, self.target_z_low]), 
                                      high=np.array([self.target_x_high, self.target_y_high, self.target_z_high]), 
                                      shape=(3,), dtype=np.float32),
                "gripper_state": spaces.Discrete(2), # {0,1}  , 1 if open 0 if closed
                "grabbed_target": spaces.Discrete(2), # {0,1}  , 1 if picked up 0 otherwise/dropped block
                "arm_reached_target": spaces.Discrete(2), # {0,1}  , 1 if arm reached target 0 otherwise
                "target_reached_destination": spaces.Discrete(2), # {0,1}  , 1 if block reached destination 0 otherwise
                "collision": spaces.Discrete(2), # {0,1}  , 1 if collision occurred 0 otherwise
                "historical_actions": spaces.Box(low=-1, high=3, shape=(5,), dtype=int),  # 5 most recent actions (t-5, t-4, t-3, t-2, t-1)
                "current_checkpoint": spaces.Discrete(3),
            }
        )

        
        
        
        
        # # xArm
        #self.action_space = spaces.Discrete(4)

        # ACTON SPACE FOR PPO CANT BE A DICTIONARY
        self.action_space = spaces.Dict(
            { 
                # movement (position) along x, y, and z axes
                "continuous": spaces.Box(low=np.array([self.agent_x_vel_low, self.agent_y_vel_low, self.agent_z_vel_low,]),
                                         high=np.array([self.agent_x_vel_high, self.agent_y_vel_high, self.agent_z_vel_high,]),
                                                     shape=(3,), dtype=np.float32),
                # gripper control simplified to just open or closed
                "gripper" : spaces.Discrete(2) #{0, 1} , 1 if open, 0 if closed
            }
        )
        
        

        
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
        self._agent_position = np.array([0,0,0,0,0,0,0,0,0,0,0, 0, 0, 0], dtype=float) # dummy values just to initialize _agent_position
        self.robot_main.move_initial()
        # self.update_agent_position()
        self._agent_position[0:6] = self.robot_main._arm.get_position()[1] # [x,y,z,roll,pitch,yaw]
        self._agent_position[6:11] = self.robot_main._arm.get_servo_angle()[1][0:5] # 5 joint angles
        

        self.robot_main.open_gripper()
        self.gripper_state = 1
        
        
        self.previous_checkpoint = 0
        # 0: starting
        # 1: gripper closed at the target (picked up block)
        # 2: block at destination (final checkpoint)
        self.current_checkpoint = 0  
        
        # ADD LATER: speed and gripper clamping width

        # self._target_position = self.get_random_target_pos()
        # # We will sample the target's location randomly until it does not coincide with the agent's location
        # while np.array_equal(self._target_position, self._agent_position[0:3]):
        #     self._target_position = self.get_random_target_pos()
        
        
        # xArm
        self._target_position = np.array([1.9,281.3,144])
        self._destination_position = np.array([211, 113.6, 268.7])
        

        self.state_last_action = { # -1
            "continuous": np.array([0,0,0], dtype=float),
            "gripper": 1
        }
        # if agent at target and robot closed gripper
        self.state_grabbed_target = 0 # if true (1), target position will be updated with agent's position
        self.state_target_reached_destination = 0
        self.state_arm_reached_target = 0
        self.state_collision = 0 # 1 if collision occurred such as arm moving to block with grippers closed
        
        
        self.prev_agent_target_distance = 99999999
        self.prev_target_dest_distance = 99999999
        
        # 5 previous actions
        self.historical_actions = np.array([ 
            { 
            "continuous": np.array([0,0,0], dtype=float),
            "gripper": 1
            },
            { 
            "continuous": np.array([0,0,0], dtype=float),
            "gripper": 1
            },
            { 
                "continuous": np.array([0,0,0], dtype=float),
                "gripper": 1
            },
            {  
                "continuous": np.array([0,0,0], dtype=float),
                "gripper": 1
            },
            { 
                "continuous": np.array([0,0,0], dtype=float),
                "gripper": 1
            }
        ])

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
        
        self.previous_checkpoint = self.current_checkpoint
        

        '''
        # action dictionary has to be defined here bc upon every step/action, the displacement changes so dictionary action must be reinitialized
        # self._actions_dict = {
        #     0: np.array([self._get_x_displacement(),0,0, 0,0,0, 0,0,0,0,0]), # x displacement
        #     1: np.array([0,self._get_y_displacement(),0, 0,0,0, 0,0,0,0,0]), # y displacement
        #     2: np.array([0,0,self._get_z_displacement(), 0,0,0, 0,0,0,0,0]), # z displacement
        # }
        
        #xArm , FOR SOME WEIRD REASON, these actions are performed sequentially (maybe initially), and: within one step() it performes all 4 actions
        # self._actions_dict = {
        #     0: self.robot_main.move_to(self._target_position),
        #     2: self.robot_main.close_gripper(),            
        #     1: self.robot_main.move_to(self._destination_position),
        #     3: self.robot_main.open_gripper()
        # }
        # DONT DO ABOVE, will literally call the functions line by line
        
        
        
        # Old Discrete Actions
        # if action == 0:
        #     yaw_rotation = -91.5

        #     # connect to camera that performs object detection (detects red or blue) to get x,y, and yaw degree rotation
        #     camera_info = self.get_xy_rotation_block() # returns [x,y, yaw degree rotation]
        #     self._target_position[0:2] = camera_info[0:2]
        #     yaw_rotation = camera_info[2]
            
        #     self.robot_main.move_to(self._target_position, yaw_rotation)
        # elif action == 1:
        #     yaw_rotation = -91.5
        #     self.robot_main.move_to(self._destination_position, -91.5)
        # elif action == 2:
        #     self.robot_main.close_gripper()
        #     self.gripper_state = 0
        # elif action == 3:
        #     self.robot_main.open_gripper()
        #     self.gripper_state = 1
        # '''
        
        # New Action Space
        # move to x y z that policy chose
        # self.robot_main.move_to(action["continuous"], yaw_rotation=-90) 

        # gripper: 1 is open, 0 is close
        if action["gripper"] == 1:
            self.robot_main.open_gripper()
            self.gripper_state = 1
        elif action["gripper"] == 0:
            self.robot_main.close_gripper()
            self.gripper_state = 0

        # set robot velocity x y z using continuous action space that policy picked
        self.robot_main.action_velocity(x_velocity=action['continuous'][0], 
                                        y_velocity=action['continuous'][1], 
                                        z_velocity=action['continuous'][2], 
                                        duration=1.5) # 1.5 seconds
        
        # to update observation for velocity x y z
        self._agent_position[11:14] = [action['continuous'][0], action['continuous'][1], action['continuous'][2]] 
            
            
        print("Action:")
        print(" Velocities  (x, y, z): ", action["continuous"])
        print("   Gripper: ", "Open" if self.gripper_state == 1 else "Closed")
        
        print("Last Action: ", self.state_last_action)
        print("   Velocities (x, y, z): ", self.state_last_action["continuous"])
        print("   Gripper: ", "Open" if self.state_last_action["gripper"] == 1 else "Closed")
        
        
        # update agent position by getting robot's current coordinates, rotation ,and joints after move
        # self.update_agent_position()
        self._agent_position[0:6] = self.robot_main._arm.get_position()[1] # [x,y,z,roll,pitch,yaw]
        self._agent_position[6:11] = self.robot_main._arm.get_servo_angle()[1][0:5] # 5 joint angles

        # round to first decimal bc precision for get_position is to the 7th decimal so agent pos never equals target pos
        self._agent_position[0:3] = np.round(self._agent_position[0:3], decimals=1) 
        
        
        xy_error = 5 # mm in x and y directions for agent to have reached target
        z_max = 151 # max z-coordinate value for block grabbing
        
        # arm has to be within xy_error mm in x and y from the target's x y, and within robot z: (143 -> 151 mm) to have reached the target 
        if(abs(self._agent_position[0]-self._target_position[0]) <= xy_error and
           abs(self._agent_position[1]-self._target_position[1]) <= xy_error and
           self.target_z_low <= self._target_position[2] <= z_max):
           # 1 = true
           self.state_arm_reached_target = 1
           

           # check to see if gripper closed when arm is at target position
           if self.gripper_state == 0: # 0 = closed
                self.state_grabbed_target = 1
                self.current_checkpoint = 1
        
        z_max_dest = 231
        # if gripper opens and is not in the destination nor target location
        if(self.gripper_state == 1 and 
           not (abs(self._agent_position[0]-self._target_position[0]) <= xy_error and
                abs(self._agent_position[1]-self._target_position[1]) <= xy_error and
                self.target_z_low <= self._target_position[2] <= z_max) and
           not (abs(self._agent_position[0]-self._destination_position[0]) <= xy_error and
                abs(self._agent_position[1]-self._destination_position[1]) <= xy_error and
                self._destination_position[2] <= self._target_position[2] <= z_max_dest)):
            self.state_grabbed_target = 0
            self.state_arm_reached_target = 0
            self.current_checkpoint = 0
 
        elif(self.gripper_state == 1): # anytime the gripper is opened assume not going to be updating target position anymore
            self.state_grabbed_target = 0
            self.current_checkpoint = 0
    
        # # if agent at target and robot closed gripper, update target position with agent's position (robot is holding block)
        # if(np.array_equal(self._agent_position[0:3], self._target_position) and self.gripper_state == 0):
        #     self.state_grabbed_target = 1
        # robot chose to open gripper while it was previously holding target
        # elif(np.array_equal(self._agent_position[0:3], self._target_position) and self.state_last_action == 3):
         
        print("STATE CHECKPOINT: ", self.state_grabbed_target)

        print("AGENT: ", self._agent_position)
        
        # print("TARGET: ", self._target_position)        

        # This assumes that the robot is holding the block so update the target position with the robot's
        if self.state_grabbed_target == 1 and self.gripper_state == 0:
            # DO NOT DO BELOW cause target variable will now point to the agent pos address so when agent pos is updated, so is target:
            # self._target_position = self._agent_position[0:3] 
            self._target_position = np.round(self.robot_main._arm.get_position()[1][0:3], decimals=1)
            # print("Updating target position")

        print("TARGET: ", self._target_position)        
   
   
        '''
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
        '''
        

        
        # CHANGE LATER: account if chosen movement resulted in robot error getting stuck, going out of bounds, colliding, speed too fast? etc
        
        #terminated = np.array_equal(self._target_position, self._destination_position)  
        
        # TERMINATION SUCCESS: if arm within destination area, target has reached destination
        if (self.state_grabbed_target and
            abs(self._agent_position[0]-self._destination_position[0]) <= xy_error and
            abs(self._agent_position[1]-self._destination_position[1]) <= xy_error and
            self._destination_position[2] <= self._target_position[2] <= z_max_dest):
            self.state_target_reached_destination = 1
            terminated = True
            self.current_checkpoint = 2
 
        # print("Terminated: ", terminated)
        
        # END EPISODE if agent moves to the target with its gripper closed 
        # if(action == 0 and self.gripper_state == 0 and terminated == False):
        #     terminated = True
        
        # if np.array_equal(self._agent_position[0:3], self._target_position):
        #     self.state_arm_reached_target = 1
        # else: self.state_arm_reached_target = 0
        
                
        info = self._get_info()

        ########################## REWARD SYSTEM ###############################

        # Robot has not yet picked up block
        if (self.current_checkpoint == 0 and 
            self.state_arm_reached_target == 0): # not yet reached target
            # reward is euclidean distance
            reward = np.sqrt(((self._agent_position[0] - self._target_position[0]) ** 2) +
                             ((self._agent_position[1] - self._target_position[1]) ** 2) + 
                             ((self._agent_position[2] - self._target_position[2]) ** 2))
            #if reward == 0: reward = 0.01 #-> should never reach this problem bc updating states above 
            reward = 1 / np.round(reward, decimals=1) # bc we're rounding the x y z 's to the first decimal, max possible reward here is 10: 1/0.1
       
        # Robot has grabbed block, but is not at destination location
        elif (self.state_arm_reached_target == 1 and
              self.state_grabbed_target == 1 and
              self.state_target_reached_destination == 0):     
            # reward is euclidean distance
            reward = np.sqrt(((self._agent_position[0] - self._destination_position[0]) ** 2) +
                             ((self._agent_position[1] - self._destination_position[1]) ** 2) + 
                             ((self._agent_position[2] - self._destination_position[2]) ** 2))
            # if reward == 0: reward = 0.01 # -> should never reach here bc updating states above
            reward = 1 / np.round(reward, decimals=1) 
        
        # (Checkpoint 1) Robot is at target location, but has not picked it up
        elif (self.current_checkpoint == 1 and 
              self.previous_checkpoint == self.current_checkpoint-1): 
            # 2nd condition ensures only receive checkpoint reward once, until checkpoint starts over to 0
            reward = 2
        # (Checkpoint 2, Final Checkpoint or END) Robot has block and is at destaination location
        elif (self.current_checkpoint == 2 and 
              self.previous_checkpoint == self.current_checkpoint-1):
            reward = 10
        
        ''' OLD REWARD SYSTEM
        # Reward System (think of it as training a dog with treats / no treats / punish for doing something 
        # -> ALSO making sure agent is not convinving itself to do something that is not ideal)
        # PROBLEM: model sometimes get stuck repeating an action even with punishment but at some point broke out of it
        # PROBLEM: with my current dense reward system, model convinced itself that within an episode it can rack up more reward from constantly going back and forth to target position then at the end finally choose to pick up the block and move to destination to maximize reward
        if terminated:
            reward = 100
        #     # reward = 1
            self.state_target_reached_destination = 1
            print("Target reached Destination!!!!!!")
        # # penalize for choosing the same action back to back
        elif (action == 0 and self.state_last_action == 0) or (action == 1 and self.state_last_action == 1) or (action == 2 and self.state_last_action == 2) or (action == 3 and self.state_last_action == 3): 
            reward = -3
            # reward = 0

        # elif info["distance_a_t"] < self.prev_agent_target_distance:
        #     # if self.prev_agent_target_distance != 99999999:
        #     #     # reward = np.abs(info["distance_a_t"] - self.prev_agent_target_distance)
        #     #     reward = 1
        #     # else: reward = 1
        #     reward = 1
        # elif info["distance_t_d"] < self.prev_target_dest_distance:
        #     # if self.prev_target_dest_distance != 99999999:
        #     #     reward = np.abs(info["distance_t_d"] - self.prev_target_dest_distance)
        #     # else: reward = 1
        #     reward = 1
        
        # provide reward for closing gripper at target position
        elif np.array_equal(self._agent_position[0:3], self._target_position) and action == 2: 
            reward = 1
            
        # punish for opening gripper at target position when it use to be closed
        elif np.array_equal(self._agent_position[0:3], self._target_position) and action == 3 and self.state_last_action == 2: 
            reward = -1
        # maybe punish for going to target with grippers closed (would lead to collision which we dont want) -> this MAY lead to model thinking moving to target is bad
        elif np.array_equal(self._agent_position[0:3], self._target_position) and action == 0 and self.gripper_state == 0:  
            # reward = -0.1
            # END EPISODE if agent moves to the target with its gripper closed 
            terminated = True
            reward = -30
            self.state_collision = 1
            # reward=0
        # provide reward for moving to target
        elif np.array_equal(self._agent_position[0:3], self._target_position) and action == 0:
            reward = 0.1
            # provide bonus reward for going to target with grippers open
            if np.array_equal(self._agent_position[0:3], self._target_position) and action == 0 and self.gripper_state == 1: 
                reward += 0.1

            
        
        # # # provide reward for going to target with grippers open    
        # elif np.array_equal(self._agent_position[0:3], self._target_position) and action == 0 and self.gripper_state == 1: 
        #     reward = 1
        
            
        # penalize for letting go of the block not at destination
        # elif np.array_equal(self._agent_position[0:3], self._target_position) and action == 3:
        #     reward = -1 
        # penalize for going to destination without target (block)
        elif action == 1 and not np.array_equal(self._agent_position[0:3], self._target_position):
            reward = -1
        else:
            reward = 0
        
        # IMPORTANT EXPERIMENT: I would provide a reward of 1 if it was what i wanted and otherwise -1, this lead to the robot at 
        # a new episode pick the ideal path sequentially: actions: 0,2,1
        # reward = input("Provide a reward for this action: ")
        '''
            
        print("Reward: ", reward)
        
        # self.prev_agent_target_distance = info["distance_a_t"]
        # self.prev_target_dest_distance = info["distance_t_d"]
        
        self.state_last_action = action
        
        
        self.update_historical_actions(action)
          

        print("New Historical Actions: ", self.historical_actions)
        
        observation = self._get_obs()
        print("Observations:", observation, "\n")

        if self.render_mode == "human":
            self._render_frame()


        # self._actions_dict = {
        #     0: np.array([self._get_x_displacement(),0,0, 0,0,0, 0,0,0,0,0]), # x displacement
        #     1: np.array([0,self._get_y_displacement(),0, 0,0,0, 0,0,0,0,0]), # y displacement
        #     2: np.array([0,0,self._get_z_displacement(), 0,0,0, 0,0,0,0,0]), # z displacement
        # }

        return observation, reward, terminated, False, info # WHAT does the False represent (truncated)
    
    
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
        
    def _get_obs(self):
         return {"agent": np.round(self._agent_position, decimals=1) , "target": np.round(self._target_position, decimals=1), 
                 "gripper_state": self.gripper_state, "historical_actions": self.historical_actions, "grabbed_target": self.state_grabbed_target,
                 "target_reached_destination": self.state_target_reached_destination, "collision": self.state_collision, "arm_reached_target": self.state_arm_reached_target,
                 "current_checkpoint": self.current_checkpoint}
    
    # distance between 2 points in 3D space (agent's and target's xyz positions)
    def _get_info(self):
        # return {"distance": np.linalg.norm(self._agent_position[0:3] - self._target_position)}
        return {"distance_a_t": math.sqrt(math.pow(self._agent_position[0] - self._target_position[0], 2) + math.pow(self._agent_position[1] - self._target_position[1], 2) + math.pow(self._agent_position[2] - self._target_position[2], 2)* 1.0),
        "distance_t_d": math.sqrt(math.pow(self._destination_position[0] - self._target_position[0], 2) + math.pow(self._destination_position[1] - self._target_position[1], 2) + math.pow(self._destination_position[2] - self._target_position[2], 2)* 1.0),
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
    
    # update history of 5 most recent actions
    def update_historical_actions(self, action: dict):
        # t = current time step
        # self.historical_actions = [t-5, t-4, t-3, t-2, t-1]
        temp_1 = -1
        temp_2 = -1
        for i in reversed(range(5)):                    
            if i == 4: # most recent action            
                temp_1 = self.historical_actions[i]
                self.historical_actions[i] = action
            else:
                temp_2 = self.historical_actions[i]
                self.historical_actions[i] = temp_1  
                temp_1 = temp_2
    
    # expect to return array of [x,y, yaw degree rotation]
    def get_xy_rotation_block(self):
        red_block, blue_block = obj_det_xyz_angle.get_pos_angle()
        if red_block.is_detected:
            return np.array([red_block.x, red_block.y, red_block.rotation])
        elif blue_block.is_detected:
            return np.array([blue_block.x, blue_block.y, blue_block.rotation])
        else:
            print("\nERROR: Object not detected\n")
            exit(-1)
