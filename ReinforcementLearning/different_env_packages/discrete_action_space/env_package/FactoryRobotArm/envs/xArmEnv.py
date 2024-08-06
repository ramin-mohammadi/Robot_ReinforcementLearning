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
        self.target_x_low = -300
        self.target_x_high = 300
        self.target_y_low = -400
        self.target_y_high = 400        
        self.target_z_low = 0
        self.target_z_high = 400
        
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
                "agent" : spaces.Box(low=np.array([self.agent_x_low, self.agent_y_low, self.agent_z_low, self.agent_roll_pitch_yaw_low, self.agent_roll_pitch_yaw_low, self.agent_roll_pitch_yaw_low, self.agent_joint1_low, self.agent_joint2_low, self.agent_joint3_low, self.agent_joint4_low, self.agent_joint5_low]), 
                high=np.array([self.agent_x_high, self.agent_y_high, self.agent_z_high, self.agent_roll_pitch_yaw_high, self.agent_roll_pitch_yaw_high, self.agent_roll_pitch_yaw_high,self.agent_joint1_high, self.agent_joint2_high, self.agent_joint3_high, self.agent_joint4_high, self.agent_joint5_high]), 
                                        shape=(11,), dtype=np.float64), 
                # the z axis is needed for the target to help guide the agent's z position to the target's -> But PROBLEM here bc z pos is the attachment point of the gripper, not the tip of the gripper -> assume z is static (figure out beforehand)
                "target" : spaces.Box(low=np.array([self.target_x_low, self.target_y_low, self.target_z_low]), 
                                        high=np.array([self.target_x_high, self.target_y_high, self.target_z_high]), 
                                        shape=(3,), dtype=np.float64),
                "gripper_state": spaces.Discrete(2), # {0,1}  , 1 if open 0 if closed
                "historical_actions": spaces.Box(low=-1, high=3, shape=(5,), dtype=int)  # 5 most recent actions (t-5, t-4, t-3, t-2, t-1)
            }
        )

        
        
        
        
        # # xArm
        self.action_space = spaces.Discrete(4)
        # self.action_space = spaces.Box(low=np.array([0]), high=np.array([4]), dtype=int)
        
        

        
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
        self._agent_position = np.array([0,0,0,0,0,0,0,0,0,0,0], dtype=float) # dummy values just to initialize _agent_position
        self.robot_main.move_initial()
        # self.update_agent_position()
        self._agent_position[0:6] = self.robot_main._arm.get_position()[1] # [x,y,z,roll,pitch,yaw]
        self._agent_position[6:11] = self.robot_main._arm.get_servo_angle()[1][0:5] # 5 joint angles
        

        self.robot_main.open_gripper()
        self.gripper_state = 1
        
        
        # ADD LATER: speed and gripper clamping width

        # self._target_position = self.get_random_target_pos()
        # # We will sample the target's location randomly until it does not coincide with the agent's location
        # while np.array_equal(self._target_position, self._agent_position[0:3]):
        #     self._target_position = self.get_random_target_pos()
        
        
        # xArm
        self._target_position = np.array([1.9,281.3,144])
        self._destination_position = np.array([211, 113.6, 268.7])
        

        self.state_last_action = -1
        # if agent at target and robot closed gripper
        self.state_robot_checkpoint = False # if true, target position will be updated with agent's position
        
        
        self.prev_agent_target_distance = 99999999
        self.prev_target_dest_distance = 99999999
        
        self.historical_actions = np.array([-1,-1,-1,-1,-1])

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
        
        #xArm , FOR SOME WEIRD REASON, these actions are performed sequentially (maybe initially), and: within one step() it performes all 4 actions
        # self._actions_dict = {
        #     0: self.robot_main.move_to(self._target_position),
        #     2: self.robot_main.close_gripper(),            
        #     1: self.robot_main.move_to(self._destination_position),
        #     3: self.robot_main.open_gripper()
        # }
        # DONT DO ABOVE, will literally call the functions line by line
        
        
        
        if action == 0:
            self.robot_main.move_to(self._target_position)
        elif action == 1:
            self.robot_main.move_to(self._destination_position)
        elif action == 2:
            self.robot_main.close_gripper()
            self.gripper_state = 0
        elif action == 3:
            self.robot_main.open_gripper()
            self.gripper_state = 1
            
            
        print("Action: ", action)
        print("Last Action: ", self.state_last_action)
        
        
        
        # THIS BRANCH is still BUGGY -> may say target reached the destination when the grippers were open when moving to the destination
        # if agent at target and robot closed gripper, update target position with agent's position (robot is holding block)
        if(np.array_equal(self._agent_position[0:3], self._target_position) and self.gripper_state == 0):
            self.state_robot_checkpoint = True
        # robot chose to open gripper while it was previously holding target
        # elif(np.array_equal(self._agent_position[0:3], self._target_position) and self.state_last_action == 3): 
        if(self.gripper_state == 1): # anytime the gripper is opened assume not going to be updating target position anymore
            self.state_robot_checkpoint = False
        
        print("STATE CHECKPOINT: ", self.state_robot_checkpoint)

        

        
        # update agent position by getting robot's current coordinates after move
        # self.update_agent_position()
        self._agent_position[0:6] = self.robot_main._arm.get_position()[1] # [x,y,z,roll,pitch,yaw]
        self._agent_position[6:11] = self.robot_main._arm.get_servo_angle()[1][0:5] # 5 joint angles
        
        # round to first decimal bc precision for get_position is to the 7th decimal so agent pos never equals target pos
        self._agent_position[0:3] = np.round(self._agent_position[0:3], decimals=1) 
        

        print("AGENT: ", self._agent_position)
        
        # print("TARGET: ", self._target_position)        

        if self.state_robot_checkpoint == True and self.gripper_state == 0:
            # DO NOT DO BELOW cause target variable will now point to the agent pos address so when agent pos is updated, so is target:
            # self._target_position = self._agent_position[0:3] 
            self._target_position = np.round(self.robot_main._arm.get_position()[1][0:3], decimals=1)
            # print("Updating target position")

        print("TARGET: ", self._target_position)        
   
        
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
        # z_offset = 0 # offset z bc the z of the agent should be more than the target since agent's z is not at tip of gripper
        # terminated = np.array_equal(self._agent_position[0:3] , self._target_position + [0,0,z_offset])  
        # reward = 1 if terminated else 0  # Binary sparse rewards (FOR NOW, CHANGE LATER and make a CUSTOM REWARD SYSTEM)
        
        terminated = np.array_equal(self._target_position, self._destination_position)  
        # print("Terminated: ", terminated)
        
        # END EPISODE if agent moves to the target with its gripper closed 
        # if(action == 0 and self.gripper_state == 0 and terminated == False):
        #     terminated = True
        
                
        info = self._get_info()
        
        # Reward System (think of it as trainging a dog with treats / no treats / punish for doing something 
        # -> ALSO making sure agent is not convinving itself to do something that is not ideal)
        # PROBLEM: model sometimes get stuck repeating an action even with punishment but at some point broke out of it
        # PROBLEM: with my current dense reward system, model convinced itself that within an episode it can rack up more reward from constantly going back and forth to target position then at the end finally choose to pick up the block and move to destination to maximize reward
        if terminated:
            reward = 100
            # reward = 1
            print("Target reached Destination!!!!!!")
            # print("STATE CHECKPOINT: ", self.state_robot_checkpoint)
        # penalize for choosing the same action back to back
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
        
    def _get_obs(self):
         return {"agent": np.round(self._agent_position, decimals=1) , "target": np.round(self._target_position, decimals=1), "gripper_state": self.gripper_state, "historical_actions": self.historical_actions}
    
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
    def update_historical_actions(self, action: int):
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
        