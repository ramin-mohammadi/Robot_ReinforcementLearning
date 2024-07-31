#import gym
# NOTE: gym seems to be old and gymnasium is new maintained environment package
import numpy as np
import gymnasium as gym
from gymnasium import spaces
import math
import time
import copy

import pygame

import mujoco
import mujoco.viewer


# directory to these local files is relative to the python package being the folder env_package
from FactoryRobotArm import xArmClass

# IMPORTANT: POSSIBLY CAN IMPLEMENT CONTROL FREQUENCY WITH time.sleep() at the end of a step()

# with mujoco.viewer.launch_passive(m, d) as viewer:
#     # Close the viewer automatically after 30 wall-seconds.
#     start = time.time()
#     viewer.sync()

class xArmEnv(gym.Env): 
    metadata = {"render_modes": ["human", "rgb_array"], "render_fps": 4}
    
    def __init__(self, render_mode=None):

        mujoco.set_mjcb_control(None)

        self.m = mujoco.MjModel.from_xml_path('/Users/albert/REU/2024/MuJoCo/model/xArm 5/scene.xml')
        self.d = mujoco.MjData(self.m)
        self.viewer = mujoco.viewer.launch_passive(self.m, self.d, show_left_ui=False)
        # self.renderer = mujoco.Renderer(model=self.m, height=480, width=640)
        # pygame.init()
        # self.screen = pygame.display.set_mode((640, 480))

        self.viewer.lock()
        self.d.ctrl = [0, 0, 0, 0, 0, 0, 0, 0]
        self.viewer.sync()
        mujoco.mj_step(self.m, self.d)
        self.viewer.sync()

        self.num_steps = 0
        self.episode_length = 10000
        # Bound: grid workspace (3D plane xyz) of conveyor belt??? -> possibly not true because the location of the robots are not in the conveyor belt grid
        # solution is to assume from the vertex (0,0,0) that me and Brian chose, the bounds is the entire robot working field? Or each robot will get a unique bound being how far they can reach in all directions
        # units is millimeters
        
        # THERE ARE DIFFERENT BOUNDARY VALUES for the smaller and bigger arms (LOOK at their manuals appendix which are the last few pages)
        
        # AGENT: xArm 5 (big arm)
        
        self.agent_x_low = -1
        self.agent_x_high = 1
        self.agent_y_low = -1
        self.agent_y_high = 1
        self.agent_z_low = 0
        self.agent_z_high = 1.2

        # NOTE: self.d.ctrl = [joint1 (-6.28, 6.28), joint2 (-2.06, 2.09), 
        #           joint3 (not in xArm 5) (-6.28, 6.28), joint4 (-0.192, 3.93), 
        #           joint5 (not in xArm5) (-6.28, 6.28), joint6 (-1.69, 3.14), 
        #           joint7 (-6.28, 6.28), gripper (0, 255)]
        self.agent_joint1_low = -6.28
        self.agent_joint1_high = 6.28
        self.agent_joint2_low = -2.06
        self.agent_joint2_high = 2.09
        self.agent_joint3_low = -0.192
        self.agent_joint3_high = 3.93
        self.agent_joint4_low = -1.69
        self.agent_joint4_high = 3.14
        self.agent_joint5_low = -6.28
        self.agent_joint5_high = 6.28
        self.agent_gripper_low = 0
        self.agent_gripper_high = 255
        
        # TARGET: object on conveyor belt
        self.target_x_low = -1 #-300
        self.target_x_high = 1 #300
        self.target_y_low = -1 #-400
        self.target_y_high = 1 # 400  
        self.target_z_low = 0 # 0
        self.target_z_high = 1 # 400        
        
        # robot arm is the agent, target is the object on the conveyor belt
        # i have specified the range of possible values for xyz THEN the shape is an array of 3 elements for xyz  [-2,200,11]
        # For the agent it is [x,y,z, Roll, Pitch, Yaw, J1,J2,J3,J4,J5] -> smaller arms have 6 joints so IMPORTANT: EACH arm will have their own custom "environment"
        # target will just have [x,y,z]
        
        # ===============================IMPORTANT=============================
        # NOTE: NEED TO NORMALIZE OBSERVATION SPACE AND ACTION SPACE TO [-1, 1]
        # normalized_x = [ (actual_x - min(x)) / (max(x) - min(x)) - 0.5] * 2
        # ===============================IMPORTANT=============================

        # (agent x, agent y, agent z, target x, target y, target z, state_gripper -> 1 if open 0 if closed,  5 most recent historical actions)
        # STATES should also be placed in the observation space for model to learn from

        self.observation_space = spaces.Dict( # later may have to account for observing if other arms are moving???
            {
                # [x_pos, y_pos, z_pos, joint1, joint2, joint3, joint4, joint5, gripper]
                "agent" : spaces.Box(low=np.array([-1, -1, -1, -1, -1, -1, -1, -1, -1]), 
                                     high=np.array([1, 1, 1, 1, 1, 1, 1, 1, 1]), 
                                     shape=(9,), dtype=np.float32), 
                
                # [x_pos, y_pos, z_pos]
                "target" : spaces.Box(low=np.array([-1, -1, -1]), 
                                      high=np.array([1, 1, 1]), 
                                      shape=(3,), dtype=np.float32),
                # "gripper_state": spaces.Discrete(2), # {0,1}  , 1 if open 0 if closed
                "grabbed_target": spaces.Discrete(2), # {0,1}  , 1 if picked up 0 otherwise/dropped block
                "arm_reached_target": spaces.Discrete(2), # {0,1}  , 1 if arm reached target 0 otherwise
                "target_reached_destination": spaces.Discrete(2), # {0,1}  , 1 if block reached destination 0 otherwise
                # "collision": spaces.Discrete(2), # {0,1}  , 1 if collision occurred 0 otherwise
                # "historical_actions": spaces.Box(low=-1, high=3, shape=(5,), dtype=np.float32),  # 5 most recent actions (t-5, t-4, t-3, t-2, t-1)
                "current_checkpoint": spaces.Discrete(3),
            }
        )
        
        # Continuous actions: [x-velocity, y-velocity, z-velocity, gripper state]
        # NOTE: gripper state >= 0 is open, gripper state < 0 is closed

        # [joint1, joint2, joint3, joint4, joint5, gripper]
        # self.action_space = spaces.Box(low=np.array([self.agent_x_vel_low, self.agent_y_vel_low, self.agent_z_vel_low, -1]),
        #                                high=np.array([self.agent_x_vel_high, self.agent_y_vel_high, self.agent_z_vel_high, 1]),
        #                                shape=(4,), dtype=np.float32)
        self.action_space = spaces.Box(low=np.array([-1, -1, -1, -1, -1, -1]),
                                       high=np.array([1, 1, 1, 1, 1, 1]),
                                       shape=(6,), dtype=np.float32)
        
        
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

        self.viewer.lock()
        self.viewer.sync()

        # xArm: Agent's initial values will be static based off of values at initial position set on the arm
        # self._agent_position = np.array([-12.1,181.6,222.3, 180,0,-91.5, 93.8,-58.8,5.2,53.6,185.3]) # [x,y,z, roll,pitch,yaw, J1,J2,J3,J4,J5]
        
        # move robot to initial position
        self._agent_position = np.array([0,0,0,0,0,0,0,0,0], dtype=float) # dummy values just to initialize _agent_position
        self.d.ctrl = [0, 0, 0, 0, 0, 0, 0, 0]
        mujoco.mj_step(self.m, self.d)
        self.viewer.sync()

        # self.self.d.ctrl[2] and self.self.d.ctrl[4] are joints 3 and 5, nonexistent on xArm5
        # [x, y, z, joint1, joint2, joint3, joint4, joint5, gripper]
        self._agent_position[0:3] = [self.d.geom("gripper_geom").xpos[0], 
                                     self.d.geom("gripper_geom").xpos[1],
                                     self.d.geom("gripper_geom").xpos[2]]
        self._agent_position[3:9] =  [self.d.ctrl[0], self.d.ctrl[1], self.d.ctrl[3], 
                                      self.d.ctrl[5], self.d.ctrl[6], self.d.ctrl[7]]
        
        self.gripper_state = 1
        
        self.previous_checkpoint = 0
        # 0: starting
        # 1: gripper closed at the target (picked up block)
        # 2: block at destination (final checkpoint)
        self.current_checkpoint = 0  
        
        # ADD LATER: speed and gripper clamping width

        self.dropped_block = False

        # Static values for now
        target_x = 0.5
        target_y = 0
        target_z = 0
        self._target_position = np.array([target_x, target_y, target_z])

        # Static values for now
        dest_x = 0
        dest_y = 1
        dest_z = 0
        self._destination_position = np.array([dest_x, dest_y, dest_z])
        

        self.state_last_action = np.array([0, 0, 0, 0, 0, 0], dtype=float)

        # if agent at target and robot closed gripper
        self.state_grabbed_target = 0 # if true (1), target position will be updated with agent's position
        self.state_target_reached_destination = 0
        self.state_arm_reached_target = 0
        self.state_collision = 0 # 1 if collision occurred such as arm moving to block with grippers closed
        
        
        self.prev_agent_target_distance = 99999999
        self.prev_target_dest_distance = 99999999
        
        # 5 previous actions
        '''
        self.historical_actions = np.array([np.array([0, 0, 0, 0, 0, 0]),
                                            np.array([0, 0, 0, 0, 0, 0]),
                                            np.array([0, 0, 0, 0, 0, 0]),
                                            np.array([0, 0, 0, 0, 0, 0]),
                                            np.array([0, 0, 0, 0, 0, 0])])
        '''

        observation = self._get_obs()
        info = self._get_info()

        if self.render_mode == "human":
            self._render_frame()

        return observation, info
        
    def step(self, action):
        """
        Returns: Given current obs and action, returns the next observation, the reward, done and optionally additional info
        """
        if not self.viewer.is_running():
            exit()

        step_start = time.time()

        self.viewer.lock()
        self.viewer.sync()

        self.num_steps += 1
        
        self.previous_checkpoint = self.current_checkpoint

        self.viewer.lock()
        # perform action
        self.d.ctrl = [self.get_actual(action[0], self.agent_joint1_low, self.agent_joint1_high),
                  self.get_actual(action[1], self.agent_joint2_low, self.agent_joint2_high), 
                  0, # inactive joint on xArm 5
                  self.get_actual(action[2], self.agent_joint3_low, self.agent_joint3_high), 
                  0, # inactive joint on xArm 5
                  self.get_actual(action[3], self.agent_joint4_low, self.agent_joint4_high),
                  self.get_actual(action[4], self.agent_joint5_low, self.agent_joint5_high),
                  self.get_actual(action[5], self.agent_gripper_low, self.agent_gripper_high)]
        
        self.viewer.sync()
        mujoco.mj_step(self.m, self.d)
        self.viewer.sync()

        # time.sleep(1)

        time_until_next_step = self.m.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)

        # to update observation
        self._agent_position[0:3] = [self.d.geom("gripper_geom").xpos[0], 
                                     self.d.geom("gripper_geom").xpos[1],
                                     self.d.geom("gripper_geom").xpos[2]]
        self._agent_position[3:9] =  [self.d.ctrl[0], self.d.ctrl[1], self.d.ctrl[3], 
                                      self.d.ctrl[5], self.d.ctrl[6], self.d.ctrl[7]]

        # gripper: positive or 0 is open, negative is close
        normalized_gripper = self.get_normalized(action[5], 
                                                 self.agent_gripper_low, 
                                                 self.agent_gripper_high)
        if normalized_gripper >= 0:
            # self.robot_main.open_gripper()
            self.gripper_state = 1
        elif normalized_gripper < 0:
            # self.robot_main.close_gripper()
            self.gripper_state = 0
            
            
        print("Action (normalized):")
        print("   Joints: ", action[:-1])
        print("   Gripper: ", "Open" if normalized_gripper >= 0 else "Closed")
        
        print("Last Action (normalized): ", self.state_last_action)
        print("   Joints: ", self.state_last_action[:-1])
        print("   Gripper: ", "Open" if self.state_last_action[5] > 100 else "Closed")
        
        
        # update agent position
        self._agent_position[0:3] = [self.d.geom("gripper_geom").xpos[0], 
                                     self.d.geom("gripper_geom").xpos[1],
                                     self.d.geom("gripper_geom").xpos[2]]
        self._agent_position[3:9] =  [self.d.ctrl[0], self.d.ctrl[1], self.d.ctrl[3], 
                                      self.d.ctrl[5], self.d.ctrl[6], self.d.ctrl[7]]

        # round to first decimal bc precision for get_position is to the 7th decimal so agent pos never equals target pos
        self._agent_position[0:3] = np.round(self._agent_position[0:3], decimals=1) 

        # NOTE: DURING SIMULATION, IF BLOCK MOVES, UPDATE POSITION
        self._target_position = [self.d.geom("block_geom").xpos[0],
                                 self.d.geom("block_geom").xpos[1],
                                 self.d.geom("block_geom").xpos[2]]
        
        
        xy_error = 0.001 # Simulation scale
        z_max = 0.112 # max z-coordinate value for block grabbing
        
        # arm has to be within xy_error mm in x and y from the target's x y, and within robot z: (143 -> 151 mm) to have reached the target 
        if(abs(self._agent_position[0]-self._target_position[0]) <= xy_error and
           abs(self._agent_position[1]-self._target_position[1]) <= xy_error and
           self.target_z_low <= self._target_position[2] <= z_max):
           # 1 = true
           self.state_arm_reached_target = 1
           
           print("CHECKPOINT 1 REACHED")
           time.sleep(3)
           # check to see if gripper closed when arm is at target position
           if self.gripper_state == 0: # 0 = closed
                self.state_grabbed_target = 1
                self.current_checkpoint = 1
        
        z_max_dest = 0.15
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
            if self.current_checkpoint >= 1:
                self.dropped_block = True
 
        elif(self.gripper_state == 1): # anytime the gripper is opened assume not going to be updating target position anymore
            self.state_grabbed_target = 0
            self.current_checkpoint = 0
         
        # print("STATE CHECKPOINT: ", self.state_grabbed_target)

        print("AGENT: ", self._agent_position)
        print("xpos: ", self.d.geom("gripper_geom").xpos)
        
        # print("TARGET: ", self._target_position)        

        # This assumes that the robot is holding the block so update the target position with the robot's
        if self.state_grabbed_target == 1 and self.gripper_state == 0:
            self._target_position = self._agent_position
            # print("Updating target position")     
        
        # CHANGE LATER: account if chosen movement resulted in robot error getting stuck, going out of bounds, colliding, speed too fast? etc
        
        #terminated = np.array_equal(self._target_position, self._destination_position)  
        
        # TERMINATION SUCCESS: if arm within destination area, target has reached destination
        terminated = False
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
        # NOTE: possibly want to normalize rewards

        # Robot has not yet picked up block
        if (self.current_checkpoint == 0 and 
            self.state_arm_reached_target == 0): # not yet reached target
            # reward is euclidean distance
            reward = np.sqrt(((self._agent_position[0] - self._target_position[0]) ** 2) +
                             ((self._agent_position[1] - self._target_position[1]) ** 2) + 
                             ((self._agent_position[2] - self._target_position[2]) ** 2))
            #if reward == 0: reward = 0.01 #-> should never reach this problem bc updating states above 
            # reward = (-100 * np.round(reward, decimals=1) / 3) + 10
            # a / (1 + x) gives max reward of a for x = 0
            # Could consider more smooth (possibly linear? such as reward = -mx + b)
            # Or quadratic? cubic?
            # Or log? e.g. -a * log(x + 1) + b

            block_id = int(self.m.geom("block_geom").id)
            gripper_id = int(self.m.geom("gripper_geom").id)
            distance = mujoco.mj_geomDistance(self.m, self.d, block_id, gripper_id, 100, None)
            print("Distance: ", distance)
            reward = 1 / np.round(distance, decimals=2) # bc we're rounding the x y z 's to the first decimal, max possible reward here is 10: 1/0.1

        # Robot has grabbed block, but is not at destination location
        elif (self.state_arm_reached_target == 1 and
              self.state_grabbed_target == 1 and
              self.state_target_reached_destination == 0):     
            # reward is euclidean distance
            reward = np.sqrt(((self._agent_position[0] - self._destination_position[0]) ** 2) +
                             ((self._agent_position[1] - self._destination_position[1]) ** 2) + 
                             ((self._agent_position[2] - self._destination_position[2]) ** 2))
            # if reward == 0: reward = 0.01 # -> should never reach here bc updating states above
            # reward = (-100 * np.round(reward, decimals=1) / 3) + 10

            block_id = int(self.m.geom("block_geom").id)
            gripper_id = int(self.m.geom("gripper_geom").id)
            distance = mujoco.mj_geomDistance(self.m, self.d, block_id, gripper_id, 100, None)
            print("Distance: ", distance)
            reward = 1 / np.round(distance, decimals=2) 
        
        # (Checkpoint 1) Robot is at target location, but has not picked it up
        elif (self.current_checkpoint == 1 and 
              self.previous_checkpoint == self.current_checkpoint - 1): 
            # 2nd condition ensures only receive checkpoint reward once, until checkpoint starts over to 0
            reward = 20
        # (Checkpoint 2, Final Checkpoint or END) Robot has block and is at destaination location
        elif (self.current_checkpoint == 2 and 
              self.previous_checkpoint == self.current_checkpoint - 1):
            reward = 100

        print("Current Checkpoint: ", self.current_checkpoint)

        if self.dropped_block:
            reward = -50 # TBD
            self.dropped_block = False
            
        print("Reward: ", reward) 
        
        # self.prev_agent_target_distance = info["distance_a_t"]
        # self.prev_target_dest_distance = info["distance_t_d"]
        
        self.state_last_action = copy.copy(action)
        
        # self.update_historical_actions(action)
        
        # print("New Historical Actions: ", self.historical_actions)
        
        observation = self._get_obs()
        # print("Observations (normalized):", observation, "\n")

        if self.render_mode == "human":
            self._render_frame()
            
        # Truncate if number of steps reaches specified episode length
        truncated = self.num_steps > self.episode_length

        return observation, reward, terminated, truncated, info
    
    
    # pybullet?
    # gymnasium-robotics??
    # IDEALLY have 3d simulation of our robot, conveyor belt and object
    def render(self, mode='rgb_array'):
        pass
        # self.renderer.update_scene(self.d)

        # img = self.renderer.render()
        # pygame_img = pygame.surfarray.make_surface(np.transpose(img, (1, 0, 2)))
        # pygame.screen.blit(pygame_img, (0, 0))
        # pygame.display.flip()

        # if self.timestep >= 20 * 60: # 20 secs at 60 fps    
        #     obs = self.reset()
        #     self.timestep = 0

        # for event in pygame.event.get():
        #     if event.type == pygame.QUIT:
        #         pygame.quit()
        #         exit()
        
    def _get_obs(self):
        # return {"agent": np.round(self._agent_position, decimals=1) , "target": np.round(self._target_position, decimals=1), 
        #         "gripper_state": self.gripper_state, "historical_actions": self.historical_actions, "grabbed_target": self.state_grabbed_target,
        #         "target_reached_destination": self.state_target_reached_destination, "collision": self.state_collision, "arm_reached_target": self.state_arm_reached_target,
        #         "current_checkpoint": self.current_checkpoint}

        # [x_pos, y_pos, z_pos, joint1, joint2, joint3, joint4, joint5, gripper]
        temp_agent = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0])
        temp_agent[0] = self.get_normalized(self._agent_position[0], self.agent_x_low, self.agent_x_high)
        temp_agent[1] = self.get_normalized(self._agent_position[1], self.agent_y_low, self.agent_y_high)
        temp_agent[2] = self.get_normalized(self._agent_position[2], self.agent_z_low, self.agent_z_high)
        temp_agent[3] = self.get_normalized(self._agent_position[3], self.agent_joint1_low, self.agent_joint1_high)
        temp_agent[4] = self.get_normalized(self._agent_position[4], self.agent_joint2_low, self.agent_joint2_high)
        temp_agent[5] = self.get_normalized(self._agent_position[5], self.agent_joint3_low, self.agent_joint3_high)
        temp_agent[6] = self.get_normalized(self._agent_position[6], self.agent_joint4_low, self.agent_joint4_high)
        temp_agent[7] = self.get_normalized(self._agent_position[7], self.agent_joint5_low, self.agent_joint5_high)
        temp_agent[8] = self.get_normalized(self._agent_position[8], self.agent_gripper_low, self.agent_gripper_high)
        
        # [x_pos, y_pos, z_pos]
        temp_target = np.array([0, 0, 0])
        temp_target[0] = self.get_normalized(self._target_position[0], self.target_x_low, self.target_x_high)
        temp_target[1] = self.get_normalized(self._target_position[1], self.target_y_low, self.target_y_high)
        temp_target[2] = self.get_normalized(self._target_position[2], self.target_z_low, self.target_z_high)

        return {
                # [x_pos, y_pos, z_pos, roll, pitch, yaw, joint1, joint2, joint3, joint4, joint5, x_vel, y_vel, z_vel]
                "agent" : np.round(temp_agent, decimals=1), 
                # [x_pos, y_pos, z_pos]
                "target" : np.round(temp_target, decimals=1),
                # "gripper_state": self.gripper_state,
                "grabbed_target": self.state_grabbed_target,
                "arm_reached_target": self.state_arm_reached_target,
                "target_reached_destination": self.state_target_reached_destination,
                # "historical_actions": self.historical_actions,
                "current_checkpoint": self.current_checkpoint,
                # "collision": self.state_collision
                }
    
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
        self._agent_position[0:3] = [self.d.geom("gripper_geom").xpos[0], 
                                     self.d.geom("gripper_geom").xpos[1],
                                     self.d.geom("gripper_geom").xpos[2]]
        self._agent_position[3:9] =  [self.d.ctrl[0], self.d.ctrl[1], self.d.ctrl[3], 
                                      self.d.ctrl[5], self.d.ctrl[6], self.d.ctrl[7]]
    
    # update history of 5 most recent actions
    def update_historical_actions(self, action):
        # t = current time step
        # self.historical_actions = [t-5, t-4, t-3, t-2, t-1]
        # temp_1 = -1
        # temp_2 = -1
        # for i in reversed(range(5)):                    
        #     if i == 4: # most recent action            
        #         temp_1 = self.historical_actions[i]
        #         self.historical_actions[i] = action
        #     else:
        #         temp_2 = self.historical_actions[i]
        #         self.historical_actions[i] = temp_1  
        #         temp_1 = temp_2

        # Shifts all elements 1 index left e.g. [1, 2, 3] --> [2, 3, 1]
        self.historical_actions = np.roll(self.historical_actions, -1)
        # Set last element to most recent action
        self.historical_actions[-1] = action
            
    def get_normalized(self, value, min, max):
        return ((value - min) / (max - min) - 0.5) * 2
    
    def get_actual(self, normalized, min, max):
        # NOTE: Should just be inverse of above
        return (((normalized / 2) + 0.5) * (max - min) + min)