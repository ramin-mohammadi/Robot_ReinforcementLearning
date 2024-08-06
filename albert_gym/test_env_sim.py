import FactoryRobotArm
import gymnasium
from stable_baselines3 import PPO
from stable_baselines3.ppo import MlpPolicy
from stable_baselines3 import DQN
import numpy as np
import time
import mujoco
import mujoco.viewer

env = gymnasium.make('FactoryRobotArm/xArm-v0')

# PPO seems to be doing a better job of reaching the destination than DQN
# model = DQN("MultiInputPolicy", env, verbose=1, learning_rate=0.001, gamma=0.8)  
# gamma is the discount factor (go for higher value bc with dense reward system, want model to go for big reward in future 
# being putting the block at the destination rather repetititve quick rewards)


# model.learn(total_timesteps=200, log_interval=1)
# model.save("DQN_xArm")


# del model # remove to demonstrate saving and loading

# model = DQN.load("DQN_xArm_20000")
# model = PPO.load("DQN_xArm")

obs, info = env.reset()

m = mujoco.MjModel.from_xml_path('PATHscene.xml')
d = mujoco.MjData(m)

RL_model = PPO("MultiInputPolicy", env, verbose=1)

RL_model.learn(total_timesteps=10000)

# with mujoco.viewer.launch_passive(m, d) as viewer:
#     # Close the viewer automatically after 30 wall-seconds.
#     start = time.time()
#     viewer.sync()

#     # while viewer.is_running() and time.time() - start < 30:
#     while viewer.is_running():
#         step_start = time.time()
#         viewer.sync()
if True: # Only here for indentation/debugging
    while True:
        action, _states = RL_model.predict(obs, deterministic=True)
        # print(action)
        # action = int(action) # action was ndarray but could not correctly index into my action dictionary
        # if type(action) == np.ndarray:
        #     action = action[np.argmax(action)] 



        # env.render()
        obs, reward, terminated, truncated, info = env.step(action)
        
        # d.ctrl = [env.get_actual(action[0], env.agent_joint1_low, env.agent_joint1_high),
        #           env.get_actual(action[1], env.agent_joint2_low, env.agent_joint2_high), 
        #           0, # inactive joint on xArm 5
        #           env.get_actual(action[2], env.agent_joint3_low, env.agent_joint3_high), 
        #           0, # inactive joint on xArm 5
        #           env.get_actual(action[3], env.agent_joint4_low, env.agent_joint4_high),
        #           env.get_actual(action[4], env.agent_joint5_low, env.agent_joint5_high),
        #           env.get_actual(action[5], env.agent_gripper_low, env.agent_gripper_high)]
        # d.ctrl = [1, 1, 1, 1, 1, 1, 1, 1]

        # mujoco.mj_step(m, d)

        # print("Distance: ", info['distance'])
        # print("Agent Pos: ", info['agent_pos'])
        # print("Target_Pos: ", info['target_pos'])
        
        # if(info['distance'] == 0)
        #     print("DONEEEEEE")
        #     print("Number of steps: ", info['num_steps'])
        #     time.sleep(3)

        if truncated:
            print("Truncated")
            time.sleep(1)
            # break
            obs, info = env.reset()
        if terminated:
            print("Terminated")
            exit()
        
        # viewer.sync()
        
# IMPORTANT: model cant reach the target bc there are so many different actions to choose from so will take FOREVER

"""
TO DO:
-look into existing environment example of moving by velocity / displacement -> implememting control frequency for velocity over time
- using history and state transitions (if distance is not changing and you already performed that action last step, the next action should be different than the precious history of actions)


- position changed by = (assumed speed mm/s * (sign of positional displacement for that axis) ) / time in seconds passed since last time step() was called
- above requires history of last action performed, and needs to account for control freq -> solution apparently is to perform the same action for multiple steps before choosing a new action


- implement physics engine and get working then connect and use the real robot and its functions? -> but once i use the real robot rather using the physics engine functions i would be calling the real robot -> so ignore physics engine and use the real robot
"""
