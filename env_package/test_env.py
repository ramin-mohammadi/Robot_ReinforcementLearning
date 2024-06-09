import FactoryRobotArm
import gymnasium
from stable_baselines3 import PPO
from stable_baselines3.ppo import MlpPolicy
from stable_baselines3 import DQN
import numpy as np
import time

env = gymnasium.make('FactoryRobotArm/xArm-v0')

# model = PPO("MultiInputPolicy", env, verbose=1)
# model.learn(total_timesteps=2000)


# obs, info = env.reset()
# while True:
#     action, _states = model.predict(obs)
#     obs, rewards, dones, info = env.step(action)
#     env.render()

model = DQN("MultiInputPolicy", env, verbose=1)
model.learn(total_timesteps=40000, log_interval=4)


obs, info = env.reset()
while True:
    action, _states = model.predict(obs, deterministic=True)
    print(action)
    action = int(action) # action was ndarray but could not correctly index into my action dictionary
    # if type(action) == np.ndarray:
    #     action = action[np.argmax(action)] 
    obs, reward, terminated, truncated, info = env.step(action)
    
    print("Distance: ", info['distance'])
    print("Agent Pos: ", info['agent_pos'])
    print("Target_Pos: ", info['target_pos'])
    
    if(info['distance'] == 0):
        print("DONEEEEEE")
        print("Number of steps: ", info['num_steps'])
        time.sleep(3)

    if terminated or truncated:
        # print("DONEEE")
        # break
        obs, info = env.reset()
        
        
        
# IMPORTANT: model cant reach the target bc there are so many different actions to choose from so will take FOREVER

"""
TO DO:
-look into existing environment example of moving by velocity / displacement -> implememting control frequency for velocity over time
- using history and state transitions (if distance is not changing and you already performed that action last step, the next action should be different than the precious history of actions)


- position changed by = (assumed speed mm/s * (sign of positional displacement for that axis) ) / time in seconds passed since last time step() was called
- above requires history of last action performed, and needs to account for control freq -> solution apparently is to perform the same action for multiple steps before choosing a new action

-figure out how to connect to robot wirelessy
-call velocity function and get position function -> for training the model, i would assume is what the physics engine is for to simulate it

- implement physics engine and get working then connect and use the real robot and its functions? -> but once i use the real robot rather using the physics engine functions i would be calling the real robot -> so ignore physics engine and use the real robot
"""