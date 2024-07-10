import FactoryRobotArm
import gymnasium
from stable_baselines3 import PPO
from stable_baselines3.ppo import MlpPolicy
from stable_baselines3 import DQN
import numpy as np
import time

env = gymnasium.make('FactoryRobotArm/xArm-v0')




# PPO seems to be doing a better job of reaching the destination than DQN
model = PPO("MultiInputPolicy", env, verbose=1)
# model = DQN("MultiInputPolicy", env, verbose=1, learning_rate=0.001, gamma=0.8)  
# gamma is the discount factor (go for higher value bc with dense reward system, want model to go for big reward in future 
# being putting the block at the destination rather repetititve quick rewards)


model.learn(total_timesteps=300, log_interval=1)
model.save("PPO_Continuous_xArm")


# del model # remove to demonstrate saving and loading
 
exit()




# model = DQN.load("DQN_xArm_20000")
# model = PPO.load("DQN_xArm")

obs, info = env.reset()
while True:
    action, _states = model.predict(obs, deterministic=True)
    # print(action)
    action = int(action) # action was ndarray but could not correctly index into my action dictionary
    # if type(action) == np.ndarray:
    #     action = action[np.argmax(action)] 
    obs, reward, terminated, truncated, info = env.step(action)
    
    # print("Distance: ", info['distance'])
    # print("Agent Pos: ", info['agent_pos'])
    # print("Target_Pos: ", info['target_pos'])
    
    # if(info['distance'] == 0):
    #     print("DONEEEEEE")
    #     print("Number of steps: ", info['num_steps'])
    #     time.sleep(3)

    if terminated or truncated:
        print("Reached destination")
        time.sleep(2)
        # break
        obs, info = env.reset()
        

        
# IMPORTANT: model cant reach the target bc there are so many different actions to choose from so will take FOREVER

"""
TO DO:
-look into existing environment example of moving by velocity / displacement -> implememting control frequency for velocity over time
- using history and state transitions (if distance is not changing and you already performed that action last step, the next action should be different than the precious history of actions)


- position changed by = (assumed speed mm/s * (sign of positional displacement for that axis) ) / time in seconds passed since last time step() was called
- above requires history of last action performed, and needs to account for control freq -> solution apparently is to perform the same action for multiple steps before choosing a new action


- implement physics engine and get working then connect and use the real robot and its functions? -> but once i use the real robot rather using the physics engine functions i would be calling the real robot -> so ignore physics engine and use the real robot
"""