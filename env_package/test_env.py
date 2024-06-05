import FactoryRobotArm
import gymnasium
from stable_baselines3 import PPO
from stable_baselines3.ppo import MlpPolicy
from stable_baselines3 import DQN
import numpy as np

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
    
    if(info['distance'] == 0):
        print("DONEEEEEE")
        

    if terminated or truncated:
        # print("DONEEE")
        # break
        obs, info = env.reset()
        
        
# IMPORTANT: model cant reach the target bc there are so many different actions to choose from so will take FOREVER