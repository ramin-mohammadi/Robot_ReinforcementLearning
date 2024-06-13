# Main Files to Look At

## xArmEnv.py
- env_package\FactoryRobotArm\envs\xArmEnv.py
- Code for the gymnaisum custom environment containing functions such as step(). __init__(), reset(), and initialzing the connection to the robot arm

## xArmClass.py
- env_package\FactoryRobotArm\xArmClass.py
- Code for the robot's class (boiler plate from a Blockly example) and contains functions that the environment in xArmEnv.py calls to perform actions with the robot (move to this position or open gripper, etc.)

## test_env.py
- env_package\test_env.py
- This can be seen as the main.py where we actually call/create our gymnasium custom environment, pick a policy (PPO or DQN), train the policy with our environment, and then test the trained policy / Reinforcement Learning model

## __init__.py
- env_package\FactoryRobotArm\__init__.py
- location where you specify the max number of steps that can be taken in an episode
