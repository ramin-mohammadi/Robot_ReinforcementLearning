## Main Files to Look At

### xArmEnv.py
- env_package\FactoryRobotArm\envs\xArmEnv.py
- Code for the gymnaisum custom environment containing functions such as step(). __init__(), reset(), and initialzing the connection to the robot arm

### xArmClass.py
- env_package\FactoryRobotArm\xArmClass.py
- Code for the robot's class (boiler plate from a Blockly example) and contains functions that the environment in xArmEnv.py calls to perform actions with the robot (move to this position or open gripper, etc.)

### test_env.py
- env_package\test_env.py
- This can be seen as the main.py where we actually call/create our gymnasium custom environment, pick a policy (PPO or DQN), train the policy with our environment, and then test the trained policy / Reinforcement Learning model
- You will run this file to train the policy

### __init__.py
- env_package\FactoryRobotArm\__init__.py
- location where you specify the max number of steps that can be taken in an episode

## Setup 

### Download
- git
- vscode
- python
- anaconda

### Steps
- open anaconda prompt
- cd into a directory where you want to place the code
- create new conda env: conda create -n xArm_Env
- enter the conda env: conda activate xArm_Env
- install python to conda env: conda install python=3.10.14
- Download xArm Software code: git clone https://github.com/xArm-Developer/xArm-Python-SDK.git
- Download dependencies: python setup.py install
- Clone this repo to get the custom gymnasium environment code: git clone https://github.com/RockLee117/Robot_ReinforcementLearning.git
- then move the downloaded env_package folder into the xArm-Python-SDK folder
- (YOU WILL HAVE TO REPEAT THIS STEP ANYTIME YOU MAKE CHANGES TO CODE IN THE CUSTOM ENVIRONMENT) In the anaconda prompt with same env activated, download the custom gymnasium env and its required dependencies:
- cd into the env_package folder
- pip install -e .
