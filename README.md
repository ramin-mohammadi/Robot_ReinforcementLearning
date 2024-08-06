# Main Files to Look At

## Reinforcement Learning using Gymnasium and xArm API

### xArmEnv.py
- ReinforcementLearning/different_env_packages/METHOD/env_package/FactoryRobotArm/envs/xArmEnv.py
- Code for the gymnaisum custom environment containing functions such as step(), __init__(), reset(), and initialzing the connection to the robot arm

### xArmClass.py
- ReinforcementLearning/different_env_packages/METHOD/env_package/FactoryRobotArm/xArmClass.py
- Code for the robot's class (boiler plate from a Blockly example) and contains functions that the environment in xArmEnv.py calls to perform actions with the robot (move to this position or open gripper, etc.)
- Utilize the Blockly software and python conversion to get a boiler plate class structure and understanding of different commands. In addition look at the xArm SDK repo's API for functions to use outside of what's available in Blockly: https://github.com/xArm-Developer/xArm-Python-SDK
   
### test_env.py / test_env_sim.py
- ReinforcementLearning/different_env_packages/METHOD/env_package/test_env.py
- This can be seen as the main.py where we actually call/create our gymnasium custom environment, pick a policy (PPO or DQN), train the policy with our environment, and then test the trained policy / Reinforcement Learning model
- You will run this file to train the policy
- test_env_sim.py is involved in the mujoco physics engine simulation implementation

### __init__.py
- ReinforcementLearning/different_env_packages/METHOD/env_package/FactoryRobotArm/ __init__.py
- location where you specify the max number of steps that can be taken in an episode

### Assemble Disassemble Demo
- ReinforcementLearning/two_agent_demo_assemble_dissassemble
- run main.py, specify n number of blocks being picked up and their locations
- Utilizes the big xArm gripper and the small gripper robots

### IMPORTANT: Various Env_Packages
- ReinforcementLearning/different_env_packages contains various custom environments:
  - 1) Perform Continous Actions (velocity x y z, and gripper) directly using physical real world xArm big gripper
  - 2) Perform Continuous Actions in Mujoco Physics Engine
  - 3) Perform Discrete Actions (move to target, move to destination, open/close gripper) directly using physical real world xArm big gripper

## Object Detection
- object_detection_calibration.py contains the openCV color recognition code, uses a resolution of 1920x1080 60fps, and uses the .npz from the camera calibration to remove the distortion.

## Camera Calibration
- Run camera_calibrations.py with the path to the set of images of whatever pattern you're using (checkerboard, circles, etc) and its size (ex: 9x6). Output will write the camera matrix, distortion coefficients, rotational vectors and translational vectors to a .npz file to be read whenever the information is needed.

## Camera Robot Coordination
-  The PNP folder contains the attempts at solving the pixel to robot coordinate system using HOMOGRAPHY approach. pnp.py uses OpenCv's solvePnp() and pnp_manual.py involved manually creating the matrices in the homography equation
-  The .xlsx file contains data collected to create Best Fit Lines for the x+ and y+ robot axes
- robot_calibration_ERROR_checking.py was used to determine if error of the xArm's movement (ex: if a command to move 10mm in x+ actually moves 10mm in real world)
- the pixel mapping points folder contains the demo of assembling blocks using the following method to obtain the block coordinates: creating multiple pixel segments along camera's x and y and mapping them to robot coordinates 

# Setup 

### Download
- git
- vscode
- python
- anaconda
- mujoco

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
- in VSCode when running test_env.py, make sure IDE is using the python installed in your conda environment (look at bottom right of screen)
- CONNECT to xArm's IP:
  - https://docs.google.com/document/d/165G8HF38ADn8U5sJKdZGC6twOD5wrw_p3mPBK9e5eR0/edit?usp=sharing

- xArm Manuals
  - https://drive.google.com/drive/folders/1LeDknZFKG332gVclHP5J5epk2ptgiheE?usp=drive_link 
