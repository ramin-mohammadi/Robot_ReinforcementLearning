from gymnasium.envs.registration import register

register(
    id='FactoryRobotArm/xArm-v0',
    entry_point='FactoryRobotArm.envs:xArmEnv',
    max_episode_steps=75,
)