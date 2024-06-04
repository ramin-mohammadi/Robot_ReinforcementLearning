from setuptools import setup

# in anaconda prompt, activate the deep_robot env, cd into C:\RaminMohammadi_REU2024\env_package, then run : pip install -e . -> everytime env changed/updated

setup(
    name="FactoryRobotArm",
    version="0.0.1",
    install_requires=["gymnasium==0.29.1", "numpy==1.26.4", "plotly==5.22.0", "pandas==2.2.2"],
)