import big_gripper
import small_gripper
from xarm import version
from xarm.wrapper import XArmAPI


big_gripper.BigGripper.pprint('xArm-Python-SDK Version:{}'.format(version.__version__))
arm = XArmAPI('192.168.1.207', baud_checkset=False)
robot_big_gripper = big_gripper.BigGripper(arm)

small_gripper.SmallGripper.pprint('xArm-Python-SDK Version:{}'.format(version.__version__))
arm = XArmAPI('192.168.1.176', baud_checkset=False)
robot_small_gripper = small_gripper.SmallGripper(arm)

height_block = 19.5
xy_blocks = [[1.4, 230], [1.4, 230], [1.4, 230], [1.4, 230]]
n_blocks = 4
# ASSEMBLE
robot_big_gripper.assemble(xy_blocks=xy_blocks, n_blocks=n_blocks, height_block=height_block)

# DISSASSEMBLE
"""
Disassemble loop:
- small gripper get in position for ith block
- big gripper grabs block
"""
for i in reversed(range(n_blocks)):
    robot_small_gripper.disassemble_ith(i=i, xy_blocks=xy_blocks, n_blocks=n_blocks, height_block=height_block)
    robot_big_gripper.disassemble_ith(i=i, xy_blocks=xy_blocks, n_blocks=n_blocks, height_block=height_block)


robot_small_gripper.move_initial()
