import time

import mujoco
import mujoco.viewer
import numpy as np

m = mujoco.MjModel.from_xml_path('/Users/albert/REU/2024/MuJoCo/model/xArm 5/scene.xml')
d = mujoco.MjData(m)

i = 0

with mujoco.viewer.launch_passive(m, d) as viewer:
  # Close the viewer automatically after 30 wall-seconds.
  start = time.time()
  viewer.sync()
  # while viewer.is_running() and time.time() - start < 30:

# while True:
#   viewer.sync()
#   i += 1
#   d.ctrl[7] = i % 255
#   viewer.sync()

  while viewer.is_running():
    step_start = time.time()

    # NOTE: d.ctrl = [joint1 (-6.28, 6.28), joint2 (-2.06, 2.09), 
    #           joint3 (not in xArm 5) (-6.28, 6.28), joint4 (-0.192, 3.93), 
    #           joint5 (not in xArm5) (-6.28, 6.28), joint6 (-1.69, 3.14), 
    #           joint7 (-6.28, 6.28), gripper (0, 255)]
    # print(d.ctrl, "\n======")

    viewer.sync()

    # mj_step can be replaced with code that also evaluates
    # a policy and applies a control signal before stepping the physics.
    mujoco.mj_step(m, d)

    # i += 1
    # if i % 255 == 244:
    #   d.ctrl = [1, 1, 1, 1, 1, 1, 1, 255]
    # elif i % 255 == 127:
    #   d.ctrl = [0, 0, 0, 0, 0, 0, 0, 0]
    

    # Example modification of a viewer option: toggle contact points every two seconds.
    # with viewer.lock():
    #   viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = int(d.time % 2)

    # Pick up changes to the physics state, apply perturbations, update options from GUI.
    viewer.sync()

    block_id = int(m.geom("block_geom").id)
    gripper_id = int(m.geom("gripper_geom").id)
    distance = mujoco.mj_geomDistance(m, d, block_id, gripper_id, 100, None)
    print("Distance: ", distance)

    block_pos = d.geom("block_geom").xpos
    gripper_pos = d.geom("gripper_geom").xpos
    distance2 = np.sqrt(((block_pos[0] - gripper_pos[0]) ** 2) +
                        ((block_pos[1] - gripper_pos[1]) ** 2) + 
                        ((block_pos[2] - gripper_pos[2]) ** 2))
    print("Distance 2: ", distance2)

    print("Block: ", block_pos)
    print("Gripper: ", gripper_pos)
    # print(1 / np.round(distance, decimals=2))

    # print(d.ctrl)

    # Rudimentary time keeping, will drift relative to wall clock.
    time_until_next_step = m.opt.timestep - (time.time() - step_start)
    if time_until_next_step > 0:
      time.sleep(time_until_next_step)

    viewer.sync()