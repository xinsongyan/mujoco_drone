import mujoco
import numpy as np
from mujoco.glfw import glfw
import mujoco.viewer 
import time

xml = """
<mujoco>
  <worldbody>
    <light name="top" pos="0 0 1"/>
    <body name="box_and_sphere" euler="0 0 -30">
      <joint name="swing" type="hinge" axis="1 -1 0" pos="-.2 -.2 -.2"/>
      <geom name="red_box" type="box" size=".2 .2 .2" rgba="1 0 0 1"/>
      <geom name="green_sphere" pos=".2 .2 .2" size=".1" rgba="0 1 0 1"/>
    </body>
  </worldbody>
</mujoco>
"""
model = mujoco.MjModel.from_xml_string(xml)
data = mujoco.MjData(model)

def get_geom_speed(model, data, geom_name):
  """Returns the speed of a geom."""
  geom_vel = np.zeros(6)
  geom_type = mujoco.mjtObj.mjOBJ_GEOM
  geom_id = data.geom(geom_name).id
  mujoco.mj_objectVelocity(model, data, geom_type, geom_id, geom_vel, 0)
  return np.linalg.norm(geom_vel)

def add_visual_capsule(scene, point1, point2, radius, rgba):
  """Adds one capsule to an mjvScene."""
  if scene.ngeom >= scene.maxgeom:
    return
  scene.ngeom += 1  # increment ngeom
  # initialise a new capsule, add it to the scene using mjv_connector
  mujoco.mjv_initGeom(scene.geoms[scene.ngeom-1],
                      mujoco.mjtGeom.mjGEOM_ARROW, np.zeros(3),
                      np.zeros(3), np.zeros(9), rgba.astype(np.float32))
#   mujoco.mjv_connector(scene.geoms[scene.ngeom-1],
#                        mujoco.mjtGeom.mjGEOM_CAPSULE, radius,
#                        point1, point2)

 # traces of time, position and speed
times = []
positions = []
speeds = []
offset = model.jnt_axis[0]/16  # offset along the joint axis

def modify_scene(scn):
  """Draw position trace, speed modifies width and colors."""
  if len(positions) > 1:
    for i in range(len(positions)-1):
      rgba=np.array((np.clip(speeds[i]/10, 0, 1),
                     np.clip(1-speeds[i]/10, 0, 1),
                     .5, 1.))
      radius=.003*(1+speeds[i]) # Scale radius by speed
      point1 = positions[i] + offset*times[i]
      point2 = positions[i+1] + offset*times[i+1]
      add_visual_capsule(scn, point1, point2, radius, rgba)

# Reset state and time.
mujoco.mj_resetData(model, data)
mujoco.mj_forward(model, data)

with mujoco.viewer.launch_passive(model, data) as viewer:
  # The viewer will run until the user closes the window.
  while viewer.is_running():
    # append data to the traces
    positions.append(data.geom_xpos[data.geom("green_sphere").id].copy())
    times.append(data.time)
    speeds.append(get_geom_speed(model, data, "green_sphere"))

    mujoco.mj_step(model, data)

    # clear old geoms and add new ones
    viewer.user_scn.ngeom = 0
    # modify_scene(viewer.user_scn)

    # Add an arrow to the scene.
    # Get the current position and orientation of the green sphere
    sphere_id = data.geom("green_sphere").id
    sphere_pos = data.geom_xpos[sphere_id]
    sphere_mat = data.geom_xmat[sphere_id] # This is the 3x3 rotation matrix, flattened
    # Scale arrow length with velocity
    sphere_velocity = get_geom_speed(model, data, "green_sphere")
    arrow_length = 0.1 + 0.5 * sphere_velocity
    if viewer.user_scn.ngeom < viewer.user_scn.maxgeom:
      viewer.user_scn.ngeom += 1 # Increment the number of geoms
      mujoco.mjv_initGeom(viewer.user_scn.geoms[viewer.user_scn.ngeom-1],
                      mujoco.mjtGeom.mjGEOM_ARROW, 
                      np.array([0.01, 0.01, arrow_length]), # size
                      sphere_pos, 
                      sphere_mat, 
                      np.array((1,0,0,1)).astype(np.float32))

    viewer.sync()
    time.sleep(model.opt.timestep) # Control simulation speed