import mujoco
import mujoco.viewer
import numpy as np

# Load a simple model
model = mujoco.MjModel.from_xml_string("""
<mujoco>
  <worldbody>
    <light diffuse=".5 .5 .5" pos="0 0 3"/>
    <geom type="plane" size="1 1 0.1" rgba=".9 0 0 1"/>
    <body name="box" pos="0 0 1">
      <joint type="free"/>
      <geom type="box" size=".1 .1 .1" rgba="0 .9 0 1"/>
    </body>
  </worldbody>
</mujoco>
""")
data = mujoco.MjData(model)

# 1. Initialize the figure
fig = mujoco.MjvFigure()
fig.title = "Real-time Body Height"
fig.xlabel = "Time (s)"
fig.gridsize = [5, 5]
fig.range[0][0] = -0.5  # X-axis min
fig.range[0][1] = 5.0   # X-axis max
fig.range[1][0] = 0.0   # Y-axis min
fig.range[1][1] = 2.0   # Y-axis max

def update_figure(fig, current_time, current_val):
    # Number of points currently in the line
    n_points = fig.linepnt[0]
    
    # If buffer is full (max 1000 pts), shift data or reset
    if n_points >= 1000:
        fig.linepnt[0] = 0
        n_points = 0
        
    # mjvFigure uses a flattened float array: [x0, y0, x1, y1, ...]
    fig.linedata[0][2 * n_points] = current_time
    fig.linedata[0][2 * n_points + 1] = current_val
    fig.linepnt[0] += 1

# 2. Custom Render Callback
def overlay_callback(viewport, context):
    # Render the figure in the bottom left
    # rect: [left, bottom, width, height] in pixels
    rect = mujoco.MjrRect(0, 0, 400, 300)
    mujoco.mjr_figure(rect, fig, context)

with mujoco.viewer.launch_passive(model, data) as viewer:
    # Register our overlay (use safe fallbacks for different viewer wrappers)
    try:
      viewer.add_overlay(mujoco.mjtGridPos.mjGRID_BOTTOMLEFT, overlay_callback)
    except AttributeError:
      # some viewer handles wrap the real viewer on `.viewer`
      if hasattr(viewer, "viewer") and hasattr(viewer.viewer, "add_overlay"):
        viewer.viewer.add_overlay(mujoco.mjtGridPos.mjGRID_BOTTOMLEFT, overlay_callback)
      elif hasattr(viewer, "add_overlay_callback"):
        viewer.add_overlay_callback(mujoco.mjtGridPos.mjGRID_BOTTOMLEFT, overlay_callback)
      else:
        print("Warning: viewer does not support add_overlay; overlay not registered")
    
    while viewer.is_running():
        step_start = data.time
        
        # Step simulation
        mujoco.mj_step(model, data)
        
        # 3. Update data points (e.g., track the z-height of the box)
        box_height = data.body("box").xpos[2]
        update_figure(fig, data.time, box_height)
        
        # Update X-axis range to "scroll"
        if data.time > fig.range[0][1]:
            fig.range[0][0] += 0.01
            fig.range[0][1] += 0.01
        
        viewer.sync()