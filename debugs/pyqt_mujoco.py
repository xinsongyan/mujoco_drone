import sys
import time
import numpy as np
import mujoco
import mujoco.viewer
from PyQt5.QtWidgets import QApplication
import pyqtgraph as pg

from general_dashboard import GeneralDashboard

# --- 2. Main Execution (Physics is the "Boss") ---
if __name__ == '__main__':
    # A. Init Qt (Must be in Main Thread)
    app = QApplication(sys.argv)
    
    # Configure plots
    my_plots = [
        {"title": "Pendulum Angle (rad)", "color": "g"},
        {"title": "Angular Velocity (rad/s)", "color": "c"}
    ]
    
    dashboard = GeneralDashboard(my_plots)
    dashboard.show()

    # B. Init MuJoCo
    xml_string = """
    <mujoco>
      <option timestep="0.002" gravity="0 0 -9.81"/>
      <visual><global offwidth="1920" offheight="1080"/></visual>
      <worldbody>
        <light diffuse=".5 .5 .5" pos="0 0 3" dir="0 0 -1"/>
        <body pos="0 0 1" euler="0 180 0">
          <joint name="pin" type="hinge" axis="0 1 0" damping="0.01"/>
          <geom type="capsule" size=".02" fromto="0 0 0 0 0 1" rgba="1 0 0 1"/>
          <body pos="0 0 1">
            <geom type="sphere" size=".1" rgba="0 1 0 1"/>
          </body>
        </body>
      </worldbody>
    </mujoco>
    """
    model = mujoco.MjModel.from_xml_string(xml_string)
    data = mujoco.MjData(model)
    data.qpos[0] = 1.57
    
    # Launch passive viewer
    viewer = mujoco.viewer.launch_passive(model, data)

    # --- C. The Main Loop ---
    # WE control the loop, not Qt.
    step_count = 0
    
    while viewer.is_running():
        # 1. Step Physics
        mujoco.mj_step(model, data)

        # 2. Sync MuJoCo Viewer
        viewer.sync()

        # 3. Update PyQt GUI (Limit update frequency)
        if step_count % 8 == 0:
            angle = data.qpos[0]
            velocity = data.qvel[0]
            
            dashboard.update_dashboard([angle, velocity], data.time)
            
            # *** CRITICAL ***
            # This line processes all GUI events (clicks, redraws, resizes).
            # Without this, the window will freeze and say "Not Responding".
            app.processEvents() 
        
        step_count += 1
        # 4. Sleep to Control Loop Rate
        time.sleep(model.opt.timestep)

    # Cleanup
    viewer.close()
    sys.exit()