import sys
import time
import numpy as np
import mujoco
import mujoco.viewer
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget
import pyqtgraph as pg

from general_dashboard import GeneralDashboard

# --- 2. Main Execution ---
if __name__ == '__main__':
    app = QApplication(sys.argv)

    # --- CONFIGURATION HERE ---
    # You can easily add/remove plots here without touching the class code
    my_plots = [
        {"title": "Pendulum Angle (rad)", "color": "g"},
        {"title": "Angular Velocity (rad/s)", "color": "c"},
        {"title": "Simulated Control Input", "color": "m"} 
    ]
    
    dashboard = GeneralDashboard(my_plots)
    dashboard.show()

    # Setup MuJoCo (Standard Pendulum)
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
    
    viewer = mujoco.viewer.launch_passive(model, data)
    step_count = 0
    
    while viewer.is_running():
        mujoco.mj_step(model, data)
        viewer.sync()

        if step_count % 10 == 0:  # Update every 10 steps
            angle = data.qpos[0]
            velocity = data.qvel[0]
            dummy_control = np.sin(data.time)
            
            dashboard.update_dashboard([angle, velocity, dummy_control], data.time)
            app.processEvents()
        
        step_count += 1
        time.sleep(model.opt.timestep)

    viewer.close()
    sys.exit()