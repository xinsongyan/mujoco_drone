import os
import sys
try:
    import pyautogui
except Exception:
    pyautogui = None

import mujoco
import mujoco.viewer 
import numpy as np
import time

from PyQt5.QtWidgets import QApplication
from mujoco_drone.drone import SimpleDrone
from mujoco_drone.dashboard import DroneSimulationDashboard
from mujoco_drone.visuals import draw_thrust_visualization, draw_pos_target_sphere

# Pause state
paused = False

# Key callback for pause/unpause
def key_callback(keycode):
    global paused
    if keycode == 32:  # Spacebar
        paused = not paused
        print(f"Simulation {'paused' if paused else 'resumed'}")


if __name__ == "__main__":
    # Initialize dashboard
    # dashboard = DroneSimulationDashboard()

    # Initialize the drone
    drone = SimpleDrone(caged=True)
    
    with mujoco.viewer.launch_passive(drone.m, drone.d, key_callback=key_callback) as viewer:
        
        # This toggles the UI (Side panels + Overlays) just like Shift+Tab
        if pyautogui is not None:
            try:
                pyautogui.hotkey('shift', 'tab')
            except Exception:
                pass

        # Set initial camera to top-down view
        viewer.cam.azimuth =0.0
        viewer.cam.elevation = -90.0
        viewer.cam.distance = 2.0
        viewer.cam.lookat[:] = [0, 0, 0]

        step_count = 0

        while viewer.is_running():

            if not paused:
                drone()

                mujoco.mj_step(drone.m, drone.d)

                # clear old geoms and add new ones
                viewer.user_scn.ngeom = 0
                
                # Visualize thrust arrows (total and per-rotor)
                draw_thrust_visualization(viewer, drone)

                # Visualize controller position target
                if hasattr(drone.controller, "pos_target"):
                    draw_pos_target_sphere(viewer, drone.controller.pos_target)

                # # Update dashboard every 10 steps
                # if step_count % 10 == 0:
                #     dashboard.update(drone, drone.d.time)

                step_count += 1
                time.sleep(drone.m.opt.timestep)

            # Display status text at the bottom (always update regardless of pause state)
            control_mode = getattr(drone, "control_mode", type(drone.controller).__name__)
            viewer.set_texts([(0, 0, f"Time: {drone.d.time:.2f}", f"Paused: {paused} | Control Mode: {control_mode}")])

            # Always sync the viewer to process events / render
            viewer.sync()
