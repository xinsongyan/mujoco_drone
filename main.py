import os
import sys
import mujoco
import mujoco.viewer 
import numpy as np
import time

from PyQt5.QtWidgets import QApplication
from mujoco_drone.drone import SimpleDrone
from mujoco_drone.dashboard import DroneSimulationDashboard
from mujoco_drone.visuals import draw_thrust_visualization

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
    dashboard = DroneSimulationDashboard()

    # Initialize the drone
    drone = SimpleDrone(caged=True)
    
    with mujoco.viewer.launch_passive(drone.m, drone.d, key_callback=key_callback) as viewer:

        step_count = 0

        while viewer.is_running():

            if not paused:
                drone()

                mujoco.mj_step(drone.m, drone.d)

                # clear old geoms and add new ones
                viewer.user_scn.ngeom = 0
                
                # Visualize thrust arrows (total and per-rotor)
                draw_thrust_visualization(viewer, drone)

                # Update dashboard every 10 steps
                if step_count % 10 == 0:
                    dashboard.update(drone, drone.d.time)

                step_count += 1
                time.sleep(drone.m.opt.timestep)

            # Display status text at the bottom (always update regardless of pause state)
            viewer.set_texts([(0, 0, f"Time: {drone.d.time:.3f}", f"Paused: {paused}")])

            # Always sync the viewer to process events / render
            viewer.sync()
