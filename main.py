import os
import sys
import mujoco
import mujoco.viewer 
import numpy as np
import time
from PyQt5.QtWidgets import QApplication

from mujoco_drone.input.key_callback import create_key_callback
from mujoco_drone.drone import SimpleDrone
from mujoco_drone.dashboard import DroneSimulationDashboard
from mujoco_drone.visuals import draw_thrust_visualization


state = {"paused": False, "drone": None}


if __name__ == "__main__":
    # Initialize dashboard
    dashboard = DroneSimulationDashboard()

    # Initialize the global drone instance, which the key_callback will use.
    drone = SimpleDrone(caged=True)
    state["drone"] = drone
    key_callback = create_key_callback(state)
    
    with mujoco.viewer.launch_passive(drone.m, drone.d, key_callback=key_callback) as viewer:

        step_count = 0

        while viewer.is_running():

            if not state["paused"]:
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
            viewer.set_texts([(0, 0, f"Time: {drone.d.time:.3f}", f"Paused: {state['paused']}")])

            # Always sync the viewer to process events / render
            viewer.sync()
