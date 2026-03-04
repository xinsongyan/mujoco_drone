import os
import sys
import csv
from datetime import datetime
try:
    import pyautogui
except Exception:
    pyautogui = None

import mujoco
import mujoco.viewer 
import numpy as np
import time
import imageio.v2 as imageio

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

    # Recording configuration
    record_duration_s = 11.0
    record_fps = 30
    record_dt = 1.0 / record_fps
    frames = []
    position_log = []
    last_record_t = None
    record_start_t = None

    with mujoco.viewer.launch_passive(drone.m, drone.d, key_callback=key_callback) as viewer:
        renderer = mujoco.Renderer(drone.m, width=1920, height=1080)
        
        # This toggles the UI (Side panels + Overlays) just like Shift+Tab
        if pyautogui is not None:
            try:
                pyautogui.hotkey('shift', 'tab')
            except Exception:
                pass

        # Set initial camera to top-down view
        viewer.cam.azimuth =225.0
        viewer.cam.elevation = -45.0
        viewer.cam.distance = 2.0
        viewer.cam.lookat[:] = [0, 0, 0]

        step_count = 0

        try:
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

                    # Record simulation time to MP4 at fixed FPS
                    if record_start_t is None:
                        record_start_t = drone.d.time
                        last_record_t = drone.d.time - record_dt

                    elapsed = drone.d.time - record_start_t
                    if elapsed <= record_duration_s and (drone.d.time - last_record_t) >= record_dt:
                        renderer.update_scene(drone.d, camera=viewer.cam)
                        frame = renderer.render()
                        frames.append(frame)

                        base_pos = drone.state_estimator.base_pos
                        control_mode = getattr(drone, "control_mode", type(drone.controller).__name__)
                        position_log.append((
                            float(drone.d.time),
                            float(elapsed),
                            float(base_pos[0]),
                            float(base_pos[1]),
                            float(base_pos[2]),
                            str(control_mode),
                        ))

                        last_record_t = drone.d.time

                    if elapsed > record_duration_s and len(frames) > 0:
                        os.makedirs("log", exist_ok=True)
                        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                        out_path = os.path.join("log", f"sim_{int(record_duration_s)}s_{timestamp}.mp4")
                        pos_path = os.path.join("log", f"sim_{int(record_duration_s)}s_{timestamp}_position.csv")
                        imageio.mimsave(out_path, frames, fps=record_fps)

                        with open(pos_path, "w", newline="") as csvfile:
                            writer = csv.writer(csvfile)
                            writer.writerow(["sim_time_s", "elapsed_s", "x_m", "y_m", "z_m", "control_mode"])
                            writer.writerows(position_log)

                        print(f"Saved recording: {out_path} ({len(frames)} frames at {record_fps} FPS)")
                        print(f"Saved position log: {pos_path} ({len(position_log)} samples)")
                        break

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
        finally:
            renderer.close()
