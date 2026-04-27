import os
from datetime import datetime
import time
import mujoco
import mujoco.viewer 
import imageio.v2 as imageio
import numpy as np
import pyautogui
import transformations as tf 

def load_drone(caged=False):
    """
    Load a drone model.
    
    Args:
        caged (bool): Whether to load the caged drone. Default: False (simple drone)
    
    Returns:
        tuple: (model, data) - MuJoCo model and data objects
    """
    if caged:
        xml_path = os.path.join('assets/simple_drone/video/simple_drone_caged_white_floor.xml')
        drone_type = "caged"
    else:
        xml_path = os.path.join('assets/simple_drone/simple_drone.xml')
        drone_type = "simple"
    
    print(f"Loading drone model: {drone_type}")
    print(f"xml_path: {xml_path} ({type(xml_path)})")
    
    model = mujoco.MjModel.from_xml_path(xml_path)
    model.vis.global_.offwidth = 1080
    model.vis.global_.offheight = 1080
    # Set gravity to zero
    model.opt.gravity[:] = [0.0, 0.0, 0.0]
    data = mujoco.MjData(model)
    return model, data


if __name__ == "__main__":
    # Load simple drone by default
    model, data = load_drone(caged=True)
    
    # set model position
    data.qpos[:3] = [0.4, 0.4, 0.4] # Set initial position (x, y, z)
    
    # set model orientation from euler angles (roll, pitch, yaw)
    data.qpos[3:7] = tf.quaternion_from_euler(0, np.deg2rad(30), np.deg2rad(-30)) # No initial rotation

    renderer = mujoco.Renderer(model, width=1080, height=1080)

    # Parameters for video
    duration_sec = 10  # slower rotation, longer video
    fps = 30
    n_frames = duration_sec * fps
    azimuth_start = 0
    azimuth_end = 360
    elevation = -25  # keep elevation fixed
    distance = 3.0  # keep distance fixed
    # Center lookat on robot (from qpos)
    lookat = [float(data.qpos[0]), float(data.qpos[1]), float(data.qpos[2])]

    frames = []
    cam = mujoco.MjvCamera()
    cam.elevation = elevation
    cam.distance = distance
    cam.lookat[:] = lookat
    for i in range(n_frames):
        azimuth = azimuth_start + (azimuth_end - azimuth_start) * i / n_frames
        cam.azimuth = azimuth
        cam.elevation = elevation  # ensure elevation stays fixed
        cam.lookat[:] = lookat    # ensure lookat stays at robot center
        renderer.update_scene(data, camera=cam)
        img = renderer.render()
        frames.append(img)
        mujoco.mj_step(model, data)

    # Save video
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    out_path = os.path.join(os.path.dirname(__file__), f"drone_rotate_{timestamp}.mp4")
    imageio.mimsave(out_path, frames, fps=fps)
    print(f"Saved video to {out_path}")
           