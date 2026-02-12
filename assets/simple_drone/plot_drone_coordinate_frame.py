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
        xml_path = os.path.join('assets/simple_drone/simple_drone_caged_white_floor.xml')
        drone_type = "caged"
    else:
        xml_path = os.path.join('assets/simple_drone/simple_drone.xml')
        drone_type = "simple"
    
    print(f"Loading drone model: {drone_type}")
    print(f"xml_path: {xml_path} ({type(xml_path)})")
    
    model = mujoco.MjModel.from_xml_path(xml_path)
    model.vis.global_.offwidth = 1080
    model.vis.global_.offheight = 1080
    data = mujoco.MjData(model)
    
    return model, data


if __name__ == "__main__":
    # Load simple drone by default
    model, data = load_drone(caged=True)
    
    # set model position
    data.qpos[:3] = [0.4, 0.4, 0.2] # Set initial position (x, y, z)
    
    # set model orientation from euler angles (roll, pitch, yaw)
    data.qpos[3:7] = tf.quaternion_from_euler(0, np.deg2rad(30), np.deg2rad(-30)) # No initial rotation

    renderer = mujoco.Renderer(model, width=1080, height=1080)
    
    
    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.cam.azimuth = 125
        viewer.cam.elevation = -25
        viewer.cam.distance = 1.2
        viewer.cam.lookat[:] = [0.25, 0.25, 0.12]
        pyautogui.hotkey('shift', 'tab')
        saved = False
        while viewer.is_running():
  
            lookat = viewer.cam.lookat
            lookat_str = f"[{lookat[0]:.2f}, {lookat[1]:.2f}, {lookat[2]:.2f}]"
            viewer.set_texts([
                (None, None,
                 f"azimuth: {viewer.cam.azimuth:.1f}  elevation: {viewer.cam.elevation:.1f}",
                 f"distance: {viewer.cam.distance:.2f}  lookat: {lookat_str}")
            ])

            if not saved:
                # save the camera view as a picture
                renderer.update_scene(data, camera=viewer.cam)
                image = renderer.render()
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                img_dir = os.path.join(os.path.dirname(__file__), "img")
                os.makedirs(img_dir, exist_ok=True)
                output_path = os.path.join(
                    img_dir,
                    f"coordinate_frame_{timestamp}.png",
                )
                imageio.imwrite(output_path, image)
                saved = True
            

            # mujoco.mj_step(model, data)
            time.sleep(model.opt.timestep)
            viewer.sync()
           