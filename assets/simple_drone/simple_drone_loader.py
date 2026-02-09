import os
import mujoco
import mujoco.viewer 


def load_drone(caged=False):
    """
    Load a drone model.
    
    Args:
        caged (bool): Whether to load the caged drone. Default: False (simple drone)
    
    Returns:
        tuple: (model, data) - MuJoCo model and data objects
    """
    if caged:
        xml_path = os.path.join('assets/simple_drone/simple_drone_caged.xml')
        drone_type = "caged"
    else:
        xml_path = os.path.join('assets/simple_drone/simple_drone.xml')
        drone_type = "simple"
    
    print(f"Loading drone model: {drone_type}")
    print(f"xml_path: {xml_path} ({type(xml_path)})")
    
    model = mujoco.MjModel.from_xml_path(xml_path)
    data = mujoco.MjData(model)
    
    return model, data


if __name__ == "__main__":
    # Load simple drone by default
    model, data = load_drone(caged=False)
    
    with mujoco.viewer.launch(model, data) as viewer:
        while viewer.is_running():
            mujoco.mj_step(model, data)
            viewer.sync()