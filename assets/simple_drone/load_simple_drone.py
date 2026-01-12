import os
import mujoco
import mujoco.viewer 


xml_path = os.path.join('assets/simple_drone/simple_drone.xml')
print(f"xml_path: {xml_path} ({type(xml_path)})")

model = mujoco.MjModel.from_xml_path(xml_path)
data = mujoco.MjData(model)


with mujoco.viewer.launch(model, data) as viewer:
    while viewer.is_running():
        mujoco.mj_step(model, data)
        viewer.sync()