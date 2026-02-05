import os
import sys
from fontTools.merge.util import current_time
import mujoco
import mujoco.viewer 
import numpy as np
from pygame.draw import line
import transformations as tf 
import time
from PyQt5.QtWidgets import QApplication

from mujoco_drone.input.key_callback import create_key_callback



from mujoco_drone.state_estimator import StateEstimator

from mujoco_drone.cascaded_controller import CascadedController
from mujoco_drone.motor_mixer import MotorMixer
from mujoco_drone.visuals import draw_drone_thrust_arrow, draw_motor_thrust_arrows
from debugs.general_dashboard import GeneralDashboard, place_dashboard_on_monitor





state = {"paused": False, "drone": None}


class SimpleDrone: 
    def __init__(self):
        self.xml_path = os.path.join('assets', 'simple_drone', 'simple_drone.xml')
        print(f"xml_path: {self.xml_path} ")
 
        self.m = mujoco.MjModel.from_xml_path(self.xml_path)
        self.d = mujoco.MjData(self.m)

        # body_id = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_BODY, "x2")  # or model.body_name2id("name")
        # inertia_diag = self.m.body_inertia[body_id]
        # print(f"Body ID: {body_id}, Inertia Diagonal: {inertia_diag}")
        # input()
        
        # self.user_cmd = UserCommand()
        self.state_estimator = StateEstimator(self.m, self.d)

        # self.stabilisation_controller = PIDController(z_des=0.5, rpy_setpoint=[0,0,0], state_estimator=self.state_estimator)
        

        self.controller = CascadedController(target_x=0.0, 
                                            target_y=0.0, 
                                            target_z=0.5, 
                                            target_yaw=np.deg2rad(15), 
                                            state_estimator=self.state_estimator)

        # The physical parameters for the motor mixer
        dx, dy, k = 0.1, 0.1, 0.02
        self.motor_mixer = MotorMixer(dx, dy, k)


    def set_pos(self, pos):
        # Set the position of the drone's base
        self.d.qpos[:3] = pos
        
    def set_quat(self, quat):
        # Set the orientation of the drone's base
        self.d.qpos[3:7] = quat

    def set_motor_cmd(self, motor_cmd):
        # Set the motor commands to the actuators
        self.d.ctrl[:4] = motor_cmd


    def __call__(self):
        # print(f"user_cmd: {self.user_cmd.get_input()}")
        # print("Base Position:", [f"{x:.3f}" for x in self.state_estimator.base_pos])
        # print("Base Quaternion:", [f"{x:.3f}" for x in self.state_estimator.base_quat])
        # print("Base RPY:", [f"{x:.3f}" for x in self.state_estimator.base_rpy])
        
        # thrust_total, torque_roll, torque_pitch, torque_yaw = self.stabilisation_controller.compute_control()
        self.thrust_total, self.torque_roll, self.torque_pitch, self.torque_yaw = self.controller.compute_control()
        

        self.motor_cmd = self.motor_mixer.mix(self.thrust_total, self.torque_roll, self.torque_pitch, self.torque_yaw)

        self.set_motor_cmd(self.motor_cmd*0)








if __name__ == "__main__":
# Initialize PyQt5 app for dashboard
    app = QApplication(sys.argv)
    
    # Configure position and control plots
    position_plots = [
        {"title": "Drone Position X (m)", "color": "r"},
        {"title": "Drone Position Y (m)", "color": "g"},
        {"title": "Drone Position Z (m)", "color": "b"},
        {"title": "Thrust Total (N)", "color": "m"},
        {"title": "Drone Roll (deg)", "color": "r"},
        {"title": "Drone Pitch (deg)", "color": "g"},
        {"title": "Drone Yaw (deg)", "color": "b"}
    ]
    
    dashboard = GeneralDashboard(position_plots)
    dashboard.show()


    # Initialize the global drone instance, which the key_callback will use.
    drone = SimpleDrone()
    state["drone"] = drone
    key_callback = create_key_callback(state)
    
    with mujoco.viewer.launch_passive(drone.m, drone.d, key_callback=key_callback) as viewer:

        step_count = 0

        while viewer.is_running():

            # print(f"Time: {drone.d.time:.3f}s")
            
            # Check if the Spacebar (or UI button) has toggled the pause state

            if not state["paused"]:
                # This displays a single line of text at the bottom
                viewer.status_msg = f"Time: {drone.d.time:.3f} | Paused: {state['paused']}"
        
                drone()

                mujoco.mj_step(drone.m, drone.d)

                # clear old geoms and add new ones
                viewer.user_scn.ngeom = 0
                
                # Visualize thrust as an arrow from drone center
                draw_drone_thrust_arrow(viewer, drone)
                # Visualize thrust for each rotor
                draw_motor_thrust_arrows(viewer, drone)

                # Always sync the viewer to process events / render
                viewer.sync()

                # Update dashboard every 10 steps
                if step_count % 10 == 0:
                    pos_x = drone.d.qpos[0]
                    pos_y = drone.d.qpos[1]
                    pos_z = drone.d.qpos[2]
                    thrust = drone.thrust_total
                    roll, pitch, yaw = drone.state_estimator.base_rpy

        
                    dashboard.update_dashboard([pos_x, pos_y, pos_z, thrust, np.degrees(roll), np.degrees(pitch), np.degrees(yaw)], drone.d.time)
                    app.processEvents()

                step_count += 1
                time.sleep(drone.m.opt.timestep) 
