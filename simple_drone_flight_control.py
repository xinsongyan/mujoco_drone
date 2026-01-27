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



from state_estimator import StateEstimator

from pid_controller import PIDController
from flight_control import FlightControl
from Debug.general_dashboard import GeneralDashboard






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
        
        self.flight_control = FlightControl(target_x=0.0, 
                                            target_y=0.0, 
                                            target_z=0.3, 
                                            target_yaw=0.0, 
                                            state_estimator=self.state_estimator)

    def set_pos(self, pos):
        # Set the position of the drone's base
        self.d.qpos[:3] = pos
        
    def set_quat(self, quat):
        # Set the orientation of the drone's base
        self.d.qpos[3:7] = quat


    def __call__(self):
        # print(f"user_cmd: {self.user_cmd.get_input()}")
        # print("Base Position:", [f"{x:.3f}" for x in self.state_estimator.base_pos])
        # print("Base Quaternion:", [f"{x:.3f}" for x in self.state_estimator.base_quat])
        # print("Base RPY:", [f"{x:.3f}" for x in self.state_estimator.base_rpy])
        
        # thrust_total, torque_roll, torque_pitch, torque_yaw = self.stabilisation_controller.compute_control()
        self.thrust_total, self.torque_roll, self.torque_pitch, self.torque_yaw = self.flight_control.compute_control()
        

        motor_cmd = self.cal_motor_cmd(self.thrust_total, self.torque_roll, self.torque_pitch, self.torque_yaw)

        self.set_motor_cmd(motor_cmd)




    def cal_motor_cmd(self, T, tau_x, tau_y, tau_z):
        dx, dy, k = 0.1, 0.1, 0.02 # distance from motor to x axis, y aixs, motor constant
        f_rr = (T - tau_x/dx + tau_y/dy + tau_z/k) / 4.0
        f_fr = (T - tau_x/dx - tau_y/dy - tau_z/k) / 4.0
        f_rl = (T + tau_x/dx + tau_y/dy - tau_z/k) / 4.0
        f_fl = (T + tau_x/dx - tau_y/dy + tau_z/k) / 4.0
        return np.array([f_rr, f_fr, f_rl, f_fl])

    def set_motor_cmd(self, motor_cmd):
        # Set the motor commands to the actuators
        self.d.ctrl[:4] = motor_cmd





def main():
    # Initialize PyQt5 app for dashboard
    app = QApplication(sys.argv)
    
    # Configure position and control plots
    position_plots = [
        {"title": "Drone Position X (m)", "color": "r"},
        {"title": "Drone Position Y (m)", "color": "g"},
        {"title": "Drone Position Z (m)", "color": "b"},
        {"title": "Thrust Total (N)", "color": "m"}
    ]
    
    dashboard = GeneralDashboard(position_plots)
    dashboard.show()

    drone = SimpleDrone()
    
    with mujoco.viewer.launch_passive(drone.m, drone.d) as viewer:
        step_count = 0

        while viewer.is_running():

            print(f"Time: {drone.d.time:.3f}s")
            
            drone()
            
        

            mujoco.mj_step(drone.m, drone.d)

            viewer.sync()

            # Update dashboard every 8 steps
            if step_count % 10 == 0:
                pos_x = drone.d.qpos[0]
                pos_y = drone.d.qpos[1]
                pos_z = drone.d.qpos[2]
                thrust = drone.thrust_total
                
                dashboard.update_dashboard([pos_x, pos_y, pos_z, thrust], drone.d.time)
                app.processEvents()

            step_count += 1
            time.sleep(drone.m.opt.timestep) 


if __name__ == "__main__":
    main()
