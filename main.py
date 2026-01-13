import os
import mujoco
import mujoco.viewer 
import numpy as np
import transformations as tf 
import time

from pid_controller import PIDController
from se3controller import SE3Controller
from state_estimator import StateEstimator
from user_command import UserCommand







class Drone: 
    def __init__(self):
        self.xml_path = os.path.join('assets', 'skydio_x2', 'scene_x2.xml')
        print(f"xml_path: {self.xml_path} ")

        self.m = mujoco.MjModel.from_xml_path(self.xml_path)
        self.d = mujoco.MjData(self.m)

        # body_id = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_BODY, "x2")  # or model.body_name2id("name")
        # inertia_diag = self.m.body_inertia[body_id]
        # print(f"Body ID: {body_id}, Inertia Diagonal: {inertia_diag}")
        # input()
        
        # self.user_cmd = UserCommand()
        self.state_estimator = StateEstimator(self.m, self.d)
        
        
        self.stabilisation_controller = PIDController(z_des=0.5, rpy_setpoint=[0,0,0], state_estimator=self.state_estimator)
        # self.se3_controller = SE3Controller(state_estimator=self.state_estimator, user_cmd=None)



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

        thrust_total, torque_roll, torque_pitch, torque_yaw = self.stabilisation_controller.compute_control()

        # thrust_total, torque_roll, torque_pitch, torque_yaw = self.se3_controller.compute_control()
        # f, M = self.se3_controller.tracking_control(pos_des=[0.5, 0.5, 0.5], heading_des=[1, 0, 0])

        motor_cmd = self.cal_motor_cmd(thrust_total, torque_roll, torque_pitch, torque_yaw)
        # motor_cmd = self.cal_motor_cmd(f, M[0], M[1], M[2])
        # print(f"Motor Commands: {motor_cmd}")

        self.set_motor_cmd(motor_cmd)

    # def cal_motor_cmd(self, thrust_total, torque_roll, torque_pitch, torque_yaw):
    #     motor_cmd = np.array([thrust_total - torque_roll + torque_pitch + torque_yaw,
    #                         thrust_total - torque_roll - torque_pitch - torque_yaw,
    #                         thrust_total + torque_roll + torque_pitch - torque_yaw,
    #                         thrust_total + torque_roll - torque_pitch + torque_yaw])

    #     return motor_cmd
    
    def cal_motor_cmd(self, T, tau_x, tau_y, tau_z):
        dx, dy, k = 0.14, 0.18, 0.02 # distance from motor to x axis, y aixs, motor constant
        f1 = (T - tau_x/dx + tau_y/dy + tau_z/k) / 4.0
        f2 = (T - tau_x/dx - tau_y/dy - tau_z/k) / 4.0
        f3 = (T + tau_x/dx + tau_y/dy - tau_z/k) / 4.0
        f4 = (T + tau_x/dx - tau_y/dy + tau_z/k) / 4.0
        return np.array([f1, f2, f3, f4])  
    
    def set_motor_cmd(self, motor_cmd):
        # Set the motor commands to the actuators
        self.d.ctrl[:4] = motor_cmd





def main():
    
    drone = Drone()
    
    with mujoco.viewer.launch_passive(drone.m, drone.d) as viewer:

        viewer.opt.frame = mujoco.mjtFrame.mjFRAME_BODY        # Show body frames
        viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_ACTUATOR] = True  # Enable site visualization

        while viewer.is_running():

            print(f"Time: {drone.d.time:.2f}s")
            
            drone()

            mujoco.mj_step(drone.m, drone.d)

            viewer.sync()

            time.sleep(drone.m.opt.timestep) 


if __name__ == "__main__":
    main()
