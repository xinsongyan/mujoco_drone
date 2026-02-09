import os
import numpy as np
import mujoco

from mujoco_drone.state_estimator import StateEstimator
from mujoco_drone.cascaded_controller import CascadedController
from mujoco_drone.se3controller import SE3Controller
from mujoco_drone.motor_mixer import MotorMixer
from assets.simple_drone.simple_drone_loader import load_drone





class SimpleDrone:
    def __init__(self, caged=False):
        """
        Initialize a SimpleDrone instance.
        
        Args:
            caged (bool): Whether to load the caged drone. Default: False (simple drone)
        """
        print(f"Loading drone: {'caged' if caged else 'simple'}")
        
        # Use the loader function to get model and data
        self.m, self.d = load_drone(caged=caged)

        # body_id = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_BODY, "x2")  # or model.body_name2id("name")
        # inertia_diag = self.m.body_inertia[body_id]
        # print(f"Body ID: {body_id}, Inertia Diagonal: {inertia_diag}")
        # input()
        
        # self.user_cmd = UserCommand()
        self.state_estimator = StateEstimator(self.m, self.d)

        # self.stabilisation_controller = PIDController(z_des=0.5, rpy_setpoint=[0,0,0], state_estimator=self.state_estimator)
        

        self.pid_controller = CascadedController(target_x=0.0, 
                                            target_y=0.0, 
                                            target_z=0.5, 
                                            target_yaw=np.deg2rad(15), 
                                            state_estimator=self.state_estimator)

        self.se3_controller = SE3Controller(state_estimator=self.state_estimator)   


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
        
        
        # self.thrust_total, self.torque_roll, self.torque_pitch, self.torque_yaw = self.pid_controller.compute_control()
        # self.motor_cmd = self.motor_mixer.mix(self.thrust_total, self.torque_roll, self.torque_pitch, self.torque_yaw)

        self.thrust_total, self.torques = self.se3_controller.tracking_control(pos_des=[0.1, 0.2, 0.5], 
                                                                               vel_des = np.array([0, 0, 0]), 
                                                                            acc_des = np.array([0, 0, 0]), 
                                                                            heading_des=[1,1,0], 
                                                                            omega_des_local = np.array([0, 0, 0]),
                                                                            omega_dot_des_local = np.array([0, 0, 0]))
        self.motor_cmd = self.motor_mixer.mix(self.thrust_total, self.torques[0], self.torques[1], self.torques[2])

        self.set_motor_cmd(self.motor_cmd)
