import os
import numpy as np
import mujoco

from mujoco_drone.input.user_input import UserInput
from mujoco_drone.state_estimator import StateEstimator
from mujoco_drone.cascaded_controller import CascadedController
from mujoco_drone.se3controller import SE3Controller
from mujoco_drone.motor_mixer import MotorMixer
from assets.simple_drone.simple_drone_loader import load_drone

from mujoco_drone.rolling_controller import RollingController




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
        
        self.user_input = UserInput()  # Initialize user input handler
        self.state_estimator = StateEstimator(self.m, self.d)

        # self.stabilisation_controller = PIDController(z_des=0.5, rpy_setpoint=[0,0,0], state_estimator=self.state_estimator)
        

        self.cascaded_controller = CascadedController(user_input=self.user_input, 
                                            state_estimator=self.state_estimator)

        self.se3_controller = SE3Controller(user_input=self.user_input, state_estimator=self.state_estimator)   

        self.rolling_controller = RollingController(state_estimator=self.state_estimator)   

        self.controller = self.se3_controller  # Choose which controller to use

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
        # print(f"user_cmd: {self.user_input.get_input()}")
        
        # self.thrust_total, self.torque_roll, self.torque_pitch, self.torque_yaw = self.cascaded_controller.step()

        # self.thrust_total, self.torque_roll, self.torque_pitch, self.torque_yaw = self.se3_controller.step(pos_des=[0.1, 0.1, 0.4], heading_des=[1,0,0])
        
        self.thrust_total, self.torque_roll, self.torque_pitch, self.torque_yaw = self.controller.step()
        
        self.motor_cmd = self.motor_mixer.mix(self.thrust_total, self.torque_roll, self.torque_pitch, self.torque_yaw)

        self.set_motor_cmd(self.motor_cmd)
