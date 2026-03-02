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
        self.m, self.d, self.cage_radius = load_drone(caged=caged)


        # body_id = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_BODY, "x2")  # or model.body_name2id("name")
        # inertia_diag = self.m.body_inertia[body_id]
        # print(f"Body ID: {body_id}, Inertia Diagonal: {inertia_diag}")
        # input()
        
        self.user_input = UserInput()  # Initialize user input handler
        self.state_estimator = StateEstimator(self.m, self.d)

        # self.stabilisation_controller = PIDController(z_des=0.5, rpy_setpoint=[0,0,0], state_estimator=self.state_estimator)
        

        self.cascaded_controller = CascadedController(user_input=self.user_input, 
                                            state_estimator=self.state_estimator)

        self.flying_controller = SE3Controller(user_input=self.user_input, state_estimator=self.state_estimator)

        self.rolling_controller = RollingController(
            user_input=self.user_input,
            state_estimator=self.state_estimator,
            cage_radius=self.cage_radius,
        )

        self.controller = self.rolling_controller  # Choose rolling as default
        self.control_mode = "Rolling"

        # Shared trajectory (can be used by multiple controllers)
        self.traj_radius = 0.2  # meters
        self.traj_omega = 0.6   # rad/s
        self.traj_center = self.state_estimator.base_pos.copy()
        self.traj_center[0] += self.traj_radius
        self.traj_center[2] = self.cage_radius
        self.pos_target = self.state_estimator.base_pos.copy()

        # Edge-triggered A/B button handling for controller switching
        self._prev_a_pressed = False
        self._prev_b_pressed = False

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

    def update_controller_selection(self):
        a_pressed = self.user_input.button_a()
        b_pressed = self.user_input.button_b()

        # A: switch to flying controller
        if a_pressed and not self._prev_a_pressed:
            self.controller = self.flying_controller
            self.control_mode = "Flying"
            print("Switched mode -> Flying (A)")

        # B: switch to rolling controller
        if b_pressed and not self._prev_b_pressed:
            self.controller = self.rolling_controller
            self.control_mode = "Rolling"
            print("Switched mode -> Rolling (B)")

        self._prev_a_pressed = a_pressed
        self._prev_b_pressed = b_pressed

    def sample_circular_pos_target(self, t):
        theta = self.traj_omega * t + np.pi
        return np.array([
            self.traj_center[0] + self.traj_radius * np.cos(theta),
            self.traj_center[1] + self.traj_radius * np.sin(theta),
            self.traj_center[2],
        ])

    def update_shared_trajectory_target(self):
        t = self.d.time
        self.pos_target = self.sample_circular_pos_target(t)

        # Automatic controller selection by circular-path phase:
        # Rolling: [0, 90), Flying: [90, 180), Rolling: [180, 270), Flying: [270, 360)
        phase_rad = (self.traj_omega * t) % (2 * np.pi)
        phase_deg = np.degrees(phase_rad)

        if phase_deg < 90.0 or (180.0 <= phase_deg < 270.0):
            next_mode = "Rolling"
            next_controller = self.rolling_controller
        else:
            next_mode = "Flying"
            next_controller = self.flying_controller

        if next_mode != self.control_mode:
            self.control_mode = next_mode
            self.controller = next_controller
            print(f"Switched mode -> {self.control_mode} (phase: {phase_deg:.1f} deg)")

        if self.control_mode == "Flying":
            self.pos_target[2] = 0.3  # Keep flying target at a fixed altitude for better visualization
            self.flying_controller.pos_target = self.pos_target.copy()  # Update flying target for visualization and control
        elif self.control_mode == "Rolling":
            self.rolling_controller.pos_target = self.pos_target.copy()  # Update rolling target for visualization, even if not controlling

    def __call__(self):
        # print(f"user_cmd: {self.user_input.get_input()}")
        # self.update_controller_selection()
        self.update_shared_trajectory_target()

        self.thrust_total, self.torque_roll, self.torque_pitch, self.torque_yaw = self.controller.step()
        
        self.motor_cmd = self.motor_mixer.mix2(self.thrust_total, self.torque_roll, self.torque_pitch, self.torque_yaw)

        self.set_motor_cmd(self.motor_cmd)
