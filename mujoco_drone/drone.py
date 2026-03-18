import os
import mujoco

from mujoco_drone.input.user_input import UserInput
from mujoco_drone.state_estimator import StateEstimator
from mujoco_drone.cascaded_controller import CascadedController
from mujoco_drone.se3controller import SE3Controller
from mujoco_drone.motor_mixer import MotorMixer
from mujoco_drone.mission import TeleoperatedMission
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

        self.controller = self.rolling_controller
        self.control_mode = "Rolling"

        # Mission is injected externally via set_mission()
        self.mission = None
        self.mission_duration = None
        self.teleop_mission = TeleoperatedMission()
        self.pos_target = self.state_estimator.base_pos.copy()

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

    def set_mission(self, mission):
        self.mission = mission
        self.mission_duration = mission.mission_duration

    def update_controller_selection(self):
        next_mode = self.teleop_mission.select_mode(self.user_input, self.control_mode)
        if next_mode != self.control_mode:
            self.control_mode = next_mode
            self.controller = self.rolling_controller if next_mode == "Rolling" else self.flying_controller
            print(f"Switched mode -> {self.control_mode} (teleop)")

    def update_shared_trajectory_target(self):
        t = self.d.time
        self.pos_target, next_mode, phase_id = self.mission.target_and_mode(t)
        next_controller = self.rolling_controller if next_mode == "Rolling" else self.flying_controller

        if next_mode != self.control_mode:
            self.control_mode = next_mode
            self.controller = next_controller
            print(f"Switched mode -> {self.control_mode} (phase: {phase_id})")

        # Targets are passed directly into controller step() calls.

    def __call__(self):
        # print(f"user_cmd: {self.user_input.get_input()}")
        # self.update_controller_selection()
        self.update_shared_trajectory_target()

        if self.control_mode == "Rolling":
            self.thrust_total, self.torque_roll, self.torque_pitch, self.torque_yaw = self.rolling_controller.step(self.pos_target)
        else:
            self.thrust_total, self.torque_roll, self.torque_pitch, self.torque_yaw = self.flying_controller.step(self.pos_target)
        
        self.motor_cmd = self.motor_mixer.mix2(self.thrust_total, self.torque_roll, self.torque_pitch, self.torque_yaw)

        self.set_motor_cmd(self.motor_cmd)
