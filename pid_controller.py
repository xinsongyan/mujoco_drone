from simple_pid import PID

import numpy as np


class PIDController:
    def __init__(self, z_des=0.5, rpy_setpoint=[0,0,0], state_estimator=None):
        self.state_estimator = state_estimator
        self.pid_alt = PID(6, 0.5, 1.25, z_des)
        self.pid_roll = PID(6, 0.5, 1.25, setpoint=rpy_setpoint[0], output_limits = (-1,1))
        self.pid_pitch = PID(6, 0.5, 1.25, setpoint=rpy_setpoint[1], output_limits = (-1,1))
        self.pid_yaw = PID(6, 0, 1.25, setpoint=rpy_setpoint[2], output_limits = (-3,3))


    def compute_control(self):
        # Compute control signals based on the current state
        thrust_total = (self.pid_alt(self.state_estimator.z) + 9.81) * self.state_estimator.total_mass/(np.cos(self.state_estimator.roll) * np.cos(self.state_estimator.pitch))
        torque_roll = self.pid_roll(self.state_estimator.roll)
        torque_pitch = self.pid_pitch(self.state_estimator.pitch)
        torque_yaw = self.pid_yaw(self.state_estimator.yaw)
        return thrust_total, torque_roll, torque_pitch, torque_yaw
