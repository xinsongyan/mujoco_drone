from simple_pid import PID

import numpy as np


class PIDController:
    def __init__(self, z_des=0.5, rpy_setpoint=[0,0,0], state_estimator=None):
        self.se = state_estimator
        self.pid_alt = PID(6, 0.5, 1.25, z_des)
        self.pid_roll = PID(6, 0.5, 1.25, setpoint=rpy_setpoint[0], output_limits = (-1,1))
        self.pid_pitch = PID(6, 0.5, 1.25, setpoint=rpy_setpoint[1], output_limits = (-1,1))
        self.pid_yaw = PID(6, 0, 1.25, setpoint=rpy_setpoint[2], output_limits = (-3,3))


    def compute_control(self):
        # Compute control signals based on the current state
        thrust_total = (self.pid_alt(self.se.z) + 9.81) * self.se.total_mass/(np.cos(self.se.roll) * np.cos(self.se.pitch))
        # thrust_total = (self.pid_alt(self.se.z) + 9.81)* self.se.total_mass
        torque_roll = self.pid_roll(self.se.roll)
        torque_pitch = self.pid_pitch(self.se.pitch)
        torque_yaw = self.pid_yaw(self.se.yaw)
        return thrust_total, torque_roll, torque_pitch, torque_yaw
