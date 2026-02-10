from datetime import tzinfo
import numpy as np
import transformations as tf 
import time

cage_radius = 0.2  # Radius of the cage in meters


def hat(omega):
    """Convert a 3D vector to a 3x3 skew-symmetric matrix."""
    return np.array([
        [0, -omega[2], omega[1]],
        [omega[2], 0, -omega[0]],
        [-omega[1], omega[0], 0]
    ])

def vee(omega_hat):
    """Convert a 3x3 skew-symmetric matrix to a 3D vector."""
    return np.array([
        omega_hat[2,1],
        omega_hat[0,2],
        omega_hat[1,0]
    ])


def vel_base_to_omega_base(vel_base=np.array([0, 0, 0])):
    pos_contact_to_base = np.array([0, 0, cage_radius])  # Vector from contact point to drone's center of mass in body frame
    omega_base = np.cross(pos_contact_to_base, vel_base) / np.linalg.norm(pos_contact_to_base)**2
    return omega_base

#todo, use the joy stick to give input to the controller
class RollingController:
    def __init__(self, state_estimator=None, user_cmd=None):
        if user_cmd is not None:
            self.user_cmd = user_cmd
            
        self.se = state_estimator
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_z = cage_radius
        self.target_yaw = 0.0
        
        # todo, tune the gains
        # Initialize any necessary parameters for SE(3) control
        self.k_pos = 30.0  # Position gain
        self.k_vel = 5.0   # Velocity gain
        self.k_rot = 30.0   # Rotation gain
        self.k_omega = 1.0 # Angular velocity gain


        self.m = self.se.total_mass  # Mass of the drone in kg
        self.g = np.array([0, 0, -9.81])  # Gravitational acceleration in m/s^2

        self.base_inertia_wrt_body = np.array([[0.00226, 0, 0],
                                                [0, 0.00289, 0],
                                                [0, 0, 0.00508]])
        
    def adjust_target_x(self, delta):
        self.target_x += delta

    def adjust_target_y(self, delta):
        self.target_y += delta

    def adjust_target_z(self, delta):
        self.target_z += delta

    def adjust_yaw(self, delta):
        self.target_yaw += delta

    def tracking_control(self, pos_des=None):
        if pos_des is None:
            pos_des = np.array([self.target_x, self.target_y, self.target_z], dtype=float)
        
        
        
        pos_des[2] = cage_radius  # Ensure desired position is on the cage surface
        vel_des = 1 * (pos_des - self.se.base_pos)  # Desired velocity based on position error
        # linear part of control
        acc_cmd = self.k_pos * (pos_des - self.se.base_pos) + \
                  self.k_vel * (vel_des - self.se.base_vel_lin_global)
        T_cmd = self.m * (acc_cmd)  # Total force command
        T_cmd_wrt_body = self.se.R.T @ T_cmd  # Transform force command
        Tz_cmd_wrt_body = np.dot(T_cmd_wrt_body, [0, 0, 1])  # Thrust command in body frame
        
        
        omega_des = vel_base_to_omega_base(vel_des)
        omega_des_wrt_body = self.se.R.T @ omega_des  # Transform desired angular velocity to body frame
        
        torque_wrt_body = self.k_omega * (omega_des_wrt_body - self.se.base_vel_ang_local) + hat(self.se.base_vel_ang_local) @ self.base_inertia_wrt_body @ self.se.base_vel_ang_local
                     
        
        return Tz_cmd_wrt_body, torque_wrt_body