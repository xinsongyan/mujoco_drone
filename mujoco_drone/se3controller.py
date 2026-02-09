import numpy as np
import transformations as tf 
import time


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

def log(R):
    """Compute the logarithm of a rotation matrix."""
    theta = np.arccos((np.trace(R) - 1) / 2)
    if np.isclose(theta, 0):
        return np.zeros(3)
    return theta / (2 * np.sin(theta)) * vee(R - R.T)


class SE3Controller:
    def __init__(self, state_estimator=None, user_cmd=None):
        if user_cmd is not None:
            self.user_cmd = user_cmd
            
        self.se = state_estimator
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
  

    def tracking_control(self, 
                         pos_des=[0.0, 0.0, 0.5], 
                         vel_des = np.array([0, 0, 0]), 
                         acc_des = np.array([0, 0, 0]), 
                         heading_des=[1,0,0], 
                         omega_des_local = np.array([0, 0, 0]),
                         omega_dot_des_local = np.array([0, 0, 0])):


        # vel_des_wrt_body = np.array([self.user_cmd.pitch(), self.user_cmd.roll(), 0])  # Desired velocity based on user input
        # vel_des = self.se.R @ vel_des_wrt_body # Convert desired velocity from body frame to inertia frame
        
        # pos_des[0]= self.se.base_pos[0] + vel_des[0] * 0.02  # Update desired position based on user input
        # pos_des[1]= self.se.base_pos[1] + vel_des[1] * 0.02  # Update desired position based on user input
        # pos_des[0] = self.se.base_pos[0]
        # pos_des[1] = self.se.base_pos[1]
        
        # yaw_des =   self.se.yaw + np.array(self.user_cmd.yaw()) * 0.1  # Desired yaw based on user input
        # heading_des = np.array([np.cos(yaw_des), np.sin(yaw_des), 0])  # Desired heading in the XY plane
        # print(f"Desired Position: {pos_des}")
        # print(f"desired yaw: {yaw_des}, heading_des: {heading_des}")
        
        acc_cmd = self.k_pos * (pos_des - self.se.base_pos) + \
                  self.k_vel * (vel_des - self.se.base_vel_lin_global) + \
                  acc_des
        err_pos = pos_des - self.se.base_pos
        # print(f"Position Error: {err_pos}, Acc Command: {acc_cmd}")
          
        T_cmd  = self.m * acc_cmd - self.m * self.g  # Total force command
        # print(f"T_cmd: {T_cmd}", "self.m * acc_cmd :", self.m * acc_cmd , "-self.m * self.g:", -self.m * self.g)
        T_cmd_wrt_body = self.se.R.T @ T_cmd  # Transform force command to body frame
        
        # f =  np.dot(T_cmd, self.se.R[:, 2])  # Project the force onto the Z-axis of the drone's frame
        Tz_cmd_wrt_body = np.dot(T_cmd_wrt_body, [0, 0, 1])
        # print(f"Tz_cmd_wrt_body: {Tz_cmd_wrt_body}")
        
        zd = T_cmd / (np.linalg.norm(T_cmd) + 1e-6)  # Normalize to get the direction of thrust
        heading_des_unit = heading_des / np.linalg.norm(heading_des)  # Desired direction of thrust
        yd = np.cross(zd, heading_des_unit)  # Orthogonal vector to b3d and b1d
        yd = yd / (np.linalg.norm(yd) + 1e-6)
        xd = np.cross(yd, zd)
        Rd = np.column_stack((xd, yd, zd))  # Desired rotation matrix
        

        # err_rot = 1/2 * vee(Rd.T @ self.se.R - self.se.R.T @ Rd)
        # err_omega_wrt_body =  self.se.base_vel_ang_local - self.se.R.T @ Rd @ omega_des_local  


        # M_wrt_body = - self.k_rot * err_rot \
        #              - self.k_omega * err_omega_wrt_body \
        #              + hat(self.se.base_vel_ang_local) @ self.base_inertia_wrt_body @ self.se.base_vel_ang_local
        #              - self.base_inertia_wrt_body @ (hat(self.se.base_vel_ang_local) @ self.se.R.T @ Rd @ omega_des_local - self.se.R.T @ Rd @ omega_dot_des_local)
            
        

        # err_rot_wrt_body = 0.5 * vee(self.se.R.T @ Rd  - Rd.T @ self.se.R)
        err_rot_wrt_body = log(self.se.R.T @ Rd)  # Logarithm of the rotation matrix difference

        err_omega_wrt_body =  omega_des_local - self.se.base_vel_ang_local 

        M_wrt_body = self.k_rot * err_rot_wrt_body \
                     + self.k_omega * err_omega_wrt_body \
                     + hat(self.se.base_vel_ang_local) @ self.base_inertia_wrt_body @ self.se.base_vel_ang_local
                     
            
        return Tz_cmd_wrt_body, M_wrt_body

