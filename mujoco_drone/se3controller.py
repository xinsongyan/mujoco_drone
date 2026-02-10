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
    def __init__(self, user_input=None, state_estimator=None):
        self.user_input = user_input
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
        
        # Initialize targets from drone's current state
        self.pos_target = self.se.base_pos.copy()
        self.yaw_target = self.se.yaw
        self.heading_target = np.array([np.cos(self.yaw_target), np.sin(self.yaw_target), 0])
        self.vel_target = np.zeros(3)
        self.acc_target = np.zeros(3)
        self.omega_target = np.zeros(3)


    def udpate_targets_from_user_input(self, dt=0.002):
        # Update target position based on user input velocity commands
        vel_des_wrt_body = np.array([self.user_input.vx(), self.user_input.vy(), self.user_input.vz()])  # Desired velocity based on user input
        vel_des = self.se.R @ vel_des_wrt_body # Convert desired velocity from body frame to inertia frame
        
        # Only integrate velocity into target when there's actual user input (non-zero)
        if np.linalg.norm(vel_des) > 1e-6:
            self.pos_target[0] += vel_des[0] * dt  # Integrate desired position based on user input
            self.pos_target[1] += vel_des[1] * dt  # Integrate desired position based on user input
            # self.pos_target[2] += vel_des[2] * dt
        
        self.pos_target[2] += self.user_input.vz() * dt # Set target z based on user input velocity in z direction
        
        # Update yaw target only when there's yaw input
        wz = self.user_input.wz()
        if abs(wz) > 1e-6:
            self.yaw_target += wz * dt  # Integrate yaw based on user input
            self.heading_target = np.array([np.cos(self.yaw_target), np.sin(self.yaw_target), 0])
        
        # print(f"yaw_des: {np.degrees(yaw_des)}, heading_target: {self.heading_target}")

    def step(self):
        print(f"pos_target: {self.pos_target}, yaw_target: {np.degrees(self.yaw_target)}"   )

        self.udpate_targets_from_user_input()  # Update targets based on user input
        
        acc_cmd = self.k_pos * (self.pos_target - self.se.base_pos) + \
                  self.k_vel * (self.vel_target - self.se.base_vel_lin_global) + \
                  self.acc_target  # Desired acceleration command in the inertia frame
 
        T_cmd  = self.m * acc_cmd - self.m * self.g  # Total force command
        # print(f"T_cmd: {T_cmd}", "self.m * acc_cmd :", self.m * acc_cmd , "-self.m * self.g:", -self.m * self.g)
        T_cmd_wrt_body = self.se.R.T @ T_cmd  # Transform force command to body frame
        
        # f =  np.dot(T_cmd, self.se.R[:, 2])  # Project the force onto the Z-axis of the drone's frame
        Tz_cmd_wrt_body = np.dot(T_cmd_wrt_body, [0, 0, 1])
        # print(f"Tz_cmd_wrt_body: {Tz_cmd_wrt_body}")
        
        zd = T_cmd / (np.linalg.norm(T_cmd) + 1e-6)  # Normalize to get the direction of thrust
        heading_target_unit = self.heading_target / np.linalg.norm(self.heading_target)  # Desired direction of thrust
        yd = np.cross(zd, heading_target_unit)  # Orthogonal vector to b3d and b1d
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

        err_omega_wrt_body =  self.omega_target - self.se.base_vel_ang_local 

        M_wrt_body = self.k_rot * err_rot_wrt_body \
                     + self.k_omega * err_omega_wrt_body \
                     + hat(self.se.base_vel_ang_local) @ self.base_inertia_wrt_body @ self.se.base_vel_ang_local
                     
            
        return Tz_cmd_wrt_body, M_wrt_body[0], M_wrt_body[1], M_wrt_body[2]

