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

def unit_vec(vec, reg=1e-6):
    """Normalize a vector to have unit length."""
    norm = np.linalg.norm(vec)
    if norm < reg:
        return vec
    return vec / norm

class SE3Controller:
    def __init__(self, user_input=None, state_estimator=None):
        self.user_input = user_input
        self.se = state_estimator
        # Initialize any necessary parameters for SE(3) control
        self.k_pos_xy = 30.0  # Position gain for x/y
        self.k_pos_z = 80.0   # Position gain for z
        self.k_vel_xy = 5.0   # Velocity gain for x/y
        self.k_vel_z = 10.0    # Velocity gain for z
        self.k_rot = 30.0   # Rotation gain
        self.k_omega = 1.0 # Angular velocity gain


        self.m = self.se.total_mass  # Mass of the drone in kg
        self.g = np.array([0, 0, -9.81])  # Gravitational acceleration in m/s^2

        self.base_inertia_wrt_body = np.array([[0.00226, 0, 0],
                                                [0, 0.00289, 0],
                                                [0, 0, 0.00508]])

    def step(self, pos_target, x_target=np.array([1.0, 0.0, 0.0]), z_target=1, vel_target=np.zeros(3), acc_target=np.zeros(3), omega_target=np.zeros(3)):
        # todo: add flight z orientation input and use it to determine the desired orientation during flying mode
        pos_target = np.array(pos_target, dtype=float)
        x_target = np.array(x_target, dtype=float)
        vel_target = np.array(vel_target, dtype=float)
        acc_target = np.array(acc_target, dtype=float)
        omega_target = np.array(omega_target, dtype=float)

        pos_err = pos_target - self.se.base_pos
        vel_err = vel_target - self.se.base_vel_lin_global
        acc_cmd = np.array(
            [
                self.k_pos_xy * pos_err[0] + self.k_vel_xy * vel_err[0] + acc_target[0],
                self.k_pos_xy * pos_err[1] + self.k_vel_xy * vel_err[1] + acc_target[1],
                self.k_pos_z * pos_err[2] + self.k_vel_z * vel_err[2] + acc_target[2],
            ],
            dtype=float,
        )  # Desired acceleration command in the inertia frame
 
        T_cmd  = self.m * acc_cmd - self.m * self.g  # Total force command
        # print(f"T_cmd: {T_cmd}", "self.m * acc_cmd :", self.m * acc_cmd , "-self.m * self.g:", -self.m * self.g)
        T_cmd_wrt_body = self.se.R.T @ T_cmd  # Transform force command to body frame
        
        # f =  np.dot(T_cmd, self.se.R[:, 2])  # Project the force onto the Z-axis of the drone's frame
        Tz_cmd_wrt_body = np.dot(T_cmd_wrt_body, [0, 0, 1])
        # print(f"Tz_cmd_wrt_body: {Tz_cmd_wrt_body}")
        
        zd = T_cmd / (np.linalg.norm(T_cmd) + 1e-6)  # Normalize to get the direction of thrust
        zd = zd * z_target # todo: check this line
        yd = np.cross(zd, unit_vec(x_target))  # Orthogonal vector to b3d and b1d
        xd = np.cross(unit_vec(yd), zd)
        Rd = np.column_stack((xd, yd, zd))  # Desired rotation matrix
        

        # err_rot = 1/2 * vee(Rd.T @ self.se.R - self.se.R.T @ Rd)
        # err_omega_wrt_body =  self.se.base_vel_ang_local - self.se.R.T @ Rd @ omega_des_local  


        # M_wrt_body = - self.k_rot * err_rot \
        #              - self.k_omega * err_omega_wrt_body \
        #              + hat(self.se.base_vel_ang_local) @ self.base_inertia_wrt_body @ self.se.base_vel_ang_local
        #              - self.base_inertia_wrt_body @ (hat(self.se.base_vel_ang_local) @ self.se.R.T @ Rd @ omega_des_local - self.se.R.T @ Rd @ omega_dot_des_local)
            
        

        # err_rot_wrt_body = 0.5 * vee(self.se.R.T @ Rd  - Rd.T @ self.se.R)
        err_rot_wrt_body = log(self.se.R.T @ Rd)  # Logarithm of the rotation matrix difference

        err_omega_wrt_body =  omega_target - self.se.base_vel_ang_local 

        M_wrt_body = self.k_rot * err_rot_wrt_body \
                     + self.k_omega * err_omega_wrt_body \
                     + hat(self.se.base_vel_ang_local) @ self.base_inertia_wrt_body @ self.se.base_vel_ang_local
                     
            
        return Tz_cmd_wrt_body, M_wrt_body[0], M_wrt_body[1], M_wrt_body[2]

