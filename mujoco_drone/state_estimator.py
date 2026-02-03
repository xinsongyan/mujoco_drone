
import transformations as tf
import mujoco
import numpy as np

class StateEstimator:
    def __init__(self, m, d):
        self.m = m  # Store the model object for later access
        self.d = d  # Store the data object for later access

    @property
    def total_mass(self):
        return np.sum(self.m.body_mass)
    
    @property
    def base_id(self, base_name="chassis"):
        id_ = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_BODY, base_name)
        if id_ == -1:
            raise ValueError(f"Body name '{base_name}' not found in the model.")
        return id_

    @property
    def base_pos(self):
        return self.d.qpos[:3]

    @property
    def x(self):
        return self.d.qpos[0]
    
    @property
    def y(self):
        return self.d.qpos[1]
    
    @property
    def z(self):
        return self.d.qpos[2]
    

    @property
    def base_quat(self):
        return self.d.qpos[3:7]

    @property
    def base_rpy(self):
        return tf.euler_from_quaternion(self.base_quat)
    
    @property
    def roll(self):
        return self.base_rpy[0]
    
    @property
    def pitch(self):
        return self.base_rpy[1]
    
    @property
    def yaw(self):
        return self.base_rpy[2]
    
    @property
    def rotation_matrix(self):
        return tf.quaternion_matrix(self.base_quat)[:3, :3]
    
    @property
    def R(self):
        return self.rotation_matrix
    

    @property
    def base_vel_lin_global(self):
        return self.d.qvel[:3]

    @property
    def base_vel_ang_local(self):
        return self.d.qvel[3:6]

    @property
    def v(self):
        return self.base_vel_lin_global
    
    @property
    def w(self):
        return self.base_vel_ang_local
    

    @property
    def base_vel_lin_local(self):
        return self.R.T @ self.base_vel_lin_global
    
    @property
    def base_vel_ang_global(self):
        return self.R @ self.base_vel_ang_local
    



