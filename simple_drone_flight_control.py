import os
import mujoco
import mujoco.viewer 
import numpy as np
import transformations as tf 
import time



from state_estimator import StateEstimator

from pid_controller import PIDController






class SimpleDrone: 
    def __init__(self):
        self.xml_path = os.path.join('assets', 'simple_drone', 'simple_drone.xml')
        print(f"xml_path: {self.xml_path} ")
 
        self.m = mujoco.MjModel.from_xml_path(self.xml_path)
        self.d = mujoco.MjData(self.m)

        # body_id = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_BODY, "x2")  # or model.body_name2id("name")
        # inertia_diag = self.m.body_inertia[body_id]
        # print(f"Body ID: {body_id}, Inertia Diagonal: {inertia_diag}")
        # input()
        
        # self.user_cmd = UserCommand()
        self.state_estimator = StateEstimator(self.m, self.d)

        self.stabilisation_controller = PIDController(z_des=0.5, rpy_setpoint=[0,0,0], state_estimator=self.state_estimator)


    def set_pos(self, pos):
        # Set the position of the drone's base
        self.d.qpos[:3] = pos
        
    def set_quat(self, quat):
        # Set the orientation of the drone's base
        self.d.qpos[3:7] = quat


    def __call__(self):
        # print(f"user_cmd: {self.user_cmd.get_input()}")
        # print("Base Position:", [f"{x:.3f}" for x in self.state_estimator.base_pos])
        # print("Base Quaternion:", [f"{x:.3f}" for x in self.state_estimator.base_quat])
        # print("Base RPY:", [f"{x:.3f}" for x in self.state_estimator.base_rpy])
        
        thrust_total, torque_roll, torque_pitch, torque_yaw = self.stabilisation_controller.compute_control()

        f_fr, f_fl, f_rr, f_rl = self.cal_motor_cmd(thrust_total, torque_roll, torque_pitch, torque_yaw)

        self.set_motor_cmd(f_fr, f_fl, f_rr, f_rl)


    def cal_motor_cmd(self, T, tau_x, tau_y, tau_z):
        dx, dy, k = 0.1, 0.1, 0.02 # distance from motor to x axis, y aixs, motor constant
        f_rr = (T - tau_x/dx + tau_y/dy + tau_z/k) / 4.0
        f_fr = (T - tau_x/dx - tau_y/dy - tau_z/k) / 4.0
        f_rl = (T + tau_x/dx + tau_y/dy - tau_z/k) / 4.0
        f_fl = (T + tau_x/dx - tau_y/dy + tau_z/k) / 4.0
        return f_fr, f_fl, f_rr, f_rl

    def set_motor_cmd(self, f_fr, f_fl, f_rr, f_rl):
        # Set the motor commands to the actuators
        self.d.ctrl[0] = np.clip(f_fr, 0, 10) # FR
        self.d.ctrl[1] = np.clip(f_fl, 0, 10) # FL
        self.d.ctrl[2] = np.clip(f_rr, 0, 10) # RR
        self.d.ctrl[3] = np.clip(f_rl, 0, 10) # RL





def main():
    
    drone = SimpleDrone()
    
    with mujoco.viewer.launch_passive(drone.m, drone.d) as viewer:


        while viewer.is_running():

            print(f"Time: {drone.d.time:.3f}s")
            
            drone()

            mujoco.mj_step(drone.m, drone.d)

            viewer.sync()

            time.sleep(drone.m.opt.timestep) 


if __name__ == "__main__":
    main()
