from re import X
import numpy as np
from simple_pid import PID



class CascadedController:
    def __init__(self, target_x=0.0, target_y=0.0, target_z=0.5, target_yaw=0.0, state_estimator=None):
        self.se = state_estimator
        self.target_x = target_x
        self.target_y = target_y
        self.target_z = target_z
        self.target_yaw = target_yaw

        # Altitude PID (Controls Thrust)
        self.pid_z = PID(40.0, 1.5, 8.0, setpoint=target_z, output_limits=(-50.0, 50.0), sample_time=0.002)
        
        # Position PIDs (Output: Desired Angles)
        # Note: Limits clamp the max lean angle (e.g., 0.5 rad ~ 28 degrees)
        self.pid_x = PID(5.0, 1.5, 1.5, setpoint=target_x, output_limits=(-0.5, 0.5), sample_time=0.002)
        self.pid_y = PID(5.0, 1.5, 1.5, setpoint=target_y, output_limits=(-0.5, 0.5), sample_time=0.002)
        
        # Attitude PIDs (Output: Motor Mixing Moments)
        self.pid_roll = PID(15.0, 2.0, 0.5, setpoint=0.0, output_limits=(-30.0, 30.0), sample_time=0.002)
        self.pid_pitch = PID(15.0, 2.0, 0.5, setpoint=0.0, output_limits=(-30.0, 30.0), sample_time=0.002)
        self.pid_yaw = PID(2.0, 0.5, 0.1, setpoint=target_yaw, output_limits=(-3.0, 3.0), sample_time=0.002)

    


    def adjust_yaw(self, delta_rad):
        """Adjust target yaw by delta_rad and update PID setpoint."""
        self.target_yaw += delta_rad
        self.pid_yaw.setpoint = self.target_yaw

    def adjust_target_x(self, delta_x):
        """Adjust target x by delta_x in heading frame and update PID setpoint."""
        if self.se is None:
            raise ValueError("State estimator not provided.")

        yaw = self.se.yaw
        delta_x_global = delta_x * np.cos(yaw)
        delta_y_global = delta_x * np.sin(yaw)

        self.target_x += delta_x_global
        self.target_y += delta_y_global
        self.pid_x.setpoint = self.target_x
        self.pid_y.setpoint = self.target_y

    def adjust_target_y(self, delta_y):
        """Adjust target y by delta_y in heading frame and update PID setpoint."""
        if self.se is None:
            raise ValueError("State estimator not provided.")

        yaw = self.se.yaw
        delta_x_global = -delta_y * np.sin(yaw)
        delta_y_global = delta_y * np.cos(yaw)

        self.target_x += delta_x_global
        self.target_y += delta_y_global
        self.pid_x.setpoint = self.target_x
        self.pid_y.setpoint = self.target_y

    def adjust_target_z(self, delta_z):
        """Adjust target z by delta_z and update PID setpoint."""
        self.target_z += delta_z
        self.pid_z.setpoint = self.target_z

    def step(self):
        # Get current state from estimator
        if self.se is None:
            raise ValueError("State estimator not provided.")

        # --- B. OUTER LOOP (Position Control) ---
        # 1. Altitude Control
        # Gravity compensation (approx 10N for 1kg) + PID adjustment
        thrust_base = 9.81 * self.se.total_mass
        thrust_total = thrust_base + self.pid_z(self.se.z)

        # 2. XY Position Control -> Desired Lean Angles
        # To move +X (Forward), we need negative Pitch (Nose down)
        # To move +Y (Left), we need negative Roll (Left side down)
        # (Note: These signs depend on your specific frame, easy to flip if wrong)
        global_des_pitch = self.pid_x(self.se.x)
        global_des_roll  = -self.pid_y(self.se.y)

        # 3. COORDINATE TRANSFORMATION 
        # We rotate the global commands by the negative of the current yaw
        current_yaw = self.se.yaw
        
        # Body-frame setpoints
        # This aligns the "tilt" command with the way the drone is actually pointing
        body_des_roll  = global_des_roll * np.cos(current_yaw) + global_des_pitch * np.sin(current_yaw)
        body_des_pitch = -global_des_roll * np.sin(current_yaw) + global_des_pitch * np.cos(current_yaw)
            
        # --- C. INNER LOOP (Attitude Control) ---
        # torque_roll  = 5*(des_roll-self.se.roll) 
        # torque_pitch = 5*(des_pitch-self.se.pitch) 
        # torque_yaw   = 5*(self.se.yaw) 
        # Use desired angles as dynamic setpoints for the attitude PIDs
        self.pid_roll.setpoint = body_des_roll
        self.pid_pitch.setpoint = body_des_pitch


        torque_roll = self.pid_roll(self.se.roll)
        torque_pitch = self.pid_pitch(self.se.pitch)
        torque_yaw = self.pid_yaw(self.se.yaw)


        return thrust_total, torque_roll, torque_pitch, torque_yaw


        # # --- D. MOTOR MIXING ---
        # # Config: FR, FL, RR, RL
        # # Thrust is distributed to all.
        # # Pitch: Front +, Rear - (To pitch up)
        # # Roll: Left +, Right - (To roll right)
        # # Yaw: CCW +, CW -

        # # Divide thrust by 4 motors
        # t = thrust_total / 4.0
        
        # # Standard Quad X Mixing
        # # FR (Front Right) - CCW
        # m_fr = t - torque_roll - torque_pitch  - torque_yaw
        
        # # FL (Front Left) - CW
        # m_fl = t + torque_roll - torque_pitch  + torque_yaw
        
        # # RR (Rear Right) - CW
        # m_rr = t - torque_roll + torque_pitch  + torque_yaw
        
        # # RL (Rear Left) - CCW
        # m_rl = t + torque_roll + torque_pitch  - torque_yaw
        
        # return np.array([m_fr, m_fl, m_rr, m_rl])
