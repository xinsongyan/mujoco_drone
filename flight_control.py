

from re import X
import numpy as np
from simple_pid import PID

class FlightControl:
    def __init__(self, target_x=0.0, target_y=0.0, target_z=0.02, target_yaw=0.0, state_estimator=None):
        self.target_x = target_x
        self.target_y = target_y
        self.target_z = target_z
        self.target_yaw = target_yaw
        self.se = state_estimator
        
        # Altitude PID (Controls Thrust)
        self.pid_z = PID(5.0, 0.5, 1.0, setpoint=target_z, sample_time=0.002)
        
        # Position PIDs (Output: Desired Angles)
        # Note: Limits clamp the max lean angle (e.g., 0.5 rad ~ 28 degrees)
        self.pid_x = PID(0.0, 0.0, 0.0, setpoint=target_x, sample_time=0.002)
        self.pid_y = PID(0.0, 0.0, 0.0, setpoint=target_y, sample_time=0.002)
        
        # Attitude PIDs (Output: Motor Mixing Moments)
        self.pid_roll = PID(5.0, 0.0, 1.0, setpoint=0.0, sample_time=0.002)
        self.pid_pitch = PID(5.0, 0.0, 1.0, setpoint=0.0, sample_time=0.002)
        self.pid_yaw = PID(5.0, 0.0, 1.0, setpoint=target_yaw, sample_time=0.002)

    def update(self, current_time):
        # Get current state from estimator
        if self.se is None:
            raise ValueError("State estimator not provided.")
        x, y, z = self.se.base_pos
        roll, pitch, yaw = self.se.base_rpy

        # --- B. OUTER LOOP (Position Control) ---
        # 1. Altitude Control
        # Gravity compensation (approx 10N for 1kg) + PID adjustment
        thrust_base = 9.81 
        thrust_adj = self.pid_z(z)
        total_thrust = thrust_base + thrust_adj

        # 2. XY Position Control -> Desired Lean Angles
        # To move +X (Forward), we need negative Pitch (Nose down)
        # To move +Y (Left), we need positive Roll (Right side up)
        # (Note: These signs depend on your specific frame, easy to flip if wrong)
        des_pitch = -1.0 * self.pid_x(x)
        des_roll  =  1.0 * self.pid_y(y)
        
        # --- C. INNER LOOP (Attitude Control) ---
        roll_cmd  = self.pid_roll(des_roll - roll)
        pitch_cmd = self.pid_pitch(des_pitch - pitch)
        yaw_cmd   = self.pid_yaw(yaw)


        # --- D. MOTOR MIXING ---
        # Config: FR, FL, RR, RL
        # Thrust is distributed to all.
        # Pitch: Front +, Rear - (To pitch up)
        # Roll: Left +, Right - (To roll right)
        # Yaw: CCW +, CW -

        # Divide thrust by 4 motors
        t = total_thrust / 4.0
        
        # Standard Quad X Mixing
        # FR (Front Right) - CCW
        m_fr = t - roll_cmd - pitch_cmd  - yaw_cmd
        
        # FL (Front Left) - CW
        m_fl = t + roll_cmd - pitch_cmd  + yaw_cmd
        
        # RR (Rear Right) - CW
        m_rr = t - roll_cmd + pitch_cmd  + yaw_cmd
        
        # RL (Rear Left) - CCW
        m_rl = t + roll_cmd + pitch_cmd  - yaw_cmd
        
        return np.array([m_fr, m_fl, m_rr, m_rl])
