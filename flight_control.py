from re import X
import numpy as np
from simple_pid import PID



class FlightControl:
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
        self.pid_x = PID(2.0, 0.1, 1.5, setpoint=target_x, output_limits=(-0.4, 0.4), sample_time=0.002)
        self.pid_y = PID(2.0, 0.1, 1.5, setpoint=target_y, output_limits=(-0.4, 0.4), sample_time=0.002)
        
        # Attitude PIDs (Output: Motor Mixing Moments)
        self.pid_roll = PID(15.0, 1.0, 1.5, setpoint=0.0, output_limits=(-10.0, 10.0), sample_time=0.002)
        self.pid_pitch = PID(15.0, 1.0, 1.5, setpoint=0.0, output_limits=(-10.0, 10.0), sample_time=0.002)
        self.pid_yaw = PID(15.0, 0.5, 0.1, setpoint=target_yaw, output_limits=(-3.0, 3.0), sample_time=0.002)

    def compute_control(self):
        # Get current state from estimator
        if self.se is None:
            raise ValueError("State estimator not provided.")

        # --- B. OUTER LOOP (Position Control) ---
        # 1. Altitude Control
        # Gravity compensation (approx 10N for 1kg) + PID adjustment
        thrust_base = 9.81 * self.se.total_mass
        thrust_adj = self.pid_z(self.se.z)
        total_thrust = thrust_base + thrust_adj

        # 2. XY Position Control -> Desired Lean Angles
        # To move +X (Forward), we need negative Pitch (Nose down)
        # To move +Y (Left), we need negative Roll (Left side down)
        # (Note: These signs depend on your specific frame, easy to flip if wrong)
        des_pitch = self.pid_x(self.se.x)
        des_roll  = -self.pid_y(self.se.y)

        # --- C. INNER LOOP (Attitude Control) ---
        # roll_cmd  = 5*(des_roll-self.se.roll) 
        # pitch_cmd = 5*(des_pitch-self.se.pitch) 
        # yaw_cmd   = 5*(self.se.yaw) 
        # Use desired angles as dynamic setpoints for the attitude PIDs
        self.pid_roll.setpoint = des_roll
        self.pid_pitch.setpoint = des_pitch


        roll_cmd = self.pid_roll(self.se.roll)
        pitch_cmd = self.pid_pitch(self.se.pitch)
        yaw_cmd = self.pid_yaw(self.se.yaw)


        return total_thrust, roll_cmd, pitch_cmd, yaw_cmd


        # # --- D. MOTOR MIXING ---
        # # Config: FR, FL, RR, RL
        # # Thrust is distributed to all.
        # # Pitch: Front +, Rear - (To pitch up)
        # # Roll: Left +, Right - (To roll right)
        # # Yaw: CCW +, CW -

        # # Divide thrust by 4 motors
        # t = total_thrust / 4.0
        
        # # Standard Quad X Mixing
        # # FR (Front Right) - CCW
        # m_fr = t - roll_cmd - pitch_cmd  - yaw_cmd
        
        # # FL (Front Left) - CW
        # m_fl = t + roll_cmd - pitch_cmd  + yaw_cmd
        
        # # RR (Rear Right) - CW
        # m_rr = t - roll_cmd + pitch_cmd  + yaw_cmd
        
        # # RL (Rear Left) - CCW
        # m_rl = t + roll_cmd + pitch_cmd  - yaw_cmd
        
        # return np.array([m_fr, m_fl, m_rr, m_rl])
