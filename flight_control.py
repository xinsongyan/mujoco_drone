from re import X
import numpy as np
# from simple_pid import PID

class PID:
    def __init__(self, kp, ki, kd, out_min=-np.inf, out_max=np.inf, i_max=None):
        """
        :param kp: Proportional gain
        :param ki: Integral gain
        :param kd: Derivative gain
        :param out_min: Minimum output value (saturation)
        :param out_max: Maximum output value (saturation)
        :param i_max: Maximum absolute value for the Integral term (anti-windup)
                      If None, defaults to same range as out_max.
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        
        self.out_min = out_min
        self.out_max = out_max
        
        # Default i_max to output limits if not specified
        self.i_max = i_max if i_max is not None else out_max
        
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_measurement = 0.0  # Used for Derivative-on-Measurement

    def update(self, setpoint, measurement, dt):
        """
        Calculates PID output.
        
        :param setpoint: The desired target value
        :param measurement: The current actual value
        :param dt: Time delta in seconds
        """
        # 1. Error Calculation
        error = setpoint - measurement
        
        # 2. Proportional Term
        p_term = self.kp * error
        
        # 3. Integral Term (with Anti-Windup)
        self.integral += error * dt
        # Clamp the integral term independently
        self.integral = max(min(self.integral, self.i_max), -self.i_max)
        i_term = self.ki * self.integral
        
        # 4. Derivative Term (Derivative on Measurement)
        # We use d(measurement)/dt instead of d(error)/dt to avoid spikes 
        # when the setpoint changes abruptly.
        # Note the negative sign: if measurement increases, error decreases.
        derivative = (measurement - self.prev_measurement) / dt
        d_term = -self.kd * derivative
        
        # 5. Total Output
        output = p_term + i_term + d_term 
        
        # 6. Output Saturation
        output = max(min(output, self.out_max), self.out_min)
        
        # 7. Update State
        self.prev_error = error
        self.prev_measurement = measurement
        
        return output

    def reset(self):
        """Reset the internal state (integral and previous error)."""
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_measurement = 0.0
        

class FlightControl:
    def __init__(self, target_x=0.0, target_y=0.0, target_z=0.5, target_yaw=0.0, state_estimator=None):
        self.se = state_estimator
        self.target_x = target_x
        self.target_y = target_y
        self.target_z = target_z
        self.target_yaw = target_yaw

        # Altitude PID (Controls Thrust)
        self.pid_z = PID(50.0, 2.0, 5.0, out_min=-20.0, out_max=20.0)
        
        # Position PIDs (Output: Desired Angles)
        # Note: Limits clamp the max lean angle (e.g., 0.5 rad ~ 28 degrees)
        self.pid_x = PID(0, 0, 0.1, out_min=-0.1, out_max=0.1)
        self.pid_y = PID(0, 0, 0.1, out_min=-0.1, out_max=0.1)
        
        # Attitude PIDs (Output: Motor Mixing Moments)
        self.pid_roll = PID(1.0, 0.0, 0.1)
        self.pid_pitch = PID(1.0, 0.0, 0.1)
        self.pid_yaw = PID(1.0, 0.0, 0.1)

    def compute_control(self):
        # Get current state from estimator
        if self.se is None:
            raise ValueError("State estimator not provided.")

        # --- B. OUTER LOOP (Position Control) ---
        # 1. Altitude Control
        # Gravity compensation (approx 10N for 1kg) + PID adjustment
        thrust_base = 9.81 * self.se.total_mass
        thrust_adj = self.pid_z.update(self.target_z, self.se.z, dt=0.002)
        total_thrust = thrust_base + thrust_adj

        # 2. XY Position Control -> Desired Lean Angles
        # To move +X (Forward), we need negative Pitch (Nose down)
        # To move +Y (Left), we need negative Roll (Left side down)
        # (Note: These signs depend on your specific frame, easy to flip if wrong)
        des_pitch = self.pid_x.update(self.target_x, self.se.x, dt=0.002)
        des_roll  = -self.pid_y.update(self.target_y, self.se.y, dt=0.002)

        # --- C. INNER LOOP (Attitude Control) ---
        # roll_cmd  = 5*(des_roll-self.se.roll) 
        # pitch_cmd = 5*(des_pitch-self.se.pitch) 
        # yaw_cmd   = 5*(self.se.yaw) 
        roll_cmd = self.pid_roll.update(des_roll, self.se.roll, dt=0.002)
        pitch_cmd = self.pid_pitch.update(des_pitch, self.se.pitch, dt=0.002)
        yaw_cmd = self.pid_yaw.update(self.target_yaw, self.se.yaw, dt=0.002)

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
