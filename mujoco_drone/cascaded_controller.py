from re import X
import numpy as np
from simple_pid import PID



class CascadedController:
    def __init__(self, user_input, state_estimator=None):
        self.user_input = user_input
        self.se = state_estimator
        
        # Initialize targets from current position if state estimator available, else use defaults
        if self.se is not None:
            self.target_x = self.se.x
            self.target_y = self.se.y
            self.target_z = self.se.z
            self.target_yaw = self.se.yaw
        else:
            self.target_x = 0.0
            self.target_y = 0.0
            self.target_z = 0.5
            self.target_yaw = 0.0

        # Altitude PID (Controls Thrust)
        self.pid_z = PID(40.0, 1.5, 8.0, setpoint=self.target_z, output_limits=(-50.0, 50.0), sample_time=0.002)
        
        # Position PIDs (Output: Desired Angles)
        # Note: Limits clamp the max lean angle (e.g., 0.5 rad ~ 28 degrees)
        self.pid_x = PID(5.0, 1.5, 1.5, setpoint=self.target_x, output_limits=(-0.5, 0.5), sample_time=0.002)
        self.pid_y = PID(5.0, 1.5, 1.5, setpoint=self.target_y, output_limits=(-0.5, 0.5), sample_time=0.002)
        
        # Attitude PIDs (Output: Motor Mixing Moments)
        self.pid_roll = PID(15.0, 2.0, 0.5, setpoint=0.0, output_limits=(-30.0, 30.0), sample_time=0.002)
        self.pid_pitch = PID(15.0, 2.0, 0.5, setpoint=0.0, output_limits=(-30.0, 30.0), sample_time=0.002)
        self.pid_yaw = PID(2.0, 0.5, 0.1, setpoint=self.target_yaw, output_limits=(-3.0, 3.0), sample_time=0.002)

    



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

    def adjust_yaw(self, delta_rad):
        """Adjust target yaw by delta_rad and update PID setpoint."""
        self.target_yaw += delta_rad
        self.pid_yaw.setpoint = self.target_yaw


    def update_targets_from_velocity(self, dt=0.002):
        """Update target position based on velocity commands from user input.
        
        Args:
            dt: Time step for integration (default 0.002s = 500Hz)
        """
        # Get velocity commands from user input
        vx = self.user_input.vx(lim=0.5)
        vy = self.user_input.vy(lim=0.5)
        vz = self.user_input.vz(lim=0.3)
        wz = self.user_input.wz(lim=np.deg2rad(30))
        
        # Integrate velocities to get position deltas
        dx = vx * dt
        dy = vy * dt
        dz = vz * dt
        dyaw = wz * dt
        
        # Update targets
        if dx != 0.0 or dy != 0.0:
            self.adjust_target_x(dx)
            self.adjust_target_y(dy)
        
        if dz != 0.0:
            self.target_z += dz
            self.pid_z.setpoint = self.target_z
        
        if dyaw != 0.0:
            self.adjust_yaw(dyaw)
            
        print(f"Updated Targets -> X: {self.target_x:.2f}, Y: {self.target_y:.2f}, Z: {self.target_z:.2f}, Yaw: {np.degrees(self.target_yaw):.1f} deg")

    def step(self):
        """Execute control step with velocity commands from user input."""
        # Update targets from user input velocity commands
        self.update_targets_from_velocity()

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


       