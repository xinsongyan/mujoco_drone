import numpy as np

class MotorMixer:
    """
    Translates desired forces and torques into individual motor commands
    based on the drone's physical configuration.
    """
    def __init__(self, dx, dy, drag_to_thrust_ratio):
        """
        Initializes the motor mixer with the drone's physical parameters.

        Args:
            dx (float): Distance from motors to the x-axis.
            dy (float): Distance from motors to the y-axis.
            drag_to_thrust_ratio (float): Ratio of drag to thrust for each motor.
        """
        self.dx = dx
        self.dy = dy
        self.k = drag_to_thrust_ratio
        
        self.allocation_matrix = np.array([
            [1, 1, 1, 1],  # Total thrust
            [-dx, -dx, dx, dx],  # Roll torque
            [dy, -dy, dy, -dy],  # Pitch torque
            [self.k, -self.k, -self.k, self.k]  # Yaw torque
        ])

    def mix(self, thrust_total, torque_x, torque_y, torque_z):
        """
        Calculates individual motor commands for a quadcopter in 'X' configuration.

        Args:
            thrust_total (float): The total desired thrust (T).
            torque_x (float): The desired roll torque (tau_x).
            torque_y (float): The desired pitch torque (tau_y).
            torque_z (float): The desired yaw torque (tau_z).

        Returns:
            np.array: An array of 4 motor commands.
        """
        # Equations for a quad-X configuration
        # The signs depend on motor placement and propeller direction.
        # This assumes:
        # motor 1 (f_rr): rear-right
        # motor 2 (f_fr): front-right
        # motor 3 (f_rl): rear-left
        # motor 4 (f_fl): front-left
        f_rr = (thrust_total - torque_x/self.dx + torque_y/self.dy + torque_z/self.k) / 4.0
        f_fr = (thrust_total - torque_x/self.dx - torque_y/self.dy - torque_z/self.k) / 4.0
        f_rl = (thrust_total + torque_x/self.dx + torque_y/self.dy - torque_z/self.k) / 4.0
        f_fl = (thrust_total + torque_x/self.dx - torque_y/self.dy + torque_z/self.k) / 4.0
        
        motor_commands = np.array([f_rr, f_fr, f_rl, f_fl])
        
        # It's good practice to clip the commands to a valid range (e.g., 0 to 1, or min/max PWM)
        # We assume the simulator handles this for now.
        
        return motor_commands
    
    def mix2(self, thrust_total, torque_roll, torque_pitch, torque_yaw):
        # Alternative mixing using the allocation matrix
        desired = np.array([thrust_total, torque_roll, torque_pitch, torque_yaw])
        motor_commands = np.linalg.solve(self.allocation_matrix, desired)
        return motor_commands
