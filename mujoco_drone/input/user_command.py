
from mujoco_drone.input.gamepad_input import GamepadInput
import time

class UserCommand:
    def __init__(self):
        """
        Initializes the UserCommand class with gamepad input.
        """
        self.reader = GamepadInput()
        print("Using Gamepad for user input.")
        self.reader.start()

    def get_input(self):
        axes = self.reader.get_axes()
        buttons = self.reader.get_buttons()
        return axes, buttons

    def stop(self):
        self.reader.stop()


    def throttle(self, lim=1.0, deadzone=0.05):
        axes, _ = self.get_input()
        val = -axes[1] * lim
        return 0 if abs(val) < deadzone else val

    def yaw(self, lim=1.0):
        axes, _ = self.get_input()
        return -axes[0] * lim
    
    
    def roll(self, lim=1.0):
        axes, _ = self.get_input()
        return -axes[3] * lim  
    
    def pitch(self, lim=1.0):
        axes, _ = self.get_input()
        return -axes[4] * lim  
    
if __name__ == "__main__":
    user_cmd = UserCommand()
    try:
        while True:
            throttle_val = user_cmd.throttle()
            yaw_val = user_cmd.yaw()
            roll_val = user_cmd.roll()
            pitch_val = user_cmd.pitch()
            print(f"Throttle: {throttle_val:.2f}, Yaw: {yaw_val:.2f}, Roll: {roll_val:.2f}, Pitch: {pitch_val:.2f}")
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("Stopping user command reader.")
        user_cmd.stop()

