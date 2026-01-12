
from gamepad_reader import GamepadReader
from keyboard_reader import KeyboardReader
import time

class UserCommand:
    def __init__(self):
        self.gamepad_reader = GamepadReader()
        self.gamepad_reader.start()
    def __init__(self, input_device='gamepad'):
        """
        Initializes the UserCommand class.

        Args:
            input_device (str): The input device to use, 'gamepad' or 'keyboard'.
                                If 'gamepad' is chosen and none is found, it will fall back to 'keyboard'.
        """
        if input_device == 'gamepad':
            try:
                self.reader = GamepadReader()
                print("Using Gamepad for user input.")
            except RuntimeError as e:
                print(f"Warning: {e}. Falling back to keyboard for user input.")
                self.reader = KeyboardReader()
        elif input_device == 'keyboard':
            self.reader = KeyboardReader()
            print("Using Keyboard for user input.")
        else:
            raise ValueError("Invalid input_device. Choose 'gamepad' or 'keyboard'.")

        self.reader.start()

    def get_input(self):
        axes = self.gamepad_reader.get_axes()
        buttons = self.gamepad_reader.get_buttons()
        axes = self.reader.get_axes()
        buttons = self.reader.get_buttons()
        return axes, buttons

    def stop(self):
        self.gamepad_reader.stop()
        self.reader.stop()

    def update_display(self):
        """
        Updates the display for the input device, if any.
        Currently only used for the keyboard reader.
        """
        if hasattr(self.reader, 'update_display'):
            self.reader.update_display()


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
    user_cmd = UserCommand(input_device='keyboard') # Or 'gamepad'
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

