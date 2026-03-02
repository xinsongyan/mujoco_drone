try:
    from mujoco_drone.input.gamepad_input import GamepadInput
except ModuleNotFoundError:
    from gamepad_input import GamepadInput
import time

class UserInput:
    def __init__(self):
        """
        Initializes the UserInput class with gamepad input.
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

    def button_a(self):
        _, buttons = self.get_input()
        return bool(buttons[0]) if len(buttons) > 0 else False

    def button_b(self):
        _, buttons = self.get_input()
        return bool(buttons[1]) if len(buttons) > 1 else False




    def vx(self, lim=1.0, deadzone=0.05):
        axes, _ = self.get_input()
        val = -axes[1] * lim
        return 0 if abs(val) < deadzone else val

    def vy(self, lim=1.0, deadzone=0.05):
        axes, _ = self.get_input()
        val = -axes[0] * lim
        return 0 if abs(val) < deadzone else val

    def vz(self, lim=1.0):
        _, buttons = self.get_input()
        # RB (buttons[5]) up, LB (buttons[4]) down for typical XInput layout
        return - (1.0 if buttons[5] else 0.0) * lim + (1.0 if buttons[4] else 0.0) * lim

    
    def wx(self, lim=1.0):
        axes, _ = self.get_input()
        return axes[3] * lim  
    
    def wy(self, lim=1.0):
        axes, _ = self.get_input()
        return -axes[4] * lim  
    
    def wz(self, lim=1.0, deadzone=0.05):
        axes, _ = self.get_input()
        # RT (axes[5]) up, LT (axes[2]) down for typical XInput layout
        val = -(axes[5] - axes[2])/2 * lim
        return 0 if abs(val) < deadzone else val


if __name__ == "__main__":
    user_cmd = UserInput()
    try:
        while True:
            
            vx_val = user_cmd.vx()
            vy_val = user_cmd.vy()
            vz_val = user_cmd.vz()
            wz_val = user_cmd.wz()
            wx_val = user_cmd.wx()
            wy_val = user_cmd.wy()
            a_val = user_cmd.button_a()
            b_val = user_cmd.button_b()
            print(
                f"Vx: {vx_val:.2f}, Vy: {vy_val:.2f}, Vz: {vz_val:.2f}, "
                f"Wx: {wx_val:.2f}, Wy: {wy_val:.2f}, Wz: {wz_val:.2f}, "
                f"A: {int(a_val)}, B: {int(b_val)}"
            )
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("Stopping user input reader.")
        user_cmd.stop()

