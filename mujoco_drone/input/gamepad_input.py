import pygame
import threading
import time
import os

class GamepadInput:
    def __init__(self, update_freq=50):
        # Set dummy video driver if no display available
        if 'SDL_VIDEODRIVER' not in os.environ:
            os.environ['SDL_VIDEODRIVER'] = 'dummy'
        
        pygame.init()
        pygame.joystick.init()

        if pygame.joystick.get_count() == 0:
            raise RuntimeError("No joystick detected.")
        
        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()
        print(f"Joystick name: {self.joystick.get_name()}")
        print("Note: F710 gamepad should be set to X mode (not D mode)")
        
        # Store states
        self.num_axes = self.joystick.get_numaxes()
        self.num_buttons = self.joystick.get_numbuttons()
        self.axis_values = [0.0] * self.num_axes
        self.button_states = [False] * self.num_buttons

        # Thread control
        self.update_freq = update_freq
        self._running = False
        self._thread = threading.Thread(target=self._update_loop, daemon=True)

                # Define labels (based on typical F710 layout in XInput mode)
        self.button_labels = [
            "A", "B", "X", "Y",  # 0-3
            "LB", "RB",          # 4-5
            "Back", "Start",     # 6-7
            "Left Stick", "Right Stick"  # 8-9
        ]

        self.axis_labels = [
            "Left Stick X", 
            "Left Stick Y", 
            "LT",
            "Right Stick X", 
            "Right Stick Y",
            "RT" 
        ]  # These may show up as axis or button depending on mode

    def start(self):
        self._running = True
        self._thread.start()

    def stop(self):
        self._running = False
        self._thread.join()

    def _update_loop(self):
        clock = pygame.time.Clock()
        while self._running:
            try:
                pygame.event.pump()
            except pygame.error as e:
                # Video system may not be available in thread, skip event pump
                pass
            try:
                for i in range(self.num_axes):
                    self.axis_values[i] = self.joystick.get_axis(i)
                for i in range(self.num_buttons):
                    self.button_states[i] = self.joystick.get_button(i)
            except pygame.error as e:
                # Joystick disconnected or system not ready, skip this update
                print(f"Gamepad error: {e}")
            except Exception as e:
                print(f"Unexpected error in gamepad update loop: {e}")
            clock.tick(self.update_freq)

    def get_axes(self):
        return self.axis_values.copy()

    def get_buttons(self):
        return self.button_states.copy()

    def print_state(self):
        print(f"============ {self.joystick.get_name()} (X mode) ============")
        print("=== Axis States ===")
        axis_states_str = ", ".join(
            f"{self.axis_labels[i] if i < len(self.axis_labels) else f'Axis {i}'}: {val:.3f}"
            for i, val in enumerate(self.axis_values)
        )
        print(axis_states_str)

        print("=== Button States ===")
        button_states_str = ", ".join(
            f"{self.button_labels[i] if i < len(self.button_labels) else f'Button {i}'}: {'P' if pressed else '-'}"
            for i, pressed in enumerate(self.button_states)
        )
        print(button_states_str)
        print()
        
if __name__ == "__main__":
    reader = GamepadInput(update_freq=20)
    reader.start()

    try:
        while True:
            reader.print_state()
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("Stopping...")
        reader.stop()
        pygame.quit()
