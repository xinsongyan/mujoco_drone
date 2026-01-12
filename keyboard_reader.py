import pygame
import threading


class KeyboardReader:
    def __init__(self, update_freq=50, sensitivity=0.1):
        """
        Initializes the KeyboardReader.
        Args:
            update_freq (int): The frequency in Hz to update the key states.
            sensitivity (float): A value between 0 and 1. Higher values mean faster response (less smoothing).
                                 A value of 1.0 means instant response (no smoothing).
        """
        pygame.init()
        if not pygame.display.get_init():
            pygame.display.init()
        if pygame.display.get_surface() is None:
            self.screen = pygame.display.set_mode((400, 300))

        self.update_freq = update_freq
        self.sensitivity = max(0.0, min(1.0, sensitivity))  # Clamp between 0 and 1
        self._running = False
        self._thread = threading.Thread(target=self._update_loop, daemon=True)

        # Align shape with GamepadReader: 6 axes, 10 buttons
        self.axis_values = [0.0] * 6
        self.button_states = [False] * 10
        self.font = pygame.font.Font(None, 24)


    def start(self):
        self._running = True
        self._thread.start()

    def stop(self):
        self._running = False
        self._thread.join()

    def _update_loop(self):
        clock = pygame.time.Clock()
        while self._running:
            keys = pygame.key.get_pressed()

            # Determine target values for axes based on key presses (-1, 0, or +1)
            # Left Stick X: A/Left = -1, D/Right = +1
            target_left_x = (-1.0 if keys[pygame.K_a] or keys[pygame.K_LEFT] else 0.0) + (
                1.0 if keys[pygame.K_d] or keys[pygame.K_RIGHT] else 0.0
            )
            # Left Stick Y: W/Up = -1, S/Down = +1
            target_left_y = (-1.0 if keys[pygame.K_w] or keys[pygame.K_UP] else 0.0) + (
                1.0 if keys[pygame.K_s] or keys[pygame.K_DOWN] else 0.0
            )
            # Triggers LT/RT: Q/E as 1.0 when held
            target_lt = 1.0 if keys[pygame.K_q] else 0.0
            target_rt = 1.0 if keys[pygame.K_e] else 0.0
            # Right Stick X/Y: J/L and I/K
            target_right_x = (-1.0 if keys[pygame.K_j] else 0.0) + (1.0 if keys[pygame.K_l] else 0.0)
            target_right_y = (-1.0 if keys[pygame.K_i] else 0.0) + (1.0 if keys[pygame.K_k] else 0.0)

            # Smoothly update axis values towards their targets
            targets = [target_left_x, target_left_y, target_lt, target_right_x, target_right_y, target_rt]
            for i in range(len(self.axis_values)):
                current_val = self.axis_values[i]
                target_val = targets[i]
                self.axis_values[i] += (target_val - current_val) * self.sensitivity

            # Map a few buttons: Space=A (0), Left Ctrl=B (1), Left Alt=X (2), Left Shift=Y (3)
            self.button_states[0] = bool(keys[pygame.K_SPACE])
            self.button_states[1] = bool(keys[pygame.K_LCTRL])
            self.button_states[2] = bool(keys[pygame.K_LALT])
            self.button_states[3] = bool(keys[pygame.K_LSHIFT])

            clock.tick(self.update_freq)

    def update_display(self):
        """Call this from your main loop to update the pygame window."""
        if self.screen:
            # Process Pygame events in the main thread
            for _ in pygame.event.get():
                pass

            self.screen.fill((0, 0, 0))  # Clear screen

            # Define labels for clarity
            axis_labels = ["Yaw", "Throttle", "LT", "Roll", "Pitch", "RT"]
            button_labels = ["A (Space)", "B (LCtrl)", "X (LAlt)", "Y (LShift)"] + [f"Btn {i}" for i in range(4, 10)]

            for i, val in enumerate(self.axis_values):
                label = axis_labels[i] if i < len(axis_labels) else f"Axis {i}"
                text = self.font.render(f"{label}: {val:.2f}", True, (255, 255, 255))
                self.screen.blit(text, (10, 10 + i * 20))

            for i, state in enumerate(self.button_states):
                label = button_labels[i] if i < len(button_labels) else f"Button {i}"
                text = self.font.render(f"{label}: {'Pressed' if state else 'Released'}", True, (255, 255, 255))
                self.screen.blit(text, (200, 10 + i * 20))

            pygame.display.flip()

    def get_axes(self):
        return self.axis_values.copy()

    def get_buttons(self):
        return self.button_states.copy()

if __name__ == "__main__":
    reader = KeyboardReader(update_freq=20, sensitivity=0.2)
    reader.start()

    try:
        while True:
            axes = reader.get_axes()
            buttons = reader.get_buttons()
            print(f"Axes: {[f'{a:.2f}' for a in axes]}, Buttons: {buttons}")
            reader.update_display()
            
            pygame.time.wait(50) # Simulate some work
            
    except KeyboardInterrupt:
        print("Stopping...")
        reader.stop()
        pygame.quit()
