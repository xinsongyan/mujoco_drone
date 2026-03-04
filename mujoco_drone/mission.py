import numpy as np


class TeleoperatedMission:
    """Joystick-driven mission for selecting control mode.

    Mapping:
    - A button -> Flying
    - B button -> Rolling
    """

    def __init__(self):
        self._prev_a_pressed = False
        self._prev_b_pressed = False

    def select_mode(self, user_input, current_mode):
        a_pressed = user_input.button_a()
        b_pressed = user_input.button_b()

        next_mode = current_mode
        if a_pressed and not self._prev_a_pressed:
            next_mode = "Flying"
        elif b_pressed and not self._prev_b_pressed:
            next_mode = "Rolling"

        self._prev_a_pressed = a_pressed
        self._prev_b_pressed = b_pressed
        return next_mode


class PhaseCircleMission:
    """Phase-based circular mission for switching rolling/flying controllers.

    Phase mapping over one full circle:
    -   0° to  90°: Rolling
    -  90° to 180°: Flying
    - 180° to 270°: Rolling
    - 270° to 360°: Flying
    """

    def __init__(
        self,
        center,
        radius=0.2,
        omega=0.6,
        rolling_z=0.2,
        flying_z=0.3,
    ):
        self.center = np.array(center, dtype=float).copy()
        self.radius = float(radius)
        self.omega = float(omega)
        self.rolling_z = float(rolling_z)
        self.flying_z = float(flying_z)

    def sample_circle_xy(self, t):
        theta = self.omega * t + np.pi
        return np.array(
            [
                self.center[0] + self.radius * np.cos(theta),
                self.center[1] + self.radius * np.sin(theta),
            ],
            dtype=float,
        )

    def mode_from_time(self, t):
        phase_deg = np.degrees((self.omega * t) % (2 * np.pi))
        if phase_deg < 90.0 or (180.0 <= phase_deg < 270.0):
            return "Rolling", phase_deg
        return "Flying", phase_deg

    def target_and_mode(self, t):
        xy = self.sample_circle_xy(t)
        mode, phase_deg = self.mode_from_time(t)
        z = self.rolling_z if mode == "Rolling" else self.flying_z
        pos_target = np.array([xy[0], xy[1], z], dtype=float)
        return pos_target, mode, phase_deg
