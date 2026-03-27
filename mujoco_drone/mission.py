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




class PhasedStraightLineMission:
    """Three-phase straight-line mission along +x.

    Phases:
    1) Rolling for 0.6 m
    2) Flying for 0.6 m at flying_z
    3) Rolling for 0.6 m
    """

    def __init__(
        self,
        start,
        segment_length=0.6,
        line_speed=0.08,
        rolling_z=0.2,
        flying_z=0.35,
        camera_azimuth=90.0,
        camera_elevation=-10.0,
        camera_distance=3.0,
        camera_lookat=(0.9, 0.0, 0.0),
    ):
        self.start = np.array(start, dtype=float).copy()
        self.segment_length = float(segment_length)
        if self.segment_length <= 0.0:
            raise ValueError("segment_length must be > 0")

        self.line_speed = float(line_speed)
        if self.line_speed <= 0.0:
            raise ValueError("line_speed must be > 0")

        self.rolling_z = float(rolling_z)
        self.flying_z = float(flying_z)
        self.camera_azimuth = float(camera_azimuth)
        self.camera_elevation = float(camera_elevation)
        self.camera_distance = float(camera_distance)
        self.camera_lookat = np.array(camera_lookat, dtype=float).copy()
        if self.camera_lookat.shape != (3,):
            raise ValueError("camera_lookat must be shape (3,)")

        self.total_length = 3.0 * self.segment_length
        self.mission_duration = self.total_length / self.line_speed

    def target_and_mode(self, t):
        s = min(max(self.line_speed * float(t), 0.0), self.total_length)
        x = self.start[0] + s
        y = self.start[1]

        if s < self.segment_length:
            mode = "Rolling"
            phase = 1
            z = self.rolling_z
        elif s < 2.0 * self.segment_length:
            mode = "Flying"
            phase = 2
            z = self.flying_z
        else:
            mode = "Rolling"
            phase = 3
            z = self.rolling_z

        pos_target = np.array([x, y, z], dtype=float)
        return pos_target, mode, phase



# Unified circular mission for both flying and rolling
class CircularMission:
    """Circular path mission for both flying and rolling modes.

    Set mode to 'Flying' or 'Rolling' to select height and mode label.
    """

    def __init__(
        self,
        center,
        radius=0.4,
        omega=0.4,
        mode="Rolling",
        flying_z=0.35,
        rolling_z=0.2,
        cycles=1,
        camera_azimuth=-125.0,
        camera_elevation=-45.0,
        camera_distance=3.0,
        camera_lookat=None,
    ):
        self.center = np.array(center, dtype=float).copy()
        if self.center.shape != (3,):
            raise ValueError("center must be shape (3,)")

        self.radius = float(radius)
        self.omega = float(omega)
        self.mode = mode  # "Flying" or "Rolling"
        self.flying_z = float(flying_z)
        self.rolling_z = float(rolling_z)
        self.cycles = int(cycles)
        self.camera_azimuth = float(camera_azimuth)
        self.camera_elevation = float(camera_elevation)
        self.camera_distance = float(camera_distance)
        if camera_lookat is None:
            camera_lookat = (
                float(center[0]), float(center[1]), self.flying_z if mode == "Flying" else self.rolling_z
            )
        self.camera_lookat = np.array(camera_lookat, dtype=float).copy()
        if self.camera_lookat.shape != (3,):
            raise ValueError("camera_lookat must be shape (3,)")

        if self.radius <= 0.0:
            raise ValueError("radius must be > 0")
        if np.isclose(self.omega, 0.0):
            raise ValueError("omega must be non-zero")
        if self.cycles <= 0:
            raise ValueError("cycles must be > 0")

        self.period = (2.0 * np.pi) / abs(self.omega)
        self.mission_duration = self.period * float(self.cycles)

    def sample_circle_xy(self, t):
        theta = self.omega * t + np.pi
        return np.array(
            [
                self.center[0] + self.radius * np.cos(theta),
                self.center[1] + self.radius * np.sin(theta),
            ],
            dtype=float,
        )

    def target_and_mode(self, t):
        t_clamped = min(max(float(t), 0.0), self.mission_duration)
        theta = self.omega * t_clamped + np.pi
        xy = np.array([
            self.center[0] + self.radius * np.cos(theta),
            self.center[1] + self.radius * np.sin(theta),
        ], dtype=float)
        phase_deg = np.degrees((self.omega * t_clamped) % (2.0 * np.pi))
        z = self.flying_z if self.mode == "Flying" else self.rolling_z
        pos_target = np.array([xy[0], xy[1], z], dtype=float)

        # Velocity target (derivative of position)
        v_xy = np.array([
            -self.radius * self.omega * np.sin(theta),
            self.radius * self.omega * np.cos(theta)
        ], dtype=float)
        v_z = 0.0
        vel_target = np.array([v_xy[0], v_xy[1], v_z], dtype=float)

        # Acceleration target (second derivative)
        a_xy = np.array([
            -self.radius * self.omega**2 * np.cos(theta),
            -self.radius * self.omega**2 * np.sin(theta)
        ], dtype=float)
        a_z = 0.0
        acc_target = np.array([a_xy[0], a_xy[1], a_z], dtype=float)

        return pos_target, vel_target, acc_target, self.mode, phase_deg

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
        radius=0.4,
        omega=0.6,
        rolling_z=0.2,
        flying_z=0.35,
        cycles=1,
        camera_azimuth=-125.0,
        camera_elevation=-45.0,
        camera_distance=3.0,
        camera_lookat=(0.4, 0.0, 0.2),
    ):
        self.center = np.array(center, dtype=float).copy()
        self.radius = float(radius)
        self.omega = float(omega)
        self.rolling_z = float(rolling_z)
        self.flying_z = float(flying_z)
        self.cycles = int(cycles)
        self.camera_azimuth = float(camera_azimuth)
        self.camera_elevation = float(camera_elevation)
        self.camera_distance = float(camera_distance)
        self.camera_lookat = np.array(camera_lookat, dtype=float).copy()

        if self.center.shape != (3,):
            raise ValueError("center must be shape (3,)")
        if self.radius <= 0.0:
            raise ValueError("radius must be > 0")
        if np.isclose(self.omega, 0.0):
            raise ValueError("omega must be non-zero")
        if self.cycles <= 0:
            raise ValueError("cycles must be > 0")
        if self.camera_lookat.shape != (3,):
            raise ValueError("camera_lookat must be shape (3,)")

        self.period = (2.0 * np.pi) / abs(self.omega)
        self.mission_duration = self.period * float(self.cycles)

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
        t_clamped = min(max(float(t), 0.0), self.mission_duration)
        theta = self.omega * t_clamped + np.pi
        xy = np.array([
            self.center[0] + self.radius * np.cos(theta),
            self.center[1] + self.radius * np.sin(theta),
        ], dtype=float)
        mode, phase_deg = self.mode_from_time(t_clamped)
        z = self.rolling_z if mode == "Rolling" else self.flying_z
        pos_target = np.array([xy[0], xy[1], z], dtype=float)

        # Velocity target (derivative of position)
        v_xy = np.array([
            -self.radius * self.omega * np.sin(theta),
            self.radius * self.omega * np.cos(theta)
        ], dtype=float)
        v_z = 0.0
        vel_target = np.array([v_xy[0], v_xy[1], v_z], dtype=float)

        # Acceleration target (second derivative)
        a_xy = np.array([
            -self.radius * self.omega**2 * np.cos(theta),
            -self.radius * self.omega**2 * np.sin(theta)
        ], dtype=float)
        a_z = 0.0
        acc_target = np.array([a_xy[0], a_xy[1], a_z], dtype=float)

        return pos_target, vel_target, acc_target, mode, phase_deg
