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


class FlyingMission:
    """Pure flying mission that cycles through target poses.

    The generated trajectory linearly interpolates between each pair of poses,
    with an optional hold duration at every target pose.
    """

    def __init__(self, poses, segment_duration=0.5, hold_duration=0.2, cycles=2):
        poses = np.array(poses, dtype=float)
        if poses.ndim != 2 or poses.shape[1] != 3 or poses.shape[0] < 2:
            raise ValueError("poses must be shape (N, 3) with N >= 2")

        self.poses = poses.copy()
        self.num_poses = int(poses.shape[0])
        self.segment_duration = float(segment_duration)
        self.hold_duration = float(hold_duration)
        self.cycles = int(cycles)

        if self.segment_duration <= 0.0:
            raise ValueError("segment_duration must be > 0")
        if self.hold_duration < 0.0:
            raise ValueError("hold_duration must be >= 0")
        if self.cycles <= 0:
            raise ValueError("cycles must be > 0")

        self.phase_duration = self.segment_duration + self.hold_duration
        self.loop_duration = float(self.num_poses) * self.phase_duration
        self.mission_duration = self.loop_duration * float(self.cycles)

    def target_and_mode(self, t):
        t_clamped = min(max(float(t), 0.0), self.mission_duration)
        t_loop = t_clamped % self.loop_duration
        phase_idx = int(t_loop // self.phase_duration)
        phase_time = t_loop - phase_idx * self.phase_duration

        p_start = self.poses[phase_idx]
        p_end = self.poses[(phase_idx + 1) % self.num_poses]

        if phase_time <= self.segment_duration:
            alpha = phase_time / self.segment_duration
            pos_target = (1.0 - alpha) * p_start + alpha * p_end
        else:
            pos_target = p_end.copy()

        return np.array(pos_target, dtype=float), "Flying", phase_idx + 1


class RollingMission:
    """Pure rolling mission that tracks a circular path."""

    def __init__(self, center, radius=0.4, omega=0.6, rolling_z=0.2, cycles=2):
        self.center = np.array(center, dtype=float).copy()
        if self.center.shape != (3,):
            raise ValueError("center must be shape (3,)")

        self.radius = float(radius)
        self.omega = float(omega)
        self.rolling_z = float(rolling_z)
        self.cycles = int(cycles)

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
        xy = self.sample_circle_xy(t_clamped)
        phase_deg = np.degrees((self.omega * t_clamped) % (2.0 * np.pi))
        pos_target = np.array([xy[0], xy[1], self.rolling_z], dtype=float)
        return pos_target, "Rolling", phase_deg

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
