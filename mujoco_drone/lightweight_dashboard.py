"""Lightweight dashboard running in a separate process via matplotlib.

The simulation loop only calls queue.put_nowait() — essentially free.
The matplotlib animation runs independently at ~10 Hz in its own process.
"""

import multiprocessing
import collections
import numpy as np
import matplotlib
matplotlib.use("TkAgg")  # works headless-friendly; falls back gracefully
import matplotlib.pyplot as plt
import matplotlib.animation as animation


BUFFER       = 300
INTERVAL_MS  = 100  # dashboard refresh interval

# Each group: (subplot title, y-label, [(series_label, color), ...])
GROUPS = [
    ("Position",        "m",       [("X", "tab:red"), ("Y", "tab:green"), ("Z", "tab:blue")]),
    ("Roll/Pitch/Yaw",  "deg",     [("Roll", "tab:red"), ("Pitch", "tab:green"), ("Yaw", "tab:blue")]),
    ("Thrust & Torques","N / N·m", [("Thrust", "tab:purple"), ("τ roll", "tab:orange"), ("τ pitch", "tab:cyan"), ("τ yaw", "tab:olive")]),
    ("Motor Commands",  "cmd",     [("M1", "tab:orange"), ("M2", "tab:cyan"), ("M3", "tab:pink"), ("M4", "tab:olive")]),
]
# Total number of data channels (sum of series per group)
N_CHANNELS = sum(len(g[2]) for g in GROUPS)


def _run_dashboard(queue: multiprocessing.Queue) -> None:
    fig, axes = plt.subplots(len(GROUPS), 1, figsize=(7, 9), sharex=False)
    fig.suptitle("Drone Dashboard", fontsize=9)
    fig.tight_layout(pad=2.0)

    times   = collections.deque(maxlen=BUFFER)
    buffers = [collections.deque(maxlen=BUFFER) for _ in range(N_CHANNELS)]
    lines   = []

    ch = 0
    for ax, (title, ylabel, series) in zip(axes, GROUPS):
        ax.set_title(title, fontsize=7, pad=2)
        ax.set_ylabel(ylabel, fontsize=7)
        ax.tick_params(labelsize=6)
        ax.grid(True, alpha=0.3)
        for label, color in series:
            (line,) = ax.plot([], [], color=color, linewidth=1.0, label=label)
            lines.append((line, ax, ch))
            ch += 1
        ax.legend(loc="upper left", fontsize=6, ncol=len(series))

    def _update(_frame):
        while True:
            try:
                t, values = queue.get_nowait()
                times.append(t)
                for i, v in enumerate(values):
                    buffers[i].append(v)
            except Exception:
                break

        if len(times) < 2:
            return [l for l, _, _ in lines]

        t_arr = list(times)
        dirty_axes = set()
        for line, ax, idx in lines:
            line.set_data(t_arr, list(buffers[idx]))
            dirty_axes.add(ax)
        for ax in dirty_axes:
            ax.relim()
            ax.autoscale_view()

        return [l for l, _, _ in lines]

    _ani = animation.FuncAnimation(
        fig, _update, interval=INTERVAL_MS, blit=False, cache_frame_data=False
    )
    plt.show()


class LightweightDashboard:
    """Dashboard running in a separate process with near-zero sim overhead."""

    def __init__(self) -> None:
        self._queue: multiprocessing.Queue = multiprocessing.Queue(maxsize=600)
        self._process = multiprocessing.Process(
            target=_run_dashboard, args=(self._queue,), daemon=True
        )
        self._process.start()

    def update(self, drone, time: float) -> None:
        """Push latest drone state to the dashboard process. Never blocks."""
        roll, pitch, yaw = drone.state_estimator.base_rpy
        motor_cmd = drone.motor_cmd if hasattr(drone, "motor_cmd") else [0.0, 0.0, 0.0, 0.0]
        # Channel order must match GROUPS definition:
        # Position(3), RPY(3), Thrust+Torques(4), Motors(4)
        values = [
            float(drone.d.qpos[0]),             # X
            float(drone.d.qpos[1]),             # Y
            float(drone.d.qpos[2]),             # Z
            float(np.degrees(roll)),            # Roll
            float(np.degrees(pitch)),           # Pitch
            float(np.degrees(yaw)),             # Yaw
            float(drone.thrust_total),          # Thrust
            float(getattr(drone, "torque_roll",  0.0)),   # τ roll
            float(getattr(drone, "torque_pitch", 0.0)),   # τ pitch
            float(getattr(drone, "torque_yaw",   0.0)),   # τ yaw
            float(motor_cmd[0]),                # M1
            float(motor_cmd[1]),                # M2
            float(motor_cmd[2]),                # M3
            float(motor_cmd[3]),                # M4
        ]
        try:
            self._queue.put_nowait((time, values))
        except Exception:
            pass  # queue full — drop this sample rather than blocking

    def stop(self) -> None:
        if self._process.is_alive():
            self._process.terminate()
            self._process.join(timeout=2)
