import os
import csv
from datetime import datetime

try:
    import pyautogui
except Exception:
    pyautogui = None

import mujoco
import imageio.v2 as imageio

from mujoco_drone.lightweight_dashboard import create_dashboard, maybe_update_dashboard
from mujoco_drone.visuals import draw_thrust_visualization, draw_pos_target_sphere
from mujoco_drone.mission import build_mission


paused = False


def key_callback(keycode):
    global paused
    if keycode == 32:  # Spacebar
        paused = not paused
        print(f"Simulation {'paused' if paused else 'resumed'}")


def is_paused():
    return paused


def init_recording_state(drone):
    mission_obj = drone.mission
    record_duration_s = float(drone.mission_duration)
    print(
        f"Mission duration: {record_duration_s:.2f} seconds "
        "(recording will stop after this time)"
    )
    return {
        "mission": mission_obj,
        "mission_name": mission_obj.__class__.__name__,
        "duration_s": record_duration_s,
        "fps": 30,
        "dt": 1.0 / 30,
        "frames": [],
        "position_log": [],
        "last_record_t": None,
        "record_start_t": None,
    }


def try_create_renderer(drone, recording_enabled, viewer=None, mission_obj=None):
    if pyautogui is not None:
        try:
            pyautogui.hotkey("shift", "tab")
        except Exception:
            pass

    if viewer is not None and mission_obj is not None:
        configure_camera(viewer, mission_obj)

    if not recording_enabled:
        return None, False
    try:
        renderer = mujoco.Renderer(drone.m, width=1920, height=1080)
        return renderer, True
    except Exception as exc:
        print(f"Recording disabled: failed to initialize renderer ({exc})")
        return None, False


def configure_camera(viewer, mission_obj):
    viewer.cam.azimuth = float(getattr(mission_obj, "camera_azimuth", 90.0))
    viewer.cam.elevation = float(getattr(mission_obj, "camera_elevation", -10.0))
    viewer.cam.distance = float(getattr(mission_obj, "camera_distance", 3.0))
    viewer.cam.lookat[:] = getattr(mission_obj, "camera_lookat", [0.9, 0.0, 0.0])


def update_scene_draw(viewer, drone, enable_debug_draw):
    viewer.user_scn.ngeom = 0
    if enable_debug_draw:
        draw_thrust_visualization(viewer, drone)
    draw_pos_target_sphere(viewer, drone.pos_target)


def maybe_record_frame(viewer, renderer, drone, recording_state, recording_enabled):
    if not recording_enabled:
        return recording_enabled

    if recording_state["record_start_t"] is None:
        recording_state["record_start_t"] = drone.d.time
        recording_state["last_record_t"] = drone.d.time - recording_state["dt"]

    elapsed = drone.d.time - recording_state["record_start_t"]
    should_sample = (
        elapsed <= recording_state["duration_s"]
        and (drone.d.time - recording_state["last_record_t"]) >= recording_state["dt"]
    )
    if not should_sample:
        return recording_enabled

    try:
        renderer.update_scene(drone.d, camera=viewer.cam)
        frame = renderer.render()
        recording_state["frames"].append(frame)

        base_pos = drone.state_estimator.base_pos
        control_mode = getattr(drone, "control_mode", type(drone.controller).__name__)
        ref_pos, *_ = recording_state["mission"].target_and_mode(drone.d.time)
        recording_state["position_log"].append(
            (
                float(drone.d.time),
                float(elapsed),
                float(base_pos[0]),
                float(base_pos[1]),
                float(base_pos[2]),
                float(ref_pos[0]),
                float(ref_pos[1]),
                float(ref_pos[2]),
                str(control_mode),
            )
        )

        recording_state["last_record_t"] = drone.d.time
        return recording_enabled
    except Exception as exc:
        recording_state["frames"].clear()
        recording_state["position_log"].clear()
        print(f"Recording disabled during runtime ({exc})")
        return False


def maybe_save_recording(recording_state, recording_enabled, sim_time):
    if not recording_enabled:
        return False
    if recording_state["record_start_t"] is None:
        return False

    elapsed = sim_time - recording_state["record_start_t"]
    if elapsed <= recording_state["duration_s"] or len(recording_state["frames"]) == 0:
        return False

    print(
        f"Recording complete: {len(recording_state['frames'])} frames "
        f"captured over {elapsed:.2f} seconds"
    )
    os.makedirs("log", exist_ok=True)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    run_name = (
        f"{timestamp}_{recording_state['mission_name']}_sim_"
        f"{int(recording_state['duration_s'])}s"
    )
    run_dir = os.path.join("log", run_name)
    os.makedirs(run_dir, exist_ok=True)
    out_path = os.path.join(run_dir, f"{run_name}.mp4")
    pos_path = os.path.join(run_dir, f"{run_name}_position.csv")

    imageio.mimsave(out_path, recording_state["frames"], fps=recording_state["fps"])

    with open(pos_path, "w", newline="") as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(
            [
                "sim_time_s",
                "elapsed_s",
                "x_m",
                "y_m",
                "z_m",
                "ref_x_m",
                "ref_y_m",
                "ref_z_m",
                "control_mode",
            ]
        )
        writer.writerows(recording_state["position_log"])

    print(
        f"Saved recording: {out_path} "
        f"({len(recording_state['frames'])} frames at {recording_state['fps']} FPS)"
    )
    print(
        f"Saved position log: {pos_path} "
        f"({len(recording_state['position_log'])} samples)"
    )
    return True
def update_status_text(viewer, drone, record_start_t, paused):
    control_mode = getattr(drone, "control_mode", type(drone.controller).__name__)
    elapsed = drone.d.time - record_start_t if record_start_t is not None else 0.0
    viewer.set_texts(
        [
            (
                0,
                0,
                f"Time: {drone.d.time:.2f}  Elapsed: {elapsed:.2f}",
                f"Paused: {paused} | Control Mode: {control_mode}",
            ),
        ]
    )
