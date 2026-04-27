
import mujoco
import mujoco.viewer 

from mujoco_drone.drone import SimpleDrone
from mujoco_drone.sim_utils import (
    build_mission,
    create_dashboard,
    init_recording_state,
    is_paused,
    key_callback,
    maybe_record_frame,
    maybe_save_recording,
    maybe_update_dashboard,
    try_create_renderer,
    update_scene_draw,
    update_status_text,
)


# ── Mission selector ─────────────────────────────────────────────────────────
# Set MISSION to "phased_straight_line", "flying", "rolling", or "phase_circle"
MISSION = "phased_straight_line"
# ─────────────────────────────────────────────────────────────────────────────


# Offscreen renderer can fail on some Linux/GLX setups. If that happens,
# recording will be disabled at runtime and simulation will continue.

ENABLE_RECORDING = False
ENABLE_DEBUG_DRAW = False  # Default, can be overridden by CLI
ENABLE_DASHBOARD = True  # Default, can be overridden by CLI




if __name__ == "__main__":
    dashboard = create_dashboard(ENABLE_DASHBOARD)

    # Initialize the drone
    drone = SimpleDrone(caged=True)

    # Build and inject mission
    mission = build_mission(drone, MISSION)
    drone.set_mission(mission)

    recording_state = init_recording_state(drone)

    with mujoco.viewer.launch_passive(drone.m, drone.d, key_callback=key_callback) as viewer:
        renderer, recording_enabled = try_create_renderer(
            drone, ENABLE_RECORDING, viewer, recording_state["mission"]
        )

        step_count = 0

        try:
            while viewer.is_running():
                if not is_paused():
                    drone()
                    mujoco.mj_step(drone.m, drone.d)

                    update_scene_draw(viewer, drone, ENABLE_DEBUG_DRAW)
                    recording_enabled = maybe_record_frame(
                        viewer, renderer, drone, recording_state, recording_enabled
                    )
                    if maybe_save_recording(
                        recording_state, recording_enabled, drone.d.time
                    ):
                        break

                    maybe_update_dashboard(dashboard, ENABLE_DASHBOARD, step_count, drone)

                    step_count += 1

                update_status_text(viewer, drone, recording_state["record_start_t"], is_paused())

                # Always sync the viewer to process events / render
                viewer.sync()
        finally:
            if dashboard is not None:
                dashboard.stop()
            if renderer is not None:
                try:
                    renderer.close()
                except Exception:
                    pass
