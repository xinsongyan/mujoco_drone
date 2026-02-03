import numpy as np


def create_key_callback(state, move_step=0.02, yaw_step_deg=5):
    """Create a key callback that updates the provided shared state."""
    yaw_step = np.deg2rad(yaw_step_deg)

    def key_callback(keycode):
        """Handles key presses for pausing and controlling the drone."""
        drone = state.get("drone")

        # The viewer might call the callback before the drone is initialized
        if drone is None:
            return

        # Print raw keycode to help map special keys
        print(f"Key pressed: code={keycode}")

        # Spacebar
        if keycode == 32:
            state["paused"] = not state.get("paused", False)
            print(f"Paused: {state['paused']}")
            return

        # --- Target Position Controls ---
        # 8 key -> Move Forward (+X)
        if keycode == 265:
            drone.flight_control.adjust_target_x(move_step)
            print(
                f"Target XYZ: ("
                f"{drone.flight_control.target_x:.2f}m, "
                f"{drone.flight_control.target_y:.2f}m, "
                f"{drone.flight_control.target_z:.2f}m)"
            )
            return

        # 2 key -> Move Backward (-X)
        if keycode == 264:
            drone.flight_control.adjust_target_x(-move_step)
            print(
                f"Target XYZ: ("
                f"{drone.flight_control.target_x:.2f}m, "
                f"{drone.flight_control.target_y:.2f}m, "
                f"{drone.flight_control.target_z:.2f}m)"
            )
            return

        # 4 key -> Move Left (+Y)
        if keycode == 324:
            drone.flight_control.adjust_target_y(move_step)
            print(
                f"Target XYZ: ("
                f"{drone.flight_control.target_x:.2f}m, "
                f"{drone.flight_control.target_y:.2f}m, "
                f"{drone.flight_control.target_z:.2f}m)"
            )
            return

        # 6 key -> Move Right (-Y)
        if keycode == 326:
            drone.flight_control.adjust_target_y(-move_step)
            print(
                f"Target XYZ: ("
                f"{drone.flight_control.target_x:.2f}m, "
                f"{drone.flight_control.target_y:.2f}m, "
                f"{drone.flight_control.target_z:.2f}m)"
            )
            return

        # --- Altitude Controls (Up/Down Arrows) ---
        # UP arrow -> Move Up (+Z)
        if keycode == 328:
            drone.flight_control.adjust_target_z(move_step)
            print(
                f"Target XYZ: ("
                f"{drone.flight_control.target_x:.2f}m, "
                f"{drone.flight_control.target_y:.2f}m, "
                f"{drone.flight_control.target_z:.2f}m)"
            )
            return

        # DOWN arrow -> Move Down (-Z)
        if keycode == 322:
            drone.flight_control.adjust_target_z(-move_step)
            print(
                f"Target XYZ: ("
                f"{drone.flight_control.target_x:.2f}m, "
                f"{drone.flight_control.target_y:.2f}m, "
                f"{drone.flight_control.target_z:.2f}m)"
            )
            return

        # --- Yaw Controls (Left/Right Arrows) ---
        # LEFT arrow -> Rotate Left
        if keycode == 263:
            drone.flight_control.adjust_yaw(yaw_step)
            print(f"Target Yaw: {np.degrees(drone.flight_control.target_yaw):.1f}°")
            return

        # RIGHT arrow -> Rotate Right
        if keycode == 262:
            drone.flight_control.adjust_yaw(-yaw_step)
            print(f"Target Yaw: {np.degrees(drone.flight_control.target_yaw):.1f}°")

    return key_callback
