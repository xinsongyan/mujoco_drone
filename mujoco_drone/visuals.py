import numpy as np
import mujoco


def draw_arrow(viewer, position, rotation_matrix, length, radius=0.0075, rgba=(1, 1, 0, 1)):
    """Draw a single arrow in the MuJoCo viewer.

    Args:
        viewer: MuJoCo viewer object
        position: Arrow base position (3,)
        rotation_matrix: 3x3 rotation matrix flattened to (9,)
        length: Arrow length
        radius: Arrow shaft radius
        rgba: Color as (R, G, B, A) tuple
    """
    if viewer.user_scn.ngeom < viewer.user_scn.maxgeom:
        viewer.user_scn.ngeom += 1
        geom = viewer.user_scn.geoms[viewer.user_scn.ngeom-1]
        mujoco.mjv_initGeom(geom,
                            mujoco.mjtGeom.mjGEOM_ARROW,
                            np.array([radius, radius, length]),  # size
                            position,
                            rotation_matrix,
                            np.array(rgba).astype(np.float32))



def draw_drone_thrust_arrow(viewer, drone, rgba=(1, 1, 0, 0.2)):
    """Draw a thrust arrow for the drone."""
    body_id = drone.m.body("base").id
    drone_pos = drone.d.xpos[body_id].copy()
    drone_mat = drone.d.xmat[body_id].copy()
    # arrow_length = np.clip(drone.thrust_total / 50.0, -10.0, 10.0)
    arrow_length = drone.thrust_total
    draw_arrow(viewer, drone_pos, drone_mat, arrow_length, radius=0.0075, rgba=rgba)


def draw_motor_thrust_arrows(
    viewer,
    drone,
    motor_sites=("motor_rr", "motor_fr", "motor_rl", "motor_fl"),
    length_scale=1.0,
    radius=0.004,
    rgba=(1, 1, 0, 0.2),
):
    """Draw thrust arrows for each rotor using motor commands.

    Args:
        viewer: MuJoCo viewer object
        drone: SimpleDrone object
        motor_sites: Site names in motor command order
        length_scale: Scale factor for arrow length
        radius: Arrow shaft radius
        rgba: Color as (R, G, B, A) tuple
    """
    if not hasattr(drone, "motor_cmd"):
        return

    for idx, site_name in enumerate(motor_sites):
        if idx >= len(drone.motor_cmd):
            break
        site_id = drone.m.site(site_name).id
        pos = drone.d.site_xpos[site_id].copy()
        mat = drone.d.site_xmat[site_id].copy()
        length = drone.motor_cmd[idx] / length_scale
        draw_arrow(viewer, pos, mat, length, radius=radius, rgba=rgba)


def draw_thrust_visualization(viewer, drone):
    """Draw all thrust visualizations: total thrust arrow and per-rotor arrows.
    
    Args:
        viewer: MuJoCo viewer object
        drone: SimpleDrone object
    """
    draw_drone_thrust_arrow(viewer, drone)
    draw_motor_thrust_arrows(viewer, drone)
