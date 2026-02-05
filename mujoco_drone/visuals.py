import numpy as np
import mujoco
import transformations as tf


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
        mujoco.mjv_initGeom(viewer.user_scn.geoms[viewer.user_scn.ngeom-1],
                            mujoco.mjtGeom.mjGEOM_ARROW,
                            np.array([radius, radius, length]),  # size
                            position,
                            rotation_matrix,
                            np.array(rgba).astype(np.float32))


def draw_drone_thrust_arrow(viewer, drone, rgba=(1, 1, 0, 1)):
    """Draw a thrust arrow for the drone."""
    body_id = drone.m.body("base").id
    drone_pos = drone.d.xpos[body_id].copy()
    drone_mat = drone.d.xmat[body_id].copy()
    arrow_length = np.clip(drone.thrust_total / 50.0, 0.01, 1.0)
    draw_arrow(viewer, drone_pos, drone_mat, arrow_length, radius=0.0075, rgba=rgba)
