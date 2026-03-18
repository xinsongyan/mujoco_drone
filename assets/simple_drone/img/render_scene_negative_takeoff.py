import os
os.environ.setdefault("MUJOCO_GL", "egl")
from pathlib import Path

import imageio.v3 as iio
import mujoco
import mujoco.viewer


CAMERA_AZIMUTH = 90.0
CAMERA_ELEVATION = 0.0
CAMERA_DISTANCE = 2.0
CAMERA_LOOKAT = (0.5, 0.0, 0.2)
IMAGE_WIDTH = 1920
IMAGE_HEIGHT = 1080
OUTPUT_IMAGE_NAME = "scene_negative_takeoff.png"


def load_scene():
    """Load the white-background caged drone MuJoCo model and data."""
    this_file = Path(__file__).resolve()
    xml_path = this_file.parent / "scene_negative_takeoff.xml"
    print(f"Loading XML: {xml_path}")
    model = mujoco.MjModel.from_xml_path(str(xml_path))
    data = mujoco.MjData(model)
    return model, data


def configure_camera(viewer):
    """Apply a deterministic free-camera view for the scene."""
    viewer.cam.azimuth = CAMERA_AZIMUTH
    viewer.cam.elevation = CAMERA_ELEVATION
    viewer.cam.distance = CAMERA_DISTANCE
    viewer.cam.lookat[:] = CAMERA_LOOKAT


def save_camera_view(model, data):
    """Render and save the configured camera view to an image file (no viewer needed)."""
    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    cam.azimuth = CAMERA_AZIMUTH
    cam.elevation = CAMERA_ELEVATION
    cam.distance = CAMERA_DISTANCE
    cam.lookat[:] = CAMERA_LOOKAT

    output_path = Path(__file__).resolve().parent / OUTPUT_IMAGE_NAME
    renderer = mujoco.Renderer(model, width=IMAGE_WIDTH, height=IMAGE_HEIGHT)
    try:
        renderer.update_scene(data, camera=cam)
        image = renderer.render()
        iio.imwrite(output_path, image)
    finally:
        renderer.close()
    print(f"Saved camera view: {output_path}")


if __name__ == "__main__":
    model, data = load_scene()
    
    print(
        "Camera:",
        {
            "azimuth": CAMERA_AZIMUTH,
            "elevation": CAMERA_ELEVATION,
            "distance": CAMERA_DISTANCE,
            "lookat": CAMERA_LOOKAT,
        },
    )

    mujoco.mj_forward(model, data)
    save_camera_view(model, data)

    try:
        with mujoco.viewer.launch_passive(
            model,
            data,
            show_left_ui=False,
            show_right_ui=False,
        ) as viewer:
            configure_camera(viewer)
            while viewer.is_running():
                mujoco.mj_step(model, data)
                viewer.sync()
                pass
    except KeyboardInterrupt:
        print("KeyboardInterrupt received; exiting...")
