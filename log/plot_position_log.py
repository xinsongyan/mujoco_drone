#!/usr/bin/env python3
import csv
import glob
import os
import shutil
import sys
from typing import List, Tuple

import imageio.v2 as imageio
import numpy as np


def find_latest_csv(log_dir: str) -> str:
    pattern = os.path.join(log_dir, "*_position.csv")
    candidates = glob.glob(pattern)
    if not candidates:
        raise FileNotFoundError(f"No position CSV found in {log_dir}")
    candidates.sort(key=os.path.getmtime)
    return candidates[-1]


def load_position_csv(csv_path: str) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, List[str]]:
    elapsed_s = []
    x_m = []
    y_m = []
    z_m = []
    control_mode = []

    with open(csv_path, "r", newline="") as f:
        reader = csv.DictReader(f)
        required = {"elapsed_s", "x_m", "y_m", "z_m", "control_mode"}
        if not required.issubset(reader.fieldnames or set()):
            raise ValueError(
                f"CSV missing required columns. Required: {sorted(required)}, got: {reader.fieldnames}"
            )

        for row in reader:
            elapsed_s.append(float(row["elapsed_s"]))
            x_m.append(float(row["x_m"]))
            y_m.append(float(row["y_m"]))
            z_m.append(float(row["z_m"]))
            control_mode.append(row["control_mode"])

    return (
        np.asarray(elapsed_s),
        np.asarray(x_m),
        np.asarray(y_m),
        np.asarray(z_m),
        control_mode,
    )


def save_plots(csv_path: str) -> Tuple[str, str, str]:
    try:
        import matplotlib as mpl
        import matplotlib.pyplot as plt
    except Exception as e:
        raise RuntimeError(
            "matplotlib is required for plotting. Install it with: pip install matplotlib"
        ) from e

    # Use LaTeX-like font styling for title/legend/labels.
    # If a full LaTeX installation is available, render with usetex.
    if shutil.which("latex") is not None:
        mpl.rcParams.update(
            {
                "text.usetex": True,
                "font.family": "serif",
                "font.serif": ["Computer Modern Roman"],
                "axes.unicode_minus": False,
            }
        )
    else:
        mpl.rcParams.update(
            {
                "text.usetex": False,
                "font.family": "serif",
                "font.serif": ["Computer Modern Roman", "CMU Serif", "DejaVu Serif"],
                "mathtext.fontset": "cm",
            }
        )

    elapsed_s, x_m, y_m, z_m, control_mode = load_position_csv(csv_path)

    stem, _ = os.path.splitext(csv_path)
    out_dir = stem
    os.makedirs(out_dir, exist_ok=True)

    traj_png = os.path.join(out_dir, "xy.png")
    xyz_png = os.path.join(out_dir, "xyz_time.png")
    z_png = os.path.join(out_dir, "z_time.png")

    # XY trajectory plot
    plt.figure(figsize=(7, 6))
    plt.plot(x_m, y_m, linewidth=2.0, label="trajectory")
    plt.scatter([x_m[0]], [y_m[0]], c="green", s=40, label="start")
    plt.scatter([x_m[-1]], [y_m[-1]], c="red", s=40, label="end")
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.title(" XY Trajectory")
    plt.axis("equal")
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.tight_layout()
    plt.savefig(traj_png, dpi=150)
    plt.close()

    # XYZ vs time plot
    plt.figure(figsize=(9, 5))
    plt.plot(elapsed_s, x_m, label="x [m]")
    plt.plot(elapsed_s, y_m, label="y [m]")
    plt.plot(elapsed_s, z_m, label="z [m]")

    # Highlight mode switches if present
    if len(control_mode) == len(elapsed_s) and len(control_mode) > 1:
        switch_indices = [
            i for i in range(1, len(control_mode)) if control_mode[i] != control_mode[i - 1]
        ]
        for idx in switch_indices:
            plt.axvline(elapsed_s[idx], color="k", linestyle="--", alpha=0.25)

    plt.xlabel("elapsed time [s]")
    plt.ylabel("position [m]")
    plt.title(" Position vs Time")
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.tight_layout()
    plt.savefig(xyz_png, dpi=150)
    plt.close()

    # Z vs time plot
    plt.figure(figsize=(9, 4))
    plt.plot(elapsed_s, z_m, color="tab:purple", linewidth=2.0, label="z [m]")

    if len(control_mode) == len(elapsed_s) and len(control_mode) > 1:
        switch_indices = [
            i for i in range(1, len(control_mode)) if control_mode[i] != control_mode[i - 1]
        ]
        for idx in switch_indices:
            plt.axvline(elapsed_s[idx], color="k", linestyle="--", alpha=0.25)

    plt.xlabel("elapsed time [s]")
    plt.ylabel("z [m]")
    plt.title(" Z Position vs Time")
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.tight_layout()
    plt.savefig(z_png, dpi=150)
    plt.close()

    return traj_png, xyz_png, z_png


def copy_artifacts(csv_path: str) -> Tuple[str, str | None, str | None, str | None]:
    stem, _ = os.path.splitext(csv_path)
    out_dir = stem
    os.makedirs(out_dir, exist_ok=True)

    csv_dst = os.path.join(out_dir, os.path.basename(csv_path))
    shutil.copy2(csv_path, csv_dst)

    deleted_csv = None
    if os.path.abspath(csv_path) != os.path.abspath(csv_dst):
        os.remove(csv_path)
        deleted_csv = csv_path

    # CSV naming is expected as: <run>_position.csv -> MP4: <run>.mp4
    run_stem = stem[:-9] if stem.endswith("_position") else stem
    mp4_src = f"{run_stem}.mp4"
    if os.path.exists(mp4_src):
        mp4_dst = os.path.join(out_dir, os.path.basename(mp4_src))
        shutil.copy2(mp4_src, mp4_dst)
        deleted_mp4 = None
        if os.path.abspath(mp4_src) != os.path.abspath(mp4_dst):
            os.remove(mp4_src)
            deleted_mp4 = mp4_src
        return csv_dst, mp4_dst, deleted_csv, deleted_mp4

    return csv_dst, None, deleted_csv, None


def save_phase_screenshots(mp4_path: str, out_dir: str) -> List[str]:
    """Save 4 screenshots representing the 4 trajectory phases from a video.

    Screenshots are taken at the midpoint of each quarter:
    12.5%, 37.5%, 62.5%, 87.5% of video duration.
    """
    saved_paths: List[str] = []
    reader = imageio.get_reader(mp4_path)
    try:
        meta = reader.get_meta_data()
        fps = float(meta.get("fps", 30.0))
        duration = meta.get("duration", None)

        target_frame_indices: List[int]
        if duration is not None and duration > 0:
            target_times = [duration * ratio for ratio in (0.125, 0.375, 0.625, 0.875)]
            target_frame_indices = [max(0, int(round(t * fps))) for t in target_times]
        else:
            frame_count = reader.count_frames()
            if frame_count <= 0:
                return saved_paths
            target_frame_indices = [
                max(0, min(frame_count - 1, int(round(frame_count * ratio))))
                for ratio in (0.125, 0.375, 0.625, 0.875)
            ]

        for phase_idx, frame_idx in enumerate(target_frame_indices, start=1):
            frame = reader.get_data(frame_idx)
            out_path = os.path.join(out_dir, f"phase_{phase_idx}.png")
            imageio.imwrite(out_path, frame)
            saved_paths.append(out_path)
    finally:
        reader.close()

    return saved_paths


def main() -> int:
    log_dir = os.path.dirname(os.path.abspath(__file__))

    if len(sys.argv) > 1:
        csv_path = os.path.abspath(sys.argv[1])
    else:
        csv_path = find_latest_csv(log_dir)

    if not os.path.exists(csv_path):
        print(f"CSV not found: {csv_path}")
        return 1

    traj_png, xyz_png, z_png = save_plots(csv_path)
    csv_copy, mp4_copy, deleted_csv, deleted_mp4 = copy_artifacts(csv_path)
    print(f"Loaded: {csv_path}")
    print(f"Saved:  {traj_png}")
    print(f"Saved:  {xyz_png}")
    print(f"Saved:  {z_png}")
    print(f"Copied: {csv_copy}")
    if mp4_copy is not None:
        print(f"Copied: {mp4_copy}")
        phase_images = save_phase_screenshots(mp4_copy, os.path.dirname(csv_copy))
        for phase_image in phase_images:
            print(f"Saved:  {phase_image}")
    else:
        print("MP4 not found for this CSV (expected same run prefix).")

    if deleted_csv is not None:
        print(f"Removed original: {deleted_csv}")
    if deleted_mp4 is not None:
        print(f"Removed original: {deleted_mp4}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
