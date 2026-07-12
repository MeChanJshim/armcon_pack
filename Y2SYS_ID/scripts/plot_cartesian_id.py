#!/usr/bin/env python3
"""Plot a Y2SYS_ID result CSV and save the figures as PNG files."""

import csv
from pathlib import Path

import matplotlib.pyplot as plt


# 플롯할 CSV 파일명을 여기에 입력하세요.
INPUT_FILENAME = "cartesian_id_20260705_084954(UR10CB3).csv"
SHOW_PLOTS = True

RESULTS_DIR = Path(__file__).resolve().parent.parent / "results"
PLOTS_DIR = RESULTS_DIR / "plots"

POSE_LABELS = ("x", "y", "z", "wx", "wy", "wz")
JOINT_LABELS = tuple(f"joint_{index}" for index in range(1, 7))
WRENCH_LABELS = ("x", "y", "z")


def parse_vector(value):
    """Convert a semicolon-separated CSV cell to a float list."""
    if not value:
        return []
    return [float(item) for item in value.split(";") if item.strip()]


def load_csv(csv_path):
    with csv_path.open(newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))

    if not rows:
        raise ValueError(f"CSV file is empty: {csv_path}")

    start_time = float(rows[0]["time_sec"])
    time_sec = [float(row["time_sec"]) - start_time for row in rows]
    return rows, time_sec


def get_paired_vectors(time_sec, rows, first_column, second_column, expected_size):
    valid_samples = []
    for sample_time, row in zip(time_sec, rows):
        first = parse_vector(row.get(first_column, ""))
        second = parse_vector(row.get(second_column, ""))
        if len(first) >= expected_size and len(second) >= expected_size:
            valid_samples.append((sample_time, first, second))

    if not valid_samples:
        return None, None, None

    valid_time = [sample[0] for sample in valid_samples]
    first_vectors = [
        [sample[1][index] for sample in valid_samples] for index in range(expected_size)
    ]
    second_vectors = [
        [sample[2][index] for sample in valid_samples] for index in range(expected_size)
    ]
    return valid_time, first_vectors, second_vectors


def plot_comparison(time_sec, rows, first_column, second_column, labels, title,
                    first_legend, second_legend, output_path):
    valid_time, first, second = get_paired_vectors(
        time_sec, rows, first_column, second_column, len(labels)
    )
    if first is None:
        print(f"skip: required columns are missing or malformed ({title})")
        return

    figure, axes = plt.subplots(3, 2, figsize=(14, 10), sharex=True)
    for index, (axis, label) in enumerate(zip(axes.flat, labels)):
        axis.plot(valid_time, first[index], label=first_legend, linewidth=1.2)
        axis.plot(valid_time, second[index], label=second_legend, linewidth=1.0)
        axis.set_title(label)
        axis.set_ylabel("value")
        axis.grid(True, alpha=0.3)
        axis.legend()
    for axis in axes[-1]:
        axis.set_xlabel("time [s]")
    figure.suptitle(title)
    figure.tight_layout()
    figure.savefig(output_path, dpi=200, bbox_inches="tight")
    print(f"saved: {output_path}")


def plot_wrench(time_sec, rows, output_path):
    valid_time, force, torque = get_paired_vectors(
        time_sec, rows, "wrench_force", "wrench_torque", 3
    )
    if force is None:
        print("skip: wrench columns are missing or malformed")
        return

    figure, axes = plt.subplots(2, 1, figsize=(14, 8), sharex=True)
    for index, label in enumerate(WRENCH_LABELS):
        axes[0].plot(valid_time, force[index], label=label, linewidth=1.0)
        axes[1].plot(valid_time, torque[index], label=label, linewidth=1.0)
    axes[0].set_title("Force")
    axes[0].set_ylabel("force")
    axes[1].set_title("Torque")
    axes[1].set_ylabel("torque")
    axes[1].set_xlabel("time [s]")
    for axis in axes:
        axis.grid(True, alpha=0.3)
        axis.legend()
    figure.suptitle("Wrench")
    figure.tight_layout()
    figure.savefig(output_path, dpi=200, bbox_inches="tight")
    print(f"saved: {output_path}")


def main():
    csv_path = RESULTS_DIR / INPUT_FILENAME
    if not csv_path.is_file():
        raise FileNotFoundError(f"Input CSV does not exist: {csv_path}")
    if csv_path.suffix.lower() != ".csv":
        raise ValueError(f"Only CSV files are supported: {csv_path.name}")

    output_dir = PLOTS_DIR / csv_path.stem
    output_dir.mkdir(parents=True, exist_ok=True)
    rows, time_sec = load_csv(csv_path)

    plot_comparison(
        time_sec, rows, "command_pose", "current_pose", POSE_LABELS,
        "Cartesian pose: command vs current", "command", "current",
        output_dir / "cartesian_pose.png",
    )
    plot_comparison(
        time_sec, rows, "target_joint", "current_joint", JOINT_LABELS,
        "Joint position: target vs current", "target", "current",
        output_dir / "joint_position.png",
    )
    plot_wrench(time_sec, rows, output_dir / "wrench.png")

    if SHOW_PLOTS:
        plt.show()
    else:
        plt.close("all")


if __name__ == "__main__":
    main()
