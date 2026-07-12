#!/usr/bin/env python3
import argparse
import csv
import math
from pathlib import Path


AXIS_INDEX = {
    "x": 0,
    "y": 1,
    "z": 2,
    "wx": 3,
    "wy": 4,
    "wz": 5,
}


def parse_vector(value):
    if not value:
        return []
    return [float(item) for item in value.split(";") if item != ""]


def load_series(csv_path, axis):
    axis_index = AXIS_INDEX[axis]
    times = []
    commands = []
    responses = []

    with open(csv_path, newline="") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            command_pose = parse_vector(row.get("command_pose", ""))
            current_pose = parse_vector(row.get("current_pose", ""))
            if len(command_pose) <= axis_index or len(current_pose) <= axis_index:
                continue
            times.append(float(row["time_sec"]))
            commands.append(command_pose[axis_index])
            responses.append(current_pose[axis_index])

    if not times:
        raise RuntimeError(f"No usable command/current pose samples found in {csv_path}")

    t0 = times[0]
    times = [t - t0 for t in times]
    return times, commands, responses


def summarize_tracking(times, commands, responses):
    errors = [u - y for u, y in zip(commands, responses)]
    rms_error = math.sqrt(sum(error * error for error in errors) / len(errors))
    peak_abs_error = max(abs(error) for error in errors)
    command_span = max(commands) - min(commands)
    response_span = max(responses) - min(responses)
    gain = response_span / command_span if abs(command_span) > 1e-12 else float("nan")
    return {
        "samples": len(times),
        "duration_sec": times[-1] - times[0],
        "command_span": command_span,
        "response_span": response_span,
        "span_gain": gain,
        "rms_error": rms_error,
        "peak_abs_error": peak_abs_error,
    }


def main():
    parser = argparse.ArgumentParser(description="Analyze a Y2SYS_ID Cartesian response CSV.")
    parser.add_argument("csv_path", type=Path)
    parser.add_argument("--axis", choices=sorted(AXIS_INDEX), default="z")
    args = parser.parse_args()

    times, commands, responses = load_series(args.csv_path, args.axis)
    summary = summarize_tracking(times, commands, responses)

    print(f"file: {args.csv_path}")
    print(f"axis: {args.axis}")
    for key, value in summary.items():
        if isinstance(value, float):
            print(f"{key}: {value:.9g}")
        else:
            print(f"{key}: {value}")


if __name__ == "__main__":
    main()
