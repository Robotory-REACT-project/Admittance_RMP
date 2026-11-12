#!/usr/bin/env python3

import argparse
import math
from pathlib import Path
from typing import List, Tuple, Union


SIGMA_PLOT_CUTOFF_MM = 100.0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Plot synchronized FT and per-sensor proximity logs."
    )
    parser.add_argument(
        "session_directory",
        nargs="?",
        type=Path,
        help="Session directory. If omitted, the newest session under --log-root is used.",
    )
    parser.add_argument(
        "--log-root",
        type=Path,
        default=(
            Path.home()
            / "ros2_ws"
            / "src"
            / "REACT_intrinsic"
            / "sensor_data_logs"
        ),
        help="Root containing REACT_intrinsic session_* directories.",
    )
    parser.add_argument(
        "--show",
        action="store_true",
        help="Open the two figures after saving them.",
    )
    return parser.parse_args()


def find_session(session_directory: Path | None, log_root: Path) -> Path:
    if session_directory is not None:
        session = session_directory.expanduser().resolve()
        if not session.is_dir():
            raise FileNotFoundError(f"Session directory does not exist: {session}")
        return session

    root = log_root.expanduser().resolve()
    candidates = [path for path in root.glob("session_*") if path.is_dir()]
    if not candidates:
        raise FileNotFoundError(f"No session directories found under: {root}")
    return max(candidates, key=lambda path: path.stat().st_mtime)


def read_numeric_rows(
    path: Path, expected_columns: Union[int, Tuple[int, ...]]
) -> List[List[float]]:
    if not path.is_file():
        raise FileNotFoundError(f"Log file does not exist: {path}")

    allowed_columns = (
        (expected_columns,)
        if isinstance(expected_columns, int)
        else expected_columns
    )
    rows: List[List[float]] = []
    with path.open("r", encoding="utf-8") as stream:
        for line_number, line in enumerate(stream, start=1):
            stripped = line.strip()
            if not stripped or stripped.startswith("#"):
                continue
            values = stripped.split()
            if len(values) not in allowed_columns:
                expected = " or ".join(str(value) for value in allowed_columns)
                raise ValueError(
                    f"{path}:{line_number}: expected {expected} columns, "
                    f"found {len(values)}"
                )
            rows.append([float(value) for value in values])
    return rows


def plot_ft(session: Path, show: bool) -> Path | None:
    import matplotlib.pyplot as plt

    ft_path = session / "ft_sensor1.txt"
    if not ft_path.is_file():
        # Backward compatibility for sessions recorded before multi-FT logging.
        ft_path = session / "ft_sensor.txt"
    rows = read_numeric_rows(ft_path, 8)
    if not rows:
        print("Primary FT log has no samples; FT plot was not created.")
        return None

    elapsed = [row[0] for row in rows]
    labels = ("X", "Y", "Z")
    colors = ("tab:red", "tab:green", "tab:blue")

    figure, axes = plt.subplots(2, 1, sharex=True, figsize=(11, 7))
    for axis_index, (label, color) in enumerate(zip(labels, colors)):
        axes[0].plot(
            elapsed,
            [row[2 + axis_index] for row in rows],
            label=f"Force {label}",
            color=color,
            linewidth=0.9,
        )
        axes[1].plot(
            elapsed,
            [row[5 + axis_index] for row in rows],
            label=f"Torque {label}",
            color=color,
            linewidth=0.9,
        )

    axes[0].set_ylabel("Force [N]")
    axes[1].set_ylabel("Torque [N·m]")
    axes[1].set_xlabel("Elapsed time [s]")
    for axis in axes:
        axis.grid(True, alpha=0.3)
        axis.legend(loc="upper right")
    figure.suptitle("Force/Torque Sensor 1 (Contact Sensing)")
    figure.tight_layout()

    output_path = session / "ft_sensor_plot.png"
    figure.savefig(output_path, dpi=160)
    if not show:
        plt.close(figure)
    return output_path


def plot_proximity(session: Path, show: bool) -> Path | None:
    import matplotlib.pyplot as plt

    sensor_data = {}
    for sensor_id in (1, 2):
        path = session / f"proximity_sensor{sensor_id}.txt"
        rows = read_numeric_rows(path, (7, 8))
        if rows:
            # Legacy logs have no cap-sigma column. Insert NaN so all rows use
            # the new eight-column layout without breaking old sessions.
            sensor_data[sensor_id] = [
                row if len(row) == 8 else row[:5] + [math.nan] + row[5:]
                for row in rows
            ]
    if not sensor_data:
        print("Proximity logs have no synchronized samples; plot was not created.")
        return None

    figure, axes = plt.subplots(5, 1, sharex=True, figsize=(12, 12))
    colors = {1: "#e69f00", 2: "#0072b2"}
    fields = (
        (3, "Filtered raw [count]"),
        (4, "ToF [m]"),
        (5, "Cap sigma [mm] (<100)"),
        (6, "Cap distance [m]"),
        (7, "Fused distance [m]"),
    )
    for sensor_id, rows in sensor_data.items():
        elapsed = [row[0] for row in rows]
        for axis, (column, ylabel) in zip(axes, fields):
            values = [row[column] for row in rows]
            if column == 5:
                excluded = sum(
                    math.isfinite(value) and value >= SIGMA_PLOT_CUTOFF_MM
                    for value in values
                )
                values = [
                    value
                    if value < SIGMA_PLOT_CUTOFF_MM
                    else math.nan
                    for value in values
                ]
                if excluded:
                    print(
                        f"Sensor {sensor_id}: excluded {excluded} cap-sigma "
                        f"samples >= {SIGMA_PLOT_CUTOFF_MM:g} mm from the plot."
                    )
            if any(math.isfinite(value) for value in values):
                axis.plot(
                    elapsed,
                    values,
                    label=f"Sensor {sensor_id}",
                    color=colors[sensor_id],
                    linewidth=0.9,
                )
            axis.set_ylabel(ylabel)
    axes[-1].set_xlabel("Elapsed time [s]")
    for axis in axes:
        axis.grid(True, alpha=0.3)
        if axis.lines:
            axis.legend(loc="upper right")
    figure.suptitle("Synchronized Proximity Sensor Topics")
    figure.tight_layout()

    output_path = session / "proximity_sensor_plot.png"
    figure.savefig(output_path, dpi=160)
    if not show:
        plt.close(figure)
    return output_path


def main() -> int:
    args = parse_args()
    session = find_session(args.session_directory, args.log_root)
    print(f"Using sensor log session: {session}")

    outputs = [plot_ft(session, args.show), plot_proximity(session, args.show)]
    for output in outputs:
        if output is not None:
            print(f"Saved: {output}")

    if args.show:
        import matplotlib.pyplot as plt

        plt.show()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
