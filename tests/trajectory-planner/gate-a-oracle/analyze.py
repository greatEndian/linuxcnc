#!/usr/bin/env python3
"""Independent analysis of a Gate A commanded-motion trace.

Two measurements of the same run are compared:

  reported  joint.N.{vel,acc,jerk}-cmd, what motion publishes
  derived   a 3-stage ddt chain on joint.N.pos-cmd, evaluated in the servo
            thread in double precision

For the circle case there is a third, analytic reference: at constant speed
on a circle of radius R the Cartesian acceleration is v^2/R and the jerk is
v^3/R^2, regardless of what the scalar path derivatives do.

Usage: analyze.py <trace.dat> --case straight|circle|tinyblocks [--period 0.001]
"""
import argparse
import math
import sys

import numpy as np

COLUMNS = [
    "tag", "line", "current_vel",
    "x_pos", "x_vel_rep", "x_acc_rep", "x_jerk_rep",
    "y_pos", "y_vel_rep", "y_acc_rep", "y_jerk_rep",
    "x_vel_der", "x_acc_der", "x_jerk_der",
    "y_vel_der", "y_acc_der", "y_jerk_der",
]


def load(path):
    rows, overruns = [], 0
    for raw in open(path):
        if "overrun" in raw:
            overruns += 1
            continue
        parts = raw.split()
        if len(parts) != len(COLUMNS):
            continue
        rows.append([float(value) for value in parts])
    if not rows:
        sys.exit("no usable samples in %s" % path)
    data = np.array(rows)
    columns = {name: data[:, index] for index, name in enumerate(COLUMNS)}
    gaps = int(np.sum(np.diff(columns["tag"]) != 1))
    return columns, overruns, gaps


def moving_window(columns, threshold=1e-6):
    moving = np.abs(columns["current_vel"]) > threshold
    if not moving.any():
        sys.exit("no motion found in trace")
    first, last = int(np.argmax(moving)), len(moving) - 1 - int(np.argmax(moving[::-1]))
    return first, last


def magnitude(columns, first, last, kind, source):
    x = columns["x_%s_%s" % (kind, source)][first:last + 1]
    y = columns["y_%s_%s" % (kind, source)][first:last + 1]
    return np.hypot(x, y)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("trace")
    parser.add_argument("--case", required=True,
                        choices=["straight", "circle", "tinyblocks"])
    parser.add_argument("--period", type=float, default=0.001)
    parser.add_argument("--radius", type=float, default=20.0)
    parser.add_argument("--label", default="")
    parser.add_argument("--expect-length", type=float, default=0.0,
                        help="commanded path length, for a period/closure check")
    args = parser.parse_args()

    columns, overruns, gaps = load(args.trace)
    first, last = moving_window(columns)
    samples = last - first + 1
    duration = samples * args.period

    speed = magnitude(columns, first, last, "vel", "der")
    path_length = float(np.sum(speed) * args.period)
    if args.expect_length:
        print("path closure        : %+.3f%% of %.3f mm commanded"
              % (100 * (path_length / args.expect_length - 1), args.expect_length))

    print("== %s ==" % (args.label or args.trace))
    print("samples in motion   : %d  (%.3f s at %.0f us)"
          % (samples, duration, args.period * 1e6))
    print("sampler overruns    : %d   tag gaps: %d" % (overruns, gaps))
    print("path length         : %.3f mm" % path_length)
    print("peak speed          : %.3f mm/s" % speed.max())

    for kind, limit in (("acc", None), ("jerk", None)):
        reported = magnitude(columns, first, last, kind, "rep")
        derived = magnitude(columns, first, last, kind, "der")
        print("peak |%-4s| reported : %10.3f      derived: %10.3f"
              % (kind, reported.max(), derived.max()))
        del limit

    if args.case == "circle":
        # Restrict to the arc blocks: the longest-running moving G-code lines.
        line = columns["line"][first:last + 1]
        durations = {}
        for value in np.unique(line):
            count = int(np.sum((line == value) & (speed > 1e-6)))
            if count > 200:
                durations[value] = count
        if not durations:
            sys.exit("no long moving block found")
        arc_lines = sorted(durations, key=durations.get, reverse=True)[:2]
        on_arc = np.isin(line, arc_lines)
        # Drop the first and last 10% of the arc to skip entry/exit ramps.
        index = np.where(on_arc)[0]
        margin = max(1, len(index) // 10)
        index = index[margin:-margin]
        arc_speed = speed[index]
        steady_mask = arc_speed > 0.999 * arc_speed.max()
        index = index[steady_mask]
        if len(index) < 50:
            sys.exit("no steady-speed portion found on the arc blocks")
        lo, hi = int(index[0]) + first, int(index[-1]) + first
        span = slice(lo, hi + 1)
        steady = np.zeros_like(speed, dtype=bool)
        steady[index] = True

        print("arc blocks          : lines %s" % ", ".join(
            "%d" % value for value in sorted(arc_lines)))
        x = columns["x_pos"][span]
        y = columns["y_pos"][span]
        # Fit the circle centre/radius from the commanded positions themselves.
        a_matrix = np.column_stack([x, y, np.ones_like(x)])
        b_vector = x ** 2 + y ** 2
        solution, *_ = np.linalg.lstsq(a_matrix, b_vector, rcond=None)
        cx, cy = solution[0] / 2.0, solution[1] / 2.0
        radius = math.sqrt(solution[2] + cx ** 2 + cy ** 2)

        v_steady = float(np.mean(speed[steady]))
        truth_acc = v_steady ** 2 / radius
        truth_jerk = v_steady ** 3 / radius ** 2
        acc_rep = magnitude(columns, lo, hi, "acc", "rep")
        acc_der = magnitude(columns, lo, hi, "acc", "der")
        jerk_rep = magnitude(columns, lo, hi, "jerk", "rep")
        jerk_der = magnitude(columns, lo, hi, "jerk", "der")

        print("-- steady-speed portion: %d samples (%.3f s) --"
              % (hi - lo + 1, (hi - lo + 1) * args.period))
        print("fitted radius       : %.6f mm   (commanded %.3f)" % (radius, args.radius))
        print("steady speed        : %.4f mm/s" % v_steady)
        print("ANALYTIC |a| = v^2/R : %10.4f mm/s^2" % truth_acc)
        print("  measured, derived  : %10.4f   (%+.2f%%)"
              % (acc_der.mean(), 100 * (acc_der.mean() / truth_acc - 1)))
        print("  measured, reported : %10.4f   (%+.2f%%)"
              % (acc_rep.mean(), 100 * (acc_rep.mean() / truth_acc - 1)))
        print("ANALYTIC |j| = v^3/R^2: %10.4f mm/s^3" % truth_jerk)
        print("  measured, derived  : %10.4f   (%+.2f%%)"
              % (jerk_der.mean(), 100 * (jerk_der.mean() / truth_jerk - 1)))
        print("  measured, reported : %10.4f   (%+.2f%%)"
              % (jerk_rep.mean(), 100 * (jerk_rep.mean() / truth_jerk - 1)))
        print("REPORTED JERK ERROR  : %.4f mm/s^3 missing (%.1f%% of truth)"
              % (truth_jerk - jerk_rep.mean(),
                 100 * (truth_jerk - jerk_rep.mean()) / truth_jerk))
    print()


if __name__ == "__main__":
    main()
