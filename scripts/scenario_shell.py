#!/usr/bin/env python3
"""Emit safe bash assignments used by ``start_sim.sh --scenario``."""

import json
import shlex
import sys


def main(path):
    with open(path, encoding="utf-8") as stream:
        scenario = json.load(stream)
    groups = {"a": [], "b": []}
    for obstacle in scenario.get("obstacles", []):
        center = obstacle["center"]
        coords = ",".join(str(float(value)) for value in center)
        size = obstacle.get("size", [0.3, 0.3, 3.0])
        spec = "|".join([
            obstacle.get("type", "pillar_box"), coords,
            ",".join(str(float(value)) for value in size),
            str(float(obstacle.get("yaw", 0.0))),
        ])
        groups[obstacle.get("group", "a")].append((coords, spec))
    for name, key in (("A", "a"), ("B", "b")):
        coords = [item[0] for item in groups[key]]
        specs = [item[1] for item in groups[key]]
        variable = "OBSTACLES" if name == "A" else "OBSTACLES_B"
        print("%s=(%s)" %
              (variable, " ".join(shlex.quote(value) for value in coords)))
        print("OBSTACLE_SPECS_%s=(%s)" %
              (name, " ".join(shlex.quote(value) for value in specs)))
    trajectory = scenario.get("trajectory", {}).get("type")
    if trajectory:
        print("TRAJECTORY=%s" % shlex.quote(str(trajectory)))
    trajectory_config = scenario.get("trajectory", {})
    lqr_args = []

    def add(name, value):
        if value is not None:
            lqr_args.append("%s:=%s" % (name, value))

    add("trajectory_type", trajectory)
    add("trajectory_duration", trajectory_config.get("duration_s"))
    add("loop_trajectory", str(bool(
        trajectory_config.get("loop", True))).lower())
    if trajectory == "linear":
        start = trajectory_config.get("start", scenario.get("start"))
        end = trajectory_config.get("end", scenario.get("goal"))
        for axis, value in zip("xyz", start or []):
            add("start_" + axis, value)
        for axis, value in zip("xyz", end or []):
            add("end_" + axis, value)
    elif trajectory == "circular":
        center = trajectory_config.get("center")
        for axis, value in zip("xyz", center or []):
            add("center_" + axis, value)
        add("radius", trajectory_config.get("radius_m"))
        add("height", trajectory_config.get("height_m"))
        add("angular_velocity",
            trajectory_config.get("angular_velocity_rad_s"))
    elif trajectory == "fixed_horizontal":
        left = trajectory_config.get("left_center")
        right = trajectory_config.get("right_center")
        for axis, value in zip("xy", left or []):
            add("fixed_horizontal_left_" + axis, value)
        for axis, value in zip("xy", right or []):
            add("fixed_horizontal_right_" + axis, value)
        add("fixed_horizontal_radius", trajectory_config.get("radius_m"))
        add("fixed_horizontal_height", trajectory_config.get("height_m"))
        add("fixed_horizontal_angular_velocity",
            trajectory_config.get("angular_velocity_rad_s"))
    elif trajectory == "figure_eight":
        center = trajectory_config.get("center")
        for axis, value in zip("xy", center or []):
            add("figure_eight_center_" + axis, value)
        add("figure_eight_radius_x", trajectory_config.get("radius_x_m"))
        add("figure_eight_radius_y", trajectory_config.get("radius_y_m"))
        add("figure_eight_height", trajectory_config.get("height_m"))
        add("figure_eight_angular_velocity",
            trajectory_config.get("angular_velocity_rad_s"))
    print("SCENARIO_LQR_ARGS=(%s)" % " ".join(
        shlex.quote(value) for value in lqr_args))
    altitude = scenario.get("altitude")
    if altitude is not None:
        print("ALTITUDE=%s" % shlex.quote(str(float(altitude))))
    print("SCENARIO_ID=%s" % shlex.quote(str(scenario.get("scenario_id", "unknown"))))
    # 主动注入的传感器失效窗口（秒，episode 内时间）；无该字段时为空列表。
    blackout = [float(value) for value in (scenario.get("sensor_blackout_s") or [])]
    print("SENSOR_BLACKOUT=%s" % shlex.quote(json.dumps(blackout)))
    print("SCENARIO_OBSTACLES=%s" % shlex.quote(json.dumps(
        scenario.get("obstacles", []), ensure_ascii=False, separators=(",", ":"))))


if __name__ == "__main__":
    if len(sys.argv) != 2:
        raise SystemExit("usage: scenario_shell.py SCENARIO.json")
    main(sys.argv[1])
