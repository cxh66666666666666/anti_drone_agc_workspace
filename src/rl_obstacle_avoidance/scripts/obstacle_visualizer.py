#!/usr/bin/env python3
"""Publish configured Gazebo obstacles as RViz markers."""

import json
import math

import rospy
from visualization_msgs.msg import Marker, MarkerArray


def make_marker(obstacle, marker_id, namespace, color):
    marker = Marker()
    marker.header.frame_id = "map"
    marker.ns = namespace
    marker.id = marker_id
    if isinstance(obstacle, dict):
        center = obstacle["center"]
        size = obstacle.get("size", [0.3, 0.3, 3.0])
        obstacle_type = obstacle.get("type", "pillar_box")
        yaw = float(obstacle.get("yaw", 0.0))
    else:
        center = obstacle
        size = [0.3, 0.3, 3.0]
        obstacle_type, yaw = "pillar_box", 0.0
    marker.type = Marker.CYLINDER if obstacle_type == "cylinder" else Marker.CUBE
    marker.action = Marker.ADD
    marker.pose.position.x = float(center[0])
    marker.pose.position.y = float(center[1])
    marker.pose.position.z = float(center[2])
    marker.pose.orientation.z = math.sin(yaw / 2.0)
    marker.pose.orientation.w = math.cos(yaw / 2.0)
    marker.scale.x = float(size[0])
    marker.scale.y = float(size[1]) if obstacle_type != "cylinder" else float(size[0])
    marker.scale.z = float(size[2])
    marker.color.r, marker.color.g, marker.color.b = color
    marker.color.a = 0.9
    return marker


def _as_obstacles(raw):
    """Normalize a ROS parameter containing typed obstacle JSON."""
    if isinstance(raw, str):
        try:
            raw = json.loads(raw)
        except (TypeError, ValueError):
            return []
    return raw if isinstance(raw, list) else []


def _load_obstacles():
    """Read the current scenario, including runtime scene rotations."""
    typed = _as_obstacles(rospy.get_param(
        "/dqn_training/scenario_obstacles_json", ""))
    if typed:
        return ([item for item in typed if item.get("group", "a") == "a"],
                [item for item in typed if item.get("group", "a") == "b"])
    return (_as_obstacles(rospy.get_param("/dqn_training/obstacles_a", [])),
            _as_obstacles(rospy.get_param("/dqn_training/obstacles_b", [])))


def _marker_array(obstacles_a, obstacles_b):
    """Build a full replacement array; DELETEALL removes stale RViz markers."""
    marker_array = MarkerArray()
    clear_marker = Marker()
    clear_marker.header.frame_id = "map"
    clear_marker.action = Marker.DELETEALL
    marker_array.markers.append(clear_marker)
    marker_array.markers.extend(
        make_marker(obstacle, index, "obstacles_a", (1.0, 0.0, 0.0))
        for index, obstacle in enumerate(obstacles_a))
    marker_array.markers.extend(
        make_marker(obstacle, index, "obstacles_b", (0.0, 0.2, 1.0))
        for index, obstacle in enumerate(obstacles_b))
    stamp = rospy.Time.now()
    for marker in marker_array.markers:
        marker.header.stamp = stamp
    return marker_array


def main():
    rospy.init_node("obstacle_visualizer")
    publisher = rospy.Publisher(
        "/dqn_training/obstacle_markers", MarkerArray,
        queue_size=1, latch=True)
    last_signature = None
    rate = rospy.Rate(2)
    while not rospy.is_shutdown():
        obstacles_a, obstacles_b = _load_obstacles()
        signature = json.dumps([obstacles_a, obstacles_b],
                               ensure_ascii=False, sort_keys=True,
                               separators=(",", ":"))
        if signature != last_signature:
            publisher.publish(_marker_array(obstacles_a, obstacles_b))
            last_signature = signature
            rospy.loginfo(
                "[obstacle_visualizer] 已更新 RViz 障碍物: a=%d, b=%d",
                len(obstacles_a), len(obstacles_b))
        rate.sleep()


if __name__ == "__main__":
    main()
