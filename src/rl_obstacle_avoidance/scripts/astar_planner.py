#!/usr/bin/env python3
"""A* 全局路径规划节点。

- 障碍物 a 列表从 /dqn_training/obstacles_a rosparam 读取（[[x,y],...]）
- 服务 /astar_planner/plan（rl_obstacle_avoidance/PlanPath）：
  输入 start/goal/safety_margin → 输出平滑路径（Pose2D[]，theta 为切向角）
"""

import math
import os
import sys

import rospy
from geometry_msgs.msg import Pose2D
from rl_obstacle_avoidance.srv import PlanPath, PlanPathResponse

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from path_planning import (anchor_path_start, astar_path, build_grid,
                           build_typed_grid, is_goal_valid, nearest_free_point,
                           smooth_path, typed_goal_valid, world_to_grid)


class AstarPlanner:
    def __init__(self):
        rospy.init_node("astar_planner")
        self.bounds = rospy.get_param(
            "~bounds", [-20.0, 20.0, -15.0, 15.0])
        self.resolution = rospy.get_param("~resolution", 0.5)
        self.pole_radius = rospy.get_param("~pole_radius", 0.15)
        self.default_margin = rospy.get_param("~default_margin", 1.0)
        self.goal_min_clearance = rospy.get_param(
            "~goal_min_clearance", 1.5)
        obstacles_param = rospy.get_param(
            "/dqn_training/obstacles_a", [])
        typed_json = rospy.get_param("/dqn_training/scenario_obstacles_json", "")
        self.typed_obstacles = []
        if typed_json:
            try:
                import json
                typed = (json.loads(typed_json)
                         if isinstance(typed_json, str) else typed_json)
                self.typed_obstacles = [item for item in typed
                                        if item.get("group", "a") == "a"]
            except (TypeError, ValueError):
                rospy.logwarn("[astar] typed scenario obstacles 解析失败，回退中心点")
        self.obstacles_a = ([(float(item["center"][0]), float(item["center"][1]))
                             for item in self.typed_obstacles]
                            if self.typed_obstacles else
                            [(float(p[0]), float(p[1])) for p in obstacles_param])
        self.plan_srv = rospy.Service(
            "/astar_planner/plan", PlanPath, self._plan_callback)
        rospy.loginfo("[astar] 规划器就绪 | a 柱数=%d | bounds=%s | 分辨率=%.2f",
                      len(self.obstacles_a), self.bounds, self.resolution)

    def plan(self, start, goal, safety_margin):
        inflation = (self.pole_radius + safety_margin
                     if not self.typed_obstacles else safety_margin)
        blocked = (build_typed_grid(self.typed_obstacles, self.bounds,
                                    self.resolution, inflation)
                   if self.typed_obstacles else
                   build_grid(self.obstacles_a, self.bounds, self.resolution,
                              inflation))
        planning_start = nearest_free_point(
            blocked, start, self.bounds, self.resolution)
        if planning_start is None:
            return []
        path = astar_path(blocked, planning_start, goal,
                          self.bounds, self.resolution)
        if not path:
            return []
        path = smooth_path(path, blocked, self.bounds, self.resolution)
        if world_to_grid(start[0], start[1], self.bounds,
                         self.resolution) in blocked:
            rospy.logwarn("[astar] 起点位于膨胀障碍区，先恢复到自由点 (%.1f, %.1f)",
                          planning_start[0], planning_start[1])
        return anchor_path_start(path, start)

    def _plan_callback(self, request):
        response = PlanPathResponse()
        start = (request.start_x, request.start_y)
        goal = (request.goal_x, request.goal_y)
        goal_ok = (typed_goal_valid(goal, self.typed_obstacles,
                                    self.goal_min_clearance)
                   if self.typed_obstacles else
                   is_goal_valid(goal, self.obstacles_a,
                                 self.goal_min_clearance))
        if not goal_ok:
            rospy.logwarn("[astar] 目标点离障碍过近: (%.1f, %.1f)", *goal)
            return response
        path = self.plan(start, goal, request.safety_margin)
        if not path:
            rospy.logwarn("[astar] 无可达路径: (%.1f,%.1f) -> (%.1f,%.1f)",
                          start[0], start[1], goal[0], goal[1])
            return response
        for index, (x, y) in enumerate(path):
            pose = Pose2D()
            pose.x = x
            pose.y = y
            if index < len(path) - 1:
                pose.theta = math.atan2(path[index + 1][1] - y,
                                        path[index + 1][0] - x)
            elif path:
                pose.theta = response.path[-1].theta
            response.path.append(pose)
        return response


if __name__ == "__main__":
    planner = AstarPlanner()
    rospy.spin()
