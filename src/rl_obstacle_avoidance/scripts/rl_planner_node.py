#!/usr/bin/env python3
"""
@file: rl_planner_node.py
@brief: RL规划节点
@author: 陈鑫豪
@date: 2026-06-10
"""

import rospy
import math
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3Stamped
from rl_obstacle_avoidance.msg import ObstacleFeatures
from training_utils import (motion_frame_offset, motion_heading,
                            limit_change, quaternion_to_yaw,
                            rotate_sectors_to_heading)

class RlPlannerNode:
    """
    @brief: RL规划初始化节点
    """
    def __init__(self):
        rospy.init_node("rl_planner_node")

        self.step_xy = rospy.get_param("~step_xy", 1.5)           # 水平避障步长
        self.step_z = rospy.get_param("~step_z", 0.3)             # 垂直避障步长
        self.min_height = rospy.get_param("~min_height", 1.0)
        self.safe_distance_norm = rospy.get_param("~safe_distance_norm", 0.35)

        self.max_offset_xy = rospy.get_param("~max_offset_xy", 1.2)  # 最大水平偏移（m）
        self.min_offset_xy = rospy.get_param("~min_offset_xy", 0.3)  # 最小水平偏移（m）
        self.max_offset_z = rospy.get_param("~max_offset_z", 0.3)
        self.emergency_dist_norm = rospy.get_param("~emergency_dist_norm", 0.15)
        self.no_obstacle_threshold = rospy.get_param("~no_obstacle_threshold", 0.95)
        self.side_avoid_gain = rospy.get_param("~side_avoid_gain", 0.5)  # 侧方障碍避让增益（0~1）
        self.offset_smooth_alpha = rospy.get_param("~offset_smooth_alpha", 0.8)  # 偏移低通滤波系数（0~1，越大响应越快）
        self.max_offset_change = rospy.get_param("~max_offset_change", 0.25)
        self.heading_min_speed = rospy.get_param("~heading_min_speed", 0.1)
        
        self.current_pose = None
        self.current_velocity = None
        self.reference_velocity = None
        self.features = None
        self.motion_heading_world = 0.0

        # 方向保持状态：避免每次0.2s重算导致左右切换振荡
        self._last_avoid_direction = None  # 'left' | 'right' | None

        # 低通滤波状态：缓存的上一帧 body 偏移量
        self._last_dx_body = 0.0
        self._last_dy_body = 0.0
        self._last_dz = 0.0
        
        rospy.Subscriber("/obstacle_features", ObstacleFeatures, self.features_callback)
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.pose_callback)
        rospy.Subscriber("/mavros/local_position/velocity_local",
                         TwistStamped, self.velocity_callback)
        rospy.Subscriber("/lqr_tracker/reference_velocity",
                         Vector3Stamped, self.reference_velocity_callback)

        self.action_pub = rospy.Publisher("/rl_planner/action", Int32, queue_size=1)
        self.local_goal_pub = rospy.Publisher("/rl_planner/local_goal", PoseStamped, queue_size=1)
        self.local_offset_pub = rospy.Publisher("/rl_planner/local_offset", Vector3Stamped, queue_size=1)

        rospy.Timer(rospy.Duration(0.2), self.plan_once)

        rospy.loginfo("[rl_planner_node] RL规划节点初始化完成")
        rospy.spin()

    """
    @brief: 无人机位姿回调函数
    """
    def pose_callback(self, msg):
        self.current_pose = msg

    """
    @brief: 障碍物特征回调函数
    """
    def features_callback(self, msg):
        self.features = msg

    def velocity_callback(self, msg):
        self.current_velocity = msg.twist.linear

    def reference_velocity_callback(self, msg):
        self.reference_velocity = msg.vector

    def _motion_sectors(self):
        velocity = (self.reference_velocity
                    if self.reference_velocity is not None
                    else self.current_velocity)
        if velocity is not None:
            self.motion_heading_world = motion_heading(
                velocity.x, velocity.y, self.motion_heading_world,
                self.heading_min_speed)
        orientation = self.current_pose.pose.orientation
        body_yaw = quaternion_to_yaw(orientation.x, orientation.y,
                                     orientation.z, orientation.w)
        return rotate_sectors_to_heading(
            list(self.features.lidar_sectors), body_yaw,
            self.motion_heading_world)

    """
    @brief: 根据扇区数据直接计算轨迹运动坐标系逃逸偏移
    @detail: 分区域处理 — 前方障碍触发绕行，侧方障碍轻推远离，全360°覆盖
    """
    def compute_escape_offset(self):
        sector = self._motion_sectors()
        if not sector:
            return (0.0, 0.0, 0.0, 0)

        n = len(sector)
        sector_size = 2.0 * math.pi / n

        # 分区扫描：前 (±90°)、左 (90°~180°)、右 (-180°~-90°)，全360°覆盖
        # 基础轨迹负责前进，避障器只输出横向/高度修正，避免在障碍物前反复后退
        front_min = float('inf')
        front_min_idx = -1
        left_min = float('inf')
        right_min = float('inf')

        for i, dist in enumerate(sector):
            angle = (i + 0.5) * sector_size - math.pi
            if abs(angle) <= math.pi / 2.0:          # 前方 ±90°
                if dist < front_min:
                    front_min = dist
                    front_min_idx = i
            elif angle > math.pi / 2.0:               # 左侧 90°~180°，全左侧覆盖
                if dist < left_min:
                    left_min = dist
            elif angle < -math.pi / 2.0:              # 右侧 -180°~-90°，全右侧覆盖
                if dist < right_min:
                    right_min = dist

        action = 0

        # 前方紧急避让：保持绕行方向，增加上升，但不输出后退分量。
        if front_min < self.emergency_dist_norm:
            rospy.logwarn_throttle(2, "[rl_planner_node] 紧急避让！前方最近=%.3f", front_min)
            direction = self._choose_or_hold_direction(left_min, right_min)
            dy_body = self.max_offset_xy if direction == 'left' else -self.max_offset_xy
            dx_body = 0.0
            dz = self.step_z
            action = 3 if direction == 'left' else 4
        elif front_min < self.safe_distance_norm:
            direction = self._choose_or_hold_direction(left_min, right_min)
            severity = 1.0 - front_min / self.safe_distance_norm
            mag = self.min_offset_xy + severity * (self.max_offset_xy - self.min_offset_xy)
            dy_body = mag if direction == 'left' else -mag
            dx_body = 0.0
            dz = 0.0
            action = 1 if direction == 'left' else 2
        elif left_min < self.safe_distance_norm or right_min < self.safe_distance_norm:
            # 当前雷达扇区的前方可能落在左右区间；侧向障碍也按主避障强度处理。
            if self._last_avoid_direction is not None:
                obstacle_dist = min(left_min, right_min)
                direction = self._last_avoid_direction
            elif left_min <= right_min:
                obstacle_dist = left_min
                direction = 'right'
            else:
                obstacle_dist = right_min
                direction = 'left'

            severity = 1.0 - obstacle_dist / self.safe_distance_norm
            mag = self.min_offset_xy + severity * (self.max_offset_xy - self.min_offset_xy)
            mag *= self.side_avoid_gain
            dx_body = 0.0
            dy_body = mag if direction == 'left' else -mag
            dz = 0.0
            action = 1 if direction == 'left' else 2
        else:
            # 各方向均安全，重置方向保持状态
            self._last_avoid_direction = None
            return (0.0, 0.0, 0.0, 0)

        rospy.loginfo_throttle(
            1.0,
            "[rl_planner_node] front=%.3f left=%.3f right=%.3f offset_body=(%.2f, %.2f, %.2f)",
            front_min, left_min, right_min, dx_body, dy_body, dz)
        return (dx_body, dy_body, dz, action)

    """
    @brief: 选择或保持避障方向
    """
    def _choose_or_hold_direction(self, left_dist, right_dist):
        if self._last_avoid_direction is not None:
            return self._last_avoid_direction

        if left_dist >= right_dist:
            self._last_avoid_direction = 'left'
        else:
            self._last_avoid_direction = 'right'
        return self._last_avoid_direction

    """
    @brief: 计划一次
    """
    def plan_once(self, event):
        if self.current_pose is None or self.features is None:
            return

        if self.features.lidar_sectors is None or len(self.features.lidar_sectors) == 0:
            return

        dx_body, dy_body, dz, action = self.compute_escape_offset()

        # 安全限幅
        dx_body = max(-self.max_offset_xy, min(self.max_offset_xy, dx_body))
        dy_body = max(-self.max_offset_xy, min(self.max_offset_xy, dy_body))
        dz = max(-self.max_offset_z, min(self.max_offset_z, dz))

        # 低通滤波 + 变化率限制：避免偏移目标突然跨过基础轨迹
        a = self.offset_smooth_alpha
        dx_body = (1.0 - a) * self._last_dx_body + a * dx_body
        dy_body = (1.0 - a) * self._last_dy_body + a * dy_body
        dz = (1.0 - a) * self._last_dz + a * dz
        dx_body = limit_change(
            self._last_dx_body, dx_body, self.max_offset_change)
        dy_body = limit_change(
            self._last_dy_body, dy_body, self.max_offset_change)
        if abs(dx_body) < 1e-3:
            dx_body = 0.0
        if abs(dy_body) < 1e-3:
            dy_body = 0.0
        if abs(dz) < 1e-3:
            dz = 0.0
        self._last_dx_body = dx_body
        self._last_dy_body = dy_body
        self._last_dz = dz

        dx, dy = motion_frame_offset(
            dx_body, dy_body, self.motion_heading_world)

        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = "map"

        goal.pose.position.x = self.current_pose.pose.position.x + dx
        goal.pose.position.y = self.current_pose.pose.position.y + dy
        goal.pose.position.z = max(self.min_height, self.current_pose.pose.position.z + dz)
        goal.pose.orientation = self.current_pose.pose.orientation

        offset = Vector3Stamped()
        offset.header.stamp = rospy.Time.now()
        offset.header.frame_id = "map"
        offset.vector.x = dx
        offset.vector.y = dy
        offset.vector.z = dz

        self.action_pub.publish(Int32(data=action))
        self.local_goal_pub.publish(goal)
        self.local_offset_pub.publish(offset)

if __name__ == "__main__":
    try:
        RlPlannerNode()
    except rospy.ROSInterruptException:
        pass
