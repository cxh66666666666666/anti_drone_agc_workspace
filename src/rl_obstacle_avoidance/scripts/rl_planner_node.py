#!/usr/bin/env python3
"""
@file: rl_planner_node.py
@brief: RL规划节点
@author: 陈鑫豪
@date: 2026-06-10
"""

import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from rl_obstacle_avoidance.msg import ObstacleFeatures

class RlPlannerNode:
    """
    @brief: RL规划初始化节点
    """
    def __init__(self):
        rospy.init_node("rl_planner_node")

        self.step_xy = rospy.get_param("~step_xy", 1.0)
        self.step_z = rospy.get_param("~step_z", 0.5)
        self.min_height = rospy.get_param("~min_height", 1.0)
        self.safe_distance_norm = rospy.get_param("~safe_distance_norm", 0.3)
        
        self.current_pose = None
        self.features = None
        
        rospy.Subscriber("/obstacle_features", ObstacleFeatures, self.features_callback)
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.pose_callback)

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

    """
    @brief: 选择动作
    @detail: 暂时规则：
            1. 前方距离安全，直接前进（return 0）
            2. 前方太近，左边更空就左移（return 1）
            3. 左边也太近，右边更空就右移（return 2）
            4. 都危险，悬停（return 5）
    """
    def choose_action(self):
        sector = list(self.features.lidar_sectors)

        if not sector:
            return 5

        n = len(sector)

        front = sector[n//2]
        left = sector[3 * n // 4]
        right = sector[n // 4]

        if front > self.safe_distance_norm:
            return 0
        if left > right and left > self.safe_distance_norm:
            return 1
        if right > self.safe_distance_norm:
            return 2

        return 5
    
    """
    @brief: 动作转换为偏移量
    """
    def action_to_offset(self, action):
        if action == 0:
            return (0, 0, self.step_z)
        elif action == 1:
            return (-self.step_xy, 0, 0)
        elif action == 2:
            return (self.step_xy, 0, 0)
        elif action == 5:
            return (0, 0, 0)
        else:
            raise ValueError("[rl_planner_node] 无效的动作选择")

    """
    @brief: 计划一次
    """
    def plan_once(self, event):
        if self.current_pose is None or self.features is None:
            return
        
        action = self.choose_action()
        dx, dy, dz = self.action_to_offset(action)

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
