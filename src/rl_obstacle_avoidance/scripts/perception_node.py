#!/usr/bin/env python3
"""
@file: perception_node.py
@brief: 视觉感知节点
@author: 陈鑫豪
@date: 2026-06-08
"""

import rospy
import numpy as np
import math
import os
import sys
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from geometry_msgs.msg import PoseStamped, TwistStamped
from rl_obstacle_avoidance.msg import ObstacleFeatures

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from perception_utils import points_to_sectors

class PerceptionNode:
    """
    @brief: 视觉感知初始化节点
    """
    def __init__(self):
        rospy.init_node('perception_node', anonymous=True)

        # ========== 配置参数 ==========
        self.num_sector = rospy.get_param('~num_sector', 12)
        self.max_range = rospy.get_param('~max_range', 10.0)
        # Reject returns immediately around the vehicle while retaining the
        # full collision-risk band starting at 0.6 m.
        self.min_range = rospy.get_param('~min_range', 0.4)
        self.safe_height = rospy.get_param('~safe_height', 1.5)
        self.goal_tolerance = rospy.get_param('~goal_tolerance', 1.0)

        # ========== 状态变量 ==========
        self.current_pose = None
        self.current_vel = None
        self.goal_pose = None
        self.last_action = 0

        self.latest_sectors = None
        self.cloud_header = None

        # ========== 订阅话题 ==========
        rospy.Subscriber('/velodyne_points', PointCloud2, self.lidar_callback,
                         queue_size=1, buff_size=2 ** 24)
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped,
                         self.pose_callback, queue_size=1)
        rospy.Subscriber('/mavros/local_position/velocity_local', TwistStamped,
                         self.vel_callback, queue_size=1)

        # ========== 发布话题 ==========
        self.feature_pub = rospy.Publisher('/obstacle_features', ObstacleFeatures, queue_size=1)

        rospy.loginfo("[perception_node] 点云感知节点初始化完成")
        rospy.spin()

    """
    @brief: 激光雷达点云回调函数
    """
    def lidar_callback(self, msg):
        try:
            points = self.read_pointcloud(msg)
            self.latest_sectors = points_to_sectors(
                points, self.num_sector, self.min_range,
                self.max_range, self.safe_height)
            self.cloud_header = msg.header
            self.publish_features()
        except Exception as e:
            rospy.logerr("[perception_node] 点云数据转换错误: %s", e)

    """
    @brief: 读取点云数据
    @note: PointCloud2的格式通常是 [x, y, z, intensity] 或 [x, y, z]
    """
    def read_pointcloud(self, cloud_msg):
        gen = point_cloud2.read_points(cloud_msg, field_names=('x', 'y', 'z'), skip_nans=True)
        flat = np.fromiter((value for point in gen for value in point),
                           dtype=np.float32)
        if flat.size == 0:
            return np.empty((0, 3), dtype=np.float32)
        return flat.reshape((-1, 3))

    """
    @brief: 无人机位姿回调函数
    """
    def pose_callback(self, msg):
        if self.current_pose is None:
            rospy.loginfo("[perception_node] 首次收到无人机位姿数据，开始发布 /obstacle_features")
        self.current_pose = msg.pose
    
    """
    @brief: 无人机速度回调函数
    """
    def vel_callback(self, msg):
        self.current_vel = msg.twist.linear

    """
    @brief: 激光雷达点转换为扇区
    @note: 核心算法
    """
    def lidar_to_sector(self, points):
        return points_to_sectors(
            points, self.num_sector, self.min_range,
            self.max_range, self.safe_height)

    """
    @brief: 计算相对于目标的方向和距离
    """
    def compute_target_info(self):
        if self.current_pose is None or self.goal_pose is None:
            return [0.0, 0.0, 0.0], 1.0
        
        curr_x = self.current_pose.position.x
        curr_y = self.current_pose.position.y
        curr_z = self.current_pose.position.z

        goal_x = self.goal_pose.position.x
        goal_y = self.goal_pose.position.y
        goal_z = self.goal_pose.position.z

        dx = goal_x - curr_x
        dy = goal_y - curr_y
        dz = goal_z - curr_z
        
        dist = math.sqrt(dx**2 + dy**2 + dz**2)

        if dist > 0.01:
            dir_x = dx / dist
            dir_y = dy / dist
            dir_z = dz / dist
        else:
            dir_x = 0.0
            dir_y = 0.0
            dir_z = 0.0
        
        # 归一化距离到0-1之间
        # 最大探索距离暂设为20m
        max_dist = 20.0
        dist = min(dist/max_dist, 1.0)
        
        return [dir_x, dir_y, dir_z], dist

    """
    @brief: 发布障碍物特征
    """
    def publish_features(self, event=None):
        if self.current_pose is None:
            rospy.logwarn_throttle(5, "[perception_node] 等待无人机位姿数据... (sim time 时钟可能未启动)")
            return

        if self.latest_sectors is None or self.cloud_header is None:
            return
        lidar_sectors = self.latest_sectors

        target_dir, target_dist = self.compute_target_info()

        if self.current_vel:
            vel = [self.current_vel.x, self.current_vel.y, self.current_vel.z]
            # 归一化速度到0-1之间
            # 最大速度暂设为5m/s
            max_vel = 5.0
            vel = [v / max_vel for v in vel]
        else:
            vel = [0.0, 0.0, 0.0]

        msg = ObstacleFeatures()
        msg.header = self.cloud_header
        msg.lidar_sectors = lidar_sectors
        msg.target_dir_x = target_dir[0]
        msg.target_dir_y = target_dir[1]
        msg.target_dir_z = target_dir[2]
        msg.target_dist_norm = target_dist
        msg.vel_x = vel[0]
        msg.vel_y = vel[1]
        msg.vel_z = vel[2]
        msg.last_action = self.last_action

        self.feature_pub.publish(msg)
        rospy.loginfo_throttle(10, "[perception_node] 已发布 /obstacle_features (sectors=%d)" % len(msg.lidar_sectors))

if __name__ == '__main__':
    try:
        perception_node = PerceptionNode()
    except rospy.ROSInterruptException:
        pass
