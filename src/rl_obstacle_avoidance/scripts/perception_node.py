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
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import PoseStamped, TwistStamped
from rl_obstacle_avoidance.msg import ObstacleFeatures

class PerceptionNode:
    """
    @brief: 视觉感知初始化节点
    """
    def __init__(self):
        rospy.init_node('perception_node', anonymous=True)

        # ========== 配置参数 ==========
        self.num_sector = rospy.get_param('~num_sector', 12)
        self.max_range = rospy.get_param('~max_range', 10.0)
        self.min_range = rospy.get_param('~min_range', 0.5)
        self.safe_height = rospy.get_param('~safe_height', 1.5)
        self.goal_tolerance = rospy.get_param('~goal_tolerance', 1.0)

        # ========== 状态变量 ==========
        self.current_pose = None
        self.current_vel = None
        self.goal_pose = None
        self.last_action = 0

        # 点云数据（用numpy数组存储）
        self.cloud_data = None

        # ========== 订阅话题 ==========
        rospy.Subscriber('/velodyne_points', PointCloud2, self.lidar_callback)
        rospy.Subscriber('mavros/local_position/pose', PoseStamped, self.pose_callback)
        rospy.Subscriber('mavros/local_position/velocity_local', TwistStamped, self.vel_callback)

        # ========== 发布话题 ==========
        self.feature_pub = rospy.Publisher('/obstacle_features', ObstacleFeatures, queue_size=1)

        # ========== 定时器（10hz） ==========
        rospy.Timer(rospy.Duration(0.1), self.publish_features)

        rospy.loginfo("[perception_node] 视觉感知节点初始化完成")
        rospy.spin()

    """
    @brief: 激光雷达点云回调函数
    """
    def lidar_callback(self, msg):
        try:
            # 将PointCloud2消息转换为numpy数组
            self.cloud_data = self.read_pointcloud(msg)
        except Exception as e:
            rospy.logerr("[perception_node] 点云数据转换错误: %s", e)

    """
    @brief: 读取点云数据
    @note: PointCloud2的格式通常是 [x, y, z, intensity] 或 [x, y, z]
    """
    def read_pointcloud(self, cloud_msg):
        fields = cloud_msg.fields

        x_idx = y_idx = z_idx = -1
        for i, field in enumerate(fields):
            if field.name == 'x':
                x_idx = i
            elif field.name == 'y':
                y_idx = i
            elif field.name == 'z':
                z_idx = i

        if x_idx == -1 or y_idx == -1 or z_idx == -1:
            rospy.logerr("[perception_node] 点云数据格式错误，缺少x, y, z字段")
            return None

        point_step = cloud_msg.point_step
        row_step = cloud_msg.row_step

        points = []

        data = np.frombuffer(cloud_msg.data, dtype=np.uint8)

        x_offset = fields[x_idx].offset
        y_offset = fields[y_idx].offset
        z_offset = fields[z_idx].offset

        num_points = cloud_msg.width * cloud_msg.height

        for i in range(num_points):
            offset = i * point_step

            x = np.float32(data[offset + x_offset:offset + x_offset + 4])
            y = np.float32(data[offset + y_offset:offset + y_offset + 4])
            z = np.float32(data[offset + z_offset:offset + z_offset + 4])

            points.append([x, y, z])
        
        return np.array(points)

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
        if points is None or len(points) == 0:
            return [self.max_range] * self.num_sector
        
        sectors = [self.max_range] * self.num_sector

        for point in points:
            x, y, z = point[0], point[1], point[2]

            dist_xy = math.sqrt(x**2 + y**2)

            if dist_xy < self.min_range or dist_xy > self.max_range:
                continue

            if abs(z) > self.safe_height:
                continue

            angle = math.atan2(x, y)

            sector_size = 2*math.pi / self.num_sector
            sector_idx = int((angle+math.pi) / sector_size)
            sector_idx = max(0, min(self.num_sector-1, sector_idx))

            sectors[sector_idx]=min(sectors[sector_idx], dist_xy)

        # 归一化到0-1之间
        sectors = [s / self.max_range for s in sectors]

        return sectors

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

        if self.cloud_data is not None:
            lidar_sectors = self.lidar_to_sector(self.cloud_data)
        else:
            lidar_sectors = [self.max_range / self.max_range] * self.num_sector

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