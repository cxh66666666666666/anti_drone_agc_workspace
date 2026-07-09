#!/usr/bin/env python3
"""
@file: experiment_logger_node.py
@brief: 实验数据记录节点 — 订阅关键话题，定时写入 CSV，不改控制链路
@author: trae
@date: 2026-07-08
"""

import os
import math
import csv
from datetime import datetime

import rospy
from std_msgs.msg import Int32, Float64
from geometry_msgs.msg import PoseStamped, Vector3Stamped


class ExperimentLoggerNode:
    """实验数据记录节点：订阅 5 个话题，按固定频率写入 CSV"""

    # CSV 列定义（与 spec 字段一一对应）
    CSV_HEADER = [
        "timestamp",
        "pose_x", "pose_y", "pose_z",
        "pose_qx", "pose_qy", "pose_qz", "pose_qw",
        "offset_x", "offset_y", "offset_z",
        "action",
        "position_error",
        "adjusted_ref_x", "adjusted_ref_y", "adjusted_ref_z",
        "is_avoiding",
        "offset_magnitude_xy",
    ]

    def __init__(self):
        rospy.init_node("experiment_logger_node")

        # 参数
        output_dir = rospy.get_param("~output_dir", os.path.expanduser("~/anti_drone_agc_workspace/experiment_data"))
        record_rate = rospy.get_param("~record_rate", 10.0)

        # 创建输出目录
        os.makedirs(output_dir, exist_ok=True)

        # 生成带时间戳的文件名
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        self._csv_path = os.path.join(output_dir, f"experiment_{ts}.csv")

        # 打开文件并写表头
        self._csv_file = open(self._csv_path, "w", newline="")
        self._csv_writer = csv.writer(self._csv_file)
        self._csv_writer.writerow(self.CSV_HEADER)
        self._csv_file.flush()

        rospy.loginfo("[experiment_logger] CSV 文件: %s", self._csv_path)

        # 缓存最新消息，初始 None（未收到时填 NaN）
        self._latest_pose = None       # PoseStamped
        self._latest_offset = None     # Vector3Stamped
        self._latest_action = None     # Int32
        self._latest_pos_err = None    # Float64
        self._latest_adj_ref = None    # PoseStamped

        # 订阅 5 个话题
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped,
                         self._pose_cb, queue_size=1)
        rospy.Subscriber("/rl_planner/local_offset", Vector3Stamped,
                         self._offset_cb, queue_size=1)
        rospy.Subscriber("/rl_planner/action", Int32,
                         self._action_cb, queue_size=1)
        rospy.Subscriber("/lqr_tracker/position_error", Float64,
                         self._pos_err_cb, queue_size=1)
        rospy.Subscriber("/lqr_tracker/adjusted_ref", PoseStamped,
                         self._adj_ref_cb, queue_size=1)

        # 定时器：按 record_rate 写入一行
        self._timer = rospy.Timer(
            rospy.Duration(1.0 / record_rate), self._write_row)

        rospy.loginfo("[experiment_logger] 实验记录节点启动完成, rate=%.1f Hz",
                      record_rate)

    # ---- 回调：只缓存最新消息 ----

    def _pose_cb(self, msg):
        self._latest_pose = msg

    def _offset_cb(self, msg):
        self._latest_offset = msg

    def _action_cb(self, msg):
        self._latest_action = msg

    def _pos_err_cb(self, msg):
        self._latest_pos_err = msg

    def _adj_ref_cb(self, msg):
        self._latest_adj_ref = msg

    # ---- 定时写入 ----

    def _write_row(self, event):
        ts = rospy.Time.now().to_sec()

        # 从缓存取值，缺失填 NaN
        pose = self._latest_pose
        offset = self._latest_offset
        action = self._latest_action
        pos_err = self._latest_pos_err
        adj_ref = self._latest_adj_ref

        px = pose.pose.position.x if pose else float("nan")
        py = pose.pose.position.y if pose else float("nan")
        pz = pose.pose.position.z if pose else float("nan")
        qx = pose.pose.orientation.x if pose else float("nan")
        qy = pose.pose.orientation.y if pose else float("nan")
        qz = pose.pose.orientation.z if pose else float("nan")
        qw = pose.pose.orientation.w if pose else float("nan")

        ox = offset.vector.x if offset else float("nan")
        oy = offset.vector.y if offset else float("nan")
        oz = offset.vector.z if offset else float("nan")

        act = action.data if action else float("nan")

        pe = pos_err.data if pos_err else float("nan")

        arx = adj_ref.pose.position.x if adj_ref else float("nan")
        ary = adj_ref.pose.position.y if adj_ref else float("nan")
        arz = adj_ref.pose.position.z if adj_ref else float("nan")

        # 衍生字段
        is_avoiding = 1 if (action is not None and action.data != 0) else 0
        offset_mag = math.sqrt(ox * ox + oy * oy) if offset else float("nan")

        row = [ts,
               px, py, pz, qx, qy, qz, qw,
               ox, oy, oz,
               act,
               pe,
               arx, ary, arz,
               is_avoiding, offset_mag]
        self._csv_writer.writerow(row)
        self._csv_file.flush()

    def shutdown(self):
        """节点关闭时关闭 CSV 文件"""
        if self._csv_file and not self._csv_file.closed:
            self._csv_file.close()
            rospy.loginfo("[experiment_logger] CSV 文件已关闭: %s", self._csv_path)


if __name__ == "__main__":
    node = ExperimentLoggerNode()
    rospy.on_shutdown(node.shutdown)
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
