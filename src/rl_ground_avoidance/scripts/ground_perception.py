#!/usr/bin/env python3
"""地面车感知节点：由真实位姿 + 静态杆位解析计算 12 扇区最小距离（虚拟激光）。

发布 /ground_obstacle_sectors（Float32MultiArray，归一化 [0,1]）。
杆位通过 ~poles 参数注入（[[x,y],...]），默认与 ground.world 一致。
"""

import os
import sys

import rospy
from gazebo_msgs.msg import LinkStates
from std_msgs.msg import Float32MultiArray

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from ground_utils import virtual_sectors, yaw_from_quaternion


class GroundPerception:
    def __init__(self):
        rospy.init_node("ground_perception", anonymous=True)
        self.num_sectors = rospy.get_param("~num_sectors", 12)
        self.max_range = rospy.get_param("~lidar_max_range", 10.0)
        self.pole_radius = rospy.get_param("~pole_radius", 0.15)
        poles_param = rospy.get_param("~poles", [[6.0, 0.0],
                                                 [12.0, 0.5],
                                                 [18.0, -0.3]])
        self.poles = [(float(p[0]), float(p[1])) for p in poles_param]
        self.sector_pub = rospy.Publisher(
            "/ground_obstacle_sectors", Float32MultiArray, queue_size=5)
        self.pose = None
        self.link_sub = rospy.Subscriber(
            "/ground_gazebo/link_states", LinkStates,
            self._link_states_callback, queue_size=5)
        self.timer = rospy.Timer(rospy.Duration(0.1), self._publish_timer)

    def _link_states_callback(self, msg):
        try:
            index = msg.name.index("ground_car::base_link")
        except ValueError:
            return
        pose = msg.pose[index]
        orientation = pose.orientation
        self.pose = (pose.position.x, pose.position.y,
                     yaw_from_quaternion(orientation.x, orientation.y,
                                         orientation.z, orientation.w))

    def _publish_timer(self, _event):
        if self.pose is None:
            return
        sectors = virtual_sectors(
            self.pose[0], self.pose[1], self.pose[2], self.poles,
            num_sectors=self.num_sectors,
            max_range=self.max_range, pole_radius=self.pole_radius)
        self.sector_pub.publish(Float32MultiArray(data=sectors))


if __name__ == "__main__":
    GroundPerception()
    rospy.spin()
