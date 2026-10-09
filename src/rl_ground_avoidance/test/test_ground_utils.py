#!/usr/bin/env python3

import math
import os
import sys
import unittest


SCRIPT_DIR = os.path.join(os.path.dirname(os.path.dirname(__file__)), "scripts")
sys.path.insert(0, SCRIPT_DIR)

from ground_utils import (  # noqa: E402
    diff_drive_command,
    forward_clearance,
    integrate_pose,
    minimum_clearance,
    reference_target,
    sectors_from_scan,
    tracking_error,
    virtual_sectors,
    wrap_angle,
    yaw_from_quaternion,
)


class GroundUtilsTest(unittest.TestCase):
    def test_sectors_from_scan_bins_ranges(self):
        # 720 样本、360 度：第 0 扇区覆盖 -180..-150 度（样本 0..59）
        ranges = [11.0] * 720    # 超量程 → 过滤，扇区默认 1.0
        ranges[0] = 2.0        # 正后方，扇区 0
        ranges[360] = 1.0      # 正前方（180 度），扇区 6
        sectors = sectors_from_scan(ranges, -math.pi, math.pi / 360.0)
        self.assertEqual(len(sectors), 12)
        self.assertAlmostEqual(sectors[0], 2.0 / 10.0)
        self.assertAlmostEqual(sectors[6], 1.0 / 10.0)
        self.assertAlmostEqual(sectors[3], 1.0)

    def test_sectors_from_scan_filters_invalid_ranges(self):
        ranges = [0.05] * 720      # 低于 min_range：视为无效
        sectors = sectors_from_scan(ranges, -math.pi, math.pi / 360.0)
        self.assertEqual(sectors, [1.0] * 12)

    def test_sector_minimum_is_taken(self):
        ranges = [8.0] * 720
        ranges[0] = 2.0
        ranges[10] = 1.0
        sectors = sectors_from_scan(ranges, -math.pi, math.pi / 360.0)
        self.assertAlmostEqual(sectors[0], 0.1)

    def test_reference_target_moves_with_speed(self):
        self.assertEqual(reference_target(0.0, 1.0, 0.0, 0.0), (0.0, 0.0))
        x, y = reference_target(10.0, 1.0, 0.3, -0.2)
        self.assertAlmostEqual(x, 10.3)
        self.assertAlmostEqual(y, -0.2)

    def test_diff_drive_command_steers_toward_target(self):
        linear, angular = diff_drive_command(
            0.0, 0.0, 0.0, 1.0, 0.0, base_speed=1.0)
        self.assertAlmostEqual(angular, 0.0)
        self.assertAlmostEqual(linear, 1.0)
        linear, angular = diff_drive_command(
            0.0, 0.0, 0.0, 1.0, 1.0, base_speed=1.0)
        self.assertGreater(angular, 0.0)   # 目标在左侧 → 左转
        self.assertLess(linear, 1.0)       # 大角度转向时减速
        linear, angular = diff_drive_command(
            0.0, 0.0, math.pi / 2.0, 10.0, 0.0, base_speed=1.0)
        self.assertLess(angular, 0.0)      # 车头朝 +y，目标在 +x → 右转

    def test_tracking_error_is_euclidean(self):
        self.assertAlmostEqual(tracking_error(0.0, 0.0, 3.0, 4.0), 5.0)

    def test_wrap_angle(self):
        self.assertAlmostEqual(wrap_angle(3.5 * math.pi), -0.5 * math.pi,
                               places=6)
        self.assertAlmostEqual(wrap_angle(-3.5 * math.pi), 0.5 * math.pi,
                               places=6)

    def test_forward_clearance_ignores_rear(self):
        sectors = [1.0] * 12
        sectors[0] = 0.1   # 正后方
        self.assertEqual(forward_clearance(sectors), 1.0)
        sectors[6] = 0.2   # 正前方
        self.assertEqual(forward_clearance(sectors), 0.2)

    def test_minimum_clearance(self):
        sectors = [0.8, 0.2, 0.5]
        self.assertEqual(minimum_clearance(sectors), 0.2)
        self.assertEqual(minimum_clearance([]), 1.0)

    def test_virtual_sectors_places_pole_in_forward_sector(self):
        # 车在原点朝向 +x，杆在正前方 6m：表面距离 5.85 → 扇区 5/6 边界
        sectors = virtual_sectors(0.0, 0.0, 0.0, [(6.0, 0.0)])
        self.assertAlmostEqual(minimum_clearance(sectors), (6.0 - 0.15) / 10.0)
        self.assertEqual(forward_clearance(sectors), minimum_clearance(sectors))

    def test_virtual_sectors_bearing_depends_on_yaw(self):
        # 车朝向 +y：杆在正后方（世界 -y），不应出现在前向锥内
        sectors = virtual_sectors(0.0, 0.0, math.pi / 2.0, [(0.0, -6.0)])
        self.assertEqual(minimum_clearance(sectors), (6.0 - 0.15) / 10.0)
        self.assertEqual(forward_clearance(sectors), 1.0)

    def test_virtual_sectors_collapse_inside_pole(self):
        sectors = virtual_sectors(6.0, 0.0, 0.0, [(6.0, 0.0)])
        self.assertEqual(sectors, [0.0] * 12)

    def test_virtual_sectors_empty_world_is_all_far(self):
        sectors = virtual_sectors(0.0, 0.0, 0.0, [])
        self.assertEqual(sectors, [1.0] * 12)

    def test_yaw_from_quaternion(self):
        self.assertAlmostEqual(yaw_from_quaternion(0, 0, 0, 1), 0.0)
        # 绕 z 轴 90 度：w=cos45, z=sin45
        s = math.sqrt(0.5)
        self.assertAlmostEqual(yaw_from_quaternion(0, 0, s, s), math.pi / 2.0)

    def test_integrate_pose_moves_forward(self):
        x, y, yaw = integrate_pose(0.0, 0.0, 0.0, 1.0, 0.0, 0.2)
        self.assertAlmostEqual(x, 0.2)
        self.assertAlmostEqual(y, 0.0)
        self.assertAlmostEqual(yaw, 0.0)

    def test_integrate_pose_turns(self):
        _, _, yaw = integrate_pose(0.0, 0.0, 0.0, 0.0, 1.0, 0.2)
        self.assertAlmostEqual(yaw, 0.2)
        # 向右转后前进应产生负 y 位移
        x, y, _ = integrate_pose(0.0, 0.0, 0.2, 1.0, 0.0, 1.0)
        self.assertAlmostEqual(y, math.sin(0.2), places=6)


if __name__ == "__main__":
    unittest.main()
