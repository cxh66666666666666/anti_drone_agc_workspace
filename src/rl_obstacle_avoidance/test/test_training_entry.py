#!/usr/bin/env python3
"""训练入口能力：跨场景轮换 + 每集轨迹相位一致复位 的单元测试。

重点覆盖不依赖 ros master 的纯逻辑：
- _compute_t0_start 对 linear/circular/figure_eight 的 t=0 起点
- 动态障碍物 SDF / 分组 / 坐标串生成
- round/random 轮换游标覆盖全部场景
- reset 失败不占用 episode 预算、只写 reset_failures.csv
"""

import csv
import json
import math
import os
import random
import shlex
import subprocess
import sys
import tempfile
import unittest
from unittest import mock
from pathlib import Path

PKG = Path(__file__).resolve().parents[1]
ROOT = PKG.parents[1]
sys.path.insert(0, str(PKG / "scripts"))

from gazebo_env import GazeboEnv
import gazebo_env

try:
    import train_dqn
    HAS_TRAIN_SCRIPT = True
except ImportError:
    HAS_TRAIN_SCRIPT = False

ROUND_COUNT = 50  # stage2_v2 train 固定场景数
LOOP_DURATION_S = 2.0 * math.pi / 0.2  # circular/figure_eight 的角速度 0.2 rad/s


def read_csv_rows(path):
    with open(path, newline="") as stream:
        return list(csv.DictReader(stream))


class FakePose:
    """仅暴露轨迹终点距离/终止判据需要的位姿字段。"""

    def __init__(self, x, y, z=2.0):
        self.pose = mock.Mock()
        self.pose.position.x = x
        self.pose.position.y = y
        self.pose.position.z = z


class TrainingEntryTest(unittest.TestCase):
    def _env(self, traj_type, traj_json, altitude=2.0):
        env = GazeboEnv.__new__(GazeboEnv)  # 跳过需 ros master 的 __init__
        env.start_x = 0.0
        env.start_y = 0.0
        env.start_z = 2.0
        env.scenario_altitude = altitude
        env.scenario_trajectory = traj_type
        env.scenario_trajectory_json = traj_json
        return env

    def test_t0_linear_uses_start(self):
        env = self._env("linear", {"type": "linear",
                                   "start": [-8.0, -0.5, 2.0]})
        self.assertEqual(env._compute_t0_start(), (-8.0, -0.5, 2.0))

    def test_t0_circular_is_center_plus_radius(self):
        env = self._env("circular", {"type": "circular",
                                     "center": [0.0, 0.0, 2.0],
                                     "radius_m": 4.2, "height_m": 2.0})
        self.assertEqual(env._compute_t0_start(), (4.2, 0.0, 2.0))

    def test_t0_figure_eight_is_center(self):
        env = self._env("figure_eight", {"type": "figure_eight",
                                         "center": [1.0, 2.0],
                                         "radius_x_m": 4.6,
                                         "radius_y_m": 3.0,
                                         "height_m": 2.5})
        self.assertEqual(env._compute_t0_start(), (1.0, 2.0, 2.5))

    def test_collision_ends_episode_after_two_confirmed_frames(self):
        env = GazeboEnv.__new__(GazeboEnv)
        env._features_are_fresh = lambda: True
        env._geometry_clearance = lambda: 0.0   # 包络接触
        env.contact_dist = 0.0
        env._tracking_error = lambda: 0.0
        env.current_pose = None
        env._collision_streak = 1
        env._collision_latched = False
        env.collision_event_count = 0
        env.last_classified_collision_event = 0
        env.last_done_reason = "running"
        with mock.patch.object(gazebo_env.rospy, "logwarn"):
            done = env.is_done()
        self.assertTrue(done)
        self.assertEqual(env.last_done_reason, "collision")
        self.assertEqual(env.collision_event_count, 1)

    def test_risk_band_clearance_does_not_end_episode(self):
        """净空落在 (contact_dist, collision_dist) 风险带内：仅惩罚，不终止。"""
        env = self._done_env("linear", {"type": "linear", "loop": True})
        env._geometry_clearance = lambda: 0.35  # 风险带内但不接触
        with mock.patch.object(gazebo_env.rospy, "logwarn"):
            self.assertFalse(env.is_done())
        self.assertEqual(env.collision_event_count, 0)
        self.assertEqual(env._collision_streak, 0)

    def test_single_frame_contact_is_not_confirmed(self):
        """仅一帧包络接触不终止，需连续两帧确认。"""
        env = self._done_env("linear", {"type": "linear", "loop": True})
        env._geometry_clearance = lambda: -0.01  # 包络重叠
        with mock.patch.object(gazebo_env.rospy, "logwarn"):
            self.assertFalse(env.is_done())
        self.assertEqual(env._collision_streak, 1)
        self.assertEqual(env.collision_event_count, 0)

    def test_terminal_states_require_hard_reset_recovery(self):
        for reason in ("collision", "shield_infeasible", "diverged",
                       "low_altitude", "perception_timeout"):
            self.assertTrue(GazeboEnv._requires_reset_recovery(reason))
        self.assertFalse(GazeboEnv._requires_reset_recovery("timeout"))

    def test_reset_recovery_survives_reset_state_clear(self):
        env = GazeboEnv.__new__(GazeboEnv)
        env._reset_recovery_pending = False
        env.last_done_reason = "shield_infeasible"
        required = (env._reset_recovery_pending or
                    env._requires_reset_recovery(env.last_done_reason))
        self.assertTrue(required)

    def test_reset_observation_wait_refreshes_trajectory_hold(self):
        """Strict reset validation must feed LQR's local-control watchdog."""
        env = GazeboEnv.__new__(GazeboEnv)
        env.trajectory_type = "linear"
        env.scenario_trajectory = "linear"
        env.scenario_trajectory_json = {}
        env.start_x = env.start_y = 0.0
        env.start_z = env.scenario_altitude = 2.0
        env._reset_started_stamp = None
        env.reset_min_clearance_norm = 0.24
        env.current_pose = env.current_velocity = None
        env.features = object()
        env._non_rear_min_distance = lambda: 1.0
        env._publish_local_control = mock.Mock()

        clock = [0.0]

        class FakeDuration:
            def __init__(self, seconds):
                self.seconds = seconds

            def to_sec(self):
                return self.seconds

        class FakeTime:
            def __init__(self, seconds=None):
                self.seconds = clock[0] if seconds is None else seconds

            @staticmethod
            def now():
                return FakeTime()

            def __sub__(self, other):
                return FakeDuration(self.seconds - other.seconds)

            def is_zero(self):
                return True

        class FakeRate:
            def sleep(self):
                clock[0] += 0.1
                env.features = object()

        with mock.patch.object(gazebo_env.rospy, "Time", FakeTime), \
                mock.patch.object(gazebo_env.rospy, "Rate", return_value=FakeRate()), \
                mock.patch.object(gazebo_env.rospy, "logwarn"):
            self.assertTrue(env._wait_for_reset_observations(
                timeout=1.0, stable_cycles=6, require_reference=False,
                require_clearance=False, maintain_hold=True))
        self.assertGreaterEqual(env._publish_local_control.call_count, 6)
        env._publish_local_control.assert_called_with(0.0, True, 1.0)

    def test_reset_hold_requires_pose_and_zero_velocity(self):
        """Recovery validation must require a fresh stationary pose."""
        env = GazeboEnv.__new__(GazeboEnv)
        env.trajectory_type = "linear"
        env.scenario_trajectory = "linear"
        env.scenario_trajectory_json = {}
        env.start_x = env.start_y = 0.0
        env.start_z = env.scenario_altitude = 2.0
        env._reset_started_stamp = None
        env.reset_min_clearance_norm = 0.24
        env.current_pose = env.current_velocity = None
        env.features = object()
        env._non_rear_min_distance = lambda: 1.0
        env._publish_local_control = mock.Mock()

        clock = [0.0]

        class FakeDuration:
            def __init__(self, seconds):
                self.seconds = seconds

            def to_sec(self):
                return self.seconds

        class FakeTime:
            def __init__(self, seconds=None):
                self.seconds = clock[0] if seconds is None else seconds

            @staticmethod
            def now():
                return FakeTime()

            def __sub__(self, other):
                return FakeDuration(self.seconds - other.seconds)

            def is_zero(self):
                return True

        class FakeRate:
            def sleep(self):
                clock[0] += 0.1
                env.features = object()

        with mock.patch.object(gazebo_env.rospy, "Time", FakeTime), \
                mock.patch.object(gazebo_env.rospy, "Rate", return_value=FakeRate()), \
                mock.patch.object(gazebo_env.rospy, "logwarn"):
            self.assertFalse(env._wait_for_reset_observations(
                timeout=0.5, stable_cycles=1, require_reference=True,
                require_clearance=False, require_pose_target=True,
                require_velocity_stable=True, maintain_hold=True))
        self.assertGreaterEqual(env._publish_local_control.call_count, 5)

    def test_obstacle_sdf_pillar_box_box_geometry(self):
        env = GazeboEnv.__new__(GazeboEnv)
        sdf = env._obstacle_sdf({"type": "pillar_box",
                                 "size": [0.25, 0.25, 2.8]}, "scenario_a0")
        self.assertIn("<box><size>0.25 0.25 2.8</size></box>", sdf)
        self.assertIn("scenario_a0", sdf)

    def test_obstacle_sdf_cylinder_radius_half_max(self):
        env = GazeboEnv.__new__(GazeboEnv)
        sdf = env._obstacle_sdf({"type": "cylinder",
                                 "size": [0.65, 0.65, 2.8]}, "scenario_b1")
        self.assertIn("<cylinder><radius>0.325</radius>", sdf)

    def test_linear_reload_uses_scenario_speed_limit(self):
        env = GazeboEnv.__new__(GazeboEnv)
        env.reload_trajectory_srv = mock.Mock()
        # _reload_lqr_trajectory 末尾会调 _apply_trajectory_budget，补齐其依赖属性
        env.trajectory_type = "linear"
        env.goal_mode = False
        env.action_duration = 0.1
        env.episode_budget_factor = 2.0
        env._max_steps_cap = 1000
        env.max_episode_steps = 1000
        trajectory = {"type": "linear", "duration_s": 40.0,
                      "start": [-8.0, -0.5, 2.0],
                      "end": [8.0, -0.5, 2.0], "loop": False}
        with mock.patch.object(gazebo_env.rospy, "set_param") as set_param, \
                mock.patch.object(gazebo_env.rospy, "wait_for_service"):
            env._reload_lqr_trajectory(trajectory, speed_limit=1.02)
        set_param.assert_any_call(
            "/lqr_tracker_node/trajectory_duration", 16.0 / 1.02)

    def _done_env(self, traj_type, trajectory, goal_mode=False,
                  duration_s=LOOP_DURATION_S, phase_s=0.0, pose=None,
                  step=0, max_steps=1000):
        """构造只含终止判据所需属性的假 env（不启动 ROS）。"""
        env = GazeboEnv.__new__(GazeboEnv)
        env.goal_mode = goal_mode
        env.trajectory_type = traj_type
        env.trajectory_duration_s = duration_s
        env.trajectory_loop = bool(trajectory.get("loop", True))
        env.trajectory_phase_s = phase_s
        env.scenario_trajectory_json = trajectory
        env.goal_reached_dist = 1.0
        env.current_pose = pose
        env._features_are_fresh = lambda: True
        env._geometry_clearance = lambda: 10.0   # 远离障碍，不接触
        env.contact_dist = 0.0
        env._tracking_error = lambda: 0.0
        env.max_terminal_tracking_error = 5.0
        env.min_height = 0.3
        env._collision_streak = 0
        env._collision_latched = False
        env.collision_event_count = 0
        env.last_classified_collision_event = 0
        env.last_done_reason = "running"
        env.goal = None
        env.avoidance_active = False
        env.episode_step = step
        env.max_episode_steps = max_steps
        env.episode_min_obstacle = 10.0
        return env

    def test_trajectory_completed_uses_end_distance_for_linear(self):
        trajectory = {"type": "linear", "loop": False,
                      "start": [-8.0, 0.0, 2.0], "end": [8.0, 0.0, 2.0]}
        near = self._done_env("linear", trajectory,
                              pose=FakePose(7.5, 0.0))
        self.assertTrue(near._trajectory_completed())
        far = self._done_env("linear", trajectory, pose=FakePose(6.5, 0.0))
        self.assertFalse(far._trajectory_completed())

    def test_trajectory_completed_uses_phase_for_loop_trajectories(self):
        for traj_type in ("circular", "figure_eight"):
            trajectory = {"type": traj_type, "loop": True}
            running = self._done_env(traj_type, trajectory, phase_s=31.0)
            self.assertFalse(running._trajectory_completed(), traj_type)
            finished = self._done_env(traj_type, trajectory, phase_s=31.5)
            self.assertTrue(finished._trajectory_completed(), traj_type)

    def test_trajectory_completed_disabled_for_goal_and_step_modes(self):
        trajectory = {"type": "linear", "loop": False,
                      "end": [8.0, 0.0, 2.0]}
        pose = FakePose(8.0, 0.0)  # 终点距离 0，若判定生效必为 True
        goal_env = self._done_env("linear", trajectory, goal_mode=True,
                                  pose=pose)
        self.assertFalse(goal_env._trajectory_completed())
        step_env = self._done_env("step", trajectory, pose=pose)
        self.assertFalse(step_env._trajectory_completed())
        no_duration = self._done_env("linear", trajectory, duration_s=None,
                                     pose=pose)
        self.assertFalse(no_duration._trajectory_completed())

    def test_is_done_reports_trajectory_complete(self):
        trajectory = {"type": "linear", "loop": False,
                      "start": [-8.0, 0.0, 2.0], "end": [8.0, 0.0, 2.0]}
        env = self._done_env("linear", trajectory, pose=FakePose(7.5, 0.0),
                             step=10)
        self.assertTrue(env.is_done())
        self.assertEqual(env.last_done_reason, "trajectory_complete")
        # 尚未到终点且未超时：episode 继续
        running = self._done_env("linear", trajectory,
                                 pose=FakePose(6.5, 0.0), step=10)
        self.assertFalse(running.is_done())
        self.assertEqual(running.last_done_reason, "running")

    def test_apply_trajectory_budget_clamps_to_cap(self):
        env = self._budget_env(cap=450, duration_s=LOOP_DURATION_S)
        env._apply_trajectory_budget()
        self.assertEqual(env.max_episode_steps, 450)   # ceil(628.32) 被 cap 夹住
        env = self._budget_env(cap=1000, duration_s=LOOP_DURATION_S)
        env._apply_trajectory_budget()
        self.assertEqual(env.max_episode_steps, 629)
        env = self._budget_env(cap=1000, duration_s=16.0 / 1.02)
        env._apply_trajectory_budget()
        self.assertEqual(env.max_episode_steps, 314)

    def test_apply_trajectory_budget_skips_goal_and_step_modes(self):
        for kwargs in ({"traj_type": "step"}, {"goal_mode": True}):
            env = self._budget_env(cap=450, duration_s=LOOP_DURATION_S, **kwargs)
            env._apply_trajectory_budget()
            self.assertEqual(env.max_episode_steps, 450)

    def _budget_env(self, cap, duration_s, traj_type="linear",
                    goal_mode=False):
        env = GazeboEnv.__new__(GazeboEnv)
        env.goal_mode = goal_mode
        env.trajectory_type = traj_type
        env.trajectory_duration_s = duration_s
        env.episode_budget_factor = 2.0
        env.action_duration = 0.1
        env._max_steps_cap = cap
        env.max_episode_steps = cap
        return env

    def test_step_accumulates_phase_with_pause_and_speed_scale(self):
        """相位按 Δt×(暂停 0 / 否则 local_speed_scale) 累积。"""
        env = self._done_env("linear", {"type": "linear", "loop": False},
                             phase_s=10.0)
        env._episode_done = True
        env.local_speed_scale = 0.5
        env.trajectory_paused = False
        env.episode_min_geometry_clearance = 0.0
        env.geometry_collision_events = 0
        env.trial_id = 0
        env.scenario_id = "s"
        env._validate_time_alignment = lambda: None
        env._geometry_clearance = lambda: 1.0
        env._episode_metrics = lambda: {}
        env.get_observation = lambda: [0.0]

        clock = [5.0]

        class FakeDuration:
            def __init__(self, seconds):
                self.seconds = seconds

            def to_sec(self):
                return self.seconds

        class FakeTime:
            def __init__(self, seconds):
                self.seconds = seconds

            @staticmethod
            def now():
                return FakeTime(clock[0])

            def __sub__(self, other):
                return FakeDuration(self.seconds - other.seconds)

        env._phase_sample_stamp = FakeTime(3.0)
        with mock.patch.object(gazebo_env.rospy, "Time", FakeTime):
            env.step(0)
            self.assertAlmostEqual(env.trajectory_phase_s, 11.0)  # 2.0s × 0.5
            env.trajectory_paused = True
            env.step(0)
            self.assertAlmostEqual(env.trajectory_phase_s, 11.0)  # 暂停不推进

    def test_scenario_configuration_does_not_spawn_obstacles(self):
        env = GazeboEnv.__new__(GazeboEnv)
        env.trajectory_type = "linear"
        env.scenario_id = "old"
        env._spawn_scenario_obstacles = mock.Mock()
        env._load_scenario_from_params = mock.Mock()
        env._reload_lqr_trajectory = mock.Mock()
        scenario = {
            "scenario_id": "new",
            "altitude": 2.0,
            "speed_limit": 1.0,
            "trajectory": {"type": "linear"},
            "obstacles": [{"group": "a", "center": [0.0, 0.0, 2.0]}],
        }

        with mock.patch.object(gazebo_env.rospy, "set_param"):
            env._configure_scenario(scenario)

        env._spawn_scenario_obstacles.assert_not_called()
        env._load_scenario_from_params.assert_called_once_with()
        env._reload_lqr_trajectory.assert_called_once_with(
            scenario["trajectory"], scenario["speed_limit"])
        self.assertEqual(env.scenario_id, "new")

    def test_partial_spawn_is_tracked_and_cleaned_up(self):
        env = GazeboEnv.__new__(GazeboEnv)
        env._deployment_serial = 0
        env._spawn_model = mock.Mock(side_effect=[True, False])
        env._delete_spawned_models = mock.Mock(return_value=True)
        scenario = {"obstacles": [
            {"group": "a", "center": [1.0, 0.0, 2.0]},
            {"group": "a", "center": [2.0, 0.0, 2.0]},
        ]}
        self.assertFalse(env._spawn_scenario_obstacles(scenario))
        self.assertEqual(env._spawned_models, ["scenario_d000001_a0"])
        env._delete_spawned_models.assert_called_once_with()

    def test_spawn_requires_all_models_visible_in_gazebo(self):
        env = GazeboEnv.__new__(GazeboEnv)
        env._deployment_serial = 0
        env._spawn_model = mock.Mock(return_value=True)
        env._delete_spawned_models = mock.Mock(return_value=True)
        env.get_world_properties_srv = mock.Mock(return_value=mock.Mock(
            model_names=["iris", "scenario_d000001_a0"]))
        scenario = {"obstacles": [
            {"group": "a", "center": [1.0, 0.0, 2.0]},
            {"group": "b", "center": [2.0, 0.0, 2.0]},
        ]}
        with mock.patch.object(gazebo_env.rospy, "wait_for_service"):
            self.assertFalse(env._spawn_scenario_obstacles(scenario))
        self.assertEqual(env._spawned_models,
                         ["scenario_d000001_a0", "scenario_d000001_b1"])
        env._delete_spawned_models.assert_called_once_with()

    def test_spawn_succeeds_when_all_models_are_visible(self):
        env = GazeboEnv.__new__(GazeboEnv)
        env._deployment_serial = 0
        env._spawn_model = mock.Mock(return_value=True)
        env.get_world_properties_srv = mock.Mock(return_value=mock.Mock(
            model_names=["iris", "scenario_d000001_a0",
                         "scenario_d000001_b1"]))
        scenario = {"obstacles": [
            {"group": "a", "center": [1.0, 0.0, 2.0]},
            {"group": "b", "center": [2.0, 0.0, 2.0]},
        ]}
        with mock.patch.object(gazebo_env.rospy, "wait_for_service"):
            self.assertTrue(env._spawn_scenario_obstacles(scenario))
        self.assertEqual(env._spawned_models,
                         ["scenario_d000001_a0", "scenario_d000001_b1"])

    def test_spawn_visibility_race_is_retried(self):
        env = GazeboEnv.__new__(GazeboEnv)
        env._deployment_serial = 0
        env._spawn_model = mock.Mock(return_value=True)
        env._world_model_names = mock.Mock(side_effect=[
            ["iris"], ["iris", "scenario_d000001_a0"]])
        scenario = {"obstacles": [
            {"group": "a", "center": [1.0, 0.0, 2.0]},
        ]}
        with mock.patch.object(gazebo_env.rospy, "sleep"):
            self.assertTrue(env._spawn_scenario_obstacles(scenario))
        self.assertEqual(env._world_model_names.call_count, 2)

    def test_failed_delete_remains_tracked(self):
        env = GazeboEnv.__new__(GazeboEnv)
        env._spawned_models = ["scenario_a0", "scenario_a1"]
        env.get_world_properties_srv = mock.Mock(side_effect=[
            mock.Mock(model_names=["iris", "scenario_a0", "scenario_a1"]),
            mock.Mock(model_names=["iris", "scenario_a1"]),
        ])
        ok = mock.Mock(success=True, status_message="")
        failed = mock.Mock(success=False, status_message="busy")
        env.delete_model_srv = mock.Mock(side_effect=[ok, failed])
        with mock.patch.object(gazebo_env.rospy, "wait_for_service"), \
                mock.patch.object(gazebo_env.rospy, "logwarn"):
            self.assertFalse(env._delete_spawned_models())
        self.assertEqual(env._spawned_models, ["scenario_a1"])

    def test_missing_gazebo_model_is_an_idempotent_delete(self):
        env = GazeboEnv.__new__(GazeboEnv)
        env._spawned_models = ["scenario_a0"]
        env.get_world_properties_srv = mock.Mock(return_value=mock.Mock(model_names=[]))
        env.delete_model_srv = mock.Mock(return_value=mock.Mock(
            success=False, status_message="DeleteModel: model does not exist"))
        with mock.patch.object(gazebo_env.rospy, "wait_for_service"):
            self.assertTrue(env._delete_spawned_models())
        self.assertEqual(env._spawned_models, [])
        env.delete_model_srv.assert_not_called()

    def test_delete_race_with_external_cleanup_is_idempotent(self):
        env = GazeboEnv.__new__(GazeboEnv)
        env._spawned_models = ["scenario_a0"]
        env.get_world_properties_srv = mock.Mock(side_effect=[
            mock.Mock(model_names=["scenario_a0"]),
            mock.Mock(model_names=[]),
        ])
        env.delete_model_srv = mock.Mock(return_value=mock.Mock(
            success=False, status_message="DeleteModel: model does not exist"))
        with mock.patch.object(gazebo_env.rospy, "wait_for_service"):
            self.assertTrue(env._delete_spawned_models())
        self.assertEqual(env._spawned_models, [])

    def test_model_query_failure_blocks_delete_and_spawn(self):
        env = GazeboEnv.__new__(GazeboEnv)
        env._spawned_models = ["scenario_a0"]
        env.get_world_properties_srv = mock.Mock(side_effect=
                                                  gazebo_env.rospy.ServiceException("offline"))
        env.delete_model_srv = mock.Mock()
        with mock.patch.object(gazebo_env.rospy, "wait_for_service"), \
                mock.patch.object(gazebo_env.rospy, "logwarn"):
            self.assertFalse(env._delete_spawned_models())
        self.assertEqual(env._spawned_models, ["scenario_a0"])
        env.delete_model_srv.assert_not_called()

    def test_stale_scenario_models_are_discovered_for_cleanup(self):
        env = GazeboEnv.__new__(GazeboEnv)
        env._spawned_models = []
        env.get_world_properties_srv = mock.Mock(side_effect=[
            mock.Mock(model_names=["iris", "scenario_a0", "scenario_b1",
                                   "other_model"]),
            mock.Mock(model_names=["iris", "other_model"]),
        ])
        env.delete_model_srv = mock.Mock(return_value=mock.Mock(
            success=True, status_message=""))
        with mock.patch.object(gazebo_env.rospy, "wait_for_service"):
            self.assertTrue(env._delete_spawned_models())
        self.assertEqual(
            env.delete_model_srv.call_args_list,
            [mock.call("scenario_a0"), mock.call("scenario_b1")])

    def test_split_obstacles_groups(self):
        env = GazeboEnv.__new__(GazeboEnv)
        obstacles = [
            {"group": "a", "center": [1.0, 2.0, 1.4]},
            {"group": "b", "center": [3.0, 4.0, 1.4]},
            {"group": "a", "center": [5.0, 6.0, 1.4]},
        ]
        a_list, b_list = env._split_obstacles_groups(obstacles, {})
        self.assertEqual(a_list, [[1.0, 2.0], [5.0, 6.0]])
        self.assertEqual(b_list, [[3.0, 4.0]])

    def test_obstacles_spec_str(self):
        env = GazeboEnv.__new__(GazeboEnv)
        obstacles = [
            {"center": [1.0, 2.0, 1.4]},
            {"center": [3.0, 4.0, 1.4]},
        ]
        self.assertEqual(env._obstacles_spec_str(obstacles), "1,2;3,4")

    def test_as_coord_list_handles_stale_json_string(self):
        # 兼容 roscore 残留的 JSON 字符串形态（历史 bug 写入过）
        env = GazeboEnv.__new__(GazeboEnv)
        self.assertEqual(env._as_coord_list("[[4.0,0.0],[8.0,3.0]]"),
                         [[4.0, 0.0], [8.0, 3.0]])
        self.assertEqual(env._as_coord_list([]), [])
        self.assertEqual(env._as_coord_list("[bad"), [])
        self.assertEqual(env._as_coord_list(None), [])

    def test_round_rotation_covers_all_scenarios(self):
        # 复现 train_dqn 的 round 轮换公式：ep % 50
        covered = {
            ep % ROUND_COUNT for ep in range(ROUND_COUNT * 2)
        }
        self.assertEqual(covered, set(range(ROUND_COUNT)))

    def test_random_rotation_bounds(self):
        random.seed(0)
        n = max(random.randint(0, ROUND_COUNT - 1)
                for _ in range(200))
        self.assertGreaterEqual(n, 0)
        self.assertLess(n, ROUND_COUNT)


class SensorBlackoutInjectionTest(unittest.TestCase):
    """§3.2 工况 7：主动注入传感器失效窗口（场景 → rosparam → 感知门控）。"""

    def _env(self, window=(), phase=0.0):
        env = GazeboEnv.__new__(GazeboEnv)
        env.sensor_blackout_window = window
        env.action_duration = 0.1
        env.episode_step = 0
        env.trajectory_phase_s = phase
        return env

    def test_window_normalization(self):
        parse = GazeboEnv._as_blackout_window
        self.assertEqual(parse([12.0, 15.0]), (12.0, 15.0))
        self.assertEqual(parse("[12, 15]"), (12.0, 15.0))
        for bad in (None, [], [12.0], [15.0, 12.0], "坏值", 12.0):
            self.assertEqual(parse(bad), (), bad)

    def test_blackout_gate_follows_trajectory_phase(self):
        """门控基准是轨迹相位（非步数×dt），左右边界均为半开区间。"""
        env = self._env(window=(5.0, 8.0))
        for phase, expected in ((0.0, False), (4.99, False), (5.0, True),
                                (7.99, True), (8.0, False)):
            env.trajectory_phase_s = phase
            self.assertIs(env._sensor_blackout_active(), expected, phase)

    def test_gate_is_inactive_without_phase_attribute(self):
        env = GazeboEnv.__new__(GazeboEnv)
        env.sensor_blackout_window = (5.0, 8.0)
        self.assertFalse(env._sensor_blackout_active())

    def test_window_is_disabled_by_default(self):
        self.assertFalse(self._env(window=())._sensor_blackout_active())

    def test_blackout_short_circuits_freshness(self):
        """窗口内必须先判失效，不得再查感知时间戳（否则会被误判为新鲜）。"""
        env = self._env(window=(5.0, 8.0), phase=6.0)
        env.features = mock.Mock()
        env.features_stamp = mock.Mock()
        with mock.patch.object(gazebo_env.rospy.Time, "now",
                               side_effect=AssertionError("窗口内不应查询时间")):
            self.assertFalse(env._features_are_fresh())

    def test_scenario_shell_emits_blackout_window(self):
        """有窗口时输出 [t0,t1]；无该字段时输出空列表（既有场景不受影响）。"""
        script = ROOT / "scripts" / "scenario_shell.py"
        base = {"scenario_id": "s3v1-blackout", "obstacles": [],
                "trajectory": {"type": "linear", "start": [-8.0, 0.0, 2.0],
                               "end": [8.0, 0.0, 2.0], "duration_s": 40.0}}
        cases = (({"sensor_blackout_s": [12.0, 15.0]}, [12.0, 15.0]), ({}, []))
        for extra, expected in cases:
            scene = dict(base, **extra)
            with tempfile.TemporaryDirectory() as tmp:
                path = Path(tmp) / "scene.json"
                path.write_text(json.dumps(scene), encoding="utf-8")
                out = subprocess.run(
                    [sys.executable, str(script), str(path)],
                    capture_output=True, text=True, check=True).stdout
            values = dict(line.split("=", 1) for line in out.splitlines()
                          if "=" in line)
            self.assertEqual(
                json.loads(shlex.split(values["SENSOR_BLACKOUT"])[0]),
                expected, extra)


class _FixedTime(gazebo_env.rospy.Time):
    """固定时刻替身：与真实 rospy.Time 同比较/相减语义，便于无 ros master 打桩。"""

    @staticmethod
    def now():
        return _FixedTime.from_sec(100.0)


class LookaheadAvoidanceGateTests(unittest.TestCase):
    """前瞻可达性门控：观测尾部 5 维特征与 allowed_actions 门控口径。"""

    def make_env(self, **overrides):
        """构造一个不依赖 ros master 的裸 GazeboEnv（属性默认值集中在此）。"""
        env = GazeboEnv.__new__(GazeboEnv)
        attrs = {
            "num_sectors": 12,
            "features": None,
            "features_stamp": None,
            "perception_timeout": 0.5,
            "obstacle_memory_timeout": 0.4,
            "_sector_memory": None,
            "_sector_memory_stamp": None,
            "_sector_memory_body_yaw": None,
            "_last_sector_features": None,
            "start_yaw": 0.0,
            "current_pose": None,
            "current_velocity": None,
            "position_error": None,
            "reference_velocity": None,
            "reference_velocity_stamp": None,
            "reference_mid_lookahead_velocity": None,
            "reference_mid_lookahead_velocity_stamp": None,
            "reference_lookahead_velocity": None,
            "reference_lookahead_velocity_stamp": None,
            "reference_velocity_timeout": 0.5,
            "motion_heading_world": 0.0,
            "heading_min_speed": 0.1,
            "avoidance_lookahead_time": 2.0,
            "safe_distance_norm": 0.24,
            "emergency_dist_norm": 0.05,
            "warning_distance_norm": 0.30,
            "collision_dist": 0.0,
            "lidar_max_range": 10.0,
            "drone_half_extents": (0.35, 0.35, 0.06),
            "path_clearance_margin": 0.30,
            "path_feature_scale_m": gazebo_env.PATH_FEATURE_SCALE_M,
            "turning_front_extra_angle": math.pi / 6.0,
            "last_offset_x": 0.0,
            "last_offset_y": 0.0,
            "last_offset_z": 0.0,
            "max_offset_xy": 1.4,
            "step_z": 0.3,
            "prev_directional_clearances": None,
            "action_duration": 0.1,
            "sensor_blackout_window": (),
            "path_points": [],
            "last_action": 0,
            "locked_avoid_side": 0,
            "avoidance_active": False,
            "avoidance_tail_seen": False,
            "avoidance_tail_completion_norm": 0.2,
            "avoidance_tail_release_norm": 0.3,
            "metric_errors": [],
            "metric_times": [],
            "metric_positions": [],
            "action_interface": "masked",
        }
        attrs.update(overrides)
        for name, value in attrs.items():
            setattr(env, name, value)
        return env

    def test_observation_dim_includes_path_features(self):
        """12 扇区时观测维度 = 12 + 4 + 11 + 6 + 5 = 38。"""
        env = self.make_env()
        self.assertEqual(gazebo_env.PATH_FEATURES, 5)
        self.assertEqual(env.observation_space_dim, 38)

    def test_observation_tail_is_five_bounded_features(self):
        """尾部 5 维（索引 33~37）为有限且落在 [0,1] 的前瞻特征。"""
        env = self.make_env()
        with mock.patch.object(gazebo_env.rospy, "Time", _FixedTime):
            obs = env.get_observation()
        self.assertEqual(len(obs), 38)
        tail = obs[33:38]
        self.assertEqual(len(tail), gazebo_env.PATH_FEATURES)
        self.assertTrue(all(math.isfinite(value) for value in tail))
        self.assertTrue(all(0.0 <= value <= 1.0 for value in tail))

    def test_missing_perception_stays_conservative(self):
        """无感知帧走全 0 扇区保守分支：reach=1、band=1、avoid_needed=1。

        索引 35/36 是左右净空归一化值，全 0 扇区意味着净空 0 m，故为 0.0。
        """
        env = self.make_env()
        with mock.patch.object(gazebo_env.rospy, "Time", _FixedTime):
            tail = env.get_observation()[33:38]
            self.assertTrue(env._avoidance_needed())
        self.assertEqual(tail, [1.0, 1.0, 0.0, 0.0, 1.0])

    def test_clear_path_gate_allows_straight_flight(self):
        """全清扇区时门控直接放行直飞，即使 _avoidance_clearance() 低于 warning。"""
        env = self.make_env()
        env._motion_sector_frames = lambda: ([1.0] * 12, 0.0, [1.0] * 12)
        with mock.patch.object(gazebo_env.rospy, "Time", _FixedTime):
            self.assertFalse(env._avoidance_needed())
            self.assertLess(env._avoidance_clearance(),
                            env.warning_distance_norm)
            self.assertEqual(env.allowed_actions(), [0])

    def test_gate_uses_body_frame_sectors(self):
        """门控必须用机体帧原始扇区：复用已扫掠扇区会重复旋转一次。"""
        body = [1.0] * 12
        body[6] = 0.15  # +15°，落在正前 ±30° 锥内，1.5 m
        env = self.make_env()
        env._motion_sector_frames = lambda: ([1.0] * 12, 0.0, body)
        with mock.patch.object(gazebo_env.rospy, "Time", _FixedTime):
            self.assertTrue(env._avoidance_needed())

    def test_side_gap_reachable_keeps_straight_flight(self):
        """侧方 2.66 m 夹缝（mirror_pair 形状）够不到：无需避障，掩码为 [0]。"""
        for index in (7, 4):  # +45° / −45°，均在侧锥内
            body = [1.0] * 12
            body[index] = 0.266
            env = self.make_env()
            env._motion_sector_frames = lambda body=body: (
                [1.0] * 12, 0.0, body)
            with mock.patch.object(gazebo_env.rospy, "Time", _FixedTime):
                self.assertFalse(env._avoidance_needed(), index)
                self.assertEqual(env.allowed_actions(), [0], index)

        # 左右同时出现夹缝仍判为无需避障
        body = [1.0] * 12
        body[7] = body[4] = 0.266
        env = self.make_env()
        env._motion_sector_frames = lambda: ([1.0] * 12, 0.0, body)
        with mock.patch.object(gazebo_env.rospy, "Time", _FixedTime):
            self.assertFalse(env._avoidance_needed())
            self.assertEqual(env.allowed_actions(), [0])

    def test_frontal_block_keeps_avoidance_mask(self):
        """正前方 1.5 m 阻挡：需要避障，掩码不含直飞 action 0。"""
        body = [1.0] * 12
        body[6] = 0.15
        env = self.make_env()
        env._motion_sector_frames = lambda: ([1.0] * 12, 0.0, body)
        with mock.patch.object(gazebo_env.rospy, "Time", _FixedTime):
            self.assertTrue(env._avoidance_needed())
            self.assertNotIn(0, env.allowed_actions())

    def test_tail_lock_overrides_gate(self):
        """尾部锁定优先于门控：全清扇区且 locked_avoid_side=0 时走 [3]。"""
        env = self.make_env(avoidance_active=True, avoidance_tail_seen=False)
        env._motion_sector_frames = lambda: ([1.0] * 12, 0.0, [1.0] * 12)
        with mock.patch.object(gazebo_env.rospy, "Time", _FixedTime):
            self.assertFalse(env._avoidance_needed())
            self.assertEqual(env.allowed_actions(), [3])

    def test_gate_releases_lateral_lock(self):
        """门控放行时复位 locked_avoid_side（净空低于 warning，非既有分支复位）。"""
        env = self.make_env(locked_avoid_side=1)
        env._motion_sector_frames = lambda: ([1.0] * 12, 0.0, [1.0] * 12)
        with mock.patch.object(gazebo_env.rospy, "Time", _FixedTime):
            self.assertLess(env._avoidance_clearance(),
                            env.warning_distance_norm)
            self.assertEqual(env.allowed_actions(), [0])
        self.assertEqual(env.locked_avoid_side, 0)

    def _motion_env(self, body_sectors, tail_active=False):
        """构造可执行 _publish_action_motion 的裸环境（发布链路已打桩）。

        tail_active=True 且 tail_seen=False 表示刚通过的障碍尚未出现在后方锥
        → 尾部锁定（此时不抑制偏移）。
        """
        env = self.make_env(avoidance_active=tail_active,
                            avoidance_tail_seen=False)
        env.features = mock.Mock(lidar_sectors=list(body_sectors))
        env.current_action = 3
        env.selected_action = 3
        env.min_offset_xy = 0.3
        env.offset_smooth_alpha = 0.8
        env.max_offset_change = 0.25
        env.local_speed_scale = 1.0
        env.trajectory_paused = False
        env.shield_escape = 0
        env.published = []
        env._publish_offset = lambda x, y, z: env.published.append((x, y, z))
        env._publish_local_control = mock.Mock()
        env.action_pub = mock.Mock()
        env._body_yaw = lambda: 0.0
        env._features_are_fresh = lambda: True
        env._minimum_obstacle_distance = lambda: min(body_sectors)
        env._forward_obstacle_distance = lambda: min(body_sectors)
        env._motion_sector_frames = lambda: ([1.0] * 12, 0.0, body_sectors)
        return env

    def test_suppressed_gate_publishes_zero_offset(self):
        """侧方夹缝判为无需避障时，即使策略选了 3 也只发布零偏移。"""
        body = [1.0] * 12
        body[7] = 0.266                      # 侧方 2.66 m，够不到
        env = self._motion_env(body)
        with mock.patch.object(gazebo_env.rospy, "Time", _FixedTime), \
                mock.patch.object(gazebo_env.rospy, "loginfo_throttle"):
            self.assertTrue(env._avoidance_suppressed())
            env._publish_action_motion()
        self.assertEqual(env.published, [(0.0, 0.0, 0.0)])

    def test_unsuppressed_gate_publishes_lateral_offset(self):
        """正前方阻挡时门控不生效，action 3 的正常横向+上升偏移必须发布。"""
        body = [1.0] * 12
        body[6] = 0.15                       # 正前 1.5 m
        env = self._motion_env(body)
        with mock.patch.object(gazebo_env.rospy, "Time", _FixedTime), \
                mock.patch.object(gazebo_env.rospy, "loginfo_throttle"):
            self.assertFalse(env._avoidance_suppressed())
            env._publish_action_motion()
        _, offset_y, offset_z = env.published[-1]
        self.assertGreater(abs(offset_y), 0.0)
        self.assertGreater(offset_z, 0.0)

    def test_tail_lock_keeps_offset_even_without_gate(self):
        """尾部锁定时不抑制偏移：刚通过的障碍仍占据后方锥。"""
        env = self._motion_env([1.0] * 12, tail_active=True)
        with mock.patch.object(gazebo_env.rospy, "Time", _FixedTime), \
                mock.patch.object(gazebo_env.rospy, "loginfo_throttle"):
            self.assertFalse(env._avoidance_suppressed())

    def test_action_motion_rejects_out_of_range_action(self):
        """越界动作仍然抛 ValueError（校验前移到分支之前，行为不变）。"""
        env = self._motion_env([1.0] * 12)
        env.current_action = 6
        with mock.patch.object(gazebo_env.rospy, "Time", _FixedTime), \
                mock.patch.object(gazebo_env.rospy, "loginfo_throttle"):
            with self.assertRaises(ValueError):
                env._publish_action_motion()

    def _interface_env(self, clearance, left, right, rear=1.0,
                       suppressed=False, action_interface="masked",
                       locked=0, avoidance_active=False, tail_seen=False,
                       non_rear=None):
        """构造仅由注入净空驱动的裸环境，用于动作接口分支测试。"""
        env = self.make_env(
            action_interface=action_interface, locked_avoid_side=locked,
            avoidance_active=avoidance_active, avoidance_tail_seen=tail_seen)
        env._avoidance_clearance = lambda: clearance
        env._side_clearances = lambda: (left, right)
        env._rear_min_distance = lambda: rear
        env._non_rear_min_distance = lambda: (
            clearance if non_rear is None else non_rear)
        env._avoidance_suppressed = lambda: suppressed
        return env

    def test_masked_allowed_actions_matches_rule_kernel(self):
        """masked 档 allowed_actions() 与纯规则内核逐元素一致，覆盖三支。"""
        cases = (
            # clearance, left, right, rear, suppressed, active, seen, locked, 期望
            (0.10, 1.0, 1.0, 1.0, True, False, False, 0, [0]),         # suppression
            (0.10, 1.0, 1.0, 1.0, False, True, False, 0, [3]),         # tail_lock
            (0.03, 0.2, 0.2, 1.0, False, False, False, 1, [3]),        # emergency
            (0.26, 1.0, 1.0, 1.0, False, False, False, 0, [0, 1, 2]),  # warning 区
            (0.50, 1.0, 1.0, 1.0, False, False, False, 1, [0]),        # clear
        )
        for (clearance, left, right, rear, suppressed, active, seen, locked,
             expected) in cases:
            env = self._interface_env(
                clearance, left, right, rear, suppressed, locked=locked,
                avoidance_active=active, tail_seen=seen)
            self.assertEqual(env.allowed_actions(), expected)
            self.assertEqual(env.rule_actions(), expected)

    def test_masked_constrain_action_remaps_into_mask(self):
        """masked 档 _constrain_action() 迟滞改写后仍重映射进距离掩码。"""
        # 左净空贴脸使锁定翻转到右侧，掩码只剩 [2,4]；迟滞结果 1 被重映射为 2。
        env = self._interface_env(0.10, 0.0, 1.0, locked=1)
        with mock.patch.object(gazebo_env.rospy, "Time", _FixedTime):
            self.assertEqual(env._constrain_action(2), 2)

    def test_raw_allowed_actions_returns_full_set_in_danger(self):
        """raw 档即使处于危险区，allowed_actions() 仍返回完整动作集。"""
        env = self._interface_env(0.03, 0.2, 0.2, action_interface="raw",
                                  locked=1)
        self.assertEqual(env.allowed_actions(), tuple(range(6)))

    def test_raw_constrain_action_keeps_hysteresis(self):
        """raw 档下对侧动作仍被改写到锁定侧（左侧锁定：2->1、4->3）。"""
        env = self._interface_env(0.10, 0.0, 1.0, action_interface="raw",
                                  locked=1)
        self.assertEqual(env._constrain_action(2), 1)
        env.locked_avoid_side = 1
        self.assertEqual(env._constrain_action(4), 3)

    def test_rule_actions_is_mode_independent_and_pure(self):
        """rule_actions() 两档输出一致，且不读写侧向锁定。"""
        state = dict(clearance=0.03, left=1.0, right=0.2, locked=1)
        masked = self._interface_env(action_interface="masked", **state)
        raw = self._interface_env(action_interface="raw", **state)
        self.assertEqual(masked.rule_actions(), raw.rule_actions())
        masked.rule_actions()
        raw.rule_actions()
        self.assertEqual(masked.locked_avoid_side, 1)
        self.assertEqual(raw.locked_avoid_side, 1)

    def test_raw_constrain_action_skips_mask_remap(self):
        """raw 档不做掩码重映射：同一输入 masked 被改写为 2，raw 保留迟滞结果 1。"""
        masked = self._interface_env(0.10, 0.0, 1.0, locked=1)
        raw = self._interface_env(0.10, 0.0, 1.0, action_interface="raw",
                                  locked=1)
        with mock.patch.object(gazebo_env.rospy, "Time", _FixedTime):
            self.assertEqual(masked._constrain_action(2), 2)
            self.assertEqual(raw._constrain_action(2), 1)


class FakeTrainingEnv:
    """训练入口替身：前 fail_times 次 reset 失败，之后成功。"""

    def __init__(self, fail_times):
        self.n_actions = 6  # training_log 固定写 action_0..action_5
        self.num_sectors = 2
        self.episode_step_limit = 1
        self.observation_space_dim = 2
        self.goal_mode = "waypoint"
        self.obstacles_a = []
        self.obstacles_b = []
        self.goal_min_distance = 0.0
        self.goal_max_distance = 0.0
        self.trial_id = 0
        self.scenario_id = "none"
        self.start_dist = 0.0
        self.start_obstacle_min = 1.0
        self.reset_ok = False
        self.reset_failure_reason = "not_started"
        self.armed = []
        self._fail_times = fail_times
        self._reset_count = 0

    def set_max_steps(self, value):
        self.max_steps = value

    def set_scenario_rotation(self, value):
        pass

    def set_phase_aligned(self, value):
        pass

    def arm_deploy(self, scenario):
        self.armed.append(scenario["scenario_id"])
        self.scenario_id = scenario["scenario_id"]

    def reset(self):
        self._reset_count += 1
        self.reset_ok = self._reset_count > self._fail_times
        if self.reset_ok:
            self.reset_failure_reason = ""
        else:
            # 复现训练日志里的出生点越界失败
            self.reset_failure_reason = "start_clearance_below_min"
            self.start_dist = 12.0
            self.start_obstacle_min = 0.18
        return [0.0] * self.observation_space_dim

    def allowed_actions(self):
        return list(range(self.n_actions))

    def step(self, action):
        return [0.0] * self.observation_space_dim, 1.0, True, {
            "applied_action": action,
            "done_reason": "timeout",
            "episode_success": True,
            "safe_success": True,
            "min_obstacle_min": 1.0,
            "motion_sectors": [1.0] * self.num_sectors,
            # 复现一次备份逃逸，验证逃逸字段能落到 step_log
            "shield_escape": True,
            "shield_escape_count": 2,
            "escape_margin": 0.5,
        }

    def clear_local_offset(self):
        pass


class FakeTrainingAgent:
    """避免测试里构造真实 DQN/优化器。"""

    def select_action(self, state, epsilon, allowed_actions=None):
        return 0

    def update(self, batch, weights):
        return 0.0, [0.0] * len(batch)

    def sync_target(self):
        pass

    def save(self, path):
        pass

    def schedule_lr(self, episode, total_episodes):
        pass

    def training_state_dict(self):
        return {}


@unittest.skipUnless(HAS_TRAIN_SCRIPT, "train_dqn 依赖的 ROS 模块不可用")
class ResetFailureAccountingTest(unittest.TestCase):
    def test_reset_failure_does_not_consume_episode_budget(self):
        """reset 失败只写 reset_failures.csv：不写 training_log 失败行、不推进 episode。"""
        scenarios = [{"scenario_id": "s-%d" % index, "split": "train",
                      "obstacles": [{"type": "wall"}]} for index in range(4)]
        with tempfile.TemporaryDirectory() as directory:
            manifest_path = os.path.join(directory, "train.json")
            test_path = os.path.join(directory, "test.json")
            with open(manifest_path, "w") as stream:
                json.dump({"scenarios": scenarios}, stream)
            with open(test_path, "w") as stream:
                json.dump({"scenarios": [{"scenario_id": "t-0", "split": "test",
                                          "obstacles": [{"type": "wall"}]}]},
                          stream)
            run_dir = os.path.join(directory, "run")
            env = FakeTrainingEnv(fail_times=train_dqn.RESET_RETRY_LIMIT)
            argv = ["train_dqn.py", "--run-dir", run_dir, "--episodes", "2",
                    "--max_steps", "1", "--no-curriculum",
                    "--scenario-manifest", manifest_path]
            with mock.patch.object(train_dqn, "GazeboEnv", return_value=env), \
                    mock.patch.object(train_dqn, "DQNAgent",
                                      return_value=FakeTrainingAgent()), \
                    mock.patch.object(train_dqn, "environment_config",
                                      return_value={}), \
                    mock.patch("sys.argv", argv):
                train_dqn.main()

            train_rows = read_csv_rows(os.path.join(run_dir, "training_log.csv"))
            reset_rows = read_csv_rows(
                os.path.join(run_dir, "reset_failures.csv"))
            step_rows = read_csv_rows(os.path.join(run_dir, "step_log.csv"))

        # reset 失败明细完整落盘
        self.assertEqual(len(reset_rows), train_dqn.RESET_RETRY_LIMIT)
        self.assertEqual([row["attempt"] for row in reset_rows],
                         [str(index) for index in
                          range(1, train_dqn.RESET_RETRY_LIMIT + 1)])
        self.assertTrue(all(row["episode"] == "0" for row in reset_rows))
        self.assertTrue(all(row["total_steps"] == "0" for row in reset_rows))
        self.assertTrue(all(row["reset_failure_reason"] ==
                            "start_clearance_below_min" for row in reset_rows))
        self.assertEqual([row["start_obstacle_min"] for row in reset_rows],
                         ["0.180"] * train_dqn.RESET_RETRY_LIMIT)
        self.assertTrue({"episode", "attempt", "scenario_id",
                         "reset_failure_reason", "start_dist",
                         "start_obstacle_min", "total_steps"} <=
                        set(reset_rows[0].keys()))
        # 每次重试换下一个场景：轮换游标被推进
        self.assertEqual([row["scenario_id"] for row in reset_rows],
                         ["s-0", "s-1", "s-2"])
        # training_log 不含 reset_failed 行，且失败的 reset 未占用 episode 序号
        self.assertEqual([row["episode"] for row in train_rows], ["0", "1"])
        self.assertEqual([row["result"] for row in train_rows],
                         ["success", "success"])
        self.assertNotIn("reset_failed",
                         [row["done_reason"] for row in train_rows])
        # 被跳过的 3 次失败尝试仍消耗了场景轮换位置，成功后从新游标继续
        self.assertEqual(env.armed, ["s-0", "s-1", "s-2", "s-3", "s-0"])
        # 备份逃逸计数可从 step_log 复查（不再只存在于 env info）
        self.assertEqual([row["shield_escape"] for row in step_rows], ["1", "1"])
        self.assertEqual([row["shield_escape_count"] for row in step_rows],
                         ["2", "2"])
        self.assertEqual([row["escape_margin"] for row in step_rows],
                         ["0.5000", "0.5000"])


if __name__ == "__main__":
    unittest.main()
