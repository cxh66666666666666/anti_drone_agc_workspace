#!/usr/bin/env python3
"""
@file: gazebo_env.py
@brief: Gazebo 强化学习局部避障环境封装

分层边界：
    - 既有 LQR 参考轨迹负责全局路径/前进。
    - RL 只输出局部避障 offset，不规划全局路线。
"""

import math
import random
import hashlib
import json

import rospy
from geometry_msgs.msg import Pose, Pose2D, PoseStamped, TwistStamped, Vector3Stamped
from nav_msgs.msg import Path as NavPath
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import (GetWorldProperties, SetModelState,
                             SpawnModel, DeleteModel)
from std_msgs.msg import Float64, Int32
from std_srvs.srv import Empty, Trigger

from rl_obstacle_avoidance.msg import ObstacleFeatures
from rl_obstacle_avoidance.srv import PlanPath
from path_planning import is_goal_valid
from training_utils import (action_reversal_cost, action_swept_clearance,
                            allowed_avoidance_actions, clearance_change_reward,
                            avoidance_completion_allowed,
                            corridor_aware_clearance,
                            avoidance_quality_reward,
                            avoidance_magnitude,
                            constrain_avoidance_action, directional_clearance,
                            episode_is_success,
                            lateral_clearances, limit_change,
                            local_control_profile,
                            motion_frame_offset, motion_heading,
                            path_lookahead_features,
                            path_swept_sectors,
                            project_path_progress,
                            proximity_penalty, quaternion_to_yaw,
                            avoidance_tail_gate,
                            retain_obstacle_sectors,
                            reset_recovery_timeout,
                            rotate_sectors_to_heading, smooth_value,
                            time_to_collision,
                            update_avoidance_event,
                            update_settle_progress,
                            validate_distance_thresholds,
                            wrap_angle)
from geometry_metrics import (DEFAULT_DRONE_HALF_EXTENTS,
                              DEFAULT_OBSTACLE_SIZE,
                              minimum_geometry_clearance)
from metrics_utils import summarize_episode
from safety_policy import select_a1_action, select_p_action


ACTION_NAMES = {
    0: "沿原路径",
    1: "左移",
    2: "右移",
    3: "左移并上升",
    4: "右移并上升",
    5: "上升悬停",
}

# These terminal states can leave the vehicle/controller far from the
# episode start.  A normal setpoint reset is not sufficient in that case;
# the next episode must first hold the trajectory while the pose is restored.
RESET_RECOVERY_DONE_REASONS = frozenset((
    "collision", "shield_infeasible", "diverged", "low_altitude",
    "perception_timeout",
))
N_ACTIONS = 6
PATH_FEATURES = 5  # 前瞻可达性特征维度
PATH_FEATURE_SCALE_M = 2.0  # 侧向净空特征的饱和量程 (m)


class GazeboEnv:
    """
    @brief: DQN 局部避障环境
    @detail: action 只转换为瞬时局部 offset；LQR 继续跟踪原参考轨迹。
    """

    def __init__(self):
        try:
            rospy.init_node("gazebo_env", anonymous=True, disable_signals=True)
        except rospy.ROSException:
            pass

        # 动作与安全参数
        self.step_z = rospy.get_param("~step_z", 0.3)
        self.min_height = rospy.get_param("~min_height", 1.0)
        # 实验变量支持通过 /gazebo_env/<name> 注入（start_sim.sh 的 rosparam set 路径），
        # 未设置全局参数时使用对应私有参数。
        # 默认 0.3 为跨场景验证值：固定 2 杆 100% / 6 杆 98%。
        # 0.05 仅在 6 杆场景略优（100%）但使固定场景退化（52%），只作场景专用注入，不做默认。
        self.min_offset_xy = rospy.get_param(
            "/gazebo_env/min_offset_xy",
            rospy.get_param("~min_offset_xy", 0.3))
        self.max_offset_xy = rospy.get_param(
            "/gazebo_env/max_offset_xy",
            rospy.get_param("~max_offset_xy", 1.4))
        self.offset_smooth_alpha = rospy.get_param("~offset_smooth_alpha", 0.8)
        self.max_offset_change = rospy.get_param("~max_offset_change", 0.25)
        self.heading_min_speed = rospy.get_param("~heading_min_speed", 0.1)
        self.turning_front_extra_angle = rospy.get_param(
            "~turning_front_extra_angle", math.pi / 6.0)
        if not 0.0 <= self.turning_front_extra_angle <= math.pi / 3.0:
            raise ValueError("turning_front_extra_angle must be in [0, pi/3]")
        self.reference_velocity_timeout = rospy.get_param(
            "~reference_velocity_timeout", 0.5)
        self.avoidance_lookahead_time = rospy.get_param(
            "/lqr_tracker_node/avoidance_lookahead_time", 2.0)
        if self.reference_velocity_timeout <= 0.0:
            raise ValueError("reference_velocity_timeout must be positive")
        if self.avoidance_lookahead_time < 0.0:
            raise ValueError("avoidance_lookahead_time must be non-negative")
        self.num_sectors = rospy.get_param(
            "/perception_node/num_sector", 12)

        # LiDAR 距离为 perception_node 输出的归一化值 [0, 1]
        self.lidar_max_range = rospy.get_param("~lidar_max_range", 10.0)
        self.lidar_min_range = rospy.get_param("~lidar_min_range", 0.4)
        self.warning_distance_norm = rospy.get_param(
            "~warning_distance_norm", 0.30)
        self.safe_distance_norm = rospy.get_param("~safe_distance_norm", 0.24)
        self.emergency_dist_norm = rospy.get_param("~emergency_dist_norm", 0.05)
        # 动作接口模式：masked 保持距离掩码（legacy 逐字节不变），raw 让掩码退出裁决。
        self.action_interface = str(rospy.get_param(
            "/gazebo_env/action_interface",
            rospy.get_param("~action_interface", "masked"))).lower()
        if self.action_interface not in ("masked", "raw"):
            raise ValueError("action_interface must be masked or raw")
        self.perception_timeout = rospy.get_param("~perception_timeout", 0.5)
        self.obstacle_memory_timeout = rospy.get_param(
            "~obstacle_memory_timeout", 0.4)
        if self.obstacle_memory_timeout < 0.0:
            raise ValueError("obstacle_memory_timeout must be non-negative")
        self.action_duration = rospy.get_param("~action_duration", 0.1)
        if self.perception_timeout <= 0.0 or self.action_duration <= 0.0:
            raise ValueError(
                "perception_timeout and action_duration must be positive")
        self.safety_filter_mode = str(rospy.get_param(
            "/gazebo_env/safety_filter_mode",
            rospy.get_param("~safety_filter_mode", "disabled"))).lower()
        if self.safety_filter_mode not in ("disabled", "a1", "p"):
            raise ValueError("safety_filter_mode must be disabled, a1 or p")
        self.safety_filter_base_margin = float(rospy.get_param(
            "/gazebo_env/safety_filter_base_margin",
            rospy.get_param("~safety_filter_base_margin", 0.0)))
        self.safety_filter_kappa = float(rospy.get_param(
            "/gazebo_env/safety_filter_kappa",
            rospy.get_param("~safety_filter_kappa", 0.2)))
        self.safety_filter_prediction_error = float(rospy.get_param(
            "/gazebo_env/safety_filter_prediction_error",
            rospy.get_param("~safety_filter_prediction_error", 0.0)))
        self.safety_filter_horizon = float(rospy.get_param(
            "/gazebo_env/safety_filter_horizon_s",
            rospy.get_param("~safety_filter_horizon_s", 1.0)))
        self.safety_filter_dt = float(rospy.get_param(
            "/gazebo_env/safety_filter_dt",
            rospy.get_param("~safety_filter_dt", 0.1)))
        self.safety_filter_alpha = float(rospy.get_param(
            "/gazebo_env/safety_filter_alpha",
            rospy.get_param("~safety_filter_alpha", 1.0)))
        self.safety_filter_epsilon_pred = float(rospy.get_param(
            "/gazebo_env/safety_filter_epsilon_pred",
            rospy.get_param("~safety_filter_epsilon_pred", 0.0)))
        # 预测用的横向/纵向速度必须是真实可达的平移速度，不能复用偏移量：
        # min_offset_xy(0.3)/step_z(0.3) 是 LQR 偏移目标的幅值区间下界，
        # 控制器每步最多变化 max_offset_change(0.25m)/action_duration(0.1s)，
        # 真实横向逃逸能力远大于 0.3 m/s，按偏移量预测会低估避障能力导致空集。
        # 默认取 max_offset_xy(1.4)：横向偏移上限即单步可达的横向速度上限。
        self.safety_filter_lateral_speed = float(rospy.get_param(
            "/gazebo_env/safety_filter_lateral_speed",
            rospy.get_param("~safety_filter_lateral_speed",
                            self.max_offset_xy)))
        self.safety_filter_vertical_speed = float(rospy.get_param(
            "/gazebo_env/safety_filter_vertical_speed",
            rospy.get_param("~safety_filter_vertical_speed", self.step_z)))
        # 默认逃逸档：安全集为空时优先在可行集内执行备份逃逸。置 True 为消融档，
        # 退回 §4.5 规则 3 严格终止并记 shield_infeasible，不得回退执行危险动作。
        self.safety_filter_empty_set_terminal = bool(rospy.get_param(
            "/gazebo_env/safety_filter_empty_set_terminal",
            rospy.get_param("~safety_filter_empty_set_terminal", False)))
        if (self.safety_filter_base_margin < 0.0 or
                self.safety_filter_kappa < 0.0 or
                self.safety_filter_prediction_error < 0.0 or
                self.safety_filter_horizon < 0.0 or
                self.safety_filter_dt <= 0.0 or
                not 0.0 < self.safety_filter_alpha <= 1.0 or
                self.safety_filter_epsilon_pred < 0.0 or
                self.safety_filter_lateral_speed <= 0.0 or
                self.safety_filter_vertical_speed <= 0.0):
            raise ValueError("invalid safety filter parameters")
        self.collision_dist = rospy.get_param("~collision_dist", 0.65)
        # 机体包络接触阈值：唯一碰撞终止判据（0 表示包络接触即碰撞）
        self.contact_dist = rospy.get_param("~contact_dist", 0.0)
        self.path_clearance_margin = rospy.get_param(
            "/gazebo_env/path_clearance_margin",
            rospy.get_param("~path_clearance_margin", 0.30))
        if self.path_clearance_margin < 0.0:
            raise ValueError("path_clearance_margin must be non-negative")
        self.path_feature_scale_m = PATH_FEATURE_SCALE_M
        validate_distance_thresholds(
            self.lidar_min_range,
            self.collision_dist,
            self.emergency_dist_norm * self.lidar_max_range,
            self.safe_distance_norm * self.lidar_max_range,
            contact_dist=self.contact_dist)
        if self.warning_distance_norm <= self.safe_distance_norm:
            raise ValueError(
                "warning_distance_norm must exceed safe_distance_norm")

        # Episode 与奖励参数
        self.max_episode_steps = rospy.get_param("~max_episode_steps", 500)
        self.safe_reward = rospy.get_param("~safe_reward", 0.02)
        self.step_penalty = rospy.get_param("~step_penalty", 0.01)
        self.action_change_penalty = rospy.get_param("~action_change_penalty", 0.005)
        self.action_reversal_penalty = rospy.get_param(
            "~action_reversal_penalty", 0.03)
        self.action_hold_steps = rospy.get_param("~action_hold_steps", 4)
        self.clearance_change_scale = rospy.get_param(
            "~clearance_change_scale", 0.20)
        self.ttc_penalty_scale = rospy.get_param("~ttc_penalty_scale", 0.20)
        self.ttc_safe_seconds = rospy.get_param("~ttc_safe_seconds", 2.0)
        self.unnecessary_action_penalty = rospy.get_param(
            "~unnecessary_action_penalty", 0.005)
        self.goal_reached_reward = rospy.get_param(
            "~goal_reached_reward", 8.0)
        self.avoidance_success_reward = rospy.get_param(
            "~avoidance_success_reward", 4.0)
        self.terminal_failure_penalty = rospy.get_param(
            "~terminal_failure_penalty", 20.0)
        self.safe_proximity_penalty = rospy.get_param(
            "~safe_proximity_penalty", 0.10)
        self.emergency_proximity_penalty = rospy.get_param(
            "~emergency_proximity_penalty", 0.50)
        if self.emergency_proximity_penalty <= self.safe_reward - self.step_penalty:
            raise ValueError(
                "emergency_proximity_penalty must exceed the base step reward")
        self.avoidance_clear_steps = rospy.get_param(
            "~avoidance_clear_steps", 3)
        self.avoidance_tail_completion_norm = rospy.get_param(
            "~avoidance_tail_completion_norm", 0.12)
        self.avoidance_tail_release_norm = rospy.get_param(
            "~avoidance_tail_release_norm", 0.16)
        self.avoidance_completion_grace_steps = rospy.get_param(
            "~avoidance_completion_grace_steps", 60)
        if not (self.emergency_dist_norm <
                self.avoidance_tail_completion_norm <=
                self.avoidance_tail_release_norm <
                self.safe_distance_norm):
            raise ValueError(
                "avoidance tail thresholds must satisfy emergency < completion <= release < safe")
        if self.avoidance_completion_grace_steps < 0:
            raise ValueError("avoidance_completion_grace_steps must be non-negative")
        self.episode_budget_factor = rospy.get_param(
            "~episode_budget_factor", 2.0)
        if self.episode_budget_factor <= 0.0:
            raise ValueError("episode_budget_factor must be positive")
        self.avoidance_tracking_tolerance = rospy.get_param(
            "~avoidance_tracking_tolerance", 1.0)
        self.tracking_error_weight = rospy.get_param(
            "/gazebo_env/tracking_error_weight",
            rospy.get_param("~tracking_error_weight", 0.1))
        self.max_tracking_error = rospy.get_param("~max_tracking_error", 10.0)
        self.max_terminal_tracking_error = rospy.get_param("~max_terminal_tracking_error", 30.0)
        self.reward_clip_abs = rospy.get_param("~reward_clip_abs", 30.0)
        self.w_progress = rospy.get_param("~w_progress", 0.5)
        self.risk_progress_scale = rospy.get_param("~risk_progress_scale", 0.25)

        # Reset 起点默认跟随 LQR 当前轨迹首点，也允许训练参数单独覆盖。
        # 已解锁飞行时优先通过 setpoint 返回起点，避免 Gazebo 传送与 PX4 EKF 状态脱节。
        self.reset_mode = rospy.get_param("~reset_mode", "setpoint")
        self.model_name = rospy.get_param("~model_name", "iris")
        self._explicit_start_x = rospy.has_param("~start_x")
        self._explicit_start_y = rospy.has_param("~start_y")
        self._explicit_start_z = rospy.has_param("~start_z")
        self.start_x = 0.0
        self.start_y = 0.0
        self.start_z = 2.0
        self._update_reset_start_from_params()
        self.start_yaw = rospy.get_param("~start_yaw", 0.0)
        self.reset_tolerance = rospy.get_param("~reset_tolerance", 0.5)
        self.moving_reset_tolerance = rospy.get_param("~moving_reset_tolerance", 1.0)
        self.reset_velocity_tolerance = rospy.get_param(
            "~reset_velocity_tolerance", 0.25)
        if self.reset_velocity_tolerance < 0.0:
            raise ValueError("reset_velocity_tolerance must be non-negative")
        self.reset_attempts = rospy.get_param("~reset_attempts", 3)
        self.reset_wait_timeout = rospy.get_param("~reset_wait_timeout", 20.0)
        self.preplan_settle_timeout = rospy.get_param(
            "~preplan_settle_timeout", 10.0)
        self.preplan_speed_tolerance = rospy.get_param(
            "~preplan_speed_tolerance", 0.15)
        self.preplan_position_tolerance = rospy.get_param(
            "~preplan_position_tolerance", 0.20)
        self.preplan_stable_steps = rospy.get_param(
            "~preplan_stable_steps", 10)
        self.preplan_attempts = rospy.get_param("~preplan_attempts", 2)
        # 复杂环境目标模式配置
        self.goal_mode = rospy.get_param("/dqn_training/goal_mode", False)
        self.goal_bounds = rospy.get_param(
            "/dqn_training/goal_bounds", [-11.0, 11.0, -11.0, 11.0])
        self.goal_reached_dist = rospy.get_param("~goal_reached_dist", 1.0)
        default_goal_clearance = (
            self.emergency_dist_norm * self.lidar_max_range +
            self.goal_reached_dist + 0.15)
        self.goal_min_clearance = rospy.get_param(
            "~goal_min_clearance", default_goal_clearance)
        self.goal_min_distance = rospy.get_param("~goal_min_distance", 5.0)
        self.goal_max_distance = rospy.get_param("~goal_max_distance", 0.0)
        self.astar_safety_margin = rospy.get_param(
            "~astar_safety_margin", 1.8)
        self.waypoint_speed = rospy.get_param("~waypoint_speed", 1.02)
        self.avoidance_event_reward = rospy.get_param(
            "~avoidance_event_reward", 8.0)
        self.avoidance_quality_bonus = rospy.get_param(
            "~avoidance_quality_bonus", 10.0)
        self.max_avoidance_reward_events = rospy.get_param(
            "~max_avoidance_reward_events", 5)
        self.avoidance_rearm_progress = rospy.get_param(
            "~avoidance_rearm_progress", 2.0)
        if (self.max_avoidance_reward_events <= 0 or
                self.avoidance_rearm_progress < 0.0):
            raise ValueError(
                "avoidance event count must be positive and rearm progress non-negative")
        if min(self.goal_reached_reward, self.avoidance_success_reward,
               self.avoidance_event_reward,
               self.avoidance_quality_bonus) < 0.0:
            raise ValueError("success and avoidance rewards must be non-negative")
        # 几何尺寸/包络在加载场景(ID 计算依赖 obstacle_geometry_size)前先赋值
        self.obstacle_geometry_size = tuple(rospy.get_param(
            "~obstacle_geometry_size", DEFAULT_OBSTACLE_SIZE))
        self.drone_half_extents = tuple(rospy.get_param(
            "~drone_half_extents", DEFAULT_DRONE_HALF_EXTENTS))
        # 从 /dqn_training/* 加载场景（障碍物、轨迹、高度、ID），换场景时可复用
        self._load_scenario_from_params()
        self.goal = None
        self.path_length = 0.0
        self._max_steps_cap = self.max_episode_steps
        # 轨迹完成判据所需的有效轨迹参数与真实相位（由 _reload_lqr_trajectory 持久化）
        self.trajectory_duration_s = None
        self.trajectory_loop = False
        self.trajectory_phase_s = 0.0
        self._phase_sample_stamp = None
        self.episode_b_encounter_steps = 0
        self.reset_min_clearance_norm = rospy.get_param(
            "~reset_min_clearance_norm", self.safe_distance_norm)
        # reset_world 会回拨仿真时间，可能破坏已运行的 PX4/EKF 状态。
        self.use_reset_world = rospy.get_param("~use_reset_world", False)
        self.trial_id = "unstarted"
        self.reset_count = 0
        # Keep hard-recovery intent across a failed reset.  Resetting
        # ``last_done_reason`` to ``running`` is necessary for a fresh
        # episode, but must not make the next retry fall back to setpoint-only
        # recovery after a shield/collision/divergence terminal.
        self._reset_recovery_pending = False

        self.current_pose = None
        self.current_velocity = None
        self.current_velocity_stamp = None
        self.reference_velocity = None
        self.reference_velocity_stamp = None
        self.reference_mid_lookahead_velocity = None
        self.reference_mid_lookahead_velocity_stamp = None
        self.reference_lookahead_velocity = None
        self.reference_lookahead_velocity_stamp = None
        self.features = None
        self.features_stamp = None
        self.position_error = None
        # 时间戳校验：记录各路上一帧时间戳，用于单调递增检查
        self._last_pose_stamp = None
        self._last_velocity_stamp = None
        self._last_features_stamp = None
        self._reset_started_stamp = None

        self.episode_step = 0
        self.cumulative_reward = 0.0
        self.last_action = 0
        self.current_action = 0
        self.last_offset_x = 0.0
        self.last_offset_y = 0.0
        self.last_offset_z = 0.0
        self.motion_heading_world = self.start_yaw
        self._last_sector_features = None
        self._sector_memory = None
        self._sector_memory_stamp = None
        self._sector_memory_body_yaw = None
        self.locked_avoid_side = 0
        self.selected_action = 0
        self.filter_reason = ""  # 仅安全过滤器（A1/P）启用时写入过滤原因
        self.action_reason = ""  # 动作层决策原因，B0/B1/B2 与 A1/P 共用
        self.predicted_h = float("nan")
        self.prediction_error = 0.0
        self.filter_sensor_fresh = False
        self.filter_margin = 0.0
        self.filter_safe_actions = ()
        self.filter_feasible_actions = ()
        self.filter_candidate_actions = ()
        self.filter_rule_actions = ()
        self.action_cost = 0.0
        self.shield_escape = 0        # 本步是否执行了备份逃逸（0/1）
        self.shield_escape_count = 0  # 回合内累计逃逸次数
        self.escape_margin = float("nan")
        self.filter_emergency = False      # 本步决策时刻的 emergency 布尔值
        self.filter_hold_remaining = 0     # 本步决策时刻的保持窗口剩余步数
        # 同量交叉验证参考量：过滤器所用门控障碍集下的瞬时净空（不减 margin），
        # 与 predicted_h + margin 属同一物理量，用于 §3.3 预测误差比较。
        self.filter_clearance_now = float("nan")
        self.last_done_reason = "running"
        self._episode_done = False
        self.collision_event_count = 0
        self.episode_min_front = 1.0
        self.episode_min_obstacle = 1.0
        self._collision_streak = 0
        self._collision_latched = False
        self.reset_ok = False
        self.reset_failure_reason = "not_started"
        self.start_dist = float("inf")
        self.start_obstacle_min = 1.0
        self.obstacle_encountered = False
        self.avoidance_active = False
        self.avoidance_clear_count = 0
        self.avoidance_complete = False
        self.avoidance_event_count = 0
        self.avoidance_event_min_clearance = 1.0
        self.avoidance_quality = 0.0
        self.avoidance_quality_sum = 0.0
        self.avoidance_reward_earned = 0.0
        self.last_avoidance_goal_distance = None
        self.avoidance_start_position = None
        self.avoidance_start_heading = self.start_yaw
        self.avoidance_tail_seen = False
        self.prev_dist_to_target = None
        self.path_points = []
        self.prev_path_progress = None
        self.prev_directional_clearances = None
        self.previous_swept_clearance = None
        self.action_hold_remaining = 0
        self.commanded_action = 0
        self.action_switch_count = 0
        self.action_reversal_count = 0
        self.local_speed_scale = 1.0
        self.trajectory_paused = False
        self.travel_distance = 0.0
        self.last_step_position = None
        self.metric_times = []
        self.metric_errors = []
        self.metric_positions = []
        self.recovery_start_time = None
        self.a_encounter_steps = 0
        self.collision_events_a = 0
        self.collision_events_b = 0
        self.last_classified_collision_event = 0
        self.geometry_collision_events = 0
        self.geometry_collision_latched = False
        self.episode_min_geometry_clearance = float("inf")

        rospy.Subscriber("/obstacle_features", ObstacleFeatures,
                         self._features_cb, queue_size=1)
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped,
                         self._pose_cb, queue_size=1)
        rospy.Subscriber("/mavros/local_position/velocity_local",
                         TwistStamped, self._velocity_cb, queue_size=1)
        rospy.Subscriber("/lqr_tracker/reference_velocity",
                         Vector3Stamped, self._reference_velocity_cb,
                         queue_size=1)
        rospy.Subscriber("/lqr_tracker/reference_lookahead_velocity",
                         Vector3Stamped, self._reference_lookahead_velocity_cb,
                         queue_size=1)
        rospy.Subscriber("/lqr_tracker/reference_mid_lookahead_velocity",
                         Vector3Stamped,
                         self._reference_mid_lookahead_velocity_cb,
                         queue_size=1)
        rospy.Subscriber("/lqr_tracker/position_error", Float64,
                         self._pos_err_cb, queue_size=1)

        self.action_pub = rospy.Publisher("/rl_planner/action", Int32, queue_size=1)
        self.local_offset_pub = rospy.Publisher(
            "/rl_planner/local_offset", Vector3Stamped, queue_size=1)
        self.local_control_pub = rospy.Publisher(
            "/rl_planner/local_control", Vector3Stamped, queue_size=1)

        self.waypoint_pub = rospy.Publisher(
            "/lqr_tracker/waypoints", NavPath, queue_size=1)
        self.astar_srv = None
        if self.goal_mode:
            try:
                self.astar_service_timeout = rospy.get_param(
                    "~astar_service_timeout", 15.0)
                rospy.wait_for_service(
                    "/astar_planner/plan", timeout=self.astar_service_timeout)
                self.astar_srv = rospy.ServiceProxy(
                    "/astar_planner/plan", PlanPath)
            except rospy.ROSException:
                rospy.logwarn("[gazebo_env] 未找到 A* 规划服务，退回无目标模式")
                self.goal_mode = False
        else:
            self.astar_service_timeout = rospy.get_param(
                "~astar_service_timeout", 15.0)

        self.reset_world_srv = rospy.ServiceProxy("/gazebo/reset_world", Empty)
        self.get_world_properties_srv = rospy.ServiceProxy(
            "/gazebo/get_world_properties", GetWorldProperties)
        self.set_model_state_srv = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
        self.reset_trajectory_srv = rospy.ServiceProxy(
            "/lqr_tracker/reset_trajectory", Trigger)
        self.reload_trajectory_srv = rospy.ServiceProxy(
            "/lqr_tracker/reload_trajectory", Trigger)
        self.spawn_model_srv = rospy.ServiceProxy(
            "/gazebo/spawn_sdf_model", SpawnModel)
        self.delete_model_srv = rospy.ServiceProxy(
            "/gazebo/delete_model", DeleteModel)

        # 训练入口能力：跨场景轮换 + 每集严格相位复位（默认关闭，向后兼容）
        self._scenario_rotation = rospy.get_param("~scenario_rotation", False)
        self._phase_aligned_reset = rospy.get_param(
            "~phase_aligned_reset", False)
        self._deployment_pending = False
        self._next_scenario = None
        self._spawned_models = []
        self._deployment_serial = 0

        rospy.loginfo("[gazebo_env] 局部避障环境初始化完成 | n_actions=%d | reset_start=(%.2f, %.2f, %.2f)",
                      N_ACTIONS, self.start_x, self.start_y, self.start_z)

    def _pose_cb(self, msg):
        self.current_pose = msg

    def _features_cb(self, msg):
        self.features = msg
        self.features_stamp = (msg.header.stamp
                               if msg.header.stamp != rospy.Time(0)
                               else rospy.Time.now())
        if (msg is not None and msg.lidar_sectors and
                len(msg.lidar_sectors) != self.num_sectors):
            rospy.logerr_throttle(
                5, "[gazebo_env] 扇区数量不一致: configured=%d received=%d",
                self.num_sectors, len(msg.lidar_sectors))

    def _velocity_cb(self, msg):
        self.current_velocity = msg.twist.linear if msg is not None else None
        self.current_velocity_stamp = (msg.header.stamp
                                       if msg is not None else None)

    def _reference_velocity_cb(self, msg):
        self.reference_velocity = msg.vector if msg is not None else None
        self.reference_velocity_stamp = (msg.header.stamp
                                         if msg is not None else None)

    def _reference_lookahead_velocity_cb(self, msg):
        self.reference_lookahead_velocity = (
            msg.vector if msg is not None else None)
        self.reference_lookahead_velocity_stamp = (
            msg.header.stamp if msg is not None else None)

    def _reference_mid_lookahead_velocity_cb(self, msg):
        self.reference_mid_lookahead_velocity = (
            msg.vector if msg is not None else None)
        self.reference_mid_lookahead_velocity_stamp = (
            msg.header.stamp if msg is not None else None)

    def _pos_err_cb(self, msg):
        self.position_error = msg.data if msg is not None else float("nan")

    def _update_reset_start_from_params(self):
        self.trajectory_type = rospy.get_param(
            "/lqr_tracker_node/trajectory_type", "fixed_horizontal")
        if self.trajectory_type == "step":
            default_start_x = rospy.get_param("/lqr_tracker_node/step_x", 0.0)
            default_start_y = rospy.get_param("/lqr_tracker_node/step_y", 0.0)
            default_start_z = rospy.get_param("/lqr_tracker_node/step_z", 5.0)
        else:
            default_start_x = rospy.get_param("/lqr_tracker_node/start_x", 0.0)
            default_start_y = rospy.get_param("/lqr_tracker_node/start_y", 0.0)
            default_start_z = rospy.get_param("/lqr_tracker_node/start_z", 2.0)
        self.start_x = rospy.get_param("~start_x", default_start_x) if self._explicit_start_x else default_start_x
        self.start_y = rospy.get_param("~start_y", default_start_y) if self._explicit_start_y else default_start_y
        self.start_z = rospy.get_param("~start_z", default_start_z) if self._explicit_start_z else default_start_z

    # ------------------------------------------------------------------
    # 训练入口能力：跨场景轮换 + 每集严格相位复位
    # ------------------------------------------------------------------
    @staticmethod
    def _as_coord_list(raw):
        """把障碍物参数规范成 [[x,y],...] 列表（兼容 JSON 字符串/列表两种形态）。"""
        if isinstance(raw, str):
            try:
                raw = json.loads(raw)
            except (TypeError, ValueError):
                raw = []
        return raw if isinstance(raw, list) else []

    @staticmethod
    def _as_blackout_window(raw):
        """把传感器失效窗口规范成 (t0, t1)；缺失或非法时返回空元组。"""
        if isinstance(raw, str):
            try:
                raw = json.loads(raw)
            except (TypeError, ValueError):
                return ()
        if not isinstance(raw, (list, tuple)) or len(raw) != 2:
            return ()
        try:
            start, end = float(raw[0]), float(raw[1])
        except (TypeError, ValueError):
            return ()
        return (start, end) if end > start else ()

    def _load_scenario_from_params(self):
        """从 /dqn_training/* 读取场景参数并刷新内部缓存（换场景时复用）。"""
        obstacles_a_param = rospy.get_param("/dqn_training/obstacles_a", [])
        obstacles_b_param = rospy.get_param("/dqn_training/obstacles_b", [])
        # 兼容 obstacles_a/b 可能是 JSON 字符串（旧写入/残留）或已是列表
        self.obstacles_a = [(float(p[0]), float(p[1]))
                            for p in self._as_coord_list(obstacles_a_param)]
        self.obstacles_b = [(float(p[0]), float(p[1]))
                            for p in self._as_coord_list(obstacles_b_param)]
        scenario_obstacles_json = rospy.get_param(
            "/dqn_training/scenario_obstacles_json", "")
        self.scenario_obstacles_typed = []
        if scenario_obstacles_json:
            try:
                self.scenario_obstacles_typed = (
                    json.loads(scenario_obstacles_json)
                    if isinstance(scenario_obstacles_json, str)
                    else scenario_obstacles_json)
            except (TypeError, ValueError):
                rospy.logwarn(
                    "[gazebo_env] 无法解析 typed scenario obstacles，回退旧格式")
        if self.scenario_obstacles_typed:
            self.obstacle_geometry_centers = [
                tuple(float(v) for v in item["center"])
                for item in self.scenario_obstacles_typed]
        else:
            self.obstacle_geometry_centers = [
                (float(p[0]), float(p[1]),
                 float(p[2]) if len(p) > 2 else self.start_z)
                for p in (self._as_coord_list(obstacles_a_param) +
                          self._as_coord_list(obstacles_b_param))]
        self.scenario_obstacles = rospy.get_param(
            "/dqn_training/scenario_obstacles", "unknown")
        self.scenario_trajectory = rospy.get_param(
            "/dqn_training/trajectory",
            rospy.get_param("/lqr_tracker_node/trajectory_type",
                            "fixed_horizontal"))
        self.scenario_altitude = rospy.get_param(
            "/dqn_training/altitude", self.start_z)
        traj_json = rospy.get_param("/dqn_training/trajectory_json", "{}")
        if isinstance(traj_json, str):
            try:
                traj_json = json.loads(traj_json)
            except (TypeError, ValueError):
                traj_json = {}
        self.scenario_trajectory_json = (
            traj_json if isinstance(traj_json, dict) else {})
        # 主动注入的传感器失效窗口（无该参数时为 ()，窗口内感知按不新鲜处理）
        self.sensor_blackout_window = self._as_blackout_window(
            rospy.get_param("/dqn_training/sensor_blackout_s", []))
        self.scenario_id = self._make_scenario_id()

    def arm_deploy(self, scenario):
        """登记待部署的场景，下一次 reset 时动态部署（跨场景轮换用）。"""
        self._next_scenario = scenario
        self._deployment_pending = True

    def set_scenario_rotation(self, enabled=True):
        self._scenario_rotation = bool(enabled)

    def set_phase_aligned(self, enabled=True):
        self._phase_aligned_reset = bool(enabled)

    @staticmethod
    def _requires_reset_recovery(done_reason):
        """Return whether the previous terminal state needs hard recovery."""
        return done_reason in RESET_RECOVERY_DONE_REASONS

    def _compute_t0_start(self):
        """按场景轨迹类型计算 t=0 起点坐标 (x,y,z)。"""
        traj = self.scenario_trajectory_json or {}
        typ = self.scenario_trajectory
        if typ == "linear":
            start = traj.get("start") or [self.start_x, self.start_y,
                                          self.start_z]
            z = float(start[2]) if len(start) > 2 else self.start_z
            return (float(start[0]), float(start[1]), z)
        if typ == "circular":
            center = traj.get("center") or [0.0, 0.0, self.start_z]
            radius = float(traj.get("radius_m", 0.0))
            z = (self.scenario_altitude if len(center) < 3
                 else float(center[2]))
            return (float(center[0]) + radius, float(center[1]), z)
        if typ == "figure_eight":
            center = traj.get("center") or [0.0, 0.0]
            z = float(traj.get("height_m", self.scenario_altitude))
            return (float(center[0]), float(center[1]), z)
        # step 及其它：保留现有起点
        return (self.start_x, self.start_y, self.start_z)

    def _set_model_at(self, x, y, z):
        """把无人机模型搬到指定坐标（严格相位复位用）。"""
        try:
            rospy.wait_for_service("/gazebo/set_model_state", timeout=2.0)
            state = ModelState()
            state.model_name = self._resolve_model_name()
            state.reference_frame = "world"
            state.pose.position.x = float(x)
            state.pose.position.y = float(y)
            state.pose.position.z = float(z)
            state.pose.orientation.z = math.sin(self.start_yaw * 0.5)
            state.pose.orientation.w = math.cos(self.start_yaw * 0.5)
            # Teleporting only the pose leaves Gazebo/PX4 with the previous
            # episode's velocity.  Clear the twist atomically with the pose.
            state.twist.linear.x = 0.0
            state.twist.linear.y = 0.0
            state.twist.linear.z = 0.0
            state.twist.angular.x = 0.0
            state.twist.angular.y = 0.0
            state.twist.angular.z = 0.0
            res = self.set_model_state_srv(state)
            if not res.success:
                rospy.logwarn("[gazebo_env] set_model_state 失败: %s",
                              res.status_message)
                return False
            self.current_pose = None
            self.current_velocity = None
            self.current_velocity_stamp = None
            self.features = None
            self.features_stamp = None
            self.position_error = None
            rospy.sleep(0.5)
            return True
        except (rospy.ROSException, rospy.ServiceException) as exc:
            rospy.logwarn("[gazebo_env] set_model_state 不可用: %s", exc)
            return False

    def _world_model_names(self):
        """返回 Gazebo 当前模型名；查询失败返回 ``None``。"""
        try:
            rospy.wait_for_service("/gazebo/get_world_properties", timeout=1.0)
            getter = getattr(self, "get_world_properties_srv", None)
            if getter is None:
                return None
            world = getter()
            # Preserve Gazebo's deterministic response order for reproducible
            # cleanup logs/tests; callers use a set for membership checks.
            return list(world.model_names)
        except (rospy.ROSException, rospy.ServiceException, AttributeError) as exc:
            rospy.logwarn("[gazebo_env] 无法查询 Gazebo 模型列表: %s", exc)
            return None

    def _delete_spawned_models(self):
        """删除本环境动态生成的全部障碍物 model。

        先读取 Gazebo 实际模型集合，只对确实存在的模型调用删除服务。
        这样外部清理/重启导致的已不存在模型会直接从跟踪列表移除，
        不会再向 Gazebo 发送必然产生 ``model does not exist`` 错误的请求。
        """
        model_names = self._world_model_names()
        if model_names is None:
            # 无法确认实际状态时保持跟踪列表，交由下一次 reset 重试；
            # 更不能继续 spawn，避免 Gazebo/RViz 进入半成功状态。
            return False

        tracked = list(self._spawned_models)
        available = set(model_names)
        # A previous trainer process may have exited after spawning a model;
        # discover its deterministic names so the next run cannot collide.
        names = list(tracked)
        names.extend(name for name in model_names
                     if name.startswith("scenario_") and name not in names)
        existing = [name for name in names if name in available]
        if not existing:
            self._spawned_models = []
            return True
        try:
            rospy.wait_for_service("/gazebo/delete_model", timeout=2.0)
        except (rospy.ROSException, rospy.ServiceException) as exc:
            rospy.logwarn("[gazebo_env] delete_model 服务不可用: %s", exc)
            return False
        failed = []
        for name in existing:
            try:
                resp = self.delete_model_srv(name)
                if not resp.success:
                    status = str(resp.status_message or "")
                    # The model may disappear between the world query and
                    # this service call (another reset/cleanup can race us).
                    # Treat that terminal state as the desired result.
                    if "does not exist" not in status.lower():
                        rospy.logwarn("[gazebo_env] 删除模型 %s 失败: %s",
                                      name, resp.status_message)
                        failed.append(name)
            except rospy.ServiceException as exc:
                if "does not exist" not in str(exc).lower():
                    rospy.logwarn("[gazebo_env] 删除模型 %s 报错: %s", name, exc)
                    failed.append(name)
        # delete_model may return before Gazebo removes the entity.  Reusing
        # the same deterministic name immediately lets the delayed deletion
        # remove the newly spawned model, which produced an invalid episode
        # after nearly every valid one.  Wait until all successful deletions
        # are visible before allowing the next deployment to reuse names.
        pending = [name for name in existing if name not in failed]
        for _ in range(30):
            if not pending:
                break
            rospy.sleep(0.1)
            model_names = self._world_model_names()
            if model_names is not None:
                available = set(model_names)
                pending = [name for name in pending if name in available]
        if pending:
            rospy.logwarn("[gazebo_env] 等待模型删除生效超时: %s",
                          ",".join(pending))
            failed.extend(name for name in pending if name not in failed)
        # Models already absent are intentionally not retained. A truly failed
        # deletion remains tracked so the next reset can retry it.
        self._spawned_models = failed
        return not failed

    def _obstacle_sdf(self, item, model_name):
        """把场景障碍物 item 转成一个静态 SDF 模型字符串。"""
        otype = item.get("type", "pillar_box")
        size = item.get("size", [0.3, 0.3, 2.8])
        sx, sy, sz = (float(v) for v in size[:3])
        if otype == "cylinder":
            radius = max(sx, sy) / 2.0
            geom = ("<cylinder><radius>%g</radius>"
                    "<length>%g</length></cylinder>" % (radius, sz))
        else:
            geom = "<box><size>%g %g %g</size></box>" % (sx, sy, sz)
        return (
            "<sdf version='1.4'>"
            "<model name='%s'><static>true</static>"
            # The spawn service supplies the model orientation.  Keeping a
            # second yaw in the SDF rotates walls twice and desynchronizes
            # Gazebo geometry from the typed scenario/RViz representation.
            "<pose>0 0 0 0 0 0</pose>"
            "<link name='link'>"
            "<collision name='collision'><geometry>%s</geometry></collision>"
            "<visual name='visual'><geometry>%s</geometry>"
            "<material><ambient>1 0 0 1</ambient>"
            "<diffuse>1 0 0 1</diffuse></material>"
            "</visual></link></model></sdf>"
        ) % (model_name, geom, geom)

    def _spawn_model(self, name, sdf, center, yaw):
        """通过 gazebo service 生成一个静态障碍物 model。"""
        try:
            rospy.wait_for_service("/gazebo/spawn_sdf_model", timeout=2.0)
        except (rospy.ROSException, rospy.ServiceException) as exc:
            rospy.logwarn("[gazebo_env] spawn_sdf_model 服务不可用: %s", exc)
            return False
        pose = Pose()
        pose.position.x = float(center[0])
        pose.position.y = float(center[1])
        pose.position.z = (float(center[2])
                           if len(center) > 2 else 0.0)
        pose.orientation.z = math.sin(yaw * 0.5)
        pose.orientation.w = math.cos(yaw * 0.5)
        try:
            resp = self.spawn_model_srv(name, sdf, "", pose, "world")
            if not resp.success:
                rospy.logwarn("[gazebo_env] spawn %s 失败: %s",
                              name, resp.status_message)
                return False
            return True
        except rospy.ServiceException as exc:
            rospy.logwarn("[gazebo_env] spawn %s 报错: %s", name, exc)
            return False

    def _spawn_scenario_obstacles(self, scenario):
        """按场景生成全部障碍物 model，使用每次部署唯一的名称。"""
        obstacles = scenario.get("obstacles", [])
        self._spawned_models = []
        self._deployment_serial += 1
        for i, item in enumerate(obstacles):
            group = item.get("group", "a")
            name = "scenario_d%06d_%s%d" % (
                self._deployment_serial, group, i)
            center = item.get("center", [])
            sdf = self._obstacle_sdf(item, name)
            if not self._spawn_model(name, sdf, center,
                                     float(item.get("yaw", 0.0))):
                self._delete_spawned_models()
                return False
            self._spawned_models.append(name)
        # spawn_sdf_model may acknowledge a request before the model is
        # visible in Gazebo. Verify the complete set before reset accepts the
        # scenario and releases the trajectory hold.
        # The delete path above guarantees that delayed deletion cannot erase
        # these same-name replacements.  Now wait for the complete new set to
        # become visible before exposing its parameters to perception.
        missing = list(self._spawned_models)
        for _ in range(30):
            model_names = self._world_model_names()
            missing = [name for name in self._spawned_models
                       if model_names is None or name not in model_names]
            if not missing:
                break
            rospy.sleep(0.1)
        if missing:
            rospy.logwarn("[gazebo_env] 场景障碍物生成后校验失败，缺少模型: %s",
                          ",".join(missing))
            self._delete_spawned_models()
            return False
        return True

    def _obstacles_spec_str(self, obstacles):
        """构造 /dqn_training/scenario_obstacles 的 ';' 分隔坐标串。"""
        coords = []
        for item in obstacles:
            center = item.get("center", [])
            if len(center) >= 2:
                coords.append("%g,%g" % (float(center[0]),
                                         float(center[1])))
        return ";".join(coords)

    def _split_obstacles_groups(self, obstacles, scenario):
        """把场景障碍物按 group 拆成 obstacles_a/b 的 [[x,y],...]。"""
        a_list, b_list = [], []
        for item in obstacles:
            center = item.get("center", [])
            if len(center) < 2:
                continue
            row = [float(center[0]), float(center[1])]
            group = item.get("group", "a")
            (a_list if group == "a" else b_list).append(row)
        if not a_list and not b_list:
            return (list(scenario.get("obstacles_a", [])),
                    list(scenario.get("obstacles_b", [])))
        return a_list, b_list

    def _reload_lqr_trajectory(self, traj, speed_limit=None):
        """把场景轨迹几何写入 /lqr_tracker_node 并触发 LQR 重载轨迹。"""
        typ = traj.get("type", getattr(self, "scenario_trajectory", "step"))
        ns = "/lqr_tracker_node"
        rospy.set_param(ns + "/trajectory_type", typ)
        duration = float(traj.get("duration_s", 40.0))
        if typ == "linear" and speed_limit is not None:
            start, end = traj.get("start", []), traj.get("end", [])
            if len(start) >= 2 and len(end) >= 2 and float(speed_limit) > 0.0:
                distance = math.sqrt(sum(
                    (float(b) - float(a)) ** 2
                    for a, b in zip(start[:3], end[:3])))
                duration = distance / float(speed_limit)
        rospy.set_param(ns + "/trajectory_duration", duration)
        rospy.set_param(ns + "/loop_trajectory",
                        bool(traj.get("loop", True)))
        # 持久化实际生效的时长/循环标志，供完成判据与步数预算复用
        self.trajectory_duration_s = duration
        self.trajectory_loop = bool(traj.get("loop", True))
        if typ == "linear":
            for axis, val in zip("xyz", traj.get("start", [])):
                rospy.set_param(ns + "/start_" + axis, float(val))
            for axis, val in zip("xyz", traj.get("end", [])):
                rospy.set_param(ns + "/end_" + axis, float(val))
        elif typ == "circular":
            for axis, val in zip("xyz", traj.get("center", [])):
                rospy.set_param(ns + "/center_" + axis, float(val))
            rospy.set_param(ns + "/radius",
                            float(traj.get("radius_m", 3.0)))
            rospy.set_param(ns + "/height",
                            float(traj.get("height_m", 5.0)))
            rospy.set_param(ns + "/angular_velocity",
                            float(traj.get("angular_velocity_rad_s", 0.2)))
        elif typ == "figure_eight":
            for axis, val in zip("xy", traj.get("center", [])):
                rospy.set_param(ns + "/figure_eight_center_" + axis,
                                float(val))
            rospy.set_param(ns + "/figure_eight_radius_x",
                            float(traj.get("radius_x_m", 5.0)))
            rospy.set_param(ns + "/figure_eight_radius_y",
                            float(traj.get("radius_y_m", 3.0)))
            rospy.set_param(ns + "/figure_eight_height",
                            float(traj.get("height_m", 2.0)))
            rospy.set_param(ns + "/figure_eight_angular_velocity",
                            float(traj.get("angular_velocity_rad_s", 0.2)))
        try:
            rospy.wait_for_service("/lqr_tracker/reload_trajectory",
                                   timeout=2.0)
            self.reload_trajectory_srv()
        except (rospy.ROSException, rospy.ServiceException) as exc:
            rospy.logwarn("[gazebo_env] LQR 轨迹重载失败: %s", exc)
        # 每次换场景都按新的轨迹时长重新推导每集步数预算
        self._apply_trajectory_budget()

    def _configure_scenario(self, scenario):
        """刷新场景参数与 LQR 轨迹；障碍物由 reset 在恢复后生成。"""
        obstacles = scenario.get("obstacles", [])
        rospy.set_param(
            "/dqn_training/scenario_obstacles_json",
            json.dumps(obstacles, ensure_ascii=False, separators=(",", ":")))
        rospy.set_param("/dqn_training/scenario_id",
                        str(scenario.get("scenario_id", "unknown")))
        # 逐场景下发失效窗口；无该字段时置空，避免上一个场景的窗口残留
        rospy.set_param("/dqn_training/sensor_blackout_s", [
            float(value) for value in (scenario.get("sensor_blackout_s") or [])])
        rospy.set_param("/dqn_training/scenario_obstacles",
                        self._obstacles_spec_str(obstacles))
        a_list, b_list = self._split_obstacles_groups(obstacles, scenario)
        # obstacles_a/b 必须存为真正的列表（rosparam 数组语义），不能存 JSON 字符串
        rospy.set_param("/dqn_training/obstacles_a", a_list)
        rospy.set_param("/dqn_training/obstacles_b", b_list)
        traj = scenario.get("trajectory", {})
        rospy.set_param("/dqn_training/trajectory",
                        str(traj.get("type", self.trajectory_type)))
        if scenario.get("altitude") is not None:
            rospy.set_param("/dqn_training/altitude",
                            float(scenario["altitude"]))
        rospy.set_param("/dqn_training/trajectory_json",
                        json.dumps(traj, ensure_ascii=False,
                                   separators=(",", ":")))
        self._load_scenario_from_params()
        self.scenario_id = scenario.get("scenario_id", self.scenario_id)
        self._reload_lqr_trajectory(traj, scenario.get("speed_limit"))
        rospy.loginfo("[gazebo_env] 场景轨迹已加载 | scenario_id=%s | 障碍物=%d",
                      self.scenario_id, len(obstacles))

    def set_max_steps(self, cap):
        """训练器设置每集最大步数上限（目标模式下按路径长度自适应缩小）。"""
        self._max_steps_cap = cap
        self.max_episode_steps = cap

    def _apply_trajectory_budget(self):
        """按轨迹时长推导步数预算，受 _max_steps_cap 约束（目标/step 模式不参与）。"""
        if (self.goal_mode or self.trajectory_type == "step" or
                self.trajectory_duration_s is None):
            return
        budget = int(math.ceil(
            self.trajectory_duration_s * self.episode_budget_factor /
            self.action_duration))
        self.max_episode_steps = max(30, min(self._max_steps_cap, budget))
        rospy.loginfo(
            "[gazebo_env] 轨迹步数预算 | duration=%.2fs factor=%.2f budget=%d steps",
            self.trajectory_duration_s, self.episode_budget_factor,
            self.max_episode_steps)

    def _sample_goal(self, start=None):
        for _ in range(200):
            gx = random.uniform(self.goal_bounds[0], self.goal_bounds[1])
            gy = random.uniform(self.goal_bounds[2], self.goal_bounds[3])
            goal = (gx, gy)
            if (start is not None and
                    math.hypot(gx - start[0], gy - start[1]) <
                    self.goal_min_distance):
                continue
            if (start is not None and self.goal_max_distance > 0.0 and
                    math.hypot(gx - start[0], gy - start[1]) >
                    self.goal_max_distance):
                continue
            if is_goal_valid(goal, self.obstacles_a + self.obstacles_b,
                              self.goal_min_clearance):
                return goal
        return None

    def _plan_and_publish_waypoints(self, start=None):
        """采样目标 → A*（仅 a 柱）→ 发布航点 → 按路径长度估算步数预算。"""
        if start is not None:
            start = (float(start[0]), float(start[1]))
        elif self.current_pose is not None:
            start = (self.current_pose.pose.position.x,
                     self.current_pose.pose.position.y)
        else:
            start = (self.start_x, self.start_y)
        for attempt in range(20):
            goal = self._sample_goal(start)
            if goal is None:
                rospy.logwarn("[gazebo_env] 无法采样合法目标点")
                return False
            try:
                response = self.astar_srv(
                    start[0], start[1], goal[0], goal[1],
                    self.astar_safety_margin)
            except rospy.ServiceException as exc:
                rospy.logwarn("[gazebo_env] A* 服务调用失败: %s", exc)
                return False
            if not response.path:
                continue
            path_msg = NavPath()
            path_msg.header.stamp = rospy.Time.now()
            path_msg.header.frame_id = "map"
            total_length = 0.0
            previous = None
            path_points = []
            for waypoint in response.path:
                pose = PoseStamped()
                pose.header = path_msg.header
                pose.pose.position.x = waypoint.x
                pose.pose.position.y = waypoint.y
                pose.pose.position.z = self.scenario_altitude
                path_msg.poses.append(pose)
                path_points.append((waypoint.x, waypoint.y))
                if previous is not None:
                    total_length += math.hypot(waypoint.x - previous[0],
                                               waypoint.y - previous[1])
                previous = (waypoint.x, waypoint.y)
            # 等待发布连接建立后双发，避免首条航点消息丢失
            rospy.sleep(0.3)
            self.waypoint_pub.publish(path_msg)
            rospy.sleep(0.2)
            self.waypoint_pub.publish(path_msg)
            self.goal = goal
            self.path_points = path_points
            self.path_length = total_length
            estimate = int(
                total_length / self.waypoint_speed /
                max(self.action_duration, 1e-3) * 2.25 + 30)
            self.max_episode_steps = max(
                30, min(self._max_steps_cap, estimate))
            rospy.loginfo("[gazebo_env] 目标(%.1f,%.1f) | 路径 %.1f m | 预算 %d 步",
                          goal[0], goal[1], total_length,
                          self.max_episode_steps)
            return True
        return False

    def reset(self):
        # 跨场景轮换先移除实体并加载轨迹。新障碍物要等飞机回到 t=0
        # 后再生成，否则返航路径会穿过障碍物并把飞机卡在碰撞面上。
        pending_scenario = None
        if self._scenario_rotation and self._deployment_pending:
            pending_scenario = self._next_scenario
            deployment_ok = self._delete_spawned_models()
            if deployment_ok:
                self._configure_scenario(pending_scenario)
            self._deployment_pending = False
            if not deployment_ok:
                self.reset_count += 1
                self.trial_id = "%s-t%04d" % (self.scenario_id,
                                               self.reset_count)
                self.reset_ok = False
                self.reset_failure_reason = "scenario_deployment_failed"
                self._reset_recovery_pending = True
                return self.get_observation()
        self.reset_count += 1
        self.trial_id = "%s-t%04d" % (self.scenario_id, self.reset_count)
        previous_done_reason = self.last_done_reason
        recovery_required = (
            bool(getattr(self, "_reset_recovery_pending", False)) or
            self._requires_reset_recovery(previous_done_reason))
        if recovery_required:
            self._reset_recovery_pending = True
        self._update_reset_start_from_params()
        active_reset_tolerance = (self.reset_tolerance
                                  if self.trajectory_type == "step"
                                  else self.moving_reset_tolerance)
        self.episode_step = 0
        self.prev_dist_to_target = None
        self.prev_path_progress = None
        self.prev_directional_clearances = None
        self.previous_swept_clearance = None
        self._last_sector_features = None
        self._sector_memory = None
        self._sector_memory_stamp = None
        self._sector_memory_body_yaw = None
        self.action_hold_remaining = 0
        self.commanded_action = 0
        self.action_switch_count = 0
        self.action_reversal_count = 0
        self.local_speed_scale = 1.0
        self.trajectory_paused = False
        # 每集归零轨迹相位，与 LQR reset_trajectory 后的相位 0 对齐
        self.trajectory_phase_s = 0.0
        self._phase_sample_stamp = None
        self.travel_distance = 0.0
        self.last_step_position = None
        self.metric_times = []
        self.metric_errors = []
        self.metric_positions = []
        self.recovery_start_time = None
        self.a_encounter_steps = 0
        self.collision_events_a = 0
        self.collision_events_b = 0
        self.last_classified_collision_event = 0
        self.geometry_collision_events = 0
        self.geometry_collision_latched = False
        self.cumulative_reward = 0.0
        self.last_action = 0
        self.current_action = 0
        self.last_offset_x = 0.0
        self.last_offset_y = 0.0
        self.last_offset_z = 0.0
        self.motion_heading_world = self.start_yaw
        self.locked_avoid_side = 0
        self.selected_action = 0
        self.filter_reason = "" if self.safety_filter_mode == "disabled" else "reset"
        self.action_reason = ""
        self.predicted_h = float("nan")
        self.prediction_error = self.safety_filter_prediction_error
        self.filter_sensor_fresh = False
        self.filter_margin = 0.0
        self.filter_safe_actions = ()
        self.filter_feasible_actions = ()
        self.filter_candidate_actions = ()
        self.filter_rule_actions = ()
        self.action_cost = 0.0
        self.shield_escape = 0        # 本步是否执行了备份逃逸（0/1）
        self.shield_escape_count = 0  # 回合内累计逃逸次数
        self.escape_margin = float("nan")
        self.filter_clearance_now = float("nan")
        self.last_done_reason = "running"
        self._episode_done = False
        self.collision_event_count = 0
        self.episode_min_front = 1.0
        self.episode_min_obstacle = 1.0
        self.episode_min_geometry_clearance = float("inf")
        self._collision_streak = 0
        self._collision_latched = False
        self.reset_ok = False
        self.reset_failure_reason = "reset_in_progress"
        self.start_dist = float("inf")
        self.start_obstacle_min = 1.0
        self.obstacle_encountered = False
        self.avoidance_active = False
        self.avoidance_clear_count = 0
        self.avoidance_complete = False
        self.avoidance_event_count = 0
        self.avoidance_event_min_clearance = 1.0
        self.avoidance_quality = 0.0
        self.avoidance_quality_sum = 0.0
        self.avoidance_reward_earned = 0.0
        self.last_avoidance_goal_distance = None
        self.avoidance_start_position = None
        self.avoidance_start_heading = self.start_yaw
        self.avoidance_tail_seen = False
        self.goal = None
        self.path_points = []
        self.path_length = 0.0
        self.episode_b_encounter_steps = 0
        # A shield/collision/divergence terminal can leave PX4/LQR with a
        # stale velocity or a large tracking error.  Freeze trajectory phase
        # while the tracker returns to the new episode's t=0 setpoint; this
        # keeps the reset transition deterministic and prevents the old
        # scenario from advancing underneath the new one.
        pose_reset_required = (
            self.trajectory_type != "step" and
            (self._phase_aligned_reset or recovery_required))
        active_reset_timeout = self.reset_wait_timeout
        if pose_reset_required and self.current_pose is not None:
            target = self._compute_t0_start()
            pose = self.current_pose.pose.position
            return_distance = math.sqrt(
                (pose.x - target[0]) ** 2 +
                (pose.y - target[1]) ** 2 +
                (pose.z - target[2]) ** 2)
            active_reset_timeout = reset_recovery_timeout(
                self.reset_wait_timeout, return_distance,
                max(self.waypoint_speed, 1e-3))
        self._publish_offset(0.0, 0.0, 0.0)
        self._publish_local_control(
            0.0 if (recovery_required or pose_reset_required) else 1.0,
            bool(recovery_required or pose_reset_required), 1.0)

        # Keep PX4's local estimator frame; recovery is performed via the held
        # offboard setpoint and trajectory reset below.
        recovery_pose_ok = True

        if self.goal_mode:
            if not self._settle_and_publish_episode_path("preplan"):
                rospy.logwarn("[gazebo_env] 新路径发布前稳定或规划失败，本集不可用")
                return self.get_observation()
            try:
                self.reset_trajectory_srv()
            except rospy.ServiceException:
                pass

        for attempt in range(1, self.reset_attempts + 1):
            self._reset_started_stamp = rospy.Time.now()
            self.current_pose = None
            self.features = None
            self.features_stamp = None
            self.position_error = None
            self.current_velocity = None
            self.current_velocity_stamp = None
            self.reference_velocity = None
            self.reference_velocity_stamp = None
            self.reference_mid_lookahead_velocity = None
            self.reference_mid_lookahead_velocity_stamp = None
            self.reference_lookahead_velocity = None
            self.reference_lookahead_velocity_stamp = None
            self._publish_offset(0.0, 0.0, 0.0)
            reset_world_ok = True
            model_pose_ok = True

            if self.reset_mode == "gazebo":
                if (self.use_reset_world and self.trajectory_type == "step"
                        and not recovery_required):
                    reset_world_ok = self._reset_world()
                # A pose teleport is only safe for the stationary ``step``
                # trajectory.  For moving/offboard episodes it desynchronizes
                # PX4's local estimator from Gazebo and causes the exact
                # runaway reset seen in the failed training run.
                if self.trajectory_type == "step" and not recovery_required:
                    model_pose_ok = self._reset_model_pose()

            # 严格相位复位：移动轨迹也搬到轨迹 t=0 起点并复位 LQR 相位，
            # 保证每个 episode 从一致的轨迹相位开始。
            if (self._phase_aligned_reset and
                    self.trajectory_type != "step"):
                # Keep PX4's estimator origin intact.  Gazebo's
                # set_model_state teleports only the simulation model; after
                # an offboard flight PX4 continues reporting its old local
                # position/velocity, so the strict reset check can never
                # converge and the controller starts diverging.  Reset the
                # trajectory phase and let LQR fly to t=0 while its phase is
                # held; this synchronizes through the normal MAVROS state
                # path instead of mixing two coordinate frames.
                model_pose_ok = True
                trajectory_reset_ok = self._reset_trajectory()
            elif recovery_required:
                # Setpoint-only reset is unsafe after a terminal event: PX4
                # may keep the old velocity and the tracker can immediately
                # fly away from the nominal start.  Use the same hard pose
                # reset as phase-aligned training, even when that option is
                # disabled for ordinary episodes.
                # Recover through the offboard setpoint path.  A Gazebo model
                # teleport leaves PX4/EKF state stale and is unsafe here.
                model_pose_ok = True
                trajectory_reset_ok = self._reset_trajectory()
            # 移动轨迹保持当前相位连续，只等待清零 offset 后重新贴合路径。
            else:
                trajectory_reset_ok = (self._reset_trajectory()
                                       if self.trajectory_type == "step" else True)
            if recovery_required or pose_reset_required:
                # reset_trajectory resets the tracker's speed/pause flags;
                # re-assert the hold after the service call.
                self._publish_local_control(0.0, True, 1.0)
            if self.goal_mode:
                observations_ok = self._wait_for_observations(
                    timeout=self.reset_wait_timeout)
                self.start_dist = self._tracking_error()
                self.start_obstacle_min = self._non_rear_min_distance()
                goal_distance = 0.0
                if observations_ok:
                    goal_distance = math.hypot(
                        self.current_pose.pose.position.x - self.goal[0],
                        self.current_pose.pose.position.y - self.goal[1])
                self.reset_ok = (
                    reset_world_ok and model_pose_ok and recovery_pose_ok and
                    trajectory_reset_ok and
                    observations_ok and
                    goal_distance > self.goal_reached_dist)
            else:
                require_clearance = (
                    self.trajectory_type == "step" or recovery_required or
                    pose_reset_required)
                observations_ok = self._wait_for_reset_observations(
                    timeout=active_reset_timeout,
                    stable_cycles=5,
                    require_reference=True,
                    require_clearance=require_clearance,
                    require_pose_target=pose_reset_required,
                    require_velocity_stable=(pose_reset_required or
                                             self.trajectory_type == "step" or
                                             recovery_required),
                    maintain_hold=(recovery_required or pose_reset_required))
                self.start_dist = (self._distance_to_start()
                                   if self.trajectory_type == "step"
                                   else self._tracking_error())
                self.start_obstacle_min = self._non_rear_min_distance()
                self.reset_ok = (
                    recovery_pose_ok and reset_world_ok and model_pose_ok and
                    trajectory_reset_ok and
                    observations_ok and
                    self.start_dist <= active_reset_tolerance and
                    (not require_clearance or
                     self.start_obstacle_min >= self.reset_min_clearance_norm))
            if self.reset_ok:
                break
            rospy.logwarn("[gazebo_env] reset 第 %d/%d 次失败: world=%s model=%s trajectory=%s obs=%s start_dist=%.2f obstacle_min=%.3f",
                          attempt, self.reset_attempts, reset_world_ok, model_pose_ok,
                          trajectory_reset_ok, observations_ok, self.start_dist,
                          self.start_obstacle_min)
            rospy.sleep(0.5)

        if self.reset_ok and pending_scenario is not None:
            if not self._spawn_scenario_obstacles(pending_scenario):
                self.reset_ok = False
                self.reset_failure_reason = "scenario_deployment_failed"
                self._reset_recovery_pending = True
            else:
                # Require observations produced after the models became
                # visible. This prevents a policy episode from consuming the
                # obstacle-free recovery scan.
                self._reset_started_stamp = rospy.Time.now()
                self.current_pose = None
                self.current_velocity = None
                self.current_velocity_stamp = None
                self.features = None
                self.features_stamp = None
                self.position_error = None
                post_spawn_ok = self._wait_for_reset_observations(
                    timeout=self.reset_wait_timeout,
                    stable_cycles=5,
                    require_reference=True,
                    require_clearance=True,
                    require_pose_target=pose_reset_required,
                    require_velocity_stable=(pose_reset_required or
                                             self.trajectory_type == "step" or
                                             recovery_required),
                    maintain_hold=(recovery_required or pose_reset_required))
                self.start_dist = (self._distance_to_start()
                                   if self.trajectory_type == "step"
                                   else self._tracking_error())
                self.start_obstacle_min = self._non_rear_min_distance()
                self.reset_ok = (
                    post_spawn_ok and
                    self.start_dist <= active_reset_tolerance and
                    self.start_obstacle_min >= self.reset_min_clearance_norm)
                if not self.reset_ok:
                    self.reset_failure_reason = (
                        "post_spawn_observation_validation_failed")
                    self._reset_recovery_pending = True
                    self._delete_spawned_models()
                else:
                    rospy.loginfo(
                        "[gazebo_env] 场景部署完成 | scenario_id=%s | 障碍物=%d",
                        self.scenario_id,
                        len(pending_scenario.get("obstacles", [])))

        if self.reset_ok:
            self.reset_failure_reason = ""
            self._reset_recovery_pending = False
            if recovery_required or pose_reset_required:
                self._publish_local_control(1.0, False, 1.0)
        elif self.reset_failure_reason == "reset_in_progress":
            self.reset_failure_reason = "reset_observation_validation_failed"
        if not self.reset_ok and (recovery_required or pose_reset_required):
            # Keep the vehicle held after a failed recovery. The trainer skips
            # this scene before another policy episode can consume the state.
            self._publish_local_control(0.0, True, 1.0)

        # 以 reset 结束时刻为相位累积起点，避免把等待/复位耗时计入相位
        self._phase_sample_stamp = rospy.Time.now()
        rospy.loginfo("[gazebo_env] episode reset 完成 | reset_ok=%s | reason=%s | start_dist=%.2f | obstacle_min=%.3f",
                      self.reset_ok, self.reset_failure_reason,
                      self.start_dist, self.start_obstacle_min)
        return self.get_observation()

    def step(self, action):
        # 真实相位累积：用上一步已发布的 缩放/暂停 状态推算，与 LQR 相位推进公式一致
        now = rospy.Time.now()
        if self._phase_sample_stamp is not None:
            dt = (now - self._phase_sample_stamp).to_sec()
            if dt > 0.0:
                self.trajectory_phase_s += dt * (
                    0.0 if self.trajectory_paused else self.local_speed_scale)
            self._phase_sample_stamp = now
        self._validate_time_alignment()
        if self._episode_done:
            # 统一成功口径：超时且无碰撞事件同样记成功
            episode_success = episode_is_success(
                self.last_done_reason, self.collision_event_count)
            info = {
                "episode_step": self.episode_step,
                "trial_id": self.trial_id,
                "scenario_id": self.scenario_id,
                "trajectory_phase_s": self.trajectory_phase_s,
                "done_reason": self.last_done_reason,
                "episode_success": episode_success,
                "safe_success": (episode_success and
                                 self.collision_event_count == 0),
                "geometry_clearance": self._geometry_clearance(),
                "min_geometry_clearance": self.episode_min_geometry_clearance,
                "geometry_collision_events": self.geometry_collision_events,
            }
            info.update(self._episode_metrics())
            return self.get_observation(), 0.0, True, info
        filter_done = self._publish_action(action)
        if filter_done:
            self._episode_done = True
            info = {
                "episode_step": self.episode_step,
                "trial_id": self.trial_id,
                "scenario_id": self.scenario_id,
                "trajectory_phase_s": self.trajectory_phase_s,
                "cumulative_reward": self.cumulative_reward,
                "done_reason": self.last_done_reason,
                # 终止行补齐普通分支同名字段，避免 step_log 出现 NaN/默认值误导分析。
                "front_min": self._front_min_distance(),
                "forward_obstacle_min": self._forward_obstacle_distance(),
                "min_front_min": self.episode_min_front,
                "obstacle_min": self._minimum_obstacle_distance(),
                "min_obstacle_min": self.episode_min_obstacle,
                "episode_success": False,
                "safe_success": False,
                "obstacle_encountered": self.obstacle_encountered,
                "avoidance_clear_count": self.avoidance_clear_count,
                "avoidance_active": self.avoidance_active,
                "avoidance_tail_seen": self.avoidance_tail_seen,
                "avoidance_event_count": self.avoidance_event_count,
                "avoidance_quality": self.avoidance_quality,
                "avoidance_reward": self.avoidance_reward_earned,
                "avoidance_reward_step": 0.0,
                "position_error": self._tracking_error(),
                "pose_x": (self.current_pose.pose.position.x
                           if self.current_pose is not None else float("nan")),
                "pose_y": (self.current_pose.pose.position.y
                           if self.current_pose is not None else float("nan")),
                "pose_z": (self.current_pose.pose.position.z
                           if self.current_pose is not None else float("nan")),
                "selected_action": self.selected_action,
                # No motion action is applied when the shield terminates.
                "applied_action": None,
                "filter_reason": self.filter_reason,
                "action_reason": self.action_reason,
                "predicted_h": self.predicted_h,
                "prediction_error": self.prediction_error,
                "filter_sensor_fresh": self.filter_sensor_fresh,
                "sensor_blackout": int(self._sensor_blackout_active()),
                "safety_filter_mode": self.safety_filter_mode,
                "filter_margin": self.filter_margin,
                "filter_safe_actions": self.filter_safe_actions,
                "filter_feasible_actions": self.filter_feasible_actions,
                "filter_candidate_actions": self.filter_candidate_actions,
                "filter_rule_actions": self.filter_rule_actions,
                "action_cost": self.action_cost,
                "filter_clearance_now": self.filter_clearance_now,
                "shield_escape": self.shield_escape,
                "shield_escape_count": self.shield_escape_count,
                "escape_margin": self.escape_margin,
                "filter_emergency": self.filter_emergency,
                "filter_hold_remaining": self.filter_hold_remaining,
                "motion_heading": self.motion_heading_world,
                "body_yaw": self._body_yaw(),
                "offset_x": self.last_offset_x,
                "offset_y": self.last_offset_y,
                "offset_z": self.last_offset_z,
                "motion_sectors": self._motion_sectors(),
                "left_clearance": self._side_clearances()[0],
                "right_clearance": self._side_clearances()[1],
                "local_speed_scale": self.local_speed_scale,
                "trajectory_paused": self.trajectory_paused,
                "geometry_clearance": self._geometry_clearance(),
                "min_geometry_clearance": self.episode_min_geometry_clearance,
                "geometry_collision_events": self.geometry_collision_events,
            }
            info.update(self._episode_metrics())
            return self.get_observation(), -self.terminal_failure_penalty, True, info
        rospy.sleep(self.action_duration)

        step_avoidance_reward = 0.0
        # Use the turn-aware cone to start avoidance before a curved
        # trajectory enters an obstacle.
        obstacle_min = self._avoidance_clearance()
        geometry_clearance = self._geometry_clearance()
        self.episode_min_geometry_clearance = min(
            self.episode_min_geometry_clearance, geometry_clearance)
        if geometry_clearance <= 0.0:
            if not self.geometry_collision_latched:
                self.geometry_collision_events += 1
            self.geometry_collision_latched = True
        else:
            self.geometry_collision_latched = False
        self.avoidance_complete = False
        if obstacle_min < self.safe_distance_norm:
            self.obstacle_encountered = True
        if (not self.avoidance_active and
                obstacle_min < self.safe_distance_norm and
                self.avoidance_event_count < self.max_avoidance_reward_events):
            self.avoidance_start_position = self._horizontal_position()
            self.avoidance_start_heading = self.motion_heading_world
            self.avoidance_tail_seen = False
            if self.recovery_start_time is None:
                self.recovery_start_time = self.episode_step * self.action_duration
        rear_clearance = self._rear_min_distance()
        if self.avoidance_active and rear_clearance < self.safe_distance_norm:
            self.avoidance_tail_seen = True
        forward_progress = self._avoidance_forward_progress()
        tail_completion_ready, tail_release_locked = avoidance_tail_gate(
            self.avoidance_active, self.avoidance_tail_seen, rear_clearance,
            self.avoidance_tail_completion_norm,
            self.avoidance_tail_release_norm)
        completion_allowed = avoidance_completion_allowed(
            forward_progress, tail_completion_ready, tail_release_locked)
        (self.avoidance_active,
         self.avoidance_clear_count,
         self.avoidance_event_count,
         self.avoidance_event_min_clearance,
         self.last_avoidance_goal_distance,
         self.avoidance_complete) = update_avoidance_event(
             self.avoidance_active,
             self.avoidance_clear_count,
             self.avoidance_event_count,
             self.avoidance_event_min_clearance,
             self.last_avoidance_goal_distance,
             obstacle_min,
             self._tracking_error(),
             self._distance_to_goal(),
             self.safe_distance_norm,
             self.avoidance_tracking_tolerance,
             self.avoidance_clear_steps,
             self.max_avoidance_reward_events,
             self.avoidance_rearm_progress,
             completion_allowed,
             completion_obstacle_distance=self._front_min_distance())
        reward = self.compute_reward()

        self.episode_step += 1
        self.episode_min_front = min(
            self.episode_min_front, self._front_min_distance())
        self.episode_min_obstacle = min(
            self.episode_min_obstacle, self._minimum_obstacle_distance())
        if self.avoidance_complete:
            event_reward, quality = avoidance_quality_reward(
                self.avoidance_event_min_clearance,
                self._tracking_error(),
                self._collision_dist_norm(), self.emergency_dist_norm,
                self.avoidance_tracking_tolerance,
                self.avoidance_event_reward, self.avoidance_quality_bonus)
            self.avoidance_quality_sum += quality
            self.avoidance_quality = (
                self.avoidance_quality_sum / self.avoidance_event_count)
            self.avoidance_reward_earned += event_reward
            step_avoidance_reward = event_reward
            reward += event_reward
            rospy.loginfo(
                "[gazebo_env] 避障完成: quality=%.3f reward=%.2f min_clearance=%.2f m recovery_error=%.2f m",
                quality, event_reward,
                self.avoidance_event_min_clearance * self.lidar_max_range,
                self._tracking_error())
            # The avoidance offset is a temporary reference displacement.  Do
            # not carry it into the next policy decision after the obstacle
            # has cleared, otherwise a safe episode can drift into runaway
            # lateral motion.
            # Let the next action-0 command decay the offset through the normal
            # smoothing/rate limit instead of teleporting the reference point.
            self.commanded_action = 0
            self.action_hold_remaining = 0
            self.locked_avoid_side = 0
        if self.current_pose is not None and self.obstacles_b:
            px = self.current_pose.pose.position.x
            py = self.current_pose.pose.position.y
            b_min = min(math.hypot(px - bx, py - by)
                        for bx, by in self.obstacles_b)
            if b_min < self.safe_distance_norm * self.lidar_max_range:
                self.episode_b_encounter_steps += 1
        if self.current_pose is not None and self.obstacles_a:
            px = self.current_pose.pose.position.x
            py = self.current_pose.pose.position.y
            a_min = min(math.hypot(px - ax, py - ay)
                        for ax, ay in self.obstacles_a)
            if a_min < self.safe_distance_norm * self.lidar_max_range:
                self.a_encounter_steps += 1

        collision_events_before = self.collision_event_count
        done = self.is_done()
        self._episode_done = done
        self._classify_collision_event()
        self._update_episode_motion_metrics()
        # Sample metrics after the terminal/state-update wait so the values
        # used for online summaries are identical to the pose written to the
        # step log.
        self._record_metrics_sample()
        if self.last_done_reason in ("goal_reached", "trajectory_complete"):
            reward += self.goal_reached_reward
        elif self.last_done_reason == "avoidance_complete":
            reward += self.avoidance_success_reward
        elif self.last_done_reason in ("diverged",
                                       "low_altitude", "timeout",
                                       "perception_timeout"):
            # A clear timeout is a conservative horizon stop rather than a crash.
            safe_timeout = (
                self.last_done_reason == "timeout" and
                self._front_min_distance() >= self.safe_distance_norm)
            reward -= (0.0 if safe_timeout else self.terminal_failure_penalty)
        if self.collision_event_count > collision_events_before:
            reward -= self.terminal_failure_penalty
        reward = self._clip_reward(reward)
        self.cumulative_reward += reward
        self.last_action = self.current_action
        obs = self.get_observation()
        self.prev_directional_clearances = self._directional_clearances()

        # 统一成功口径：超时且无碰撞事件同样记成功
        episode_success = episode_is_success(
            self.last_done_reason, self.collision_event_count)
        info = {
            "episode_step": self.episode_step,
            "trial_id": self.trial_id,
            "scenario_id": self.scenario_id,
            "trajectory_phase_s": self.trajectory_phase_s,
            "cumulative_reward": self.cumulative_reward,
            "done_reason": self.last_done_reason,
            "front_min": self._front_min_distance(),
            "forward_obstacle_min": self._forward_obstacle_distance(),
            "min_front_min": self.episode_min_front,
            "obstacle_min": self._minimum_obstacle_distance(),
            "min_obstacle_min": self.episode_min_obstacle,
            "episode_success": episode_success,
            "safe_success": (episode_success and
                             self.collision_event_count == 0),
            "obstacle_encountered": self.obstacle_encountered,
            "avoidance_clear_count": self.avoidance_clear_count,
            "avoidance_active": self.avoidance_active,
            "avoidance_tail_seen": self.avoidance_tail_seen,
            "avoidance_event_count": self.avoidance_event_count,
            "avoidance_quality": self.avoidance_quality,
            "avoidance_reward": self.avoidance_reward_earned,
            "avoidance_reward_step": step_avoidance_reward,
            "position_error": self._tracking_error(),
            "pose_x": (self.current_pose.pose.position.x
                       if self.current_pose is not None else float("nan")),
            "pose_y": (self.current_pose.pose.position.y
                       if self.current_pose is not None else float("nan")),
            "pose_z": (self.current_pose.pose.position.z
                       if self.current_pose is not None else float("nan")),
            "selected_action": self.selected_action,
            "applied_action": self.current_action,
            "filter_reason": self.filter_reason,
            "action_reason": self.action_reason,
            "predicted_h": self.predicted_h,
            "prediction_error": self.prediction_error,
            "filter_sensor_fresh": self.filter_sensor_fresh,
            "safety_filter_mode": self.safety_filter_mode,
            "filter_margin": self.filter_margin,
            "filter_safe_actions": self.filter_safe_actions,
            "filter_feasible_actions": self.filter_feasible_actions,
            "filter_candidate_actions": self.filter_candidate_actions,
            "filter_rule_actions": self.filter_rule_actions,
            "action_cost": self.action_cost,
            "filter_clearance_now": self.filter_clearance_now,
            "shield_escape": self.shield_escape,
            "shield_escape_count": self.shield_escape_count,
            "escape_margin": self.escape_margin,
            "filter_emergency": self.filter_emergency,
            "filter_hold_remaining": self.filter_hold_remaining,
            "motion_heading": self.motion_heading_world,
            "body_yaw": self._body_yaw(),
            "offset_x": self.last_offset_x,
            "offset_y": self.last_offset_y,
            "offset_z": self.last_offset_z,
            "motion_sectors": self._motion_sectors(),
            "left_clearance": self._side_clearances()[0],
            "right_clearance": self._side_clearances()[1],
            "reset_ok": self.reset_ok,
            "reset_failure_reason": self.reset_failure_reason,
            "start_dist": self.start_dist,
            "start_obstacle_min": self.start_obstacle_min,
            "goal_x": self.goal[0] if self.goal else 0.0,
            "goal_y": self.goal[1] if self.goal else 0.0,
            "path_length": self.path_length,
            "b_encounter_steps": self.episode_b_encounter_steps,
            "a_encounter_steps": self.a_encounter_steps,
            "collision_events": self.collision_event_count,
            "collision_events_a": self.collision_events_a,
            "collision_events_b": self.collision_events_b,
            "geometry_clearance": self._geometry_clearance(),
            "min_geometry_clearance": self.episode_min_geometry_clearance,
            "geometry_collision_events": self.geometry_collision_events,
            "geometry_collision": (self.episode_min_geometry_clearance <= 0.0),
            "travel_distance": self.travel_distance,
            "collision_events_per_meter": (self.collision_event_count /
                                           max(self.travel_distance, 1e-6)),
            "action_switch_count": self.action_switch_count,
            "action_switch_rate": (self.action_switch_count /
                                   max(1, self.episode_step - 1)),
            "action_reversal_rate": (self.action_reversal_count /
                                     max(1, self.episode_step - 1)),
            "local_speed_scale": self.local_speed_scale,
            "trajectory_paused": self.trajectory_paused,
            # 前瞻门控判据落盘：avoid_needed 是原始判据，avoid_suppressed 是
            # 叠加尾部锁定后的最终判定（是否真的压制了偏移），便于核查一致性。
            "avoid_needed": int(self._avoidance_needed()),
            "avoid_suppressed": int(self._avoidance_suppressed()),
            "sensor_blackout": int(self._sensor_blackout_active()),
        }
        info.update(self._episode_metrics())
        return obs, reward, done, info

    def get_observation(self):
        if self._features_are_fresh() and self.features.lidar_sectors:
            sectors = self._motion_sectors()
        else:
            sectors = [1.0] * self.num_sectors
        if len(sectors) != self.num_sectors:
            sectors = (sectors[:self.num_sectors] +
                       [1.0] * max(0, self.num_sectors - len(sectors)))

        if self.current_pose is not None:
            px = self._clip_observation_value(self.current_pose.pose.position.x / 10.0)
            py = self._clip_observation_value(self.current_pose.pose.position.y / 10.0)
            pz = self._clip_observation_value(self.current_pose.pose.position.z / 10.0)
        else:
            px, py, pz = 0.0, 0.0, 0.0

        pe = self._tracking_error()
        pe = min(pe / 5.0, 1.0)
        velocity = self.current_velocity
        vx = velocity.x if velocity is not None else 0.0
        vy = velocity.y if velocity is not None else 0.0
        vz = velocity.z if velocity is not None else 0.0
        cosine = math.cos(self.motion_heading_world)
        sine = math.sin(self.motion_heading_world)
        velocity_forward = (vx * cosine + vy * sine) / 3.0
        velocity_left = (-vx * sine + vy * cosine) / 3.0
        ref = self.reference_velocity
        reference_speed = math.hypot(ref.x, ref.y) / 3.0 if ref is not None else 0.0
        clearances = self._directional_clearances()
        previous = self.prev_directional_clearances or clearances
        clearance_rates = [
            (current - old) / max(self.action_duration, 1e-3)
            for current, old in zip(clearances, previous)
        ]
        _, path_error = self._path_progress()
        last_action_one_hot = [0.0] * N_ACTIONS
        last_action_one_hot[self.last_action] = 1.0
        dynamic = [
            max(-1.0, min(1.0, velocity_forward)),
            max(-1.0, min(1.0, velocity_left)),
            max(-1.0, min(1.0, vz / 3.0)),
            max(0.0, min(1.0, reference_speed)),
            self.last_offset_x / max(self.max_offset_xy, 1e-6),
            self.last_offset_y / max(self.max_offset_xy, 1e-6),
            self.last_offset_z / max(self.step_z, 1e-6),
            *[max(-1.0, min(1.0, rate)) for rate in clearance_rates],
            min(path_error / 5.0, 1.0),
        ]
        return sectors + [px, py, pz, pe] + dynamic + last_action_one_hot + \
            self._path_lookahead_features()

    def compute_reward(self):
        reward = self.safe_reward - self.step_penalty

        front, left, right = self._directional_clearances()
        obstacle_min = action_swept_clearance(
            self.current_action, front, left, right)
        reward -= proximity_penalty(
            obstacle_min, self._collision_dist_norm(),
            self.emergency_dist_norm, self.safe_distance_norm,
            self.safe_proximity_penalty, self.emergency_proximity_penalty)

        pe_norm = min(self._tracking_error(), self.max_tracking_error)
        pe_norm /= max(self.max_tracking_error, 1e-6)
        reward -= self.tracking_error_weight * pe_norm

        reward -= action_reversal_cost(
            self.last_action, self.current_action, self.action_change_penalty,
            self.action_reversal_penalty)
        previous_swept = (action_swept_clearance(
            self.current_action, *self.prev_directional_clearances)
            if self.prev_directional_clearances is not None else None)
        reward += clearance_change_reward(
            previous_swept, obstacle_min,
            self.safe_distance_norm, self.clearance_change_scale)
        if previous_swept is not None:
            closing_rate = max(
                0.0, (previous_swept - obstacle_min) /
                max(self.action_duration, 1e-3))
            ttc = time_to_collision(
                obstacle_min, closing_rate, self._collision_dist_norm())
            if ttc < self.ttc_safe_seconds:
                reward -= self.ttc_penalty_scale * (
                    1.0 - ttc / max(self.ttc_safe_seconds, 1e-6))
        if obstacle_min >= self.safe_distance_norm and self.current_action != 0:
            reward -= self.unnecessary_action_penalty

        # 只在安全区奖励真实目标进度，避障绕行时不惩罚短暂远离目标。
        path_progress, _ = self._path_progress()
        if path_progress is not None and self.prev_path_progress is not None:
            progress_scale = (1.0 if obstacle_min >= self.safe_distance_norm
                              else self.risk_progress_scale)
            reward += self.w_progress * progress_scale * max(
                -0.5, min(0.5, path_progress - self.prev_path_progress))
        self.prev_path_progress = path_progress
        self.previous_swept_clearance = obstacle_min
        return self._clip_reward(reward)

    def is_done(self):
        if not self._features_are_fresh():
            self.last_done_reason = "perception_timeout"
            rospy.logwarn("[gazebo_env] episode 终止: perception timeout")
            return True

        if self._geometry_clearance() <= self.contact_dist + 1e-9:
            self._collision_streak += 1
            # 单帧几何跳变不作数；连续两帧确认机体包络接触后立即结束 episode。
            # 碰撞后继续推进会把无人机留在障碍物内，随后每个控制周期
            # 都会重复扣分并污染 replay buffer，最终表现为训练发散。
            if self._collision_streak >= 2 and not self._collision_latched:
                self.collision_event_count += 1
                self._collision_latched = True
                self._classify_collision_event()
                self.last_done_reason = "collision"
                rospy.logwarn(
                    "[gazebo_env] collision event=%d，终止当前 episode",
                    self.collision_event_count)
                return True
        else:
            self._collision_streak = 0
            self._collision_latched = False

        if self._tracking_error() > self.max_terminal_tracking_error:
            self.last_done_reason = "diverged"
            rospy.logwarn("[gazebo_env] episode 终止: tracking diverged (pos_err=%.2f)",
                          self._tracking_error())
            return True

        if self.current_pose is not None and self.current_pose.pose.position.z < self.min_height:
            self.last_done_reason = "low_altitude"
            rospy.logwarn("[gazebo_env] episode 终止: low altitude")
            return True

        if self._trajectory_completed():
            self.last_done_reason = "trajectory_complete"
            rospy.loginfo(
                "[gazebo_env] episode 终止: trajectory_complete (phase=%.2fs/%.2fs, 终点距离=%.2fm)",
                self.trajectory_phase_s, self.trajectory_duration_s,
                self._trajectory_end_distance())
            return True

        if (self.goal_mode and self.goal is not None and
                self.current_pose is not None):
            goal_dist = math.hypot(
                self.current_pose.pose.position.x - self.goal[0],
                self.current_pose.pose.position.y - self.goal[1])
            if goal_dist < self.goal_reached_dist:
                self.last_done_reason = "goal_reached"
                rospy.loginfo("[gazebo_env] episode 终止: goal_reached (%.1f,%.1f) (%d steps, 误差 %.2f m)",
                              self.goal[0], self.goal[1], self.episode_step,
                              goal_dist)
                return True

        completion_grace_active = (
            not self.goal_mode and self.avoidance_active and
            self.episode_step < self.episode_step_limit)
        if (self.episode_step >= self.max_episode_steps and
                not completion_grace_active):
            self.last_done_reason = "timeout"
            rospy.loginfo("[gazebo_env] episode 终止: timeout (%d steps, min_obstacle=%.3f)",
                          self.episode_step, self.episode_min_obstacle)
            return True

        self.last_done_reason = "running"
        return False

    def _trajectory_end_distance(self):
        """当前位置到 linear 轨迹终点的水平距离（不可用时返回 nan）。"""
        end = (self.scenario_trajectory_json or {}).get("end")
        if end is None or len(end) < 2 or self.current_pose is None:
            return float("nan")
        return math.hypot(
            self.current_pose.pose.position.x - float(end[0]),
            self.current_pose.pose.position.y - float(end[1]))

    def _trajectory_completed(self):
        """非目标模式下判定整条场景轨迹是否已飞完（相位或终点几何）。"""
        if (self.goal_mode or self.trajectory_duration_s is None or
                self.trajectory_type not in ("linear", "circular",
                                             "figure_eight")):
            return False
        if not self.trajectory_loop:
            # 非循环轨迹优先用终点距离判定，缺终点时退化为相位判据
            end_dist = self._trajectory_end_distance()
            if not math.isnan(end_dist):
                return end_dist < self.goal_reached_dist
        return self.trajectory_phase_s >= self.trajectory_duration_s

    def _publish_action(self, action):
        desired_left, desired_z = 0.0, 0.0
        self.selected_action = int(action)
        self.filter_margin = 0.0
        self.filter_safe_actions = ()
        self.filter_feasible_actions = ()
        self.filter_candidate_actions = ()
        self.filter_rule_actions = ()
        self.action_cost = 0.0
        self.shield_escape = 0
        self.escape_margin = float("nan")
        self.filter_emergency = False
        self.filter_hold_remaining = 0
        self.filter_sensor_fresh = self._features_are_fresh()
        if self.safety_filter_mode != "disabled":
            if self.current_pose is None or self.current_velocity is None:
                position = velocity = None
                tracking_error = 0.0
                sensor_valid = False
            else:
                position = (self.current_pose.pose.position.x,
                            self.current_pose.pose.position.y,
                            self.current_pose.pose.position.z)
                velocity = (self.current_velocity.x, self.current_velocity.y,
                            self.current_velocity.z)
                tracking_error = self._tracking_error()
                sensor_valid = (self.filter_sensor_fresh and
                                math.isfinite(tracking_error))
            obstacles = self._safety_obstacles() if sensor_valid else None
            emergency = self._non_rear_min_distance() < self.emergency_dist_norm
            self.filter_emergency = bool(emergency)
            self.filter_hold_remaining = int(self.action_hold_remaining)
            # 环境可行性层：与 B2 完全一致。_constrain_action 会写入侧向锁定
            # 并把候选收敛到动作掩码内，A1/P 因此不再丢失侧向一致性约束。
            feasible_candidate = self._constrain_action(self.selected_action)
            feasible_actions = self.allowed_actions()
            if (self.action_interface == "masked" and
                    feasible_candidate not in feasible_actions):
                # 时隙竞态兜底：收敛到掩码内与候选最接近的动作。
                # raw 档不做此兜底——掩码已退出裁决，兜底会掩盖真实候选。
                feasible_candidate = min(
                    feasible_actions, key=lambda a: abs(a - feasible_candidate))
            previous_action = (self.commanded_action
                               if self.commanded_action in feasible_actions
                               else None)
            hold_kwargs = {
                "previous_action": previous_action,
                "hold_remaining": self.action_hold_remaining,
                "emergency": emergency,
                "feasible_actions": feasible_actions,
                "rule_actions": self.rule_actions(),
            }
            if self.safety_filter_mode == "a1":
                next_action, diagnostics = select_a1_action(
                    feasible_candidate, position, velocity, obstacles,
                    base_margin=self.safety_filter_base_margin,
                    horizon_s=self.safety_filter_horizon,
                    dt=self.safety_filter_dt,
                    alpha=self.safety_filter_alpha,
                    epsilon_pred=self.safety_filter_epsilon_pred,
                    drone_half_extents=self.drone_half_extents,
                    lateral_speed=self.safety_filter_lateral_speed,
                    vertical_speed=self.safety_filter_vertical_speed,
                    empty_set_terminal=self.safety_filter_empty_set_terminal,
                    clearance_floor=self.collision_dist,
                    sensor_valid=sensor_valid, **hold_kwargs)
            else:
                next_action, diagnostics = select_p_action(
                    feasible_candidate, position, velocity, obstacles,
                    base_margin=self.safety_filter_base_margin,
                    tracking_error=max(0.0, tracking_error) if sensor_valid else 0.0,
                    kappa=self.safety_filter_kappa,
                    prediction_error=self.safety_filter_prediction_error,
                    horizon_s=self.safety_filter_horizon,
                    dt=self.safety_filter_dt,
                    alpha=self.safety_filter_alpha,
                    epsilon_pred=self.safety_filter_epsilon_pred,
                    drone_half_extents=self.drone_half_extents,
                    lateral_speed=self.safety_filter_lateral_speed,
                    vertical_speed=self.safety_filter_vertical_speed,
                    empty_set_terminal=self.safety_filter_empty_set_terminal,
                    clearance_floor=self.collision_dist,
                    sensor_valid=sensor_valid, **hold_kwargs)
            self.filter_reason = diagnostics.get("filter_reason", "sensor_invalid")
            # 过滤器启用时，过滤结果同时就是动作层原因
            self.action_reason = self.filter_reason
            self.predicted_h = diagnostics.get("predicted_h", float("nan"))
            self.prediction_error = diagnostics.get(
                "prediction_error", self.safety_filter_prediction_error)
            self.filter_margin = diagnostics.get("margin", 0.0)
            self.filter_safe_actions = tuple(diagnostics.get("safe_actions", ()))
            # 诊断用：本步的 A_feasible。空集终止时用于区分「DCBF 判据判空」与
            # 「可行性层先把动作收窄到只剩 {0}」两种成因。
            self.filter_feasible_actions = feasible_actions
            self.filter_candidate_actions = tuple(
                diagnostics.get("candidate_actions", ()))
            self.filter_rule_actions = tuple(diagnostics.get("rule_actions", ()))
            self.action_cost = diagnostics.get("action_cost")
            # 同量参考：用过滤器同一门控障碍集算当前瞬时净空（不减 margin）。
            # predicted_h + filter_margin 与它同物理量，差值即预测时域带来的保守量。
            if sensor_valid and obstacles is not None:
                self.filter_clearance_now = minimum_geometry_clearance(
                    position, obstacles, drone_half_extents=self.drone_half_extents)
            else:
                self.filter_clearance_now = float("nan")
            # 备份逃逸：安全集为空但仍发布了动作，记录计数与逃逸裕量。
            self.shield_escape = int(bool(diagnostics.get("shield_escape", False)))
            if self.shield_escape:
                self.shield_escape_count += 1
                # escape_action 无裕量可算时返回 None（键存在），而 %.3f 不接受
                # None，故与 L1973 的默认值一致归一为 nan（缺失哨兵）。
                self.escape_margin = diagnostics.get("escape_margin")
                if self.escape_margin is None:
                    self.escape_margin = float("nan")
                rospy.logwarn_throttle(
                    1.0,
                    "[gazebo_env] 安全集为空，执行备份逃逸动作 applied=%s 裕量=%.3f",
                    diagnostics.get("actual_action"), self.escape_margin)
            if diagnostics.get("done", False) or next_action is None:
                self.last_done_reason = diagnostics.get(
                    "done_reason", "shield_infeasible")
                self.current_action = 0
                self.commanded_action = 0
                # Stop sending the previous avoidance offset after a terminal
                # shield decision.  Leaving it active lets the LQR tracker
                # continue steering during reset and can move the vehicle far
                # from the next episode's start pose.
                self._publish_offset(0.0, 0.0, 0.0)
                self._publish_local_control(0.0, True, 1.0)
                return True
            next_action = int(next_action)
            if next_action != self.commanded_action:
                self.action_switch_count += 1
                old_side = (1 if self.commanded_action in (1, 3)
                            else -1 if self.commanded_action in (2, 4) else 0)
                new_side = (1 if next_action in (1, 3)
                            else -1 if next_action in (2, 4) else 0)
                if old_side * new_side < 0:
                    self.action_reversal_count += 1
            self.current_action = next_action
            self.commanded_action = next_action
            if self.action_reason == "action_hold":
                self.action_hold_remaining = diagnostics["hold_remaining"]
            else:
                self.action_hold_remaining = max(0, self.action_hold_steps - 1)
        else:
            return self._publish_action_legacy(action)

        self._publish_action_motion()
        return False

    def _publish_action_legacy(self, action):
        """Existing B0/B1/B2 action-mask path, kept behaviorally unchanged."""
        self.selected_action = int(action)
        emergency = self._non_rear_min_distance() < self.emergency_dist_norm
        self.filter_emergency = bool(emergency)
        self.filter_hold_remaining = int(self.action_hold_remaining)
        if (self.action_hold_remaining > 0 and not emergency and
                self.commanded_action in self.allowed_actions()):
            self.current_action = self.commanded_action
            self.action_reason = "action_hold"
            self.action_hold_remaining -= 1
        else:
            next_action = self._constrain_action(self.selected_action)
            self.action_reason = ("candidate_safe" if next_action == self.selected_action
                                  else "candidate_filtered")
            self.action_cost = 0.0 if next_action == self.selected_action else 1.0
            if next_action != self.commanded_action:
                self.action_switch_count += 1
                old_side = (1 if self.commanded_action in (1, 3)
                            else -1 if self.commanded_action in (2, 4) else 0)
                new_side = (1 if next_action in (1, 3)
                            else -1 if next_action in (2, 4) else 0)
                if old_side * new_side < 0:
                    self.action_reversal_count += 1
            self.commanded_action = next_action
            self.current_action = next_action
            self.action_hold_remaining = max(0, self.action_hold_steps - 1)
        self.predicted_h = self._geometry_clearance()
        self._publish_action_motion()
        return False

    def _avoidance_suppressed(self):
        """前瞻判据认定"沿原航迹无需避障"，且尾部不处于锁定状态。

        观测特征、动作掩码与偏移发布共用这一个判据，避免三处阈值漂移。
        """
        if self._avoidance_tail_locked():
            return False
        return not self._avoidance_needed()

    def _avoidance_tail_locked(self):
        """刚通过的障碍仍占据后方锥时保持侧向偏移，不急于切回直飞。"""
        _, tail_lock = avoidance_tail_gate(
            self.avoidance_active, self.avoidance_tail_seen,
            self._rear_min_distance(),
            self.avoidance_tail_completion_norm,
            self.avoidance_tail_release_norm)
        return tail_lock

    def _publish_action_motion(self):
        desired_left, desired_z = 0.0, 0.0
        if self.current_action not in range(N_ACTIONS):
            raise ValueError("action must be in range(6), got %d" %
                             self.current_action)
        # 门控收口：判据认定无需避障时忽略动作的空间分量，只发布零偏移。
        # a1/p 安全滤波只按"候选是否安全"筛选，不会阻止策略继续侧向偏移，
        # 必须在动作发布处收口，"不需要反应就不反应"才能真正生效。
        suppressed = self._avoidance_suppressed()
        # 曲线下动作解锁后，若仅用前向锥距离，input 高时幅度≈0、offset 恒为 0；
        # 改以融合后的 clearance 作为幅度输入，让转弯内侧障碍也能驱动偏移。
        magnitude = avoidance_magnitude(
            self._avoidance_clearance(),
            self.warning_distance_norm,
            self.min_offset_xy,
            self.max_offset_xy)

        if suppressed or self.current_action == 0:   # 沿原路径
            pass
        elif self.current_action == 1:    # 左移
            desired_left = magnitude
        elif self.current_action == 2:    # 右移
            desired_left = -magnitude
        elif self.current_action == 3:    # 左移并上升
            desired_left = magnitude
            desired_z = self.step_z
        elif self.current_action == 4:    # 右移并上升
            desired_left = -magnitude
            desired_z = self.step_z
        elif self.current_action == 5:    # 上升悬停
            desired_z = self.step_z

        desired_x, desired_y = motion_frame_offset(
            0.0, desired_left, self.motion_heading_world)
        offset_x = smooth_value(
            self.last_offset_x, desired_x, self.offset_smooth_alpha)
        offset_y = smooth_value(
            self.last_offset_y, desired_y, self.offset_smooth_alpha)
        offset_z = smooth_value(
            self.last_offset_z, desired_z, self.offset_smooth_alpha)
        offset_x = limit_change(
            self.last_offset_x, offset_x, self.max_offset_change)
        offset_y = limit_change(
            self.last_offset_y, offset_y, self.max_offset_change)
        self.last_offset_x = 0.0 if abs(offset_x) < 1e-3 else offset_x
        self.last_offset_y = 0.0 if abs(offset_y) < 1e-3 else offset_y
        self.last_offset_z = 0.0 if abs(offset_z) < 1e-3 else offset_z
        self._publish_offset(
            self.last_offset_x, self.last_offset_y, self.last_offset_z)
        # 曲线转弯内侧贴身障碍可能在前向/扫掠锥外，再与融合后 clearance 取 min
        # 以触发减速（speed_scale 0.65/0.3）。
        control_clearance = min(
            action_swept_clearance(
                self.current_action, *self._directional_clearances()),
            self._avoidance_clearance())
        (self.local_speed_scale,
         self.trajectory_paused, _) = local_control_profile(
             control_clearance, self.emergency_dist_norm,
             self.safe_distance_norm, self.warning_distance_norm)
        if self.avoidance_active and self.local_speed_scale > 0.8:
            self.local_speed_scale = 0.8
        # 备份逃逸动作未经安全认证，必须压制速度并暂停轨迹推进，避免逃逸本身撞上去。
        if self.shield_escape:
            self.local_speed_scale = min(self.local_speed_scale, 0.3)
            self.trajectory_paused = True
        self._publish_local_control(
            self.local_speed_scale, self.trajectory_paused, 1.0)
        self.action_pub.publish(Int32(data=self.current_action))
        # 结构化诊断：动作掩码、宽/严格锥净空、航向与参考速度新鲜度
        allowed = self.allowed_actions()
        wide_clearance = self._avoidance_clearance()
        strict_clearance = self._front_min_distance()
        actual = self.current_velocity
        actual_heading = None
        if (actual is not None and
                math.hypot(actual.x, actual.y) >= self.heading_min_speed):
            actual_heading = math.atan2(actual.y, actual.x)
        actual_heading_str = ("None" if actual_heading is None
                              else "%.2f" % actual_heading)
        if self.reference_velocity_stamp is not None:
            reference_age = (rospy.Time.now()
                             - self.reference_velocity_stamp).to_sec()
            reference_age_str = "%.3f" % reference_age
        else:
            reference_age = -1.0
            reference_age_str = "-1"
        reference_fresh = (
            self.reference_velocity is not None and
            self.reference_velocity_stamp is not None and
            0.0 <= reference_age <= self.reference_velocity_timeout and
            math.hypot(self.reference_velocity.x,
                       self.reference_velocity.y) >= self.heading_min_speed)
        rospy.loginfo_throttle(
            1.0,
            "[gazebo_env] selected=%d applied=%d allowed=%s wide=%.3f strict=%.3f heading=%.2f actual_heading=%s yaw=%.2f ref_age=%s ref_fresh=%d forward=%.3f speed_scale=%.2f paused=%d offset=(%.2f, %.2f, %.2f) omni=%.3f feat_fresh=%d",
            self.selected_action, self.current_action,
            sorted(allowed), wide_clearance, strict_clearance,
            self.motion_heading_world, actual_heading_str, self._body_yaw(),
            reference_age_str, int(reference_fresh),
            self._forward_obstacle_distance(),
            self.local_speed_scale, int(self.trajectory_paused),
            self.last_offset_x, self.last_offset_y, self.last_offset_z,
            self._minimum_obstacle_distance(), int(self._features_are_fresh()))

    def _safety_obstacles(self):
        """Return configured geometry gated by the current LiDAR observation.

        Scenario geometry is used only to provide the shape for a return that
        is already inside the current sensor range.  An all-clear or stale
        LiDAR frame therefore exposes no obstacles to the filter; future
        layout geometry is never used as a preview.
        """
        if self.current_pose is None:
            return None
        if not self._features_are_fresh() or not self.features.lidar_sectors:
            return None
        observed_range = min(float(value) for value in self.features.lidar_sectors)
        if observed_range >= 0.999:
            return []
        px = self.current_pose.pose.position.x
        py = self.current_pose.pose.position.y
        source = self.scenario_obstacles_typed
        if not source:
            source = [{"center": [cx, cy, cz],
                       "size": self.obstacle_geometry_size,
                       "type": "pillar_box", "yaw": 0.0}
                      for cx, cy, cz in self.obstacle_geometry_centers]
        return [obstacle for obstacle in source
                if math.hypot(float(obstacle["center"][0]) - px,
                              float(obstacle["center"][1]) - py)
                <= min(self.lidar_max_range,
                       max(self.lidar_min_range,
                           observed_range * self.lidar_max_range + 1.0))]

    def _constrain_action(self, action):
        if int(action) not in range(N_ACTIONS):
            raise ValueError("action must be in range(6), got %r" % action)
        action = int(action)
        clearance = self._avoidance_clearance()
        action, self.locked_avoid_side = constrain_avoidance_action(
            action, self.locked_avoid_side,
            clearance, self.warning_distance_norm)
        if self.action_interface == "raw":
            # raw 档：只做侧向迟滞改写，不再重映射进距离掩码。
            return action
        allowed = self.allowed_actions()
        if action not in allowed:
            # 时隙竞态：clearance 在两次调用间从安全区突变到贴脸区，
            # 上一时刻合法的动作当前可能被掩码排除；收敛到最近似合法动作而非崩溃。
            rospy.logwarn_throttle(
                1.0,
                "[gazebo_env] action %d not allowed now (allowed=%s), remapped",
                int(action), sorted(allowed))
            action = min(allowed, key=lambda a: abs(a - int(action)))
        return action

    def rule_actions(self, clearance=None, left_clearance=None,
                     right_clearance=None, tail_lock=None):
        """距离阈值规则掩码（纯规则输出，不读写侧向锁定）。

        B1 冻结规则避障器的动作集合，也是 masked 档 allowed_actions() 的内核。
        """
        if clearance is None:
            clearance = self._avoidance_clearance()
        if left_clearance is None or right_clearance is None:
            left_clearance, right_clearance = self._side_clearances()
        if tail_lock is None:
            _, tail_lock = avoidance_tail_gate(
                self.avoidance_active, self.avoidance_tail_seen,
                self._rear_min_distance(),
                self.avoidance_tail_completion_norm,
                self.avoidance_tail_release_norm)
        if self._avoidance_suppressed():
            return [0]
        if tail_lock:
            return self._tail_locked_actions(clearance, left_clearance,
                                             right_clearance)
        return allowed_avoidance_actions(
            clearance, self.emergency_dist_norm, self.safe_distance_norm,
            self.warning_distance_norm, self.locked_avoid_side,
            left_clearance, right_clearance,
            omni_min=self._non_rear_min_distance())

    def allowed_actions(self):
        """Return actions consistent with the active avoidance side lock."""
        clearance = self._avoidance_clearance()
        left_clearance, right_clearance = self._side_clearances()
        rear_clearance = self._rear_min_distance()
        # Keep the lateral offset until the tail has cleared.  Returning to
        # action 0 while the rear cone is still occupied can swing the body
        # back into the obstacle that was just passed.
        _, tail_lock = avoidance_tail_gate(
            self.avoidance_active, self.avoidance_tail_seen, rear_clearance,
            self.avoidance_tail_completion_norm,
            self.avoidance_tail_release_norm)
        if clearance >= self.warning_distance_norm and not tail_lock:
            self.locked_avoid_side = 0
        elif (self.locked_avoid_side > 0 and
              left_clearance < self.emergency_dist_norm):
            self.locked_avoid_side = (
                -1 if right_clearance >= self.emergency_dist_norm else 0)
        elif (self.locked_avoid_side < 0 and
              right_clearance < self.emergency_dist_norm):
            self.locked_avoid_side = (
                1 if left_clearance >= self.emergency_dist_norm else 0)
        suppressed = self._avoidance_suppressed()
        if suppressed:
            # 前瞻判据认定沿原航迹无需避障：释放侧向锁定。
            self.locked_avoid_side = 0
        if self.action_interface == "raw":
            # raw 档：距离掩码退出安全裁决，仅保留侧向锁定维护。
            return tuple(range(N_ACTIONS))
        if suppressed:
            return [0]
        if tail_lock:
            return self._tail_locked_actions(clearance, left_clearance,
                                             right_clearance)
        return allowed_avoidance_actions(
            clearance, self.emergency_dist_norm, self.safe_distance_norm,
            self.warning_distance_norm, self.locked_avoid_side,
            left_clearance, right_clearance,
            omni_min=self._non_rear_min_distance())

    def _tail_locked_actions(self, clearance, left_clearance, right_clearance):
        """Return actions that preserve the chosen side while the tail clears."""
        if self.locked_avoid_side > 0 and left_clearance >= self.emergency_dist_norm:
            return [3] if clearance < self.emergency_dist_norm else [1, 3]
        if self.locked_avoid_side < 0 and right_clearance >= self.emergency_dist_norm:
            return [4] if clearance < self.emergency_dist_norm else [2, 4]
        if left_clearance >= right_clearance and left_clearance >= self.emergency_dist_norm:
            return [3]
        if right_clearance >= self.emergency_dist_norm:
            return [4]
        return [5]

    def _side_clearances(self):
        return lateral_clearances(self._motion_sectors())

    @property
    def episode_step_limit(self):
        """Hard loop bound including a finite active-avoidance completion grace."""
        return self.max_episode_steps + self.avoidance_completion_grace_steps

    def _directional_clearances(self):
        left, right = self._side_clearances()
        return self._front_min_distance(), left, right

    def _path_progress(self):
        if self.current_pose is None or len(self.path_points) < 2:
            return None, self._tracking_error()
        position = (self.current_pose.pose.position.x,
                    self.current_pose.pose.position.y)
        return project_path_progress(position, self.path_points)

    def _horizontal_position(self):
        if self.current_pose is None:
            return None
        return (self.current_pose.pose.position.x,
                self.current_pose.pose.position.y)

    def _avoidance_forward_progress(self):
        position = self._horizontal_position()
        if position is None or self.avoidance_start_position is None:
            return 0.0
        dx = position[0] - self.avoidance_start_position[0]
        dy = position[1] - self.avoidance_start_position[1]
        return (dx * math.cos(self.avoidance_start_heading) +
                dy * math.sin(self.avoidance_start_heading))

    def _update_episode_motion_metrics(self):
        if self.current_pose is None:
            return
        position = (self.current_pose.pose.position.x,
                    self.current_pose.pose.position.y)
        if self.last_step_position is not None:
            self.travel_distance += math.hypot(
                position[0] - self.last_step_position[0],
                position[1] - self.last_step_position[1])
        self.last_step_position = position

    def _classify_collision_event(self):
        if (self.current_pose is None or
                self.collision_event_count <= self.last_classified_collision_event):
            return
        px = self.current_pose.pose.position.x
        py = self.current_pose.pose.position.y
        a_min = min((math.hypot(px - x, py - y) for x, y in self.obstacles_a),
                    default=float("inf"))
        b_min = min((math.hypot(px - x, py - y) for x, y in self.obstacles_b),
                    default=float("inf"))
        if a_min <= b_min:
            self.collision_events_a += 1
        else:
            self.collision_events_b += 1
        self.last_classified_collision_event = self.collision_event_count

    def _publish_offset(self, dx, dy, dz):
        offset_msg = Vector3Stamped()
        offset_msg.header.stamp = rospy.Time.now()
        offset_msg.header.frame_id = "map"
        offset_msg.vector.x = dx
        offset_msg.vector.y = dy
        offset_msg.vector.z = dz
        self.local_offset_pub.publish(offset_msg)

    def _publish_local_control(self, speed_scale, pause_progress,
                               phase_direction=1.0):
        control = Vector3Stamped()
        control.header.stamp = rospy.Time.now()
        control.header.frame_id = "map"
        control.vector.x = max(0.0, min(1.0, speed_scale))
        control.vector.y = 1.0 if pause_progress else 0.0
        control.vector.z = -1.0 if phase_direction < 0.0 else 1.0
        self.local_control_pub.publish(control)

    def _rear_min_distance(self):
        """运动方向后方锥内的最小障碍距离（归一化）。"""
        sectors = self._motion_sectors()
        if not sectors:
            return 1.0
        # 将扇区旋转 180° 后复用前方锥计算
        shift = len(sectors) // 2
        rotated = sectors[shift:] + sectors[:shift]
        return directional_clearance(rotated, half_angle=math.pi / 3.0)

    def _non_rear_min_distance(self):
        """排除运动后方锥的最小障碍距离（归一化）。

        已飞过的后方障碍不影响后续飞行，不应触发碰撞/紧急/绕行判定；
        后方锥与 _rear_min_distance 一致（运动反方向 ±60°）。
        features 不新鲜时返回 0.0（视为最紧急，与 _minimum_obstacle_distance 一致）。
        """
        if not self._features_are_fresh() or not self.features.lidar_sectors:
            return 0.0
        sectors = self._motion_sectors()
        if not sectors:
            return 0.0
        count = len(sectors)
        sector_size = 2.0 * math.pi / count
        rear_half = math.pi / 3.0
        candidates = [
            distance for index, distance in enumerate(sectors)
            if abs((index + 0.5) * sector_size - math.pi) <= math.pi - rear_half
        ]
        return min(candidates) if candidates else 0.0

    def clear_local_offset(self):
        """Return control to the unmodified LQR reference trajectory."""
        self.current_action = 0
        self.last_action = 0
        self.last_offset_x = 0.0
        self.last_offset_y = 0.0
        self.last_offset_z = 0.0
        self.locked_avoid_side = 0
        self.local_speed_scale = 1.0
        self.trajectory_paused = False
        self._publish_offset(0.0, 0.0, 0.0)
        self._publish_local_control(1.0, False, 1.0)

    def _front_min_distance(self):
        # Use the same short-lived sector memory as the observation/action
        # mask.  Previously this path read only the newest frame, so a single
        # all-clear LiDAR frame could immediately restore action 0 while the
        # obstacle was still in the vehicle's swept path.
        sectors = self._motion_sectors()
        if not sectors:
            return 0.0
        return directional_clearance(
            sectors, half_angle=math.pi / 3.0)

    def _avoidance_clearance(self):
        """Clearance used to trigger avoidance before entering a turn.

        Use a stable wide cone rather than body-yaw lag as a curvature proxy:
        the vehicle yaw is not required to follow the trajectory tangent.  The
        wide cone is only an early-warning signal and must not be reused as the
        collision distance.

        曲线（8字形）转弯内侧/侧方障碍落在前向锥之外，仅靠方向性锥
        无法感知，故融合排除后方的兜底距离。后方已飞过的障碍不影响
        后续飞行，不参与触发；features 不新鲜时 _non_rear_min_distance
        返回 0.0，可接受（视为最紧急）。
        """
        sectors = self._motion_sectors()
        if not sectors:
            return 0.0
        half_angle = math.pi / 3.0 + self.turning_front_extra_angle
        directional = directional_clearance(sectors, half_angle=half_angle)
        omni = self._non_rear_min_distance()
        central = directional_clearance(sectors, half_angle=math.pi / 6.0)
        left, right = lateral_clearances(sectors)
        result = corridor_aware_clearance(
            directional, omni, central, left, right,
            self._collision_dist_norm(), self.safe_distance_norm,
            self.warning_distance_norm)
        rospy.loginfo_throttle(0.5, "[gazebo_env] CLEARANCE_DEBUG directional=%.3f omni=%.3f central=%.3f result=%.3f features_fresh=%d", directional, omni, central, result, int(self._features_are_fresh()))
        return result

    def _forward_obstacle_distance(self):
        return self._front_min_distance()

    def _motion_sectors(self):
        return self._motion_sector_frames()[0]

    def _motion_sector_frames(self):
        """返回 (运动帧扫掠扇区, 采集时偏航, 机体帧原始扇区)。

        前瞻可达性特征需要机体帧原始扇区：它自己会按前瞻航向重新旋转，
        若复用已扫掠到运动帧的结果会重复旋转一次（误差 = 偏航跟踪滞后）。
        """
        features_fresh = self._features_are_fresh()
        raw_sectors = (self.features.lidar_sectors
                       if features_fresh and self.features is not None
                       else None)
        using_memory_frame = not raw_sectors
        # During a brief sensor gap, keep the last obstacle frame alive.  A
        # zero-filled replacement would both erase the observation and make
        # the policy oscillate back to action 0 on the next control tick.
        if not raw_sectors:
            now = rospy.Time.now()
            memory_age = ((now - self._sector_memory_stamp).to_sec()
                          if self._sector_memory_stamp is not None else None)
            raw_sectors = retain_obstacle_sectors(
                [], self._sector_memory, memory_age,
                self.obstacle_memory_timeout)
            if not raw_sectors or self._sector_memory_body_yaw is None:
                empty = [0.0] * self.num_sectors
                return empty, self.start_yaw, empty
        self._update_motion_heading()
        sectors = [max(0.0, min(float(distance), 1.0))
                   for distance in raw_sectors]
        body_yaw = (self._sector_memory_body_yaw
                    if using_memory_frame else self._body_yaw())
        mid_heading, lookahead_heading = self._lookahead_headings()
        headings = [self.motion_heading_world, mid_heading, lookahead_heading]
        actual = self.current_velocity
        actual_heading = None
        if (actual is not None and
                math.hypot(actual.x, actual.y) >= self.heading_min_speed):
            # 实际航向放在 sweep 首段，覆盖"实际航向→参考切线"之间的滞后区间，
            # 转弯内侧障碍才能进入锥内。
            actual_heading = math.atan2(actual.y, actual.x)
            headings = [actual_heading] + headings
        current = path_swept_sectors(sectors, body_yaw, headings)

        # Retain raw body-frame sectors and the yaw at capture time. Reproject
        # them on every call so a turn cannot move remembered obstacles into an
        # unrelated motion-frame direction.
        now = rospy.Time.now()
        if features_fresh and self.features is not self._last_sector_features:
            has_return = any(value < 0.999 for value in sectors)
            if has_return:
                self._sector_memory = list(sectors)
                self._sector_memory_stamp = now
                self._sector_memory_body_yaw = body_yaw
            self._last_sector_features = self.features
        memory_valid = (
            self._sector_memory is not None and
            self._sector_memory_stamp is not None and
            self._sector_memory_body_yaw is not None and
            (now - self._sector_memory_stamp).to_sec() <=
            self.obstacle_memory_timeout)
        if memory_valid:
            remembered = path_swept_sectors(
                self._sector_memory, self._sector_memory_body_yaw,
                headings)
            current = retain_obstacle_sectors(
                current, remembered,
                (now - self._sector_memory_stamp).to_sec(),
                self.obstacle_memory_timeout)
        return current, body_yaw, sectors

    def _fresh_reference_heading(self, velocity, stamp, default):
        if velocity is None or stamp is None:
            return default
        age = (rospy.Time.now() - stamp).to_sec()
        if not 0.0 <= age <= self.reference_velocity_timeout:
            return default
        if math.hypot(velocity.x, velocity.y) < self.heading_min_speed:
            return default
        return math.atan2(velocity.y, velocity.x)

    def _lookahead_headings(self):
        mid = self._fresh_reference_heading(
            self.reference_mid_lookahead_velocity,
            self.reference_mid_lookahead_velocity_stamp,
            self.motion_heading_world)
        end = self._fresh_reference_heading(
            self.reference_lookahead_velocity,
            self.reference_lookahead_velocity_stamp, mid)
        return mid, end

    def _path_clearance_body_margin(self):
        """机体半宽 + 额外净空余量，用于判断"够不够宽通过"。"""
        return float(self.drone_half_extents[0]) + self.path_clearance_margin

    def _path_speed_xy(self):
        """轨迹切向速度 (m/s)：优先新鲜 reference_velocity，其次实测速度。"""
        reference = self.reference_velocity
        if reference is not None and self.reference_velocity_stamp is not None:
            age = (rospy.Time.now() - self.reference_velocity_stamp).to_sec()
            if 0.0 <= age <= self.reference_velocity_timeout:
                return math.hypot(reference.x, reference.y)
        velocity = self.current_velocity
        if velocity is None:
            return 0.0
        return math.hypot(velocity.x, velocity.y)

    def _path_travel_distances(self):
        """沿原航迹到 3 个前瞻时刻的可行距离 (m)。

        距离下限取 safe_distance_norm * lidar_max_range - body_margin，
        使当前时刻（Δt=0）也保留 2.4 m 的即时制动/感知预留，避免低速悬停时漏判。
        """
        horizon = self.avoidance_lookahead_time
        speed = self._path_speed_xy()
        floor_distance = max(
            0.0, self.safe_distance_norm * self.lidar_max_range -
            self._path_clearance_body_margin())
        return [max(speed * fraction * horizon, floor_distance)
                for fraction in (0.0, 0.5, 1.0)]

    def _path_lookahead_features(self):
        """5 维前瞻可达性特征（与掩码门控同源）。

        必须用机体帧原始扇区：本函数会按每个前瞻航向重新旋转一次，
        复用 `_motion_sectors()` 的运动帧结果会造成重复旋转。
        """
        _, body_yaw, body_sectors = self._motion_sector_frames()
        mid_heading, lookahead_heading = self._lookahead_headings()
        features, _ = path_lookahead_features(
            body_sectors, body_yaw,
            (self.motion_heading_world, mid_heading, lookahead_heading),
            self._path_travel_distances(), self.lidar_max_range,
            self._path_clearance_body_margin(), self.path_feature_scale_m)
        return features

    def _avoidance_needed(self):
        """前瞻判据：是否需要避障反应（1 表示需要）。"""
        return bool(self._path_lookahead_features()[4] > 0.5)

    def _update_motion_heading(self):
        reference_is_fresh = False
        if (self.reference_velocity is not None and
                self.reference_velocity_stamp is not None):
            age = (rospy.Time.now() - self.reference_velocity_stamp).to_sec()
            reference_is_fresh = (
                0.0 <= age <= self.reference_velocity_timeout)
        velocity = (self.reference_velocity if reference_is_fresh
                    else self.current_velocity)
        if velocity is None:
            return
        self.motion_heading_world = motion_heading(
            velocity.x, velocity.y,
            self.motion_heading_world, self.heading_min_speed)

    def _body_yaw(self):
        if self.current_pose is None:
            return self.start_yaw
        orientation = self.current_pose.pose.orientation
        return quaternion_to_yaw(orientation.x, orientation.y,
                                 orientation.z, orientation.w)

    def _minimum_obstacle_distance(self):
        if not self._features_are_fresh() or not self.features.lidar_sectors:
            return 0.0
        return min(float(distance) for distance in self.features.lidar_sectors)

    def _geometry_clearance(self):
        """Return signed clearance to the configured Gazebo collision boxes.

        This is a deterministic geometry proxy reconstructed from the same
        spawn parameters as ``start_sim.sh``. It is deliberately separate
        from LiDAR distance and does not claim to be a Gazebo contact event.
        """
        if self.current_pose is None:
            return float("inf")
        point = (self.current_pose.pose.position.x,
                 self.current_pose.pose.position.y,
                 self.current_pose.pose.position.z)
        obstacles = (self.scenario_obstacles_typed if self.scenario_obstacles_typed
                     else [(center, self.obstacle_geometry_size)
                           for center in self.obstacle_geometry_centers])
        return minimum_geometry_clearance(
            point, obstacles, self.drone_half_extents)

    def _sensor_blackout_active(self):
        """轨迹相位是否落在注入的传感器失效窗口内（§3.2 工况 7）。

        以 trajectory_phase_s 而非「步数 × dt」判定：实测真实相位推进比名义
        步时长快约 1.2 倍，用名义时间会把窗口错位到航迹末段。
        """
        if not self.sensor_blackout_window:
            return False
        start, end = self.sensor_blackout_window
        phase = getattr(self, "trajectory_phase_s", 0.0) or 0.0
        return start <= phase < end

    def _features_are_fresh(self):
        if self._sensor_blackout_active():
            return False
        if self.features is None or self.features_stamp is None:
            return False
        age = (rospy.Time.now() - self.features_stamp).to_sec()
        return 0.0 <= age <= self.perception_timeout

    def _record_metrics_sample(self):
        if self.current_pose is None:
            return
        self.metric_times.append(self.episode_step * self.action_duration)
        self.metric_errors.append(self._tracking_error())
        self.metric_positions.append((self.current_pose.pose.position.x,
                                      self.current_pose.pose.position.y))

    def _episode_metrics(self):
        summary = summarize_episode(
            self.metric_errors, self.metric_times, self.metric_positions,
            steady_window_s=1.0,
            recovery_start_time=self.recovery_start_time,
            recovery_tolerance=self.avoidance_tracking_tolerance,
            recovery_hold_s=self.action_duration * max(1, self.avoidance_clear_steps))
        return {
            "tracking_rmse": summary["rmse"],
            "tracking_max_error": summary["max_error"],
            "tracking_steady_error": summary["steady_error"],
            "recovery_time": summary["recovery_time"],
            "metric_path_length": summary["path_length"],
        }

    def _make_scenario_id(self):
        """Build a stable short ID from the frozen scenario parameters."""
        payload = {
            "obstacles_a": self.obstacles_a,
            "obstacles_b": self.obstacles_b,
            "obstacle_geometry_centers": self.obstacle_geometry_centers,
            "obstacle_geometry_size": self.obstacle_geometry_size,
            "scenario_obstacles_typed": self.scenario_obstacles_typed,
            "trajectory": self.scenario_trajectory,
            "altitude": self.scenario_altitude,
        }
        encoded = json.dumps(payload, sort_keys=True, separators=(",", ":"))
        return "s-" + hashlib.sha1(encoded.encode("utf-8")).hexdigest()[:10]

    def _validate_time_alignment(self):
        """校验 pose/velocity/features 三路时间戳对齐情况。

        检查项：无 Time(0)、各路相对上一帧单调递增、三路间最大延迟
        小于 reference_velocity_timeout。异常时 throttle 告警，不阻断主流程。
        """
        pose_stamp = (self.current_pose.header.stamp
                      if self.current_pose is not None else None)
        vel_stamp = self.current_velocity_stamp
        feat_stamp = self.features_stamp

        streams = (("pose", pose_stamp, "_last_pose_stamp"),
                   ("velocity", vel_stamp, "_last_velocity_stamp"),
                   ("features", feat_stamp, "_last_features_stamp"))
        missing = [name for name, stamp, _ in streams if stamp is None]
        if missing:
            rospy.logwarn_throttle(
                5.0, "[gazebo_env] 时间戳校验: 缺少 %s", ",".join(missing))
            return False

        valid = True
        order = []
        for name, stamp, attr in streams:
            if stamp == rospy.Time(0):
                valid = False
            last = getattr(self, attr)
            if last is not None and stamp < last:
                valid = False
            setattr(self, attr, stamp)
            order.append((name, stamp))

        delay = (max(s for _, s in order) - min(s for _, s in order)).to_sec()
        delay_ok = delay < self.reference_velocity_timeout
        if not (valid and delay_ok):
            rospy.logwarn_throttle(
                5.0,
                "[gazebo_env] 时间戳校验未通过: %s delay=%.3fs timeout=%.2fs",
                "; ".join("%s=%.3f" % (n, s.to_sec()) for n, s in order),
                delay, self.reference_velocity_timeout)
        return valid and delay_ok

    def _collision_dist_norm(self):
        return self.collision_dist / max(self.lidar_max_range, 1e-6)

    def _tracking_error(self):
        pe = self.position_error if self.position_error is not None else 0.0
        if pe is None or math.isnan(pe):
            return 0.0
        return max(0.0, float(pe))

    def _clip_reward(self, reward):
        limit = max(self.reward_clip_abs, 1e-6)
        return max(-limit, min(limit, reward))

    def _clip_observation_value(self, value):
        return max(-2.0, min(2.0, value))

    def _distance_to_start(self):
        if self.current_pose is None:
            return float("inf")
        cx = self.current_pose.pose.position.x
        cy = self.current_pose.pose.position.y
        cz = self.current_pose.pose.position.z
        return math.sqrt((cx - self.start_x) ** 2 +
                         (cy - self.start_y) ** 2 +
                         (cz - self.start_z) ** 2)

    def _distance_to_goal(self):
        if not self.goal_mode or self.goal is None or self.current_pose is None:
            return None
        return math.hypot(
            self.current_pose.pose.position.x - self.goal[0],
            self.current_pose.pose.position.y - self.goal[1])

    def _reset_model_pose(self):
        try:
            rospy.wait_for_service("/gazebo/set_model_state", timeout=2.0)
            state = ModelState()
            state.model_name = self._resolve_model_name()
            state.reference_frame = "world"
            state.pose.position.x = self.start_x
            state.pose.position.y = self.start_y
            state.pose.position.z = (self.scenario_altitude
                                     if self.goal_mode else self.start_z)
            state.pose.orientation.z = math.sin(self.start_yaw * 0.5)
            state.pose.orientation.w = math.cos(self.start_yaw * 0.5)
            state.twist.linear.x = 0.0
            state.twist.linear.y = 0.0
            state.twist.linear.z = 0.0
            state.twist.angular.x = 0.0
            state.twist.angular.y = 0.0
            state.twist.angular.z = 0.0
            res = self.set_model_state_srv(state)
            if not res.success:
                rospy.logwarn("[gazebo_env] set_model_state 失败: %s", res.status_message)
                return False
            self.current_pose = None
            self.current_velocity = None
            self.current_velocity_stamp = None
            self.features = None
            self.features_stamp = None
            self.position_error = None
            rospy.sleep(0.5)
            return True
        except (rospy.ROSException, rospy.ServiceException) as exc:
            rospy.logwarn("[gazebo_env] set_model_state 不可用: %s", exc)
            return False

    def _reset_world(self):
        try:
            rospy.wait_for_service("/gazebo/reset_world", timeout=2.0)
            self.reset_world_srv()
            rospy.sleep(0.5)
            return True
        except (rospy.ROSException, rospy.ServiceException) as exc:
            rospy.logwarn("[gazebo_env] reset_world 不可用: %s", exc)
            return False

    def _reset_trajectory(self):
        try:
            rospy.wait_for_service("/lqr_tracker/reset_trajectory", timeout=2.0)
            res = self.reset_trajectory_srv()
            if not res.success:
                rospy.logwarn("[gazebo_env] LQR 轨迹复位失败: %s", res.message)
                return False
            return True
        except (rospy.ROSException, rospy.ServiceException) as exc:
            rospy.logwarn("[gazebo_env] LQR 轨迹复位服务不可用: %s", exc)
            return False

    def _wait_for_observations(self, timeout=3.0):
        start = rospy.Time.now()
        rate = rospy.Rate(10)
        while (rospy.Time.now() - start).to_sec() < timeout:
            if (self.current_pose is not None and self._features_are_fresh() and
                    self.position_error is not None):
                return True
            rate.sleep()
        rospy.logwarn("[gazebo_env] 等待新观测超时 (%.1fs)", timeout)
        return False

    def _wait_for_preplan_settle(self):
        """Command a hold point, then wait until the actual vehicle settles."""
        if self.current_pose is None:
            if not self._wait_for_observations(timeout=self.reset_wait_timeout):
                return None
        hold_point = (self.current_pose.pose.position.x,
                      self.current_pose.pose.position.y,
                      self.current_pose.pose.position.z)
        self._publish_hold_waypoint(hold_point)
        start = rospy.Time.now()
        rate = rospy.Rate(10)
        stable_steps = 0
        last_velocity = None
        speed = float("inf")
        while ((rospy.Time.now() - start).to_sec() <
               self.preplan_settle_timeout):
            if (self.current_pose is not None and
                    self.current_velocity is not None and
                    self.current_velocity is not last_velocity):
                position = (self.current_pose.pose.position.x,
                            self.current_pose.pose.position.y)
                speed = math.hypot(self.current_velocity.x,
                                   self.current_velocity.y)
                stable_steps = update_settle_progress(
                    hold_point, stable_steps, position, speed,
                    self.preplan_speed_tolerance,
                    self.preplan_position_tolerance)
                last_velocity = self.current_velocity
                if stable_steps >= self.preplan_stable_steps:
                    rospy.loginfo("[gazebo_env] 新路径发布前已稳定: pos=(%.2f,%.2f) speed=%.3f m/s observations=%d",
                                  position[0], position[1], speed,
                                  stable_steps)
                    return hold_point[:2]
            rate.sleep()
        rospy.logwarn("[gazebo_env] 新路径发布前稳定等待超时: speed=%.3f m/s stable=%d/%d",
                      speed, stable_steps, self.preplan_stable_steps)
        return None

    def _settle_and_publish_episode_path(self, stage):
        """Retry the hold-and-plan transition before declaring reset failure."""
        attempts = max(1, int(self.preplan_attempts))
        last_failure = "settle_timeout"
        for attempt in range(1, attempts + 1):
            settled_start = self._wait_for_preplan_settle()
            if settled_start is None:
                last_failure = "settle_timeout"
            elif self._plan_and_publish_waypoints(start=settled_start):
                return True
            else:
                last_failure = "planning_failed"
            rospy.logwarn("[gazebo_env] %s 稳定/规划第 %d/%d 次失败: %s",
                          stage, attempt, attempts, last_failure)
            self._publish_offset(0.0, 0.0, 0.0)
            rospy.sleep(0.2)
        self.reset_failure_reason = "%s_%s" % (stage, last_failure)
        return False

    def _publish_hold_waypoint(self, point):
        """Publish a one-point Path; the tracker interprets it as a hold."""
        path_msg = NavPath()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = "map"
        pose = PoseStamped()
        pose.header = path_msg.header
        pose.pose.position.x = point[0]
        pose.pose.position.y = point[1]
        pose.pose.position.z = point[2]
        path_msg.poses.append(pose)
        self.waypoint_pub.publish(path_msg)
        rospy.sleep(0.1)
        self.waypoint_pub.publish(path_msg)

    def _wait_for_reset_observations(self, timeout=3.0, stable_cycles=1,
                                     require_reference=True,
                                     require_clearance=True,
                                     require_pose_target=False,
                                     require_velocity_stable=False,
                                     maintain_hold=False):
        start = rospy.Time.now()
        rate = rospy.Rate(10)
        on_reference = False
        on_pose_target = not require_pose_target
        velocity_stable = not require_velocity_stable
        clear_count = 0
        last_features = None
        target = (self._compute_t0_start()
                  if require_pose_target and self.trajectory_type != "step"
                  else None)
        reset_stamp = self._reset_started_stamp
        while (rospy.Time.now() - start).to_sec() < timeout:
            # LQR's local-control watchdog expires after 0.5 s.  Keep the
            # trajectory paused while strict reset validation is in progress
            # so the phase cannot run away from the teleported t=0 pose.
            if maintain_hold:
                self._publish_local_control(0.0, True, 1.0)
            fresh_pose = self.current_pose is not None
            fresh_velocity = self.current_velocity is not None
            fresh_features = self.features is not None
            if reset_stamp is not None and not reset_stamp.is_zero():
                fresh_pose = (fresh_pose and
                              self.current_pose.header.stamp >= reset_stamp)
                fresh_velocity = (fresh_velocity and
                                  self.current_velocity_stamp is not None and
                                  self.current_velocity_stamp >= reset_stamp)
                fresh_features = (fresh_features and
                                  self.features_stamp is not None and
                                  self.features_stamp >= reset_stamp)
            if self.trajectory_type == "step":
                on_reference = (fresh_pose and
                                self._distance_to_start() <= self.reset_tolerance)
            else:
                if require_pose_target and target is not None:
                    # PositionError is a legacy scalar without a timestamp;
                    # during a reset it may still describe the previous
                    # scenario.  Use the fresh pose-to-t0 distance as the
                    # authoritative reference check.
                    on_reference = on_pose_target
                else:
                    on_reference = (fresh_pose and
                                    self.position_error is not None and
                                    self._tracking_error() <=
                                    self.moving_reset_tolerance)
            if target is not None and fresh_pose:
                pose = self.current_pose.pose.position
                on_pose_target = math.sqrt(
                    (pose.x - target[0]) ** 2 +
                    (pose.y - target[1]) ** 2 +
                    (pose.z - target[2]) ** 2) <= self.moving_reset_tolerance
            if fresh_velocity:
                velocity = self.current_velocity
                velocity_stable = math.sqrt(
                    velocity.x ** 2 + velocity.y ** 2 + velocity.z ** 2
                ) <= self.reset_velocity_tolerance
            has_clearance = (fresh_features and
                             self._non_rear_min_distance() >=
                             self.reset_min_clearance_norm)
            if fresh_features and self.features is not last_features:
                stable = ((on_reference or not require_reference) and
                          (on_pose_target or not require_pose_target) and
                          (velocity_stable or not require_velocity_stable) and
                          (has_clearance or not require_clearance))
                clear_count = clear_count + 1 if stable else 0
                last_features = self.features
            if clear_count >= stable_cycles:
                return True
            rate.sleep()
        rospy.logwarn("[gazebo_env] 等待 reset 恢复超时 (%.1fs): on_reference=%s pose_target=%s velocity_stable=%s obstacle_min=%.3f required=%.3f stable=%d/%d",
                      timeout, on_reference, on_pose_target, velocity_stable,
                      self._non_rear_min_distance(),
                      self.reset_min_clearance_norm, clear_count, stable_cycles)
        return False

    def _resolve_model_name(self):
        try:
            rospy.wait_for_service("/gazebo/get_world_properties", timeout=1.0)
            res = self.get_world_properties_srv()
            names = list(res.model_names)
            if self.model_name in names:
                return self.model_name
            for candidate in ("iris", "iris_3d_lidar"):
                if candidate in names:
                    self.model_name = candidate
                    return candidate
            for name in names:
                if "iris" in name:
                    self.model_name = name
                    return name
        except (rospy.ROSException, rospy.ServiceException) as exc:
            rospy.logwarn("[gazebo_env] 无法查询 Gazebo 模型列表: %s", exc)
        return self.model_name

    @property
    def observation_space_dim(self):
        return self.num_sectors + 4 + 11 + N_ACTIONS + PATH_FEATURES

    @property
    def n_actions(self):
        return N_ACTIONS
