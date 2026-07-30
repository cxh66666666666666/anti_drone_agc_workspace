#!/usr/bin/env python3
"""
@file: gazebo_env.py
@brief: Gazebo 强化学习局部避障环境封装

分层边界：
    - 既有 LQR 参考轨迹负责全局路径/前进。
    - RL 只输出局部避障 offset，不规划全局路线。
"""

import math

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3Stamped
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetWorldProperties, SetModelState
from std_msgs.msg import Float64, Int32
from std_srvs.srv import Empty, Trigger

from rl_obstacle_avoidance.msg import ObstacleFeatures
from training_utils import (action_change_cost, avoidance_magnitude,
                            constrain_avoidance_action, directional_clearance,
                            episode_succeeded,
                            limit_change, motion_frame_offset, motion_heading,
                            proximity_penalty, quaternion_to_yaw,
                            rotate_sectors_to_heading, smooth_value,
                            update_avoidance_progress,
                            validate_distance_thresholds)


ACTION_NAMES = {
    0: "沿原路径",
    1: "左移",
    2: "右移",
    3: "左移并上升",
    4: "右移并上升",
}
N_ACTIONS = 5


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
        self.min_offset_xy = rospy.get_param("~min_offset_xy", 0.3)
        self.max_offset_xy = rospy.get_param("~max_offset_xy", 1.2)
        self.offset_smooth_alpha = rospy.get_param("~offset_smooth_alpha", 0.8)
        self.max_offset_change = rospy.get_param("~max_offset_change", 0.25)
        self.heading_min_speed = rospy.get_param("~heading_min_speed", 0.1)
        self.num_sectors = rospy.get_param(
            "/perception_node/num_sector", 12)

        # LiDAR 距离为 perception_node 输出的归一化值 [0, 1]
        self.lidar_max_range = rospy.get_param("~lidar_max_range", 10.0)
        self.lidar_min_range = rospy.get_param("~lidar_min_range", 0.5)
        self.safe_distance_norm = rospy.get_param("~safe_distance_norm", 0.35)
        self.emergency_dist_norm = rospy.get_param("~emergency_dist_norm", 0.15)
        self.collision_dist = rospy.get_param("~collision_dist", 0.6)
        validate_distance_thresholds(
            self.lidar_min_range,
            self.collision_dist,
            self.emergency_dist_norm * self.lidar_max_range,
            self.safe_distance_norm * self.lidar_max_range)

        # Episode 与奖励参数
        self.max_episode_steps = rospy.get_param("~max_episode_steps", 500)
        self.safe_reward = rospy.get_param("~safe_reward", 0.05)
        self.step_penalty = rospy.get_param("~step_penalty", 0.02)
        self.action_change_penalty = rospy.get_param("~action_change_penalty", 0.01)
        self.unnecessary_action_penalty = rospy.get_param(
            "~unnecessary_action_penalty", 0.01)
        self.avoidance_success_reward = rospy.get_param(
            "~avoidance_success_reward", 30.0)
        self.terminal_failure_penalty = rospy.get_param(
            "~terminal_failure_penalty", 30.0)
        self.safe_proximity_penalty = rospy.get_param(
            "~safe_proximity_penalty", 0.01)
        self.emergency_proximity_penalty = rospy.get_param(
            "~emergency_proximity_penalty", 0.05)
        self.avoidance_clear_steps = rospy.get_param(
            "~avoidance_clear_steps", 10)
        self.avoidance_tracking_tolerance = rospy.get_param(
            "~avoidance_tracking_tolerance", 1.0)
        self.tracking_error_weight = rospy.get_param("~tracking_error_weight", 0.02)
        self.max_tracking_error = rospy.get_param("~max_tracking_error", 10.0)
        self.max_terminal_tracking_error = rospy.get_param("~max_terminal_tracking_error", 30.0)
        self.reward_clip_abs = rospy.get_param("~reward_clip_abs", 30.0)

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
        self.reset_attempts = rospy.get_param("~reset_attempts", 3)
        self.reset_wait_timeout = rospy.get_param("~reset_wait_timeout", 20.0)
        self.reset_min_clearance_norm = rospy.get_param(
            "~reset_min_clearance_norm", self.emergency_dist_norm)
        # reset_world 会回拨仿真时间，可能破坏已运行的 PX4/EKF 状态。
        self.use_reset_world = rospy.get_param("~use_reset_world", False)
        self.scenario_obstacles = rospy.get_param(
            "/dqn_training/scenario_obstacles", "unknown")
        self.scenario_trajectory = rospy.get_param(
            "/dqn_training/trajectory", self.trajectory_type)
        self.scenario_altitude = rospy.get_param(
            "/dqn_training/altitude", self.start_z)

        self.current_pose = None
        self.current_velocity = None
        self.reference_velocity = None
        self.features = None
        self.position_error = None

        self.episode_step = 0
        self.cumulative_reward = 0.0
        self.last_action = 0
        self.current_action = 0
        self.last_offset_x = 0.0
        self.last_offset_y = 0.0
        self.last_offset_z = 0.0
        self.motion_heading_world = self.start_yaw
        self.locked_avoid_side = 0
        self.selected_action = 0
        self.last_done_reason = "running"
        self.episode_min_front = 1.0
        self.episode_min_obstacle = 1.0
        self.reset_ok = False
        self.start_dist = float("inf")
        self.start_obstacle_min = 1.0
        self.obstacle_encountered = False
        self.avoidance_clear_count = 0
        self.avoidance_complete = False

        rospy.Subscriber("/obstacle_features", ObstacleFeatures, self._features_cb)
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self._pose_cb)
        rospy.Subscriber("/mavros/local_position/velocity_local",
                         TwistStamped, self._velocity_cb)
        rospy.Subscriber("/lqr_tracker/reference_velocity",
                         Vector3Stamped, self._reference_velocity_cb)
        rospy.Subscriber("/lqr_tracker/position_error", Float64, self._pos_err_cb)

        self.action_pub = rospy.Publisher("/rl_planner/action", Int32, queue_size=1)
        self.local_offset_pub = rospy.Publisher(
            "/rl_planner/local_offset", Vector3Stamped, queue_size=1)

        self.reset_world_srv = rospy.ServiceProxy("/gazebo/reset_world", Empty)
        self.get_world_properties_srv = rospy.ServiceProxy(
            "/gazebo/get_world_properties", GetWorldProperties)
        self.set_model_state_srv = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
        self.reset_trajectory_srv = rospy.ServiceProxy(
            "/lqr_tracker/reset_trajectory", Trigger)

        rospy.loginfo("[gazebo_env] 局部避障环境初始化完成 | n_actions=%d | reset_start=(%.2f, %.2f, %.2f)",
                      N_ACTIONS, self.start_x, self.start_y, self.start_z)

    def _pose_cb(self, msg):
        self.current_pose = msg

    def _features_cb(self, msg):
        self.features = msg
        if (msg is not None and msg.lidar_sectors and
                len(msg.lidar_sectors) != self.num_sectors):
            rospy.logerr_throttle(
                5, "[gazebo_env] 扇区数量不一致: configured=%d received=%d",
                self.num_sectors, len(msg.lidar_sectors))

    def _velocity_cb(self, msg):
        self.current_velocity = msg.twist.linear if msg is not None else None

    def _reference_velocity_cb(self, msg):
        self.reference_velocity = msg.vector if msg is not None else None

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

    def reset(self):
        self._update_reset_start_from_params()
        active_reset_tolerance = (self.reset_tolerance
                                  if self.trajectory_type == "step"
                                  else self.moving_reset_tolerance)
        self.episode_step = 0
        self.cumulative_reward = 0.0
        self.last_action = 0
        self.current_action = 0
        self.last_offset_x = 0.0
        self.last_offset_y = 0.0
        self.last_offset_z = 0.0
        self.motion_heading_world = self.start_yaw
        self.locked_avoid_side = 0
        self.selected_action = 0
        self.last_done_reason = "running"
        self.episode_min_front = 1.0
        self.episode_min_obstacle = 1.0
        self.reset_ok = False
        self.start_dist = float("inf")
        self.start_obstacle_min = 1.0
        self.obstacle_encountered = False
        self.avoidance_clear_count = 0
        self.avoidance_complete = False
        self.current_pose = None
        self.current_velocity = None
        self.reference_velocity = None
        self.features = None
        self.position_error = None
        self._publish_offset(0.0, 0.0, 0.0)

        for attempt in range(1, self.reset_attempts + 1):
            self.current_pose = None
            self.features = None
            self.position_error = None
            self._publish_offset(0.0, 0.0, 0.0)
            reset_world_ok = True
            model_pose_ok = True

            if self.reset_mode == "gazebo":
                if self.use_reset_world:
                    reset_world_ok = self._reset_world()
                model_pose_ok = self._reset_model_pose()

            # 移动轨迹保持当前相位连续，只等待清零 offset 后重新贴合路径。
            trajectory_reset_ok = (self._reset_trajectory()
                                   if self.trajectory_type == "step" else True)
            observations_ok = self._wait_for_reset_observations(timeout=self.reset_wait_timeout)
            self.start_dist = (self._distance_to_start()
                               if self.trajectory_type == "step"
                               else self._tracking_error())
            self.start_obstacle_min = self._minimum_obstacle_distance()
            self.reset_ok = (reset_world_ok and model_pose_ok and
                             trajectory_reset_ok and observations_ok and
                             self.start_dist <= active_reset_tolerance and
                             self.start_obstacle_min >= self.reset_min_clearance_norm)
            if self.reset_ok:
                break
            rospy.logwarn("[gazebo_env] reset 第 %d/%d 次失败: world=%s model=%s trajectory=%s obs=%s start_dist=%.2f obstacle_min=%.3f",
                          attempt, self.reset_attempts, reset_world_ok, model_pose_ok,
                          trajectory_reset_ok, observations_ok, self.start_dist,
                          self.start_obstacle_min)
            rospy.sleep(0.5)

        rospy.loginfo("[gazebo_env] episode reset 完成 | reset_ok=%s | start_dist=%.2f | obstacle_min=%.3f",
                      self.reset_ok, self.start_dist, self.start_obstacle_min)
        return self.get_observation()

    def step(self, action):
        self._publish_action(action)
        rospy.sleep(0.2)

        obstacle_min = self._forward_obstacle_distance()
        self.obstacle_encountered, self.avoidance_clear_count, self.avoidance_complete = (
            update_avoidance_progress(
                self.obstacle_encountered,
                self.avoidance_clear_count,
                obstacle_min,
                self._tracking_error(),
                self.safe_distance_norm,
                self.avoidance_tracking_tolerance,
                self.avoidance_clear_steps))
        reward = self.compute_reward()

        self.episode_step += 1
        self.episode_min_front = min(
            self.episode_min_front, self._front_min_distance())
        self.episode_min_obstacle = min(
            self.episode_min_obstacle, self._minimum_obstacle_distance())
        done = self.is_done()
        if self.last_done_reason == "avoidance_complete":
            reward += self.avoidance_success_reward
        elif self.last_done_reason in ("collision", "diverged",
                                       "low_altitude", "timeout"):
            reward = -self.terminal_failure_penalty
        reward = self._clip_reward(reward)
        self.cumulative_reward += reward
        self.last_action = self.current_action
        obs = self.get_observation()

        info = {
            "episode_step": self.episode_step,
            "cumulative_reward": self.cumulative_reward,
            "done_reason": self.last_done_reason,
            "front_min": self._front_min_distance(),
            "forward_obstacle_min": self._forward_obstacle_distance(),
            "min_front_min": self.episode_min_front,
            "obstacle_min": self._minimum_obstacle_distance(),
            "min_obstacle_min": self.episode_min_obstacle,
            "episode_success": episode_succeeded(
                self.last_done_reason),
            "obstacle_encountered": self.obstacle_encountered,
            "avoidance_clear_count": self.avoidance_clear_count,
            "position_error": self._tracking_error(),
            "selected_action": self.selected_action,
            "applied_action": self.current_action,
            "motion_heading": self.motion_heading_world,
            "body_yaw": self._body_yaw(),
            "offset_x": self.last_offset_x,
            "offset_y": self.last_offset_y,
            "offset_z": self.last_offset_z,
            "motion_sectors": self._motion_sectors(),
            "reset_ok": self.reset_ok,
            "start_dist": self.start_dist,
            "start_obstacle_min": self.start_obstacle_min,
        }
        return obs, reward, done, info

    def get_observation(self):
        if self.features is not None and self.features.lidar_sectors:
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
        last_action_one_hot = [0.0] * N_ACTIONS
        last_action_one_hot[self.last_action] = 1.0
        return sectors + [px, py, pz, pe] + last_action_one_hot

    def compute_reward(self):
        reward = self.safe_reward - self.step_penalty

        obstacle_min = self._forward_obstacle_distance()
        reward -= proximity_penalty(
            obstacle_min, self._collision_dist_norm(),
            self.emergency_dist_norm, self.safe_distance_norm,
            self.safe_proximity_penalty, self.emergency_proximity_penalty)

        pe_norm = min(self._tracking_error(), self.max_tracking_error)
        pe_norm /= max(self.max_tracking_error, 1e-6)
        reward -= self.tracking_error_weight * pe_norm

        reward -= action_change_cost(
            self.last_action, self.current_action, self.action_change_penalty)
        if obstacle_min >= self.safe_distance_norm and self.current_action != 0:
            reward -= self.unnecessary_action_penalty

        return self._clip_reward(reward)

    def is_done(self):
        if self._minimum_obstacle_distance() < self._collision_dist_norm():
            self.last_done_reason = "collision"
            rospy.loginfo("[gazebo_env] episode 终止: collision")
            return True

        if self._tracking_error() > self.max_terminal_tracking_error:
            self.last_done_reason = "diverged"
            rospy.logwarn("[gazebo_env] episode 终止: tracking diverged (pos_err=%.2f)",
                          self._tracking_error())
            return True

        if self.current_pose is not None and self.current_pose.pose.position.z < self.min_height:
            self.last_done_reason = "low_altitude"
            rospy.logwarn("[gazebo_env] episode 终止: low altitude")
            return True

        if self.avoidance_complete:
            self.last_done_reason = "avoidance_complete"
            rospy.loginfo("[gazebo_env] episode 终止: avoidance_complete (%d steps, min_obstacle=%.3f)",
                          self.episode_step, self.episode_min_obstacle)
            return True

        if self.episode_step >= self.max_episode_steps:
            self.last_done_reason = "timeout"
            rospy.loginfo("[gazebo_env] episode 终止: timeout (%d steps, min_obstacle=%.3f)",
                          self.episode_step, self.episode_min_obstacle)
            return True

        self.last_done_reason = "running"
        return False

    def _publish_action(self, action):
        desired_left, desired_z = 0.0, 0.0
        self.selected_action = int(action)
        self.current_action = self._constrain_action(self.selected_action)

        magnitude = avoidance_magnitude(
            self._forward_obstacle_distance(),
            self.safe_distance_norm,
            self.min_offset_xy,
            self.max_offset_xy)

        if self.current_action == 0:      # 沿原路径
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
        else:
            rospy.logwarn("[gazebo_env] 未知动作 %d，回退到沿原路径",
                          self.current_action)
            self.current_action = 0

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
        self.action_pub.publish(Int32(data=self.current_action))
        rospy.loginfo_throttle(
            1.0,
            "[gazebo_env] selected=%d applied=%d heading=%.2f yaw=%.2f forward=%.3f offset=(%.2f, %.2f, %.2f)",
            self.selected_action, self.current_action, self.motion_heading_world,
            self._body_yaw(), self._forward_obstacle_distance(),
            self.last_offset_x, self.last_offset_y, self.last_offset_z)

    def _constrain_action(self, action):
        action, self.locked_avoid_side = constrain_avoidance_action(
            action, self.locked_avoid_side,
            self._minimum_obstacle_distance(), self.safe_distance_norm)
        return action

    def allowed_actions(self):
        """Return actions consistent with the active avoidance side lock."""
        if self._minimum_obstacle_distance() >= self.safe_distance_norm:
            self.locked_avoid_side = 0
            return list(range(N_ACTIONS))
        if self.locked_avoid_side > 0:
            return [0, 1, 3]
        if self.locked_avoid_side < 0:
            return [0, 2, 4]
        return list(range(N_ACTIONS))

    def _publish_offset(self, dx, dy, dz):
        offset_msg = Vector3Stamped()
        offset_msg.header.stamp = rospy.Time.now()
        offset_msg.header.frame_id = "map"
        offset_msg.vector.x = dx
        offset_msg.vector.y = dy
        offset_msg.vector.z = dz
        self.local_offset_pub.publish(offset_msg)

    def clear_local_offset(self):
        """Return control to the unmodified LQR reference trajectory."""
        self.current_action = 0
        self.last_action = 0
        self.last_offset_x = 0.0
        self.last_offset_y = 0.0
        self.last_offset_z = 0.0
        self.locked_avoid_side = 0
        self._publish_offset(0.0, 0.0, 0.0)

    def _front_min_distance(self):
        if self.features is None or not self.features.lidar_sectors:
            return 1.0
        return directional_clearance(
            self._motion_sectors(), half_angle=math.pi / 3.0)

    def _forward_obstacle_distance(self):
        return directional_clearance(self._motion_sectors())

    def _motion_sectors(self):
        if self.features is None or not self.features.lidar_sectors:
            return [1.0] * self.num_sectors
        self._update_motion_heading()
        sectors = [max(0.0, min(float(distance), 1.0))
                   for distance in self.features.lidar_sectors]
        return rotate_sectors_to_heading(
            sectors, self._body_yaw(), self.motion_heading_world)

    def _update_motion_heading(self):
        velocity = (self.reference_velocity
                    if self.reference_velocity is not None
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
        if self.features is None or not self.features.lidar_sectors:
            return 1.0
        return min(float(distance) for distance in self.features.lidar_sectors)

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

    def _reset_model_pose(self):
        try:
            rospy.wait_for_service("/gazebo/set_model_state", timeout=2.0)
            state = ModelState()
            state.model_name = self._resolve_model_name()
            state.reference_frame = "world"
            state.pose.position.x = self.start_x
            state.pose.position.y = self.start_y
            state.pose.position.z = self.start_z
            state.pose.orientation.z = math.sin(self.start_yaw * 0.5)
            state.pose.orientation.w = math.cos(self.start_yaw * 0.5)
            res = self.set_model_state_srv(state)
            if not res.success:
                rospy.logwarn("[gazebo_env] set_model_state 失败: %s", res.status_message)
                return False
            self.current_pose = None
            self.features = None
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

    def _wait_for_reset_observations(self, timeout=3.0):
        start = rospy.Time.now()
        rate = rospy.Rate(10)
        on_reference = False
        while (rospy.Time.now() - start).to_sec() < timeout:
            if self.trajectory_type == "step":
                on_reference = (self.current_pose is not None and
                                self._distance_to_start() <= self.reset_tolerance)
            else:
                on_reference = (self.position_error is not None and
                                self._tracking_error() <= self.moving_reset_tolerance)
            has_clearance = (self.features is not None and
                             self._minimum_obstacle_distance() >=
                             self.reset_min_clearance_norm)
            if on_reference and has_clearance:
                return True
            rate.sleep()
        rospy.logwarn("[gazebo_env] 等待 reset 恢复超时 (%.1fs): on_reference=%s obstacle_min=%.3f required=%.3f",
                      timeout, on_reference, self._minimum_obstacle_distance(),
                      self.reset_min_clearance_norm)
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
        return self.num_sectors + 4 + N_ACTIONS

    @property
    def n_actions(self):
        return N_ACTIONS
