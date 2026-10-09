#!/usr/bin/env python3
"""地面车 DQN 避障环境（运动学方案）— 镜像 gazebo_env.py 的无人机环境语义。

分层边界：
    - 参考轨迹 = 沿 +x 匀速直线（1 m/s），由环境内部生成。
    - RL 只输出局部避障偏移（左右横移/加速），不规划全局路线。
实现方式：
    - 运动学单车模型在环境内部积分（integrate_pose），每步通过
      /ground_gazebo/set_model_state 传送车模；Gazebo 负责可视化与杆体存在。
    - 感知用虚拟激光（virtual_sectors）：由位姿 + 静态杆位解析计算 12 扇区距离，
      确定、可复现，不受 Gazebo 传感器插件影响。
"""

import math

import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState

from training_utils import (action_change_cost, avoidance_magnitude,
                            constrain_avoidance_action, episode_succeeded,
                            limit_change, proximity_penalty, smooth_value,
                            update_avoidance_progress)

from ground_utils import (diff_drive_command, forward_clearance,
                          integrate_pose, minimum_clearance, reference_target,
                          tracking_error, virtual_sectors)

ACTION_NAMES = {
    0: "沿原路径",
    1: "左移",
    2: "右移",
    3: "左移并加速",
    4: "右移并加速",
}
N_ACTIONS = 5
STEP_DT = 0.2  # 环境步长时间（秒）


class GroundEnv:
    def __init__(self):
        try:
            rospy.init_node("ground_env", anonymous=True, disable_signals=True)
        except rospy.ROSException:
            pass

        # 参考轨迹与动作参数（支持全局或私有参数注入）
        self.reference_speed = rospy.get_param(
            "/ground_env/reference_speed",
            rospy.get_param("~reference_speed", 1.0))
        self.fast_speed = rospy.get_param(
            "/ground_env/fast_speed", rospy.get_param("~fast_speed", 1.3))
        self.min_offset_xy = rospy.get_param(
            "/ground_env/min_offset_xy",
            rospy.get_param("~min_offset_xy", 0.3))
        self.max_offset_xy = rospy.get_param(
            "/ground_env/max_offset_xy",
            rospy.get_param("~max_offset_xy", 1.2))
        self.offset_smooth_alpha = rospy.get_param("~offset_smooth_alpha", 0.8)
        self.max_offset_change = rospy.get_param("~max_offset_change", 0.25)

        # 虚拟激光与安全距离（与无人机环境相同的归一化约定）
        self.num_sectors = rospy.get_param("~num_sectors", 12)
        self.lidar_max_range = rospy.get_param("~lidar_max_range", 10.0)
        self.pole_radius = rospy.get_param("~pole_radius", 0.15)
        poles_param = rospy.get_param("~poles", [[6.0, 0.0],
                                                 [12.0, 0.5],
                                                 [18.0, -0.3]])
        self.poles = [(float(p[0]), float(p[1])) for p in poles_param]
        self.safe_distance_norm = rospy.get_param("~safe_distance_norm", 0.35)
        self.emergency_dist_norm = rospy.get_param("~emergency_dist_norm", 0.15)
        self.collision_dist = rospy.get_param("~collision_dist", 0.6)
        self.collision_dist_norm = self.collision_dist / max(
            self.lidar_max_range, 1e-6)

        # Episode 与奖励参数（镜像无人机环境）
        self.max_episode_steps = rospy.get_param("~max_episode_steps", 150)
        self.safe_reward = rospy.get_param("~safe_reward", 0.15)
        self.step_penalty = rospy.get_param("~step_penalty", 0.005)
        self.action_change_penalty = rospy.get_param(
            "~action_change_penalty", 0.005)
        self.unnecessary_action_penalty = rospy.get_param(
            "~unnecessary_action_penalty", 0.005)
        self.avoidance_success_reward = rospy.get_param(
            "~avoidance_success_reward", 25.0)
        self.terminal_failure_penalty = rospy.get_param(
            "~terminal_failure_penalty", 20.0)
        self.safe_proximity_penalty = rospy.get_param(
            "~safe_proximity_penalty", 0.005)
        self.emergency_proximity_penalty = rospy.get_param(
            "~emergency_proximity_penalty", 0.02)
        self.avoidance_clear_steps = rospy.get_param(
            "~avoidance_clear_steps", 10)
        self.avoidance_tracking_tolerance = rospy.get_param(
            "~avoidance_tracking_tolerance", 1.0)
        self.tracking_error_weight = rospy.get_param(
            "/ground_env/tracking_error_weight",
            rospy.get_param("~tracking_error_weight", 0.1))
        self.max_tracking_error = rospy.get_param("~max_tracking_error", 10.0)
        self.reward_clip_abs = rospy.get_param("~reward_clip_abs", 30.0)

        # Reset 参数
        self.reset_min_clearance_norm = rospy.get_param(
            "~reset_min_clearance_norm", self.emergency_dist_norm)
        self.model_name = rospy.get_param("~model_name", "ground_car")
        self.start_x = rospy.get_param("~start_x", 0.0)
        self.start_y = rospy.get_param("~start_y", 0.0)
        self.start_z = rospy.get_param("~start_z", 0.11)
        self.start_yaw = rospy.get_param("~start_yaw", 0.0)

        # Gazebo 传送服务（可视化同步）
        rospy.wait_for_service("/ground_gazebo/set_model_state", timeout=20.0)
        self.set_model_state_srv = rospy.ServiceProxy(
            "/ground_gazebo/set_model_state", SetModelState)

        # 状态
        self.pose_x = self.start_x
        self.pose_y = self.start_y
        self.pose_yaw = self.start_yaw
        self.sectors = [1.0] * self.num_sectors
        self.episode_step = 0
        self.last_done_reason = "running"
        self.reset_ok = False
        self.locked_avoid_side = 0
        self.selected_action = 0
        self.current_action = 0
        self.last_action = 0
        self.last_offset_x = 0.0
        self.last_offset_y = 0.0
        self.episode_min_obstacle = 1.0
        self.obstacle_encountered = False
        self.avoidance_clear_count = 0
        self.avoidance_complete = False
        self.cumulative_reward = 0.0

    # ---------------- 内部工具 ----------------

    def _update_sectors(self):
        self.sectors = virtual_sectors(
            self.pose_x, self.pose_y, self.pose_yaw, self.poles,
            num_sectors=self.num_sectors,
            max_range=self.lidar_max_range, pole_radius=self.pole_radius)

    def _forward_clearance(self):
        return forward_clearance(self.sectors)

    def _minimum_clearance(self):
        return minimum_clearance(self.sectors)

    def _tracking_error(self):
        target_x, target_y = reference_target(
            self.episode_step * STEP_DT, self.reference_speed,
            self.last_offset_x, self.last_offset_y)
        return tracking_error(self.pose_x, self.pose_y, target_x, target_y)

    def _teleport(self):
        state = ModelState()
        state.model_name = self.model_name
        state.reference_frame = "world"
        state.pose.position.x = self.pose_x
        state.pose.position.y = self.pose_y
        state.pose.position.z = self.start_z
        state.pose.orientation.z = math.sin(self.pose_yaw * 0.5)
        state.pose.orientation.w = math.cos(self.pose_yaw * 0.5)
        try:
            result = self.set_model_state_srv(state)
            return result.success
        except rospy.ServiceException as exc:
            rospy.logwarn("[ground_env] set_model_state 失败: %s", exc)
            return False

    def _constrain_action(self, action):
        action, self.locked_avoid_side = constrain_avoidance_action(
            action, self.locked_avoid_side,
            self._minimum_clearance(), self.safe_distance_norm)
        return action

    def allowed_actions(self):
        if self._minimum_clearance() >= self.safe_distance_norm:
            self.locked_avoid_side = 0
            return list(range(N_ACTIONS))
        if self.locked_avoid_side > 0:
            return [0, 1, 3]
        if self.locked_avoid_side < 0:
            return [0, 2, 4]
        return list(range(N_ACTIONS))

    def _clip_reward(self, reward):
        limit = max(self.reward_clip_abs, 1e-6)
        return max(-limit, min(limit, reward))

    # ---------------- 环境接口 ----------------

    def reset(self):
        self.last_done_reason = "running"
        self.episode_step = 0
        self.last_action = 0
        self.current_action = 0
        self.last_offset_x = 0.0
        self.last_offset_y = 0.0
        self.locked_avoid_side = 0
        self.selected_action = 0
        self.episode_min_obstacle = 1.0
        self.obstacle_encountered = False
        self.avoidance_clear_count = 0
        self.avoidance_complete = False
        self.cumulative_reward = 0.0
        self.pose_x = self.start_x
        self.pose_y = self.start_y
        self.pose_yaw = self.start_yaw
        self._teleport()
        self._update_sectors()
        self.reset_ok = (self._minimum_clearance() >=
                         self.reset_min_clearance_norm)
        rospy.loginfo("[ground_env] reset 完成 | reset_ok=%s | obstacle_min=%.3f",
                      self.reset_ok, self._minimum_clearance())
        return self.get_observation()

    def step(self, action):
        self.selected_action = int(action)
        self.current_action = self._constrain_action(self.selected_action)

        magnitude = avoidance_magnitude(
            self._forward_clearance(), self.safe_distance_norm,
            self.min_offset_xy, self.max_offset_xy)
        speed = self.reference_speed
        desired_x, desired_y = 0.0, 0.0
        if self.current_action == 0:  # 沿原路径
            pass
        elif self.current_action == 1:  # 左移
            desired_y = magnitude
        elif self.current_action == 2:  # 右移
            desired_y = -magnitude
        elif self.current_action == 3:  # 左移并加速
            desired_y = magnitude
            speed = self.fast_speed
        elif self.current_action == 4:  # 右移并加速
            desired_y = -magnitude
            speed = self.fast_speed
        else:
            raise ValueError("action must be in range(5), got %d" % action)

        offset_x = smooth_value(self.last_offset_x, desired_x,
                                self.offset_smooth_alpha)
        offset_y = smooth_value(self.last_offset_y, desired_y,
                                self.offset_smooth_alpha)
        offset_x = limit_change(self.last_offset_x, offset_x,
                                self.max_offset_change)
        offset_y = limit_change(self.last_offset_y, offset_y,
                                self.max_offset_change)
        self.last_offset_x = 0.0 if abs(offset_x) < 1e-3 else offset_x
        self.last_offset_y = 0.0 if abs(offset_y) < 1e-3 else offset_y

        # 运动学积分推进
        target_x, target_y = reference_target(
            self.episode_step * STEP_DT, speed,
            self.last_offset_x, self.last_offset_y)
        linear, angular = diff_drive_command(
            self.pose_x, self.pose_y, self.pose_yaw, target_x, target_y,
            base_speed=speed,
            position_error=tracking_error(
                self.pose_x, self.pose_y, target_x, target_y))
        self.pose_x, self.pose_y, self.pose_yaw = integrate_pose(
            self.pose_x, self.pose_y, self.pose_yaw,
            linear, angular, STEP_DT)
        self._teleport()
        rospy.sleep(STEP_DT)
        self._update_sectors()

        forward_min = self._forward_clearance()
        self.obstacle_encountered, self.avoidance_clear_count, \
            self.avoidance_complete = update_avoidance_progress(
                self.obstacle_encountered, self.avoidance_clear_count,
                forward_min, self._tracking_error(),
                self.safe_distance_norm, self.avoidance_tracking_tolerance,
                self.avoidance_clear_steps)
        reward = self.compute_reward()

        self.episode_step += 1
        self.episode_min_obstacle = min(
            self.episode_min_obstacle, self._minimum_clearance())
        done = self.is_done()
        if self.last_done_reason == "avoidance_complete":
            reward += self.avoidance_success_reward
        elif self.last_done_reason in ("collision", "timeout"):
            reward -= self.terminal_failure_penalty
        reward = self._clip_reward(reward)
        self.cumulative_reward += reward
        self.last_action = self.current_action

        info = {
            "episode_step": self.episode_step,
            "cumulative_reward": self.cumulative_reward,
            "done_reason": self.last_done_reason,
            "forward_obstacle_min": forward_min,
            "obstacle_min": self._minimum_clearance(),
            "min_obstacle_min": self.episode_min_obstacle,
            "episode_success": episode_succeeded(self.last_done_reason),
            "obstacle_encountered": self.obstacle_encountered,
            "avoidance_clear_count": self.avoidance_clear_count,
            "position_error": self._tracking_error(),
            "selected_action": self.selected_action,
            "applied_action": self.current_action,
            "offset_x": self.last_offset_x,
            "offset_y": self.last_offset_y,
            "pose_x": self.pose_x,
            "pose_y": self.pose_y,
            "reset_ok": self.reset_ok,
        }
        return self.get_observation(), reward, done, info

    def get_observation(self):
        sectors = list(self.sectors[:self.num_sectors])
        sectors += [1.0] * max(0, self.num_sectors - len(sectors))
        px = max(-2.0, min(2.0, self.pose_x / 10.0))
        py = max(-2.0, min(2.0, self.pose_y / 10.0))
        pe = min(self._tracking_error() / 5.0, 1.0)
        last_action_one_hot = [0.0] * N_ACTIONS
        last_action_one_hot[self.last_action] = 1.0
        return sectors + [px, py, pe] + last_action_one_hot

    def compute_reward(self):
        reward = self.safe_reward - self.step_penalty
        forward_min = self._forward_clearance()
        reward -= proximity_penalty(
            forward_min, self.collision_dist_norm,
            self.emergency_dist_norm, self.safe_distance_norm,
            self.safe_proximity_penalty, self.emergency_proximity_penalty)
        pe_norm = min(self._tracking_error(), self.max_tracking_error)
        pe_norm /= max(self.max_tracking_error, 1e-6)
        reward -= self.tracking_error_weight * pe_norm
        reward -= action_change_cost(
            self.last_action, self.current_action,
            self.action_change_penalty)
        if forward_min >= self.safe_distance_norm and self.current_action != 0:
            reward -= self.unnecessary_action_penalty
        return self._clip_reward(reward)

    def is_done(self):
        if self._minimum_clearance() < self.collision_dist_norm:
            self.last_done_reason = "collision"
            rospy.loginfo("[ground_env] episode 终止: collision")
            return True
        if self.avoidance_complete:
            self.last_done_reason = "avoidance_complete"
            rospy.loginfo("[ground_env] episode 终止: avoidance_complete (%d steps, min_obstacle=%.3f)",
                          self.episode_step, self.episode_min_obstacle)
            return True
        if self.episode_step >= self.max_episode_steps:
            self.last_done_reason = "timeout"
            rospy.loginfo("[ground_env] episode 终止: timeout (%d steps, min_obstacle=%.3f)",
                          self.episode_step, self.episode_min_obstacle)
            return True
        self.last_done_reason = "running"
        return False

    def clear_local_offset(self):
        self.current_action = 0
        self.last_action = 0
        self.last_offset_x = 0.0
        self.last_offset_y = 0.0
        self.locked_avoid_side = 0

    @property
    def observation_space_dim(self):
        return self.num_sectors + 2 + 1 + N_ACTIONS

    @property
    def n_actions(self):
        return N_ACTIONS
