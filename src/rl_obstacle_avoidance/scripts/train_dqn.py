#!/usr/bin/env python3
"""Train and evaluate the local obstacle-avoidance DQN."""

import argparse
import csv
from datetime import datetime
import json
import os
import random
import sys

import torch


sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from dqn_agent import DQNAgent
from gazebo_env import ACTION_NAMES, GazeboEnv
from replay_buffer import PrioritizedReplayBuffer, ReplayBuffer
from training_utils import (count_termination_reason, epsilon_linear_at_step,
                            evaluation_score, parse_hidden_dims,
                            rolling_evaluation_metrics, update_best_score,
                            NStepAccumulator)


BUFFER_CAPACITY = 100000
EPSILON_START = 1.0
EPSILON_END = 0.05
PER_ALPHA = 0.6
PER_BETA = 0.4
# A reset failure means the simulator/controller state is not trustworthy.
# Allow one retry for a transient sensor/service delay, but do not let later
# scene rotations train on stale state after repeated failures.
MAX_CONSECUTIVE_RESET_FAILURES = 2
# reset 失败属于基础设施故障（出生点离障碍物过近等）：每个 episode 最多换场景
# 重试 RESET_RETRY_LIMIT 次，仍失败则跳过该回合，且不占用 episode 预算。
RESET_RETRY_LIMIT = 3
EXPECTED_EPISODE_STEPS = 150
HEALTH_CHECK_EPISODES = 20
MIN_HEALTH_CHECK_STEPS = 100
TRAINING_CHECKPOINT_VERSION = 1

TRAIN_FIELDS = [
    "episode", "trial_id", "scenario_id", "trajectory_phase_s",
    "total_reward", "steps", "total_steps", "epsilon",
    "tracking_rmse", "tracking_max_error", "tracking_steady_error",
    "recovery_time", "metric_path_length",
    "avg_loss", "result", "done_reason", "final_front_min",
    "min_front_min", "final_obstacle_min", "min_obstacle_min",
    "final_pos_error", "reset_ok", "reset_failure_reason", "start_dist",
    "start_obstacle_min",
    "obstacle_encountered", "avoidance_clear_count", "avoidance_events",
    "avoidance_quality", "avoidance_reward",
    "goal_x", "goal_y", "path_length", "a_encounter_steps", "b_encounter_steps",
    "collision_events", "collision_events_a", "collision_events_b",
    "geometry_collision_events", "min_geometry_clearance",
    "collision_events_per_meter", "travel_distance",
    "safe_success", "action_switch_rate", "action_reversal_rate",
    "action_0", "action_1", "action_2", "action_3", "action_4", "action_5",
]
STEP_FIELDS = [
    "episode", "trial_id", "scenario_id", "trajectory_phase_s",
    "episode_step", "total_step", "selected_action",
    "applied_action", "reward", "done", "done_reason",
    "pose_x", "pose_y", "pose_z",
    "tracking_rmse", "tracking_max_error", "tracking_steady_error",
    "recovery_time", "metric_path_length",
    "front_min", "forward_obstacle_min", "obstacle_min",
    "left_clearance", "right_clearance",
    "position_error",
    "motion_heading", "body_yaw", "offset_x", "offset_y",
    "offset_z", "motion_sectors", "collision_events", "geometry_clearance",
    # 前瞻可达性门控：avoid_needed=原始判据，avoid_suppressed=叠加尾部锁定后的最终判定
    "avoid_needed", "avoid_suppressed",
    "avoidance_quality", "avoidance_reward_step", "safe_success",
    "action_switch_rate", "action_reversal_rate", "local_speed_scale",
    "trajectory_paused", "filter_reason", "action_reason", "predicted_h", "prediction_error",
    "filter_sensor_fresh", "safety_filter_mode", "filter_margin",
    "filter_safe_actions", "action_cost",
    # 备份逃逸审计：shield_escape 标记本步是否逃逸，shield_escape_count 为回合内累计次数
    "shield_escape", "shield_escape_count", "escape_margin",
]


# reset 失败明细日志（不进入 training_log.csv，避免污染回合统计）
RESET_FAILURE_FIELDS = [
    "episode", "attempt", "scenario_id",
    "reset_failure_reason", "start_dist", "start_obstacle_min",
    "total_steps",
]


def format_metric(value, default=float("nan")):
    """Format optional metric values without turning an absent result into a crash."""
    if value is None:
        value = default
    try:
        return "%.4f" % float(value)
    except (TypeError, ValueError):
        return "nan"


EVAL_FIELDS = [
    "evaluation", "after_episode", "episodes", "valid_episodes",
    "scenario_ids",
    "success_rate", "safe_success_rate", "collision_rate", "risk_entry_rate",
    "geometry_collision_rate", "mean_min_geometry_clearance",
    "timeout_rate", "shield_infeasible_rate", "reset_failure_rate",
    "trajectory_complete_rate",
    "mean_reward", "mean_steps",
    "mean_min_obstacle", "mean_final_pos_error", "reset_failures",
    "action_0_rate", "action_1_rate", "action_2_rate", "action_3_rate",
    "action_4_rate", "action_5_rate",
    "rolling_success_rate", "rolling_collision_rate", "rolling_timeout_rate",
    "rolling_shield_infeasible_rate", "rolling_reset_failure_rate",
    "rolling_trajectory_complete_rate",
    "rolling_safe_success_rate", "rolling_risk_entry_rate",
    "rolling_mean_reward", "window_used", "termination_reasons",
]


def parse_args():
    parser = argparse.ArgumentParser(description="DQN 局部避障训练")
    parser.add_argument("--episodes", type=int, default=300,
                        help="训练 episode 数（默认: 300）")
    parser.add_argument("--max_steps", type=int, default=450,
                        help="每个 episode 最大步数（默认: 450）")
    parser.add_argument("--run-dir", default=None,
                        help="本轮产物目录；默认在 training_runs 下按时间创建")
    parser.add_argument("--eval-interval", type=int, default=100,
                        help="每隔多少个训练 episode 做一次贪心评估（默认: 100）")
    parser.add_argument("--eval-episodes", type=int, default=30,
                        help="每次贪心评估的 episode 数")
    parser.add_argument("--eval-window", type=int, default=3,
                        help="最佳模型选择的滚动评估窗口大小")
    parser.add_argument("--epsilon-decay-steps", type=int, default=0,
                        help="epsilon 衰减步数；0 表示总预算的 50%%")
    parser.add_argument("--learning-rate", type=float, default=1e-4,
                        help="Adam 学习率（默认: 1e-4）")
    parser.add_argument("--gamma", type=float, default=0.99,
                        help="折扣因子（默认: 0.99）")
    parser.add_argument("--batch-size", type=int, default=64,
                        help="每次梯度更新的采样批大小（默认: 64）")
    parser.add_argument("--target-update-steps", type=int, default=500,
                        help="目标网络同步周期，单位环境步（默认: 500）")
    parser.add_argument("--train-start-steps", type=int, default=256,
                        help="开始训练前至少积累的环境步数（默认: 256）")
    parser.add_argument("--hidden-dims", default="128,128",
                        help="隐藏层维度，逗号分隔（默认: 128,128）")
    parser.add_argument("--use-lr-schedule", action="store_true",
                        help="启用余弦退火学习率调度（默认关闭）")
    parser.add_argument("--resume", default=None,
                        help="从 training_state.pt 完整续训，或从旧 .pt 仅加载权重")
    parser.add_argument("--seed", type=int, default=0, help="随机种子")
    parser.add_argument("--n-step", type=int, default=3,
                        help="多步回报长度（默认: 3）")
    curriculum = parser.add_mutually_exclusive_group()
    curriculum.add_argument("--curriculum", dest="curriculum",
                            action="store_true",
                            help="按训练进度从低难度逐步引入高难度场景（默认启用）")
    curriculum.add_argument("--no-curriculum", dest="curriculum",
                            action="store_false",
                            help="关闭课程学习，立即使用全部训练场景")
    parser.set_defaults(curriculum=True)
    parser.add_argument("--scenario-manifest", default=None,
                        help="训练场景 manifest JSON（如 manifests/train.json），"
                             "启用跨场景轮换（单 run 内逐集切换）")
    parser.add_argument("--eval-manifest", default=None,
                        help="评估场景 manifest；默认使用训练 manifest 同目录的 test.json")
    parser.add_argument("--eval-split", default="test",
                        help="评估 manifest 中使用的 split 字段（默认 test）；"
                             "正式流程须传 validation 以选择 checkpoint，避免读取 test")
    parser.add_argument("--rotation-mode", default="round",
                        choices=["round", "random"],
                        help="场景轮换方式: round 轮流 / random 随机（默认 round）")
    args = parser.parse_args()

    if args.episodes <= 0 or args.max_steps <= 0:
        parser.error("--episodes 和 --max_steps 必须为正数")
    if args.eval_interval <= 0 or args.eval_episodes <= 0:
        parser.error("评估间隔和评估 episode 数必须为正数")
    if args.eval_window < 1:
        parser.error("--eval-window 必须为正整数")
    if args.epsilon_decay_steps < 0:
        parser.error("--epsilon-decay-steps 不能为负数")
    if args.learning_rate <= 0:
        parser.error("--learning-rate 必须为正数")
    if not 0.0 < args.gamma <= 1.0:
        parser.error("--gamma 必须在 (0, 1] 范围内")
    if args.batch_size <= 0:
        parser.error("--batch-size 必须为正数")
    if args.target_update_steps <= 0:
        parser.error("--target-update-steps 必须为正数")
    if args.train_start_steps < 0:
        parser.error("--train-start-steps 不能为负数")
    if args.n_step <= 0:
        parser.error("--n-step 必须为正整数")
    try:
        args.hidden_dims = parse_hidden_dims(args.hidden_dims)
    except ValueError as exc:
        parser.error(str(exc))
    return args


def create_run_dir(requested_path):
    if requested_path:
        path = os.path.abspath(requested_path)
    else:
        run_id = datetime.now().strftime("dqn_%Y%m%d_%H%M%S_%f")
        path = os.path.abspath(os.path.join("training_runs", run_id))
    os.makedirs(path, exist_ok=False)
    return path


def initialize_csv(path, fields):
    with open(path, "w", newline="") as stream:
        csv.DictWriter(stream, fieldnames=fields).writeheader()


def append_csv(path, fields, row):
    with open(path, "a", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writerow(row)


def write_json(path, data):
    with open(path, "w") as stream:
        json.dump(data, stream, indent=2, sort_keys=True)
        stream.write("\n")


def clone_model_state(agent):
    return {
        key: value.detach().cpu().clone()
        for key, value in agent.q_network.state_dict().items()
    }


def save_training_checkpoint(path, agent, buffer, progress,
                             eval_history, best_score, best_model_state,
                             decay_steps):
    """Atomically save everything required to continue training."""
    payload = {
        "training_checkpoint_version": TRAINING_CHECKPOINT_VERSION,
        "agent": agent.training_state_dict(),
        "replay_buffer": buffer.state_dict(),
        "progress": dict(progress),
        "eval_history": list(eval_history),
        "best_score": best_score,
        "best_model_state": best_model_state,
        "epsilon_decay_steps": int(decay_steps),
        "python_random_state": random.getstate(),
        "torch_rng_state": torch.get_rng_state(),
    }
    if torch.cuda.is_available():
        payload["torch_cuda_rng_state"] = torch.cuda.get_rng_state_all()
    temporary_path = path + ".tmp"
    torch.save(payload, temporary_path)
    os.replace(temporary_path, path)


def load_training_checkpoint(path, agent, buffer):
    """Load a full checkpoint, or fall back to a legacy weights-only file."""
    try:
        payload = torch.load(path, map_location=agent.device,
                             weights_only=False)
    except TypeError:
        payload = torch.load(path, map_location=agent.device)
    if not (isinstance(payload, dict) and
            payload.get("training_checkpoint_version") ==
            TRAINING_CHECKPOINT_VERSION):
        agent.load(path)
        return None
    agent.load_training_state_dict(payload["agent"])
    buffer.load_state_dict(payload["replay_buffer"])
    random.setstate(payload["python_random_state"])
    torch.set_rng_state(payload["torch_rng_state"].cpu())
    if torch.cuda.is_available() and "torch_cuda_rng_state" in payload:
        torch.cuda.set_rng_state_all(
            [state.cpu() for state in payload["torch_cuda_rng_state"]])
    return payload


def environment_config(env):
    names = [
        "trajectory_type", "step_z", "min_height", "min_offset_xy",
        "max_offset_xy", "offset_smooth_alpha", "max_offset_change",
        "num_sectors",
        "heading_min_speed", "turning_front_extra_angle",
        "reference_velocity_timeout", "avoidance_lookahead_time",
        "lidar_max_range", "lidar_min_range", "warning_distance_norm",
        "path_feature_scale_m", "safe_distance_norm",
        "emergency_dist_norm", "perception_timeout", "action_duration",
        "collision_dist", "path_clearance_margin", "safe_reward",
        "step_penalty", "action_change_penalty", "unnecessary_action_penalty",
        "action_reversal_penalty", "action_hold_steps",
        "clearance_change_scale", "ttc_penalty_scale", "ttc_safe_seconds",
        "risk_progress_scale",
        "goal_reached_reward", "avoidance_success_reward",
        "avoidance_event_reward", "avoidance_quality_bonus",
        "max_avoidance_reward_events", "avoidance_rearm_progress",
        "terminal_failure_penalty",
        "waypoint_speed", "astar_safety_margin", "astar_service_timeout",
        "goal_min_distance", "goal_max_distance",
        "safe_proximity_penalty", "emergency_proximity_penalty",
        "avoidance_clear_steps",
        "avoidance_tracking_tolerance", "reset_min_clearance_norm",
        "avoidance_tail_completion_norm", "avoidance_tail_release_norm",
        "avoidance_completion_grace_steps",
        "tracking_error_weight",
        "max_tracking_error", "max_terminal_tracking_error",
        "reward_clip_abs", "reset_mode", "reset_tolerance",
        "moving_reset_tolerance", "reset_velocity_tolerance", "reset_attempts", "reset_wait_timeout",
        "preplan_settle_timeout", "preplan_speed_tolerance",
        "preplan_position_tolerance", "preplan_stable_steps",
        "preplan_attempts",
        "scenario_obstacles", "scenario_trajectory", "scenario_altitude",
        "obstacle_geometry_size", "drone_half_extents",
        "action_interface",
        "safety_filter_mode", "safety_filter_base_margin",
        "safety_filter_kappa", "safety_filter_prediction_error",
        "safety_filter_horizon", "safety_filter_dt",
        "safety_filter_alpha", "safety_filter_epsilon_pred",
        "safety_filter_lateral_speed", "safety_filter_vertical_speed",
        "safety_filter_empty_set_terminal",
        "scenario_id",
    ]
    return {name: getattr(env, name) for name in names}


def scenario_difficulty(scenario):
    """Return the ordered curriculum tier for a generated scenario."""
    layout = scenario.get("layout", {}).get("type", "")
    tiers = {
        "frontal_block": 0,
        "single_side": 0,
        "mirror_pair": 1,
        "dense_corridor": 2,
        "curve_challenge": 2,
    }
    return tiers.get(layout, min(3, len(scenario.get("obstacles", []))))


def curriculum_pool(scenarios, fraction):
    """Progressively add difficulty tiers while retaining earlier lessons."""
    if not scenarios:
        return []
    levels = sorted({scenario_difficulty(item) for item in scenarios})
    stage = min(len(levels) - 1,
                int(max(0.0, min(1.0, fraction)) * len(levels)))
    maximum = levels[stage]
    return [item for item in scenarios
            if scenario_difficulty(item) <= maximum]


def load_split_scenarios(path, split):
    if not path or not os.path.isfile(path):
        raise SystemExit("[错误] 找不到 %s manifest: %s" % (split, path))
    with open(path, encoding="utf-8") as stream:
        manifest = json.load(stream)
    scenarios = [item for item in manifest.get("scenarios", [])
                 if item.get("split", split) == split and item.get("obstacles")]
    if not scenarios:
        raise SystemExit("[错误] manifest 中没有 %s 场景: %s" % (split, path))
    return scenarios


def evaluate(agent, env, episode_count, max_steps, scenarios=None):
    rewards = []
    steps_per_episode = []
    minimum_obstacles = []
    final_errors = []
    successes = 0
    safe_successes = 0
    collisions = 0
    geometry_collisions = 0
    geometry_clearances = []
    risk_entries = 0
    timeouts = 0
    shield_infeasible = 0
    reset_failures = 0
    trajectory_complete = 0
    action_counts = [0] * env.n_actions
    reason_counts = {}
    scenario_ids = []

    for episode in range(episode_count):
        if scenarios:
            scenario = scenarios[episode % len(scenarios)]
            env.arm_deploy(scenario)
            scenario_ids.append(str(scenario.get("scenario_id", "unknown")))
        state = env.reset()
        if not env.reset_ok:
            reset_failures += 1
            continue

        total_reward = 0.0
        final_info = {}
        episode_steps = 0
        for _ in range(env.episode_step_limit):
            action = agent.select_action(
                state, epsilon=0.0, allowed_actions=env.allowed_actions())
            state, reward, done, final_info = env.step(action)
            applied_action = final_info.get("applied_action", action)
            if applied_action is not None:
                action_counts[applied_action] += 1
            total_reward += reward
            episode_steps += 1
            if done:
                break

        count_termination_reason(
            reason_counts, final_info.get("done_reason"))
        rewards.append(total_reward)
        steps_per_episode.append(episode_steps)
        minimum_obstacles.append(final_info.get("min_obstacle_min", 1.0))
        final_errors.append(final_info.get("position_error", 0.0))
        successes += int(bool(final_info.get("episode_success", False)))
        safe_successes += int(bool(final_info.get("safe_success", False)))
        collisions += int(final_info.get("done_reason") == "collision")
        geometry_collisions += int(final_info.get(
            "geometry_collision_events", 0) > 0)
        geometry_clearances.append(final_info.get(
            "min_geometry_clearance", float("inf")))
        risk_entries += int(final_info.get("collision_events", 0) > 0)
        timeouts += int(final_info.get("done_reason") == "timeout")
        shield_infeasible += int(
            final_info.get("done_reason") == "shield_infeasible")
        trajectory_complete += int(
            final_info.get("done_reason") == "trajectory_complete")

    valid = len(rewards)
    denominator = float(episode_count)
    total_actions = float(sum(action_counts))
    metrics = {
        "episodes": episode_count,
        "valid_episodes": valid,
        "scenario_ids": scenario_ids,
        "success_rate": successes / denominator,
        "safe_success_rate": safe_successes / denominator,
        "collision_rate": collisions / denominator,
        "risk_entry_rate": risk_entries / denominator,
        "geometry_collision_rate": geometry_collisions / denominator,
        "mean_min_geometry_clearance": (
            sum(geometry_clearances) / valid if valid else 0.0),
        "timeout_rate": timeouts / denominator,
        "shield_infeasible_rate": shield_infeasible / denominator,
        "reset_failure_rate": reset_failures / denominator,
        "trajectory_complete_rate": trajectory_complete / denominator,
        "mean_reward": sum(rewards) / valid if valid else 0.0,
        "mean_steps": sum(steps_per_episode) / valid if valid else 0.0,
        "mean_min_obstacle": (sum(minimum_obstacles) / valid
                              if valid else 0.0),
        "mean_final_pos_error": sum(final_errors) / valid if valid else 0.0,
        "reset_failures": reset_failures,
        "termination_reasons": reason_counts,
    }
    for action in range(env.n_actions):
        metrics["action_%d_rate" % action] = (
            action_counts[action] / total_actions if total_actions else 0.0)
    return metrics


def main():
    args = parse_args()
    random.seed(args.seed)
    torch.manual_seed(args.seed)

    decay_steps = args.epsilon_decay_steps
    if decay_steps == 0:
        expected_steps = min(args.max_steps, EXPECTED_EPISODE_STEPS)
        decay_steps = max(1, int(args.episodes * expected_steps * 0.5))

    print("[初始化] 创建 GazeboEnv ...")
    env = GazeboEnv()
    env.set_max_steps(args.max_steps)

    # 跨场景轮换：加载训练场景清单，启用单 run 内逐集切换 + 严格相位复位
    train_scenarios = []
    eval_scenarios = []
    if args.scenario_manifest:
        train_scenarios = load_split_scenarios(args.scenario_manifest, "train")
        eval_manifest = (args.eval_manifest or os.path.join(
            os.path.dirname(os.path.abspath(args.scenario_manifest)), "test.json"))
        eval_scenarios = load_split_scenarios(eval_manifest, args.eval_split)
        args.eval_manifest = eval_manifest
        env.set_scenario_rotation(True)
        env.set_phase_aligned(True)
        print("[初始化] 启用跨场景轮换 | train=%d | %s=%d | 模式=%s"
              % (len(train_scenarios), args.eval_split, len(eval_scenarios),
                 args.rotation_mode))

    agent = DQNAgent(env.observation_space_dim, env.n_actions,
                     lr=args.learning_rate, gamma=args.gamma,
                     hidden_dims=args.hidden_dims)
    buffer = PrioritizedReplayBuffer(BUFFER_CAPACITY, alpha=PER_ALPHA, beta=PER_BETA)
    resume_state = None
    if args.resume:
        resume_state = load_training_checkpoint(args.resume, agent, buffer)
        resume_type = "完整训练状态" if resume_state else "仅模型权重"
        print("[恢复] 已加载%s: %s" % (resume_type, args.resume))
        if resume_state:
            decay_steps = int(resume_state["epsilon_decay_steps"])
            if int(resume_state["progress"]["completed_episodes"]) > args.episodes:
                raise SystemExit(
                    "[错误] --episodes 必须不少于 checkpoint 已完成的 episode 数")

    run_dir = create_run_dir(args.run_dir)
    train_log_path = os.path.join(run_dir, "training_log.csv")
    eval_log_path = os.path.join(run_dir, "evaluation_log.csv")
    step_log_path = os.path.join(run_dir, "step_log.csv")
    reset_log_path = os.path.join(run_dir, "reset_failures.csv")
    best_model_path = os.path.join(run_dir, "best_model.pt")
    last_model_path = os.path.join(run_dir, "last_model.pt")
    training_state_path = os.path.join(run_dir, "training_state.pt")
    initialize_csv(train_log_path, TRAIN_FIELDS)
    initialize_csv(eval_log_path, EVAL_FIELDS)
    initialize_csv(step_log_path, STEP_FIELDS)
    initialize_csv(reset_log_path, RESET_FAILURE_FIELDS)

    config = {
        "command": sys.argv,
        "created_at": datetime.now().astimezone().isoformat(),
        "episodes": args.episodes,
        "max_steps": args.max_steps,
        "eval_interval": args.eval_interval,
        "eval_episodes": args.eval_episodes,
        "eval_window": args.eval_window,
        "use_lr_schedule": args.use_lr_schedule,
        "seed": args.seed,
        "batch_size": args.batch_size,
        "buffer_capacity": BUFFER_CAPACITY,
        "buffer_type": "prioritized",
        "per_alpha": PER_ALPHA,
        "per_beta": PER_BETA,
        "train_start_steps": args.train_start_steps,
        "target_update_steps": args.target_update_steps,
        "health_check_episodes": HEALTH_CHECK_EPISODES,
        "minimum_health_check_steps": MIN_HEALTH_CHECK_STEPS,
        "epsilon_start": EPSILON_START,
        "epsilon_end": EPSILON_END,
        "epsilon_decay_steps": decay_steps,
        "gamma": args.gamma,
        "n_step": args.n_step,
        "curriculum": args.curriculum,
        "scenario_manifest": args.scenario_manifest,
        "eval_manifest": args.eval_manifest,
        "eval_split": args.eval_split,
        "learning_rate": args.learning_rate,
        "hidden_dims": list(args.hidden_dims),
        "observation_dim": env.observation_space_dim,
        "observation_frame": "trajectory_motion",
        "reward_version": 8,
        "action_names": ACTION_NAMES,
        "goal_mode": env.goal_mode,
        "obstacles_a": list(env.obstacles_a),
        "obstacles_b": list(env.obstacles_b),
        "environment": environment_config(env),
        "resume_type": ("full" if resume_state else
                        "weights_only" if args.resume else None),
    }
    write_json(os.path.join(run_dir, "config.json"), config)

    total_steps = 0
    update_steps = 0
    completed_episodes = 0
    evaluation_index = 0
    last_evaluated_episode = 0
    consecutive_reset_failures = 0
    best_score = None
    best_model_state = None
    eval_history = []
    status = "running"

    if resume_state:
        progress = resume_state["progress"]
        total_steps = int(progress["total_steps"])
        update_steps = int(progress["update_steps"])
        completed_episodes = int(progress["completed_episodes"])
        evaluation_index = int(progress["evaluation_index"])
        last_evaluated_episode = int(progress["last_evaluated_episode"])
        eval_history = list(resume_state["eval_history"])
        best_score = resume_state["best_score"]
        best_model_state = resume_state.get("best_model_state")
        if best_model_state is not None:
            torch.save(best_model_state, best_model_path)

    # 场景轮换游标：与 episode 计数解耦，reset 失败换场景时会额外推进
    scenario_cursor = completed_episodes
    # 连续被跳过的回合数：按回合（而非每次尝试）计，避免少数坏场景集中出现时误判仿真不可用
    consecutive_reset_skips = 0

    print("=" * 72)
    print("开始训练 | episodes=%d | max_steps=%d | epsilon_decay_steps=%d" %
          (args.episodes, args.max_steps, decay_steps))
    print("产物目录: %s" % run_dir)
    print("=" * 72)

    try:
        # 用 while + completed_episodes 作循环条件：reset 失败被跳过时不推进计数，
        # 因此失败的 reset 不会占用 episode 预算、也不会占用 episode 序号。
        while completed_episodes < args.episodes:
            ep = completed_episodes
            if args.curriculum:
                fraction = float(ep) / max(1, args.episodes - 1)
                env.goal_min_distance = 3.0
                env.goal_max_distance = (7.0 if fraction < 0.33
                                         else 14.0 if fraction < 0.67
                                         else 0.0)
            # 跨场景轮换 + reset 重试：每次重试前推进场景游标换下一个场景，
            # reset 失败只记入 reset_failures.csv，不写入 training_log.csv。
            state = None
            for attempt in range(1, RESET_RETRY_LIMIT + 1):
                if train_scenarios:
                    pool = (curriculum_pool(train_scenarios, fraction)
                            if args.curriculum else train_scenarios)
                    if args.rotation_mode == "round":
                        idx = scenario_cursor % len(pool)
                        scenario_cursor += 1
                    else:
                        idx = random.randint(0, len(pool) - 1)
                    env.arm_deploy(pool[idx])
                state = env.reset()
                if env.reset_ok:
                    break
                consecutive_reset_failures += 1
                append_csv(reset_log_path, RESET_FAILURE_FIELDS, {
                    "episode": ep,
                    "attempt": attempt,
                    "scenario_id": env.scenario_id,
                    "reset_failure_reason": env.reset_failure_reason,
                    "start_dist": "%.3f" % env.start_dist,
                    "start_obstacle_min": "%.3f" % env.start_obstacle_min,
                    "total_steps": total_steps,
                })
                print("Episode %d | reset failed (%s) | attempt=%d/%d | "
                      "scenario=%s | start_obstacle_min=%.3f | consecutive=%d" %
                      (ep, env.reset_failure_reason, attempt,
                       RESET_RETRY_LIMIT, env.scenario_id,
                       env.start_obstacle_min, consecutive_reset_failures))
            if not env.reset_ok:
                # 换场景重试仍失败：跳过本回合。不推进 completed_episodes，
                # 也不终止 learner，下一轮会从新的场景游标位置继续尝试。
                print("Episode %d | reset 重试 %d 次仍失败，跳过该回合（不占用回合预算）" %
                      (ep, RESET_RETRY_LIMIT))
                consecutive_reset_skips += 1
                if consecutive_reset_skips >= MAX_CONSECUTIVE_RESET_FAILURES:
                    # 连续多个回合完全无法 reset，说明仿真/部署已不可用；
                    # 此时必须中止，否则 while 循环会无上界地重试下去。
                    status = "aborted_reset_failures"
                    print("[中止] 连续 %d 个回合 reset 失败（阈值 %d），停止训练" %
                          (consecutive_reset_skips,
                           MAX_CONSECUTIVE_RESET_FAILURES))
                    break
                continue

            consecutive_reset_failures = 0
            consecutive_reset_skips = 0
            episode_reward = 0.0
            losses = []
            final_info = {}
            episode_steps = 0
            action_counts = [0] * env.n_actions
            n_step = NStepAccumulator(args.n_step, args.gamma)

            for _ in range(env.episode_step_limit):
                epsilon = epsilon_linear_at_step(
                    EPSILON_START, EPSILON_END, decay_steps, total_steps)
                action = agent.select_action(
                    state, epsilon, allowed_actions=env.allowed_actions())
                next_state, reward, done, final_info = env.step(action)
                applied_action = final_info.get("applied_action", action)
                replay_action = action if applied_action is None else applied_action
                if applied_action is not None:
                    action_counts[applied_action] += 1
                priority_hint = (3.0 if final_info.get("collision_events", 0) > 0
                                 else 2.0 if final_info.get("avoidance_active", False)
                                 else 1.0)
                transition = (state, replay_action, reward, next_state, done,
                              env.allowed_actions(), priority_hint)
                for ready in n_step.append(transition):
                    buffer.push(*ready[:6], bootstrap_discount=ready[6],
                                priority_hint=ready[7])
                sectors = final_info.get(
                    "motion_sectors", [1.0] * env.num_sectors)
                step_row = {
                    "episode": ep,
                    "trial_id": final_info.get("trial_id", env.trial_id),
                    "scenario_id": final_info.get("scenario_id", env.scenario_id),
                    "trajectory_phase_s": "%.3f" % final_info.get(
                        "trajectory_phase_s", 0.0),
                    "tracking_rmse": format_metric(final_info.get("tracking_rmse")),
                    "tracking_max_error": format_metric(final_info.get("tracking_max_error")),
                    "tracking_steady_error": format_metric(final_info.get("tracking_steady_error")),
                    "recovery_time": format_metric(final_info.get("recovery_time")),
                    "metric_path_length": format_metric(
                        final_info.get("metric_path_length"), 0.0),
                    "episode_step": episode_steps + 1,
                    "total_step": total_steps + 1,
                    "selected_action": action,
                    "applied_action": applied_action,
                    "reward": "%.4f" % reward,
                    "done": int(done),
                    "done_reason": final_info.get("done_reason", "running"),
                    "pose_x": "%.6f" % final_info.get("pose_x", float("nan")),
                    "pose_y": "%.6f" % final_info.get("pose_y", float("nan")),
                    "pose_z": "%.6f" % final_info.get("pose_z", float("nan")),
                    "front_min": "%.4f" % final_info.get("front_min", 1.0),
                    "forward_obstacle_min": "%.4f" % final_info.get(
                        "forward_obstacle_min", 1.0),
                    "obstacle_min": "%.4f" % final_info.get("obstacle_min", 1.0),
                    "left_clearance": "%.4f" % final_info.get(
                        "left_clearance", 1.0),
                    "right_clearance": "%.4f" % final_info.get(
                        "right_clearance", 1.0),
                    "position_error": "%.6f" % final_info.get("position_error", 0.0),
                    "motion_heading": "%.4f" % final_info.get("motion_heading", 0.0),
                    "body_yaw": "%.4f" % final_info.get("body_yaw", 0.0),
                    "offset_x": "%.4f" % final_info.get("offset_x", 0.0),
                    "offset_y": "%.4f" % final_info.get("offset_y", 0.0),
                    "offset_z": "%.4f" % final_info.get("offset_z", 0.0),
                    "motion_sectors": json.dumps(
                        [round(float(value), 4) for value in sectors]),
                    "collision_events": final_info.get("collision_events", 0),
                    "geometry_clearance": "%.4f" % final_info.get(
                        "geometry_clearance", float("inf")),
                    "avoid_needed": int(final_info.get("avoid_needed", -1)),
                    "avoid_suppressed": int(final_info.get(
                        "avoid_suppressed", -1)),
                    "avoidance_quality": "%.4f" % final_info.get(
                        "avoidance_quality", 0.0),
                    "avoidance_reward_step": "%.4f" % final_info.get(
                        "avoidance_reward_step", 0.0),
                    "safe_success": int(bool(final_info.get("safe_success", False))),
                    "action_switch_rate": "%.4f" % final_info.get(
                        "action_switch_rate", 0.0),
                    "action_reversal_rate": "%.4f" % final_info.get(
                        "action_reversal_rate", 0.0),
                    "local_speed_scale": "%.2f" % final_info.get(
                        "local_speed_scale", 1.0),
                    "trajectory_paused": int(bool(final_info.get(
                        "trajectory_paused", False))),
                    "filter_reason": final_info.get("filter_reason", ""),
                    "action_reason": final_info.get("action_reason", ""),
                    "predicted_h": format_metric(final_info.get("predicted_h")),
                    "prediction_error": format_metric(final_info.get("prediction_error"), 0.0),
                    "filter_sensor_fresh": int(bool(final_info.get("filter_sensor_fresh", False))),
                    "safety_filter_mode": final_info.get("safety_filter_mode", "disabled"),
                    "filter_margin": format_metric(final_info.get("filter_margin"), 0.0),
                    "filter_safe_actions": json.dumps(list(final_info.get("filter_safe_actions", ()))),
                    "action_cost": format_metric(final_info.get("action_cost"), 0.0),
                }
                # 备份逃逸审计字段：env info 已计数，这里落盘以便从产物复查
                step_row["shield_escape"] = int(bool(
                    final_info.get("shield_escape", 0)))
                step_row["shield_escape_count"] = final_info.get(
                    "shield_escape_count", 0)
                step_row["escape_margin"] = format_metric(
                    final_info.get("escape_margin"))
                append_csv(step_log_path, STEP_FIELDS, step_row)

                if len(buffer) >= args.train_start_steps:
                    result = buffer.sample(args.batch_size)
                    if result is not None:
                        batch, indices, weights = result
                        loss, td_errors = agent.update(batch, weights)
                        update_steps += 1
                        if loss is not None:
                            losses.append(loss)
                            buffer.update_priorities(indices, td_errors)

                total_steps += 1
                if total_steps % args.target_update_steps == 0:
                    agent.sync_target()

                state = next_state
                episode_reward += reward
                episode_steps += 1
                if done:
                    break

            completed_episodes = ep + 1
            done_reason = final_info.get("done_reason", "train_cutoff")
            success = bool(final_info.get("episode_success", False))
            if success:
                result = "success"
            elif done_reason == "timeout":
                result = "timeout"
            else:
                result = "failed"
            epsilon = epsilon_linear_at_step(
                EPSILON_START, EPSILON_END, decay_steps, total_steps)
            avg_loss = sum(losses) / len(losses) if losses else 0.0

            append_csv(train_log_path, TRAIN_FIELDS, {
                "episode": ep,
                "trial_id": env.trial_id,
                "scenario_id": env.scenario_id,
                "trajectory_phase_s": "%.3f" % final_info.get(
                    "trajectory_phase_s", 0.0),
                "tracking_rmse": format_metric(final_info.get("tracking_rmse")),
                "tracking_max_error": format_metric(final_info.get("tracking_max_error")),
                "tracking_steady_error": format_metric(final_info.get("tracking_steady_error")),
                "recovery_time": format_metric(final_info.get("recovery_time")),
                "metric_path_length": format_metric(
                    final_info.get("metric_path_length"), 0.0),
                "total_reward": "%.2f" % episode_reward,
                "steps": episode_steps,
                "total_steps": total_steps,
                "epsilon": "%.4f" % epsilon,
                "avg_loss": "%.6f" % avg_loss,
                "result": result,
                "done_reason": done_reason,
                "final_front_min": "%.3f" % final_info.get("front_min", 1.0),
                "min_front_min": "%.3f" % final_info.get("min_front_min", 1.0),
                "final_obstacle_min": "%.3f" % final_info.get("obstacle_min", 1.0),
                "min_obstacle_min": "%.3f" % final_info.get("min_obstacle_min", 1.0),
                "final_pos_error": "%.3f" % final_info.get("position_error", 0.0),
                "reset_ok": 1,
                "reset_failure_reason": "",
                "start_dist": "%.3f" % env.start_dist,
                "start_obstacle_min": "%.3f" % env.start_obstacle_min,
                "obstacle_encountered": int(bool(
                    final_info.get("obstacle_encountered", False))),
                "avoidance_clear_count": final_info.get(
                    "avoidance_clear_count", 0),
                "avoidance_events": final_info.get(
                    "avoidance_event_count", 0),
                "avoidance_quality": "%.4f" % final_info.get(
                    "avoidance_quality", 0.0),
                "avoidance_reward": "%.4f" % final_info.get(
                    "avoidance_reward", 0.0),
                "goal_x": "%.2f" % final_info.get("goal_x", 0.0),
                "goal_y": "%.2f" % final_info.get("goal_y", 0.0),
                "path_length": "%.2f" % final_info.get("path_length", 0.0),
                "a_encounter_steps": final_info.get("a_encounter_steps", 0),
                "b_encounter_steps": final_info.get("b_encounter_steps", 0),
                "collision_events": final_info.get("collision_events", 0),
                "collision_events_a": final_info.get("collision_events_a", 0),
                "collision_events_b": final_info.get("collision_events_b", 0),
                "geometry_collision_events": final_info.get(
                    "geometry_collision_events", 0),
                "min_geometry_clearance": "%.3f" % final_info.get(
                    "min_geometry_clearance", float("inf")),
                "collision_events_per_meter": "%.4f" % final_info.get(
                    "collision_events_per_meter", 0.0),
                "travel_distance": "%.3f" % final_info.get(
                    "travel_distance", 0.0),
                "safe_success": int(bool(final_info.get("safe_success", False))),
                "action_switch_rate": "%.4f" % final_info.get(
                    "action_switch_rate", 0.0),
                "action_reversal_rate": "%.4f" % final_info.get(
                    "action_reversal_rate", 0.0),
                "action_0": action_counts[0],
                "action_1": action_counts[1],
                "action_2": action_counts[2],
                "action_3": action_counts[3],
                "action_4": action_counts[4],
                "action_5": action_counts[5],
            })
            dominant_action = max(range(env.n_actions), key=action_counts.__getitem__)
            print("Episode %d | reward=%+.2f | steps=%d | epsilon=%.3f | "
                  "loss=%.4f | result=%s | reason=%s | min_obstacle=%.3f | "
                  "dominant_action=%s" %
                  (ep, episode_reward, episode_steps, epsilon, avg_loss, result,
                   done_reason, final_info.get("min_obstacle_min", 1.0),
                   ACTION_NAMES[dominant_action]))

            if args.use_lr_schedule:
                agent.schedule_lr(ep, args.episodes)

            if (completed_episodes == HEALTH_CHECK_EPISODES and
                    total_steps < MIN_HEALTH_CHECK_STEPS):
                status = "aborted_invalid_environment"
                print("[中止] 前 %d 个 episode 仅产生 %d 步，环境在智能体学习前过早终止" %
                      (HEALTH_CHECK_EPISODES, total_steps))
                break

            if completed_episodes % args.eval_interval == 0:
                evaluation_index += 1
                metrics = evaluate(agent, env, args.eval_episodes, args.max_steps,
                                   eval_scenarios)
                eval_history.append(metrics)
                rolling = rolling_evaluation_metrics(eval_history, args.eval_window)
                row = {"evaluation": evaluation_index,
                       "after_episode": completed_episodes, **metrics,
                       **rolling}
                row["termination_reasons"] = json.dumps(
                    row["termination_reasons"], sort_keys=True)
                row["scenario_ids"] = json.dumps(row["scenario_ids"])
                append_csv(eval_log_path, EVAL_FIELDS, row)
                last_evaluated_episode = completed_episodes
                score = evaluation_score(rolling)
                new_best = update_best_score(best_score, score,
                                             metrics["valid_episodes"])
                if new_best is not best_score:
                    best_score = new_best
                    agent.save(best_model_path)
                    best_model_state = clone_model_state(agent)
                reason_text = " ".join(
                    "%s:%d" % (reason, count) for reason, count in
                    sorted(metrics["termination_reasons"].items()))
                print("[评估 %d] success=%.1f%% | collision=%.1f%% | timeout=%.1f%% | "
                      "reward=%+.2f | min_obstacle=%.3f | "
                      "rolling_success=%.1f%% (window=%d) | reasons {%s}" %
                      (evaluation_index, metrics["success_rate"] * 100.0,
                       metrics["collision_rate"] * 100.0,
                       metrics["timeout_rate"] * 100.0,
                       metrics["mean_reward"], metrics["mean_min_obstacle"],
                       rolling["rolling_success_rate"] * 100.0,
                       rolling["window_used"], reason_text))

        if status == "running":
            status = "completed" if update_steps > 0 else "completed_without_updates"
        if (status == "completed" and completed_episodes > 0 and
                last_evaluated_episode != completed_episodes):
            evaluation_index += 1
            metrics = evaluate(agent, env, args.eval_episodes, args.max_steps,
                               eval_scenarios)
            eval_history.append(metrics)
            rolling = rolling_evaluation_metrics(eval_history, args.eval_window)
            row = {"evaluation": evaluation_index,
                   "after_episode": completed_episodes, **metrics,
                   **rolling}
            row["termination_reasons"] = json.dumps(
                row["termination_reasons"], sort_keys=True)
            row["scenario_ids"] = json.dumps(row["scenario_ids"])
            append_csv(eval_log_path, EVAL_FIELDS, row)
            score = evaluation_score(rolling)
            new_best = update_best_score(best_score, score,
                                         metrics["valid_episodes"])
            if new_best is not best_score:
                best_score = new_best
                agent.save(best_model_path)
                best_model_state = clone_model_state(agent)
    except KeyboardInterrupt:
        status = "interrupted"
        print("\n[中断] 收到用户中断，正在保存本轮产物...")
    except Exception:
        status = "failed"
        raise
    finally:
        env.clear_local_offset()
        if status == "aborted_reset_failures":
            # Keep the simulator held after an invalid reset; releasing the
            # phase here would allow the stale controller state to drift while
            # no learner is consuming observations.
            env._publish_local_control(0.0, True, 1.0)
        if total_steps > 0:
            agent.save(last_model_path)
            save_training_checkpoint(
                training_state_path, agent, buffer, {
                    "completed_episodes": completed_episodes,
                    "total_steps": total_steps,
                    "update_steps": update_steps,
                    "evaluation_index": evaluation_index,
                    "last_evaluated_episode": last_evaluated_episode,
                }, eval_history, best_score, best_model_state, decay_steps)
        write_json(os.path.join(run_dir, "status.json"), {
            "status": status,
            "finished_at": datetime.now().astimezone().isoformat(),
            "completed_episodes": completed_episodes,
            "total_steps": total_steps,
            "update_steps": update_steps,
            "best_model_saved": os.path.exists(best_model_path),
            "last_model_saved": os.path.exists(last_model_path),
            "training_state_saved": os.path.exists(training_state_path),
        })

    print("=" * 72)
    print("训练状态: %s" % status)
    print("产物目录: %s" % run_dir)
    print("=" * 72)


if __name__ == "__main__":
    main()
