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
from replay_buffer import ReplayBuffer
from training_utils import epsilon_at_step


BATCH_SIZE = 32
BUFFER_CAPACITY = 50000
EPSILON_START = 1.0
EPSILON_END = 0.05
GAMMA = 0.99
LEARNING_RATE = 1e-3
TARGET_UPDATE_STEPS = 500
TRAIN_START_STEPS = 1000
MAX_CONSECUTIVE_RESET_FAILURES = 3
HEALTH_CHECK_EPISODES = 20
MIN_HEALTH_CHECK_STEPS = 100

TRAIN_FIELDS = [
    "episode", "total_reward", "steps", "total_steps", "epsilon",
    "avg_loss", "result", "done_reason", "final_front_min",
    "min_front_min", "final_obstacle_min", "min_obstacle_min",
    "final_pos_error", "reset_ok", "start_dist", "start_obstacle_min",
    "obstacle_encountered", "avoidance_clear_count",
    "action_0", "action_1", "action_2", "action_3", "action_4",
]
STEP_FIELDS = [
    "episode", "episode_step", "total_step", "selected_action",
    "applied_action", "reward", "done", "done_reason",
    "front_min", "forward_obstacle_min", "obstacle_min",
    "position_error",
    "motion_heading", "body_yaw", "offset_x", "offset_y",
    "offset_z", "motion_sectors",
]
EVAL_FIELDS = [
    "evaluation", "after_episode", "episodes", "valid_episodes",
    "success_rate", "collision_rate", "timeout_rate", "mean_reward", "mean_steps",
    "mean_min_obstacle", "mean_final_pos_error", "reset_failures",
    "action_0_rate", "action_1_rate", "action_2_rate", "action_3_rate",
    "action_4_rate",
]


def parse_args():
    parser = argparse.ArgumentParser(description="DQN 局部避障训练")
    parser.add_argument("--episodes", type=int, default=300,
                        help="训练 episode 数（默认: 300）")
    parser.add_argument("--max_steps", type=int, default=150,
                        help="每个 episode 最大步数（默认: 150）")
    parser.add_argument("--run-dir", default=None,
                        help="本轮产物目录；默认在 training_runs 下按时间创建")
    parser.add_argument("--eval-interval", type=int, default=20,
                        help="每隔多少个训练 episode 做一次贪心评估")
    parser.add_argument("--eval-episodes", type=int, default=5,
                        help="每次贪心评估的 episode 数")
    parser.add_argument("--epsilon-decay-steps", type=int, default=0,
                        help="epsilon 衰减步数；0 表示总预算的 80%%")
    parser.add_argument("--seed", type=int, default=0, help="随机种子")
    args = parser.parse_args()

    if args.episodes <= 0 or args.max_steps <= 0:
        parser.error("--episodes 和 --max_steps 必须为正数")
    if args.eval_interval <= 0 or args.eval_episodes <= 0:
        parser.error("评估间隔和评估 episode 数必须为正数")
    if args.epsilon_decay_steps < 0:
        parser.error("--epsilon-decay-steps 不能为负数")
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


def environment_config(env):
    names = [
        "trajectory_type", "step_z", "min_height", "min_offset_xy",
        "max_offset_xy", "offset_smooth_alpha", "max_offset_change",
        "num_sectors",
        "heading_min_speed",
        "lidar_max_range", "lidar_min_range", "safe_distance_norm",
        "emergency_dist_norm", "collision_dist", "safe_reward",
        "step_penalty", "action_change_penalty", "unnecessary_action_penalty",
        "avoidance_success_reward", "terminal_failure_penalty",
        "safe_proximity_penalty", "emergency_proximity_penalty",
        "avoidance_clear_steps",
        "avoidance_tracking_tolerance", "reset_min_clearance_norm",
        "tracking_error_weight",
        "max_tracking_error", "max_terminal_tracking_error",
        "reward_clip_abs", "reset_mode", "reset_tolerance",
        "moving_reset_tolerance", "reset_attempts", "reset_wait_timeout",
        "scenario_obstacles", "scenario_trajectory", "scenario_altitude",
    ]
    return {name: getattr(env, name) for name in names}


def evaluate(agent, env, episode_count, max_steps):
    rewards = []
    steps_per_episode = []
    minimum_obstacles = []
    final_errors = []
    successes = 0
    collisions = 0
    timeouts = 0
    reset_failures = 0
    action_counts = [0] * env.n_actions

    for _ in range(episode_count):
        state = env.reset()
        if not env.reset_ok:
            reset_failures += 1
            continue

        total_reward = 0.0
        final_info = {}
        episode_steps = 0
        for _ in range(max_steps):
            action = agent.select_action(
                state, epsilon=0.0, allowed_actions=env.allowed_actions())
            state, reward, done, final_info = env.step(action)
            action_counts[final_info.get("applied_action", action)] += 1
            total_reward += reward
            episode_steps += 1
            if done:
                break

        rewards.append(total_reward)
        steps_per_episode.append(episode_steps)
        minimum_obstacles.append(final_info.get("min_obstacle_min", 1.0))
        final_errors.append(final_info.get("position_error", 0.0))
        successes += int(bool(final_info.get("episode_success", False)))
        collisions += int(final_info.get("done_reason") == "collision")
        timeouts += int(final_info.get("done_reason") == "timeout")

    valid = len(rewards)
    denominator = float(episode_count)
    total_actions = float(sum(action_counts))
    metrics = {
        "episodes": episode_count,
        "valid_episodes": valid,
        "success_rate": successes / denominator,
        "collision_rate": collisions / denominator,
        "timeout_rate": timeouts / denominator,
        "mean_reward": sum(rewards) / valid if valid else 0.0,
        "mean_steps": sum(steps_per_episode) / valid if valid else 0.0,
        "mean_min_obstacle": (sum(minimum_obstacles) / valid
                              if valid else 0.0),
        "mean_final_pos_error": sum(final_errors) / valid if valid else 0.0,
        "reset_failures": reset_failures,
    }
    for action in range(env.n_actions):
        metrics["action_%d_rate" % action] = (
            action_counts[action] / total_actions if total_actions else 0.0)
    return metrics


def evaluation_score(metrics):
    return (metrics["success_rate"], -metrics["collision_rate"],
            metrics["mean_reward"])


def main():
    args = parse_args()
    random.seed(args.seed)
    torch.manual_seed(args.seed)

    decay_steps = args.epsilon_decay_steps
    if decay_steps == 0:
        decay_steps = max(1, int(args.episodes * args.max_steps * 0.8))

    print("[初始化] 创建 GazeboEnv ...")
    env = GazeboEnv()
    env.max_episode_steps = args.max_steps
    agent = DQNAgent(env.observation_space_dim, env.n_actions,
                     lr=LEARNING_RATE, gamma=GAMMA)
    buffer = ReplayBuffer(BUFFER_CAPACITY)

    run_dir = create_run_dir(args.run_dir)
    train_log_path = os.path.join(run_dir, "training_log.csv")
    eval_log_path = os.path.join(run_dir, "evaluation_log.csv")
    step_log_path = os.path.join(run_dir, "step_log.csv")
    best_model_path = os.path.join(run_dir, "best_model.pt")
    last_model_path = os.path.join(run_dir, "last_model.pt")
    initialize_csv(train_log_path, TRAIN_FIELDS)
    initialize_csv(eval_log_path, EVAL_FIELDS)
    initialize_csv(step_log_path, STEP_FIELDS)

    config = {
        "command": sys.argv,
        "created_at": datetime.now().astimezone().isoformat(),
        "episodes": args.episodes,
        "max_steps": args.max_steps,
        "eval_interval": args.eval_interval,
        "eval_episodes": args.eval_episodes,
        "seed": args.seed,
        "batch_size": BATCH_SIZE,
        "buffer_capacity": BUFFER_CAPACITY,
        "train_start_steps": TRAIN_START_STEPS,
        "target_update_steps": TARGET_UPDATE_STEPS,
        "health_check_episodes": HEALTH_CHECK_EPISODES,
        "minimum_health_check_steps": MIN_HEALTH_CHECK_STEPS,
        "epsilon_start": EPSILON_START,
        "epsilon_end": EPSILON_END,
        "epsilon_decay_steps": decay_steps,
        "gamma": GAMMA,
        "learning_rate": LEARNING_RATE,
        "observation_dim": env.observation_space_dim,
        "observation_frame": "trajectory_motion",
        "reward_version": 2,
        "action_names": ACTION_NAMES,
        "environment": environment_config(env),
    }
    write_json(os.path.join(run_dir, "config.json"), config)

    total_steps = 0
    update_steps = 0
    completed_episodes = 0
    evaluation_index = 0
    last_evaluated_episode = 0
    consecutive_reset_failures = 0
    best_score = None
    status = "running"

    print("=" * 72)
    print("开始训练 | episodes=%d | max_steps=%d | epsilon_decay_steps=%d" %
          (args.episodes, args.max_steps, decay_steps))
    print("产物目录: %s" % run_dir)
    print("=" * 72)

    try:
        for ep in range(args.episodes):
            state = env.reset()
            if not env.reset_ok:
                consecutive_reset_failures += 1
                append_csv(train_log_path, TRAIN_FIELDS, {
                    "episode": ep,
                    "total_reward": "0.00",
                    "steps": 0,
                    "total_steps": total_steps,
                    "epsilon": "%.4f" % epsilon_at_step(
                        EPSILON_START, EPSILON_END, decay_steps, total_steps),
                    "avg_loss": "0.000000",
                    "result": "failed",
                    "done_reason": "reset_failed",
                    "final_front_min": "1.000",
                    "min_front_min": "1.000",
                    "final_obstacle_min": "1.000",
                    "min_obstacle_min": "1.000",
                    "final_pos_error": "0.000",
                    "reset_ok": 0,
                    "start_dist": "%.3f" % env.start_dist,
                    "start_obstacle_min": "%.3f" % env.start_obstacle_min,
                    "obstacle_encountered": 0,
                    "avoidance_clear_count": 0,
                    "action_0": 0,
                    "action_1": 0,
                    "action_2": 0,
                    "action_3": 0,
                    "action_4": 0,
                })
                print("Episode %d | reset failed | consecutive=%d" %
                      (ep, consecutive_reset_failures))
                if consecutive_reset_failures >= MAX_CONSECUTIVE_RESET_FAILURES:
                    status = "aborted_reset_failures"
                    break
                continue

            consecutive_reset_failures = 0
            episode_reward = 0.0
            losses = []
            final_info = {}
            episode_steps = 0
            action_counts = [0] * env.n_actions

            for _ in range(args.max_steps):
                epsilon = epsilon_at_step(
                    EPSILON_START, EPSILON_END, decay_steps, total_steps)
                action = agent.select_action(
                    state, epsilon, allowed_actions=env.allowed_actions())
                next_state, reward, done, final_info = env.step(action)
                applied_action = final_info.get("applied_action", action)
                action_counts[applied_action] += 1
                buffer.push(state, applied_action, reward, next_state, done,
                            env.allowed_actions())
                sectors = final_info.get(
                    "motion_sectors", [1.0] * env.num_sectors)
                step_row = {
                    "episode": ep,
                    "episode_step": episode_steps + 1,
                    "total_step": total_steps + 1,
                    "selected_action": action,
                    "applied_action": applied_action,
                    "reward": "%.4f" % reward,
                    "done": int(done),
                    "done_reason": final_info.get("done_reason", "running"),
                    "front_min": "%.4f" % final_info.get("front_min", 1.0),
                    "forward_obstacle_min": "%.4f" % final_info.get(
                        "forward_obstacle_min", 1.0),
                    "obstacle_min": "%.4f" % final_info.get("obstacle_min", 1.0),
                    "position_error": "%.4f" % final_info.get("position_error", 0.0),
                    "motion_heading": "%.4f" % final_info.get("motion_heading", 0.0),
                    "body_yaw": "%.4f" % final_info.get("body_yaw", 0.0),
                    "offset_x": "%.4f" % final_info.get("offset_x", 0.0),
                    "offset_y": "%.4f" % final_info.get("offset_y", 0.0),
                    "offset_z": "%.4f" % final_info.get("offset_z", 0.0),
                    "motion_sectors": json.dumps(
                        [round(float(value), 4) for value in sectors]),
                }
                append_csv(step_log_path, STEP_FIELDS, step_row)

                if len(buffer) >= TRAIN_START_STEPS:
                    batch = buffer.sample(BATCH_SIZE)
                    loss = agent.update(batch)
                    update_steps += 1
                    if loss is not None:
                        losses.append(loss)

                total_steps += 1
                if total_steps % TARGET_UPDATE_STEPS == 0:
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
                result = "unsafe_timeout"
            else:
                result = "failed"
            epsilon = epsilon_at_step(
                EPSILON_START, EPSILON_END, decay_steps, total_steps)
            avg_loss = sum(losses) / len(losses) if losses else 0.0

            append_csv(train_log_path, TRAIN_FIELDS, {
                "episode": ep,
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
                "start_dist": "%.3f" % env.start_dist,
                "start_obstacle_min": "%.3f" % env.start_obstacle_min,
                "obstacle_encountered": int(bool(
                    final_info.get("obstacle_encountered", False))),
                "avoidance_clear_count": final_info.get(
                    "avoidance_clear_count", 0),
                "action_0": action_counts[0],
                "action_1": action_counts[1],
                "action_2": action_counts[2],
                "action_3": action_counts[3],
                "action_4": action_counts[4],
            })
            dominant_action = max(range(env.n_actions), key=action_counts.__getitem__)
            print("Episode %d | reward=%+.2f | steps=%d | epsilon=%.3f | "
                  "loss=%.4f | result=%s | reason=%s | min_obstacle=%.3f | "
                  "dominant_action=%s" %
                  (ep, episode_reward, episode_steps, epsilon, avg_loss, result,
                   done_reason, final_info.get("min_obstacle_min", 1.0),
                   ACTION_NAMES[dominant_action]))

            if (completed_episodes == HEALTH_CHECK_EPISODES and
                    total_steps < MIN_HEALTH_CHECK_STEPS):
                status = "aborted_invalid_environment"
                print("[中止] 前 %d 个 episode 仅产生 %d 步，环境在智能体学习前过早终止" %
                      (HEALTH_CHECK_EPISODES, total_steps))
                break

            if completed_episodes % args.eval_interval == 0:
                evaluation_index += 1
                metrics = evaluate(agent, env, args.eval_episodes, args.max_steps)
                row = {"evaluation": evaluation_index,
                       "after_episode": completed_episodes, **metrics}
                append_csv(eval_log_path, EVAL_FIELDS, row)
                last_evaluated_episode = completed_episodes
                score = evaluation_score(metrics)
                if metrics["valid_episodes"] > 0 and (
                        best_score is None or score > best_score):
                    best_score = score
                    agent.save(best_model_path)
                print("[评估 %d] success=%.1f%% | collision=%.1f%% | timeout=%.1f%% | "
                      "reward=%+.2f | min_obstacle=%.3f" %
                      (evaluation_index, metrics["success_rate"] * 100.0,
                       metrics["collision_rate"] * 100.0,
                       metrics["timeout_rate"] * 100.0,
                       metrics["mean_reward"], metrics["mean_min_obstacle"]))

        if status == "running":
            status = "completed" if update_steps > 0 else "completed_without_updates"
        if (status == "completed" and completed_episodes > 0 and
                last_evaluated_episode != completed_episodes):
            evaluation_index += 1
            metrics = evaluate(agent, env, args.eval_episodes, args.max_steps)
            row = {"evaluation": evaluation_index,
                   "after_episode": completed_episodes, **metrics}
            append_csv(eval_log_path, EVAL_FIELDS, row)
            score = evaluation_score(metrics)
            if metrics["valid_episodes"] > 0 and (
                    best_score is None or score > best_score):
                best_score = score
                agent.save(best_model_path)
    except KeyboardInterrupt:
        status = "interrupted"
        print("\n[中断] 收到用户中断，正在保存本轮产物...")
    except Exception:
        status = "failed"
        raise
    finally:
        env.clear_local_offset()
        if total_steps > 0:
            agent.save(last_model_path)
        write_json(os.path.join(run_dir, "status.json"), {
            "status": status,
            "finished_at": datetime.now().astimezone().isoformat(),
            "completed_episodes": completed_episodes,
            "total_steps": total_steps,
            "update_steps": update_steps,
            "best_model_saved": os.path.exists(best_model_path),
            "last_model_saved": os.path.exists(last_model_path),
        })

    print("=" * 72)
    print("训练状态: %s" % status)
    print("产物目录: %s" % run_dir)
    print("=" * 72)


if __name__ == "__main__":
    main()
