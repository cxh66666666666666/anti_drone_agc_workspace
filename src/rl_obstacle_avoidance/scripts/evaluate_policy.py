#!/usr/bin/env python3
"""加载已训练模型，在 Gazebo 中运行评估（不训练）。"""

import argparse
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from dqn_agent import DQNAgent
from gazebo_env import ACTION_NAMES, GazeboEnv
from training_utils import count_termination_reason, parse_hidden_dims


def evaluate(agent, env, episode_count, max_steps):
    rewards = []
    steps_per_episode = []
    minimum_obstacles = []
    final_errors = []
    b_encounter_steps = []
    successes = 0
    safe_successes = 0
    collisions = 0
    risk_entries = 0
    timeouts = 0
    shield_infeasible = 0
    reset_failures = 0
    trajectory_complete = 0
    action_counts = [0] * env.n_actions
    reason_counts = {}

    for ep in range(episode_count):
        state = env.reset()
        if not env.reset_ok:
            reset_failures += 1
            print(f"  [跳过] episode {ep + 1}: reset 失败")
            continue

        total_reward = 0.0
        final_info = {}
        episode_steps = 0
        # Match the training/evaluation helper: active avoidance gets the
        # finite completion grace period before the episode is cut off.
        for _ in range(env.episode_step_limit):
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
        b_encounter_steps.append(final_info.get("b_encounter_steps", 0))
        successes += int(bool(final_info.get("episode_success", False)))
        safe_successes += int(bool(final_info.get("safe_success", False)))
        collisions += int(final_info.get("done_reason") == "collision")
        risk_entries += int(final_info.get("collision_events", 0) > 0)
        timeouts += int(final_info.get("done_reason") == "timeout")
        shield_infeasible += int(
            final_info.get("done_reason") == "shield_infeasible")
        trajectory_complete += int(
            final_info.get("done_reason") == "trajectory_complete")

        reason = final_info.get("done_reason", "?")
        count_termination_reason(reason_counts, reason)
        print(f"  episode {ep + 1}: reward={total_reward:+.2f}  steps={episode_steps}  "
              f"reason={reason}  min_obstacle={final_info.get('min_obstacle_min', 1.0):.3f}")

    valid = len(rewards)
    denominator = float(episode_count)
    total_actions = float(sum(action_counts))
    return {
        "episodes": episode_count,
        "valid_episodes": valid,
        "success_rate": successes / denominator,
        "safe_success_rate": safe_successes / denominator,
        "collision_rate": collisions / denominator,
        "risk_entry_rate": risk_entries / denominator,
        "timeout_rate": timeouts / denominator,
        "shield_infeasible_rate": shield_infeasible / denominator,
        "reset_failure_rate": reset_failures / denominator,
        "trajectory_complete_rate": trajectory_complete / denominator,
        "mean_reward": sum(rewards) / valid if valid else 0.0,
        "mean_steps": sum(steps_per_episode) / valid if valid else 0.0,
        "mean_min_obstacle": (sum(minimum_obstacles) / valid if valid else 0.0),
        "mean_final_pos_error": sum(final_errors) / valid if valid else 0.0,
        "mean_b_encounter_steps": (sum(b_encounter_steps) / valid
                                   if valid else 0.0),
        "reset_failures": reset_failures,
        "termination_reasons": reason_counts,
        "action_rates": {
            a: action_counts[a] / total_actions if total_actions else 0.0
            for a in range(env.n_actions)
        },
    }


def parse_args():
    parser = argparse.ArgumentParser(description="加载模型并评估")
    parser.add_argument("model", help="模型 .pt 文件路径")
    parser.add_argument("--episodes", type=int, default=50,
                        help="评估 episode 数（默认: 50）")
    parser.add_argument("--max_steps", type=int, default=450,
                        help="每个 episode 最大步数（默认: 450）")
    parser.add_argument("--hidden-dims", default="128,128",
                        help="模型隐藏层维度，逗号分隔（默认: 128,128）")
    args = parser.parse_args()

    if args.episodes <= 0 or args.max_steps <= 0:
        parser.error("--episodes 和 --max_steps 必须为正数")
    try:
        args.hidden_dims = parse_hidden_dims(args.hidden_dims)
    except ValueError as exc:
        parser.error(str(exc))
    return args


def main():
    args = parse_args()

    print("[初始化] 创建 GazeboEnv ...")
    env = GazeboEnv()
    env.set_max_steps(args.max_steps)

    agent = DQNAgent(env.observation_space_dim, env.n_actions,
                     hidden_dims=args.hidden_dims)
    agent.load(args.model)
    print(f"[加载] 模型: {args.model}")

    print("=" * 60)
    print(f"开始评估 | episodes={args.episodes} | max_steps={args.max_steps}")
    print("=" * 60)

    metrics = evaluate(agent, env, args.episodes, args.max_steps)

    print("=" * 60)
    print("评估结果:")
    print(f"  成功率:      {metrics['success_rate'] * 100:.1f}%")
    print(f"  安全成功率:  {metrics['safe_success_rate'] * 100:.1f}%")
    print(f"  碰撞率:      {metrics['collision_rate'] * 100:.1f}%")
    print(f"  风险区进入率: {metrics['risk_entry_rate'] * 100:.1f}%")
    print(f"  超时率:      {metrics['timeout_rate'] * 100:.1f}%")
    print(f"  平均奖励:    {metrics['mean_reward']:+.2f}")
    print(f"  平均步数:    {metrics['mean_steps']:.1f}")
    print(f"  平均最小障碍: {metrics['mean_min_obstacle']:.3f}")
    print(f"  平均跟踪误差: {metrics['mean_final_pos_error']:.3f}")
    print(f"  平均 b 柱遭遇步数: {metrics['mean_b_encounter_steps']:.1f}")
    print(f"  有效 episode: {metrics['valid_episodes']}/{metrics['episodes']}")
    if metrics["valid_episodes"] < metrics["episodes"]:
        print(f"  [警告] 有效 episode 数小于请求数："
              f"{metrics['episodes'] - metrics['valid_episodes']} 个 episode "
              f"因 reset 失败被跳过，未计入成功。")
    print("  终止原因:")
    for reason, count in sorted(metrics["termination_reasons"].items()):
        print(f"    {reason}: {count}")
    print("  动作分布:")
    for a in range(env.n_actions):
        rate = metrics["action_rates"][a]
        print(f"    {ACTION_NAMES[a]}: {rate * 100:.1f}%")
    print("=" * 60)

    env.clear_local_offset()


if __name__ == "__main__":
    main()
