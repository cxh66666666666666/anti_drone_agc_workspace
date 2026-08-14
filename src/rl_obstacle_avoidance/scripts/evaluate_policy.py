#!/usr/bin/env python3
"""加载已训练模型，在 Gazebo 中运行评估（不训练）。"""

import argparse
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from dqn_agent import DQNAgent
from gazebo_env import ACTION_NAMES, GazeboEnv


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

    for ep in range(episode_count):
        state = env.reset()
        if not env.reset_ok:
            reset_failures += 1
            print(f"  [跳过] episode {ep + 1}: reset 失败")
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

        reason = final_info.get("done_reason", "?")
        print(f"  episode {ep + 1}: reward={total_reward:+.2f}  steps={episode_steps}  "
              f"reason={reason}  min_obstacle={final_info.get('min_obstacle_min', 1.0):.3f}")

    valid = len(rewards)
    denominator = float(episode_count)
    total_actions = float(sum(action_counts))
    return {
        "episodes": episode_count,
        "valid_episodes": valid,
        "success_rate": successes / denominator,
        "collision_rate": collisions / denominator,
        "timeout_rate": timeouts / denominator,
        "mean_reward": sum(rewards) / valid if valid else 0.0,
        "mean_steps": sum(steps_per_episode) / valid if valid else 0.0,
        "mean_min_obstacle": (sum(minimum_obstacles) / valid if valid else 0.0),
        "mean_final_pos_error": sum(final_errors) / valid if valid else 0.0,
        "reset_failures": reset_failures,
        "action_rates": {
            a: action_counts[a] / total_actions if total_actions else 0.0
            for a in range(env.n_actions)
        },
    }


def parse_args():
    parser = argparse.ArgumentParser(description="加载模型并评估")
    parser.add_argument("model", help="模型 .pt 文件路径")
    parser.add_argument("--episodes", type=int, default=10,
                        help="评估 episode 数（默认: 10）")
    parser.add_argument("--max_steps", type=int, default=150,
                        help="每个 episode 最大步数（默认: 150）")
    return parser.parse_args()


def main():
    args = parse_args()

    print("[初始化] 创建 GazeboEnv ...")
    env = GazeboEnv()
    env.max_episode_steps = args.max_steps

    agent = DQNAgent(env.observation_space_dim, env.n_actions)
    agent.load(args.model)
    print(f"[加载] 模型: {args.model}")

    print("=" * 60)
    print(f"开始评估 | episodes={args.episodes} | max_steps={args.max_steps}")
    print("=" * 60)

    metrics = evaluate(agent, env, args.episodes, args.max_steps)

    print("=" * 60)
    print("评估结果:")
    print(f"  成功率:      {metrics['success_rate'] * 100:.1f}%")
    print(f"  碰撞率:      {metrics['collision_rate'] * 100:.1f}%")
    print(f"  超时率:      {metrics['timeout_rate'] * 100:.1f}%")
    print(f"  平均奖励:    {metrics['mean_reward']:+.2f}")
    print(f"  平均步数:    {metrics['mean_steps']:.1f}")
    print(f"  平均最小障碍: {metrics['mean_min_obstacle']:.3f}")
    print(f"  平均跟踪误差: {metrics['mean_final_pos_error']:.3f}")
    print(f"  有效 episode: {metrics['valid_episodes']}/{metrics['episodes']}")
    print("  动作分布:")
    for a in range(env.n_actions):
        rate = metrics["action_rates"][a]
        print(f"    {ACTION_NAMES[a]}: {rate * 100:.1f}%")
    print("=" * 60)

    env.clear_local_offset()


if __name__ == "__main__":
    main()