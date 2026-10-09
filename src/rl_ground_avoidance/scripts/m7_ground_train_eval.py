#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""M7 地面车 RL 单体：短训 DQN + 固定 val/test 评估（离线实验工具，Python）。

在已启动的地面单体仿真（`scripts/start_ground_sim.sh`）之上：
  1) 在固定 train 场景（初始位姿/参考速度）上短训地面 DQN；
  2) 在固定 validation / test 场景上贪心评估，落盘整回合结果、失败分母、轨迹。

用法:
  python3 src/rl_ground_avoidance/scripts/m7_ground_train_eval.py \
      --episodes 200 --out-dir experiments/m7/ground --run-dir training_runs/car_m7
"""

import argparse
import json
import os
import random
import sys
import time
from datetime import datetime

import torch

WS = "/home/ubuntu/anti_drone_agc_workspace"
WS_SCRIPTS = os.path.join(WS, "src/rl_ground_avoidance/scripts")
DRONE_SCRIPTS = os.path.join(WS, "src/rl_obstacle_avoidance/scripts")
sys.path.insert(0, WS_SCRIPTS)
sys.path.insert(0, DRONE_SCRIPTS)

from dqn_agent import DQNAgent
from replay_buffer import PrioritizedReplayBuffer
from training_utils import epsilon_linear_at_step

from ground_env import GroundEnv
from m7_ground_scenarios import build_splits, apply_scenario, split_sha256

BUFFER_CAPACITY = 100000
EPS_START, EPS_END = 1.0, 0.05
PER_ALPHA, PER_BETA = 0.6, 0.4

TRAJ_FIELDS = (
    "episode_step", "pose_x", "pose_y", "position_error", "selected_action",
    "applied_action", "offset_x", "offset_y", "obstacle_min",
    "forward_obstacle_min", "done_reason",
)


def greedy_eval(env, agent, scenarios, max_steps, out_dir, split):
    """固定场景贪心评估，导出整回合结果与逐帧轨迹。"""
    os.makedirs(out_dir, exist_ok=True)
    traj_dir = os.path.join(out_dir, "trajectories")
    os.makedirs(traj_dir, exist_ok=True)
    ep_path = os.path.join(out_dir, "episodes.jsonl")
    env.max_episode_steps = max_steps

    action_counts = {i: 0 for i in range(env.n_actions)}
    actions_total = 0
    reasons = {}
    successes = collisions = timeouts = reset_failures = 0
    tracking_errors = []
    min_obstacles = []

    with open(ep_path, "w", encoding="utf-8") as stream:
        for idx, scn in enumerate(scenarios):
            apply_scenario(env, scn)
            sid = scn["scenario_id"]
            t0 = time.time()
            state = env.reset()
            if not env.reset_ok:
                reset_failures += 1
                stream.write(json.dumps({
                    "episode_index": idx, "scenario_id": sid, "reset_ok": False,
                    "done_reason": "reset_failure",
                    "episode_success": False}, ensure_ascii=False) + "\n")
                continue
            traj_path = os.path.join(traj_dir, "ep%02d_%s.jsonl" % (idx, sid))
            final_info = {}
            steps = 0
            ep_actions = []
            sum_err = 0.0
            with open(traj_path, "w", encoding="utf-8") as traj:
                for _ in range(max_steps):
                    action = agent.select_action(
                        state, epsilon=0.0,
                        allowed_actions=env.allowed_actions())
                    state, _reward, done, final_info = env.step(action)
                    applied = final_info.get("applied_action", action)
                    action_counts[applied] = action_counts.get(applied, 0) + 1
                    actions_total += 1
                    ep_actions.append(applied)
                    sum_err += float(final_info.get("position_error", 0.0) or 0.0)
                    row = {"episode_index": idx, "scenario_id": sid}
                    for field in TRAJ_FIELDS:
                        row[field] = final_info.get(field)
                    traj.write(json.dumps(row) + "\n")
                    steps += 1
                    if done:
                        break
            reason = final_info.get("done_reason", "?")
            reasons[reason] = reasons.get(reason, 0) + 1
            success = bool(final_info.get("episode_success", False))
            successes += int(success)
            collisions += int(reason == "collision")
            timeouts += int(reason == "timeout")
            tracking_errors.append(sum_err / steps if steps else 0.0)
            min_obstacles.append(float(final_info.get("min_obstacle_min", 1.0)))
            stream.write(json.dumps({
                "episode_index": idx, "scenario_id": sid, "reset_ok": True,
                "steps": steps, "done_reason": reason, "episode_success": success,
                "mean_tracking_error": round(sum_err / steps if steps else 0.0, 4),
                "min_obstacle_min": final_info.get("min_obstacle_min"),
                "action_counts": {str(a): ep_actions.count(a)
                                  for a in sorted(set(ep_actions))},
                "wall_s": round(time.time() - t0, 2),
                "trajectory_file": os.path.relpath(traj_path, out_dir),
            }, ensure_ascii=False) + "\n")

    denom = float(len(scenarios))
    valid = len(tracking_errors)
    metrics = {
        "split": split, "episodes": len(scenarios), "valid_episodes": valid,
        "failure_denominator": denom,
        "success_rate": successes / denom,
        "collision_rate": collisions / denom,
        "timeout_rate": timeouts / denom,
        "reset_failure_rate": reset_failures / denom,
        "mean_tracking_error": sum(tracking_errors) / valid if valid else 0.0,
        "mean_min_obstacle": sum(min_obstacles) / valid if valid else 0.0,
        "termination_reasons": reasons,
        "action_rates": {str(k): (action_counts[k] / actions_total
                                  if actions_total else 0.0)
                         for k in sorted(action_counts)},
    }
    with open(os.path.join(out_dir, "metrics.json"), "w",
              encoding="utf-8") as stream:
        json.dump(metrics, stream, ensure_ascii=False, indent=2)
        stream.write("\n")
    return metrics


def train(env, agent, buffer, scenarios, args):
    decay = max(1, int(args.episodes * args.max_steps * 0.5))
    total_steps = 0
    update_steps = 0
    best_score = None
    best_model_path = os.path.join(args.run_dir, "best_model.pt")
    os.makedirs(args.run_dir, exist_ok=True)
    for ep in range(args.episodes):
        apply_scenario(env, scenarios[ep % len(scenarios)])
        state = env.reset()
        if not env.reset_ok:
            continue
        done = False
        final_info = {}
        steps = 0
        ep_reward = 0.0
        for _ in range(args.max_steps):
            eps = epsilon_linear_at_step(EPS_START, EPS_END, decay, total_steps)
            action = agent.select_action(state, eps,
                                         allowed_actions=env.allowed_actions())
            next_state, reward, done, final_info = env.step(action)
            ep_reward += float(reward)
            applied = final_info.get("applied_action", action)
            buffer.push(state, applied, reward, next_state, done,
                        env.allowed_actions())
            if len(buffer) >= args.train_start_steps:
                result = buffer.sample(args.batch_size)
                if result is not None:
                    batch, indices, weights = result
                    # DQNAgent.update 返回 (loss, td_errors)
                    output = agent.update(batch, weights)
                    update_steps += 1
                    if output is not None:
                        _, td_errors = output
                        buffer.update_priorities(indices, td_errors)
            total_steps += 1
            if total_steps % args.target_update_steps == 0:
                agent.sync_target()
            state = next_state
            steps += 1
            if done:
                break
        score = ep_reward
        if best_score is None or score > best_score:
            best_score = score
            agent.save(best_model_path)
        if (ep + 1) % 50 == 0:
            print("  [训练] ep%03d reason=%s steps=%d total_steps=%d reward=%.2f"
                  % (ep + 1, final_info.get("done_reason"), steps, total_steps,
                     ep_reward), flush=True)
    agent.save(os.path.join(args.run_dir, "last_model.pt"))
    return {"episodes": args.episodes, "total_steps": total_steps,
            "update_steps": update_steps,
            "best_model": best_model_path}


def parse_args():
    p = argparse.ArgumentParser(description="M7 地面车 RL 短训+评估")
    p.add_argument("--episodes", type=int, default=200)
    p.add_argument("--max-steps", type=int, default=150)
    p.add_argument("--batch-size", type=int, default=64)
    p.add_argument("--target-update-steps", type=int, default=500)
    p.add_argument("--train-start-steps", type=int, default=1000)
    p.add_argument("--learning-rate", type=float, default=1e-4)
    p.add_argument("--gamma", type=float, default=0.99)
    p.add_argument("--hidden-dims", default="128,128")
    p.add_argument("--seed", type=int, default=101)
    p.add_argument("--run-dir", required=True)
    p.add_argument("--out-dir", required=True)
    return p.parse_args()


def main():
    args = parse_args()
    random.seed(args.seed)
    torch.manual_seed(args.seed)
    args.hidden_dims = [int(x) for x in args.hidden_dims.split(",") if x]
    splits = build_splits()
    print("[M7] 地面场景: train=%d val=%d test=%d sha=%s"
          % (len(splits["train"]), len(splits["validation"]),
             len(splits["test"]), split_sha256()[:12]))

    env = GroundEnv()
    env.max_episode_steps = args.max_steps
    agent = DQNAgent(env.observation_space_dim, env.n_actions,
                     lr=args.learning_rate, gamma=args.gamma,
                     hidden_dims=args.hidden_dims)
    buffer = PrioritizedReplayBuffer(BUFFER_CAPACITY, alpha=PER_ALPHA,
                                     beta=PER_BETA)

    train_summary = train(env, agent, buffer, splits["train"], args)

    agent.load(os.path.join(args.run_dir, "best_model.pt"))
    val_metrics = greedy_eval(env, agent, splits["validation"], args.max_steps,
                              os.path.join(args.out_dir, "validation"),
                              "validation")
    test_metrics = greedy_eval(env, agent, splits["test"], args.max_steps,
                               os.path.join(args.out_dir, "test"), "test")

    os.makedirs(args.out_dir, exist_ok=True)
    report = {
        "created_at": datetime.now().isoformat(), "seed": args.seed,
        "scenario_split_sha256": split_sha256(),
        "scenario_splits": {k: [s["scenario_id"] for s in v]
                            for k, v in splits.items()},
        "train": train_summary, "validation": val_metrics, "test": test_metrics,
    }
    with open(os.path.join(args.out_dir, "ground_report.json"), "w",
              encoding="utf-8") as stream:
        json.dump(report, stream, ensure_ascii=False, indent=2)
        stream.write("\n")
    print("[M7] val: success=%.3f collision=%.3f timeout=%.3f mean_err=%.3f"
          % (val_metrics["success_rate"], val_metrics["collision_rate"],
             val_metrics["timeout_rate"], val_metrics["mean_tracking_error"]))
    print("[M7] test: success=%.3f collision=%.3f timeout=%.3f mean_err=%.3f"
          % (test_metrics["success_rate"], test_metrics["collision_rate"],
             test_metrics["timeout_rate"], test_metrics["mean_tracking_error"]))


if __name__ == "__main__":
    main()