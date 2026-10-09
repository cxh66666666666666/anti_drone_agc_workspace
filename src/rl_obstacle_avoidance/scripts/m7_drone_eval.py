#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""M7 无人机 RL 单体评估器（离线实验工具，非实时链路）。

在已启动的无人机单体仿真（`scripts/start_sim.sh lqr --training`）之上，
按固定 train/val/test 场景清单逐回合评估已训练 DQN 策略，落盘：
  - 逐回合整回合结果（终态、失败原因、失败分母口径）
  - 逐帧轨迹（位姿、跟踪误差、动作、净空、竖直偏移）
  - 汇总指标（跟踪误差、安全触发、失败率、三维动作可用性）

用法:
  python3 src/rl_obstacle_avoidance/scripts/m7_drone_eval.py \
      --model training_runs/dqn_v5_raw_s101/best_model.pt \
      --manifest experiments/stage2_v4/manifests/test.json --split test \
      --out-dir experiments/m7/drone_test
"""

import argparse
import json
import os
import random
import sys
import time
from datetime import datetime

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from dqn_agent import DQNAgent
from gazebo_env import GazeboEnv
from train_dqn import load_split_scenarios

# 竖直动作（含上升分量），用于三维逃逸可用性统计
VERTICAL_ACTIONS = (3, 4, 5)
# "不破坏基础跟踪"判据：跟踪误差峰值不超过环境发散阈值，且终态未发散
TRACK_DIVERGE_M = 10.0
BAD_TRACKING_REASONS = ("diverged", "low_altitude")
# 轨迹导出保留字段（控制落盘体积）
TRAJ_FIELDS = (
    "episode_step", "trajectory_phase_s", "pose_x", "pose_y", "pose_z",
    "position_error", "selected_action", "applied_action", "offset_x",
    "offset_y", "offset_z", "min_obstacle_min", "geometry_clearance",
    "front_min", "collision_events", "avoidance_active",
)


def _sha256(path):
    import hashlib
    h = hashlib.sha256()
    with open(path, "rb") as stream:
        for chunk in iter(lambda: stream.read(65536), b""):
            h.update(chunk)
    return h.hexdigest()


def run_eval(env, agent, scenarios, max_steps, out_dir):
    os.makedirs(out_dir, exist_ok=True)
    traj_dir = os.path.join(out_dir, "trajectories")
    os.makedirs(traj_dir, exist_ok=True)
    episodes_path = os.path.join(out_dir, "episodes.jsonl")
    env.set_max_steps(max_steps)

    episode_records = []
    action_counts = {i: 0 for i in range(env.n_actions)}
    vertical_actions_total = 0
    actions_total = 0
    reasons = {}
    successes = collisions = timeouts = shield_infeasible = reset_failures = 0
    tracking_errors = []
    vertical_episodes = 0
    vertical_tracking_ok = 0

    with open(episodes_path, "w", encoding="utf-8") as ep_stream:
        for idx, scenario in enumerate(scenarios):
            sid = str(scenario.get("scenario_id", "unknown"))
            env.arm_deploy(scenario)
            reset_episode_start = time.time()
            state = env.reset()
            if not env.reset_ok:
                reset_failures += 1
                rec = {"episode_index": idx, "scenario_id": sid,
                       "reset_ok": False,
                       "reset_failure_reason": getattr(
                           env, "reset_failure_reason", ""),
                       "done_reason": "reset_failure", "episode_success": False}
                ep_stream.write(json.dumps(rec, ensure_ascii=False) + "\n")
                episode_records.append(rec)
                print("  [跳过] ep%02d %s: reset 失败" % (idx + 1, sid))
                continue

            traj_path = os.path.join(traj_dir, "ep%02d_%s.jsonl" % (idx, sid))
            total_reward = 0.0
            steps = 0
            final_info = {}
            ep_vertical = 0
            ep_actions = []
            max_track_err = 0.0
            sum_track_err = 0.0
            with open(traj_path, "w", encoding="utf-8") as traj_stream:
                for _ in range(env.episode_step_limit):
                    action = agent.select_action(
                        state, epsilon=0.0,
                        allowed_actions=env.allowed_actions())
                    state, reward, done, final_info = env.step(action)
                    applied = final_info.get("applied_action", action)
                    if applied is not None:
                        action_counts[applied] = action_counts.get(applied, 0) + 1
                        actions_total += 1
                        ep_actions.append(applied)
                        if applied in VERTICAL_ACTIONS:
                            ep_vertical += 1
                            vertical_actions_total += 1
                    pe = float(final_info.get("position_error", 0.0) or 0.0)
                    sum_track_err += pe
                    max_track_err = max(max_track_err, pe)
                    row = {"episode_index": idx, "scenario_id": sid}
                    for field in TRAJ_FIELDS:
                        row[field] = final_info.get(field)
                    traj_stream.write(json.dumps(row) + "\n")
                    total_reward += reward
                    steps += 1
                    if done:
                        break

            reason = final_info.get("done_reason", "?")
            reasons[reason] = reasons.get(reason, 0) + 1
            success = bool(final_info.get("episode_success", False))
            successes += int(success)
            collisions += int(reason == "collision")
            timeouts += int(reason == "timeout")
            shield_infeasible += int(reason == "shield_infeasible")
            mean_track_err = sum_track_err / steps if steps else 0.0
            tracking_errors.append(mean_track_err)
            # 三维逃逸：该回合触发过竖直动作，且跟踪误差保持在容差内
            ep_vertical_ratio = ep_vertical / steps if steps else 0.0
            if ep_vertical > 0:
                vertical_episodes += 1
                if (reason not in BAD_TRACKING_REASONS
                        and max_track_err <= TRACK_DIVERGE_M):
                    vertical_tracking_ok += 1

            rec = {
                "episode_index": idx, "scenario_id": sid, "reset_ok": True,
                "steps": steps, "done_reason": reason,
                "episode_success": success,
                "total_reward": round(total_reward, 4),
                "mean_tracking_error": round(mean_track_err, 4),
                "max_tracking_error": round(max_track_err, 4),
                "min_obstacle_min": final_info.get("min_obstacle_min"),
                "min_geometry_clearance": final_info.get(
                    "min_geometry_clearance"),
                "collision_events": final_info.get("collision_events"),
                "vertical_action_count": ep_vertical,
                "vertical_action_ratio": round(ep_vertical_ratio, 4),
                "wall_s": round(time.time() - reset_episode_start, 2),
                "trajectory_file": os.path.relpath(traj_path, out_dir),
            }
            ep_stream.write(json.dumps(rec, ensure_ascii=False) + "\n")
            episode_records.append(rec)
            print("  ep%02d %s: reason=%s steps=%d mean_err=%.3f "
                  "vert=%d min_obs=%.3f wall=%.1fs"
                  % (idx + 1, sid, reason, steps, mean_track_err, ep_vertical,
                     float(final_info.get("min_obstacle_min") or 1.0),
                     rec["wall_s"]))

    denom = float(len(scenarios))
    valid = len(tracking_errors)
    metrics = {
        "episodes": len(scenarios),
        "valid_episodes": valid,
        "failure_denominator": denom,  # 整回合分母，含 reset 失败
        "success_rate": successes / denom,
        "collision_rate": collisions / denom,
        "timeout_rate": timeouts / denom,
        "shield_infeasible_rate": shield_infeasible / denom,
        "reset_failure_rate": reset_failures / denom,
        "mean_tracking_error": (sum(tracking_errors) / valid
                                if valid else 0.0),
        "termination_reasons": reasons,
        "action_counts": {str(k): v for k, v in sorted(action_counts.items())},
        "action_rates": {
            str(k): (action_counts[k] / actions_total if actions_total else 0.0)
            for k in sorted(action_counts)},
        "vertical_action_rate": (vertical_actions_total / actions_total
                                 if actions_total else 0.0),
        "vertical_episodes": vertical_episodes,
        "vertical_episodes_tracking_within_tolerance": vertical_tracking_ok,
        "vertical_escape_available": vertical_actions_total > 0,
        "vertical_escape_preserves_tracking": (
            vertical_episodes > 0 and vertical_tracking_ok == vertical_episodes),
    }
    with open(os.path.join(out_dir, "metrics.json"), "w",
              encoding="utf-8") as stream:
        json.dump(metrics, stream, ensure_ascii=False, indent=2)
        stream.write("\n")
    return metrics, episode_records


def parse_args():
    parser = argparse.ArgumentParser(description="M7 无人机 RL 单体评估")
    parser.add_argument("--model", required=True, help="DQN 模型 .pt 路径")
    parser.add_argument("--manifest", required=True, help="场景 manifest JSON")
    parser.add_argument("--split", default="test")
    parser.add_argument("--limit", type=int, default=0,
                        help="每 split 评估场景数上限（0=全部）")
    parser.add_argument("--max-steps", type=int, default=450)
    parser.add_argument("--hidden-dims", default="128,128")
    parser.add_argument("--out-dir", required=True)
    parser.add_argument("--seed", type=int, default=101)
    return parser.parse_args()


def main():
    args = parse_args()
    random.seed(args.seed)
    os.makedirs(args.out_dir, exist_ok=True)
    hidden = [int(x) for x in args.hidden_dims.split(",") if x]

    scenarios = load_split_scenarios(args.manifest, args.split)
    if args.limit > 0:
        scenarios = scenarios[:args.limit]
    print("[M7] split=%s 场景数=%d" % (args.split, len(scenarios)))

    env = GazeboEnv()
    env.set_max_steps(args.max_steps)
    env.set_scenario_rotation(True)   # 跨场景动态部署（与训练一致）
    env.set_phase_aligned(True)
    agent = DQNAgent(env.observation_space_dim, env.n_actions,
                     hidden_dims=hidden)
    agent.load(args.model)
    print("[M7] 模型=%s obs_dim=%d n_actions=%d"
          % (args.model, env.observation_space_dim, env.n_actions))

    metrics, _episode_records = run_eval(env, agent, scenarios, args.max_steps,
                                         args.out_dir)
    meta = {
        "created_at": datetime.now().isoformat(),
        "model": os.path.abspath(args.model),
        "model_sha256": _sha256(args.model),
        "manifest": os.path.abspath(args.manifest),
        "manifest_sha256": _sha256(args.manifest),
        "split": args.split, "seed": args.seed, "max_steps": args.max_steps,
        "scenario_ids": [str(s.get("scenario_id")) for s in scenarios],
        "vertical_actions": list(VERTICAL_ACTIONS),
    }
    with open(os.path.join(args.out_dir, "run_meta.json"), "w",
              encoding="utf-8") as stream:
        json.dump(meta, stream, ensure_ascii=False, indent=2)
        stream.write("\n")
    env.clear_local_offset()
    print("[M7] 关键指标: 成功率=%.3f 碰撞=%.3f 超时=%.3f 三维动作率=%.4f "
          "竖直回合=%d 竖直保跟踪=%d"
          % (metrics["success_rate"], metrics["collision_rate"],
             metrics["timeout_rate"], metrics["vertical_action_rate"],
             metrics["vertical_episodes"],
             metrics["vertical_episodes_tracking_within_tolerance"]))


if __name__ == "__main__":
    main()