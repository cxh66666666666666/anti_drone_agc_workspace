#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""N5 地面车 RL 单体：多布局短训 DQN + 按布局隔离的 val/test 评估。

在 N5 场景集（多个命名障碍布局，按布局隔离 train/val/test）之上：
  1) 启动一次地面单体仿真（scripts/start_ground_sim.sh，见 part_ground 约定）；
  2) 在 train 布局（field_a/field_b）上短训地面 DQN；
  3) 在 validation（field_c）与 test（未见布局 field_d/field_e）上贪心评估，
     落盘整回合结果、失败分母、逐帧轨迹与**逐布局分解**。
  4) 结束后停止仿真。

关键：障碍布局通过 `n5_ground_scenarios.apply_scenario` 注入 `env.poles`
（虚拟激光解析），无需切换 world 文件。

用法:
  python3 scripts/n5_ground_train_eval.py --episodes 300 --max-steps 150 \
      --run-dir training_runs/car_n5 --out-dir experiments/n5/ground --seed 101
"""

import argparse
import hashlib
import json
import os
import random
import signal
import subprocess
import sys
import time
from datetime import datetime

import torch

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WS_SCRIPTS = os.path.join(WS, "src/rl_ground_avoidance/scripts")
DRONE_SCRIPTS = os.path.join(WS, "src/rl_obstacle_avoidance/scripts")
sys.path.insert(0, WS_SCRIPTS)
sys.path.insert(0, DRONE_SCRIPTS)

from dqn_agent import DQNAgent
from replay_buffer import PrioritizedReplayBuffer
from training_utils import epsilon_linear_at_step

from ground_env import GroundEnv
from n5_ground_scenarios import (LAYOUTS, LAYOUT_SPLIT, apply_scenario,
                                 assert_layout_isolation, build_splits,
                                 split_sha256)

BUFFER_CAPACITY = 100000
EPS_START, EPS_END = 1.0, 0.05
PER_ALPHA, PER_BETA = 0.6, 0.4

TRAJ_FIELDS = (
    "episode_step", "pose_x", "pose_y", "position_error", "selected_action",
    "applied_action", "offset_x", "offset_y", "obstacle_min",
    "forward_obstacle_min", "done_reason",
)


def _env():
    env = dict(os.environ)
    env["ROS_LOG_DIR"] = "/tmp/ros_logs"
    env["PYTHONUNBUFFERED"] = "1"
    env.setdefault("DISPLAY", ":0")
    return env


def wait_service(name, max_wait=120):
    """等待地面传送服务就绪。"""
    deadline = time.time() + max_wait
    while time.time() < deadline:
        out = subprocess.run(
            ["bash", "-c", "source /opt/ros/noetic/setup.bash && "
             "source devel/setup.bash && rosservice list"],
            cwd=WS, env=_env(), capture_output=True, text=True).stdout
        if name in out:
            return True
        time.sleep(2)
    return False


def start_sim(gui, log_path):
    """启动一次地面仿真（与 run_m7_single_agent.part_ground 一致）。"""
    cmd = ["bash", "scripts/start_ground_sim.sh",
           "gui:=%s" % ("true" if gui else "false")]
    print("[N5] 启动仿真: %s" % " ".join(cmd), flush=True)
    log = open(log_path, "w", encoding="utf-8")
    proc = subprocess.Popen(cmd, cwd=WS, env=_env(), stdout=log,
                            stderr=subprocess.STDOUT, start_new_session=True)
    return proc, log


def stop_sim(proc, log):
    """干净停止地面仿真进程。"""
    print("[N5] 停止仿真", flush=True)
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
    except ProcessLookupError:
        pass
    time.sleep(5)
    subprocess.run(["bash", "-c",
                    "pkill -9 -f ground_gazebo; pkill -9 -f ground_perception; "
                    "pkill -9 -f gzclient; pkill -9 -f rosmaster; "
                    "pkill -9 -f rosout"], cwd=WS, env=_env())
    if log is not None:
        log.close()


def greedy_eval(env, agent, scenarios, max_steps, out_dir, split):
    """固定场景贪心评估，导出整回合结果、逐帧轨迹与逐布局分解。"""
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
    # 逐布局分解
    layout_stats = {}

    def _ls(layout):
        return layout_stats.setdefault(layout, {
            "episodes": 0, "failure_denominator": 0, "success": 0,
            "collision": 0, "timeout": 0})

    with open(ep_path, "w", encoding="utf-8") as stream:
        for idx, scn in enumerate(scenarios):
            apply_scenario(env, scn)  # 注入初始位姿 + 障碍布局
            sid = scn["scenario_id"]
            layout = scn["layout"]
            _ls(layout)["episodes"] += 1
            _ls(layout)["failure_denominator"] += 1
            t0 = time.time()
            state = env.reset()
            if not env.reset_ok:
                reset_failures += 1
                stream.write(json.dumps({
                    "episode_index": idx, "scenario_id": sid, "layout": layout,
                    "reset_ok": False, "done_reason": "reset_failure",
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
                    row = {"episode_index": idx, "scenario_id": sid,
                           "layout": layout}
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
            _ls(layout)["success"] += int(success)
            _ls(layout)["collision"] += int(reason == "collision")
            _ls(layout)["timeout"] += int(reason == "timeout")
            tracking_errors.append(sum_err / steps if steps else 0.0)
            min_obstacles.append(float(final_info.get("min_obstacle_min", 1.0)))
            stream.write(json.dumps({
                "episode_index": idx, "scenario_id": sid, "layout": layout,
                "reset_ok": True, "steps": steps, "done_reason": reason,
                "episode_success": success,
                "mean_tracking_error": round(sum_err / steps if steps else 0.0, 4),
                "min_obstacle_min": final_info.get("min_obstacle_min"),
                "action_counts": {str(a): ep_actions.count(a)
                                  for a in sorted(set(ep_actions))},
                "wall_s": round(time.time() - t0, 2),
                "trajectory_file": os.path.relpath(traj_path, out_dir),
            }, ensure_ascii=False) + "\n")

    denom = float(len(scenarios))
    valid = len(tracking_errors)
    # 逐布局成功率/碰撞率
    for layout, st in layout_stats.items():
        d = float(st["failure_denominator"]) or 1.0
        st["success_rate"] = st["success"] / d
        st["collision_rate"] = st["collision"] / d
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
        "layout_breakdown": layout_stats,
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
    layouts_seen = set()
    for ep in range(args.episodes):
        scn = scenarios[ep % len(scenarios)]
        apply_scenario(env, scn)  # 训练布局轮换注入
        layouts_seen.add(scn["layout"])
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
            "update_steps": update_steps, "best_model": best_model_path,
            "train_layouts": sorted(layouts_seen)}


def parse_args():
    p = argparse.ArgumentParser(description="N5 地面车多布局 RL 短训+评估")
    p.add_argument("--episodes", type=int, default=300)
    p.add_argument("--max-steps", type=int, default=150)
    p.add_argument("--batch-size", type=int, default=64)
    p.add_argument("--target-update-steps", type=int, default=500)
    p.add_argument("--train-start-steps", type=int, default=1000)
    p.add_argument("--learning-rate", type=float, default=1e-4)
    p.add_argument("--gamma", type=float, default=0.99)
    p.add_argument("--hidden-dims", default="128,128")
    p.add_argument("--seed", type=int, default=101)
    p.add_argument("--run-dir", default="training_runs/car_n5")
    p.add_argument("--out-dir", default="experiments/n5/ground")
    p.add_argument("--eval-only", action="store_true",
                   help="跳过训练，仅评估已有/指定模型")
    p.add_argument("--model", default="", help="eval-only 时加载的模型路径")
    p.add_argument("--eval-splits", default="train,validation,test",
                   help="eval-only 时评估的 split（逗号分隔）")
    p.add_argument("--tag", default="", help="eval-only 输出子目录后缀")
    gui = p.add_mutually_exclusive_group()
    gui.add_argument("--gui", action="store_true", help="启动 Gazebo GUI")
    gui.add_argument("--headless", action="store_true",
                     help="无头（默认；保留为显式写法）")
    return p.parse_args()


def _sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as stream:
        for chunk in iter(lambda: stream.read(65536), b""):
            h.update(chunk)
    return h.hexdigest()


def _eval_only(env, agent, splits, args, run_dir, out_dir):
    """跳过训练，评估指定/已有模型（用于诊断与对照）。"""
    model_path = args.model or os.path.join(run_dir, "best_model.pt")
    agent.load(model_path)
    sub = os.path.join(out_dir, args.tag) if args.tag else out_dir
    os.makedirs(sub, exist_ok=True)
    results = {}
    for split in [s for s in args.eval_splits.split(",") if s]:
        if split not in splits:
            continue
        metrics = greedy_eval(env, agent, splits[split], args.max_steps,
                              os.path.join(sub, split), split)
        results[split] = metrics
        print("[N5-eval] %-11s success=%.3f collision=%.3f timeout=%.3f" % (
            split, metrics["success_rate"], metrics["collision_rate"],
            metrics["timeout_rate"]), flush=True)
    out = {"created_at": datetime.now().isoformat(), "eval_only": True,
           "model": model_path, "model_sha256": _sha256(model_path),
           "tag": args.tag, "scenario_split_sha256": split_sha256(),
           "layout_split": {k: LAYOUT_SPLIT[k] for k in LAYOUTS},
           "results": results}
    path = os.path.join(sub, "n5_ground_evalonly.json")
    with open(path, "w", encoding="utf-8") as stream:
        json.dump(out, stream, ensure_ascii=False, indent=2)
        stream.write("\n")
    print("[N5-eval] 报告: %s" % path, flush=True)


def main():
    args = parse_args()
    random.seed(args.seed)
    torch.manual_seed(args.seed)
    args.hidden_dims = [int(x) for x in args.hidden_dims.split(",") if x]
    splits = build_splits()
    assert_layout_isolation(splits)
    out_dir = os.path.join(WS, args.out_dir)
    run_dir = os.path.join(WS, args.run_dir)
    args.out_dir, args.run_dir = out_dir, run_dir
    os.makedirs(out_dir, exist_ok=True)
    print("[N5] 地面布局: %s | train=%d val=%d test=%d sha=%s"
          % (LAYOUT_SPLIT, len(splits["train"]), len(splits["validation"]),
             len(splits["test"]), split_sha256()[:12]))

    sim = log = None
    try:
        sim, log = start_sim(args.gui, os.path.join(out_dir, "ground_sim.log"))
        if not wait_service("/ground_gazebo/set_model_state"):
            print("[N5] 地面仿真未就绪，退出", flush=True)
            return 1

        env = GroundEnv()
        env.max_episode_steps = args.max_steps
        agent = DQNAgent(env.observation_space_dim, env.n_actions,
                         lr=args.learning_rate, gamma=args.gamma,
                         hidden_dims=args.hidden_dims)
        if args.eval_only:
            _eval_only(env, agent, splits, args, run_dir, out_dir)
            return 0
        buffer = PrioritizedReplayBuffer(BUFFER_CAPACITY, alpha=PER_ALPHA,
                                         beta=PER_BETA)

        train_summary = train(env, agent, buffer, splits["train"], args)
        agent.load(os.path.join(run_dir, "best_model.pt"))
        val_metrics = greedy_eval(env, agent, splits["validation"],
                                  args.max_steps,
                                  os.path.join(out_dir, "validation"),
                                  "validation")
        test_metrics = greedy_eval(env, agent, splits["test"], args.max_steps,
                                   os.path.join(out_dir, "test"), "test")
    finally:
        if sim is not None:
            stop_sim(sim, log)

    report = {
        "created_at": datetime.now().isoformat(), "seed": args.seed,
        "episodes": args.episodes, "max_steps": args.max_steps,
        "scenario_split_sha256": split_sha256(),
        "layout_split": {k: LAYOUT_SPLIT[k] for k in LAYOUTS},
        "layouts_poles": {k: LAYOUTS[k] for k in LAYOUTS},
        "scenario_splits": {k: [s["scenario_id"] for s in v]
                            for k, v in splits.items()},
        "train": train_summary,
        "validation": val_metrics,
        "test": test_metrics,
        "notes": [
            "test 布局 %s 在训练/验证中均未出现（按布局隔离）"
            % sorted({s["layout"] for s in splits["test"]}),
            "障碍布局经 apply_scenario 注入 env.poles（虚拟激光解析），"
            "不依赖 world 文件切换；worlds/n5/*.world 仅作可视化核对。",
            "失败分母 = 该 split 全部场景数（含 reset 失败）。",
        ],
    }
    report_path = os.path.join(out_dir, "n5_ground_report.json")
    with open(report_path, "w", encoding="utf-8") as stream:
        json.dump(report, stream, ensure_ascii=False, indent=2)
        stream.write("\n")
    print("[N5] val: success=%.3f collision=%.3f timeout=%.3f mean_err=%.3f"
          % (val_metrics["success_rate"], val_metrics["collision_rate"],
             val_metrics["timeout_rate"], val_metrics["mean_tracking_error"]))
    print("[N5] test(未见布局): success=%.3f collision=%.3f timeout=%.3f"
          % (test_metrics["success_rate"], test_metrics["collision_rate"],
             test_metrics["timeout_rate"]))
    print("[N5] 报告: %s" % report_path)
    return 0


if __name__ == "__main__":
    sys.exit(main())