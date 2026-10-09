#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""阶段3 D2：地面 DQN 未见布局独立复评（臂 A 复用 + 臂 B 重训）。

在 n5「按布局隔离」场景集之上做两臂独立复评，结果落
`experiments/d/d2_ground_reeval.json`：

  臂 A（reuse）：复用既有 best_model.pt，在 val 布局(field_c) 与 test 布局
      (field_d/field_e，训练未见) 上，以扩大样本量（建议 val>=20 / test>=30）、
      每布局多场景变体、随机化起始位姿/朝向、独立评估种子做贪心评估。
  臂 B（retrain）：在 train 布局(field_a/field_b) 上以更大预算（建议 episodes>=800，
      其余超参与 n5 脚本一致）重训，再用与臂 A **完全相同的独立 test 集**评估。

两臂都计算：成功率点估计 + Wilson 95% 区间；失败模式占比（collision/timeout/
reset_failure/other）+ Wilson 区间；泛化 gap = train 布局成功率 − test 布局成功率。
逐回合保留原始索引（scenario_id/episode/seed/done_reason/run 目录）以便审计。

容忍性：无仿真时不得崩溃——若 ground 服务不可用则打印清晰错误并非零退出。
本脚本会自行启动一次地面仿真；离线规划用 `--plan-only`（不依赖 Gazebo），
Wilson 对拍用 `--self-test`。

用法：
  python3 scripts/d2_ground_reeval.py --mode both --episodes 800 \
      --val-n 20 --test-n 30 --max-steps 150 --seed 101 \
      --out experiments/d/d2_ground_reeval.json
"""

import argparse
import glob
import hashlib
import json
import os
import random
import signal
import subprocess
import sys
import time
from datetime import datetime

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WS_SCRIPTS = os.path.join(WS, "src/rl_ground_avoidance/scripts")
DRONE_SCRIPTS = os.path.join(WS, "src/rl_obstacle_avoidance/scripts")
sys.path.insert(0, WS_SCRIPTS)
sys.path.insert(0, DRONE_SCRIPTS)

# 场景集为纯数据（无 ROS 依赖），可安全顶层导入
from n5_ground_scenarios import (LAYOUTS, LAYOUT_SPLIT, apply_scenario,
                                 assert_layout_isolation, build_splits,
                                 split_sha256)
# train() 用的 epsilon 线性衰减（与 n5 脚本一致，模块级导入避免作用域缺失）
from training_utils import epsilon_linear_at_step

# 与 n5 脚本一致的超参
BUFFER_CAPACITY = 100000
EPS_START, EPS_END = 1.0, 0.05
PER_ALPHA, PER_BETA = 0.6, 0.4
DEFAULT_MODEL = os.path.join(WS, "training_runs/car_n5/best_model.pt")

# 失败模式：success 之外的全部终局归入以下类别，分母均为该 split 全场景数
FAILURE_MODES = ("collision", "timeout", "reset_failure", "other")

TRAJ_FIELDS = (
    "episode_step", "pose_x", "pose_y", "position_error", "selected_action",
    "applied_action", "offset_x", "offset_y", "obstacle_min",
    "forward_obstacle_min", "done_reason",
)


# ---------------------------------------------------------------- 统计原语
# 口径与 src/air_ground_core/scripts/d2_stat_report.py 完全一致
Z95 = 1.959963984540054


def wilson(k, n, z=Z95):
    """Wilson score 95% 置信区间；n<=0 返回 (None, None)。"""
    if n <= 0:
        return (None, None)
    p = k / n
    denom = 1.0 + z * z / n
    center = (p + z * z / (2.0 * n)) / denom
    half = z * ((p * (1.0 - p) / n + z * z / (4.0 * n * n)) ** 0.5) / denom
    return (max(0.0, center - half), min(1.0, center + half))


def rate_block(k, n):
    """点估计 + Wilson 95% 区间 + n（与 d2_stat_report.rate_block 同口径）。"""
    lo, hi = wilson(k, n)
    return {"n": n, "count": k, "point": (k / n) if n > 0 else None,
            "wilson95": [lo, hi]}


def _sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as stream:
        for chunk in iter(lambda: stream.read(65536), b""):
            h.update(chunk)
    return h.hexdigest()


# ---------------------------------------------------------------- 场景规划
def generate_eval_scenarios(layouts, n_per_layout, seed, prefix):
    """为每个布局生成 n_per_layout 个独立场景变体（随机化起始位姿/朝向）。

    起始 x∈[-3.5,-1.5]（首个杆位 x>=5，远离障碍，保证 reset 净空），
    y∈[-1.2,1.2]，yaw∈[-0.15,0.15]。随机种子由调用方传入（独立评估种子）。
    """
    rng = random.Random(seed)
    out = []
    for layout in layouts:
        for i in range(n_per_layout):
            out.append({
                "scenario_id": "%s-%s-%02d" % (prefix, layout, i),
                "layout": layout,
                "poles": [list(p) for p in LAYOUTS[layout]],
                "start_x": round(rng.uniform(-3.5, -1.5), 4),
                "start_y": round(rng.uniform(-1.2, 1.2), 4),
                "start_yaw": round(rng.uniform(-0.15, 0.15), 4),
                "reference_speed": 1.0,
                "eval_seed": seed + i,
            })
    return out


def scenario_set_sha256(scenarios):
    """扩展场景集的稳定指纹（含布局与起始位姿）。"""
    blob = json.dumps(scenarios, sort_keys=True, separators=(",", ":"))
    return hashlib.sha256(blob.encode("utf-8")).hexdigest()


def assert_expanded_isolation(sets):
    """校验扩展后的 train/val/test 布局仍互不相交（训练-测试按布局隔离）。"""
    tr = {s["layout"] for s in sets["train"]}
    va = {s["layout"] for s in sets["validation"]}
    te = {s["layout"] for s in sets["test"]}
    assert not (te & tr), "test 与 train 布局重叠: %s" % (te & tr)
    assert not (te & va), "test 与 validation 布局重叠: %s" % (te & va)
    assert not (tr & va), "train 与 validation 布局重叠: %s" % (tr & va)
    return True


def build_plan(args):
    """构造扩展评估场景集（train/validation/test），并做隔离断言。"""
    base = build_splits()
    assert_layout_isolation(base)  # 原始 n5 场景集隔离

    def layouts_of(name):
        return sorted({s["layout"] for s in base[name]})

    eval_seed = args.seed + 1000  # 与训练随机性解耦的独立评估种子
    sets = {
        "train": generate_eval_scenarios(layouts_of("train"), args.train_n,
                                         eval_seed, "d2tr"),
        "validation": generate_eval_scenarios(layouts_of("validation"),
                                              args.val_n, eval_seed + 1, "d2v"),
        "test": generate_eval_scenarios(layouts_of("test"), args.test_n,
                                        eval_seed + 2, "d2t"),
    }
    assert_expanded_isolation(sets)  # 扩展集隔离断言
    return base, sets, eval_seed


def plan_summary(base, sets, eval_seed):
    """场景规划摘要（离线可打印，不依赖 Gazebo）。"""
    return {
        "eval_seed": eval_seed,
        "n5_scenario_split_sha256": split_sha256(base),
        "layout_split": {k: LAYOUT_SPLIT[k] for k in LAYOUTS},
        "sets": {
            name: {
                "layouts": sorted({s["layout"] for s in scns}),
                "episodes": len(scns),
                "sha256": scenario_set_sha256(scns),
            }
            for name, scns in sets.items()
        },
    }


# ---------------------------------------------------------------- 仿真生命周期
def _env_vars():
    env = dict(os.environ)
    env["ROS_LOG_DIR"] = "/tmp/ros_logs"
    env["PYTHONUNBUFFERED"] = "1"
    env.setdefault("DISPLAY", ":0")
    return env


def wait_service(name, max_wait=120):
    """等待地面传送服务就绪；超时返回 False（不抛异常）。"""
    deadline = time.time() + max_wait
    while time.time() < deadline:
        out = subprocess.run(
            ["bash", "-c", "source /opt/ros/noetic/setup.bash && "
             "source devel/setup.bash && rosservice list"],
            cwd=WS, env=_env_vars(), capture_output=True, text=True).stdout
        if name in out:
            return True
        time.sleep(2)
    return False


def start_sim(gui, log_path):
    """启动一次地面仿真（与 n5 脚本一致）。"""
    cmd = ["bash", "scripts/start_ground_sim.sh",
           "gui:=%s" % ("true" if gui else "false")]
    print("[D2] 启动地面仿真: %s" % " ".join(cmd), flush=True)
    log = open(log_path, "w", encoding="utf-8")
    proc = subprocess.Popen(cmd, cwd=WS, env=_env_vars(), stdout=log,
                            stderr=subprocess.STDOUT, start_new_session=True)
    return proc, log


def stop_sim(proc, log):
    """干净停止地面仿真进程（明示会清理 ground 相关进程与 rosmaster）。"""
    print("[D2] 停止地面仿真", flush=True)
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
    except ProcessLookupError:
        pass
    time.sleep(5)
    subprocess.run(["bash", "-c",
                    "pkill -9 -f ground_gazebo; pkill -9 -f ground_perception; "
                    "pkill -9 -f gzclient; pkill -9 -f rosmaster; "
                    "pkill -9 -f rosout"], cwd=WS, env=_env_vars())
    if log is not None:
        log.close()


# ---------------------------------------------------------------- 评估
def greedy_eval(env, agent, scenarios, max_steps, out_dir, split):
    """固定场景贪心评估：逐回合结果 + 逐布局分解 + Wilson 区间。"""
    os.makedirs(out_dir, exist_ok=True)
    traj_dir = os.path.join(out_dir, "trajectories")
    os.makedirs(traj_dir, exist_ok=True)
    env.max_episode_steps = max_steps

    denom = len(scenarios)
    mode_counts = {m: 0 for m in FAILURE_MODES}
    success = 0
    reasons = {}
    layout_stats = {}
    records = []
    tracking_errors = []
    min_obstacles = []

    def _ls(layout):
        return layout_stats.setdefault(layout, {
            "episodes": 0, "success": 0, "collision": 0,
            "timeout": 0, "reset_failure": 0, "other": 0})

    ep_path = os.path.join(out_dir, "episodes.jsonl")
    with open(ep_path, "w", encoding="utf-8") as stream:
        for idx, scn in enumerate(scenarios):
            apply_scenario(env, scn)  # 注入起始位姿 + 障碍布局
            sid, layout = scn["scenario_id"], scn["layout"]
            ls = _ls(layout)
            ls["episodes"] += 1
            rec = {"scenario_id": sid, "layout": layout, "episode": idx,
                   "seed": scn.get("eval_seed", idx), "split": split,
                   "run_dir": out_dir}
            t0 = time.time()
            state = env.reset()
            if not env.reset_ok:
                # reset 失败同样计入失败分母
                mode_counts["reset_failure"] += 1
                ls["reset_failure"] += 1
                rec.update({"reset_ok": False, "steps": 0,
                            "done_reason": "reset_failure",
                            "episode_success": False,
                            "wall_s": round(time.time() - t0, 2)})
                records.append(rec)
                stream.write(json.dumps(rec, ensure_ascii=False) + "\n")
                continue

            traj_path = os.path.join(traj_dir, "ep%03d_%s.jsonl" % (idx, sid))
            final_info = {}
            steps = 0
            sum_err = 0.0
            with open(traj_path, "w", encoding="utf-8") as traj:
                for _ in range(max_steps):
                    action = agent.select_action(
                        state, epsilon=0.0,
                        allowed_actions=env.allowed_actions())
                    state, _reward, done, final_info = env.step(action)
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
            ok = bool(final_info.get("episode_success", False))
            success += int(ok)
            # 归一失败模式
            if ok:
                mode = None
            elif reason == "collision":
                mode = "collision"
            elif reason == "timeout":
                mode = "timeout"
            else:
                mode = "other"
            if mode:
                mode_counts[mode] += 1
                ls[mode] += 1
            ls["success"] += int(ok)
            tracking_errors.append(sum_err / steps if steps else 0.0)
            min_obstacles.append(float(final_info.get("min_obstacle_min", 1.0)))
            rec.update({
                "reset_ok": True, "steps": steps, "done_reason": reason,
                "episode_success": ok,
                "failure_mode": mode or "success",
                "min_obstacle_min": final_info.get("min_obstacle_min"),
                "wall_s": round(time.time() - t0, 2),
                "trajectory_file": os.path.relpath(traj_path, out_dir),
            })
            records.append(rec)
            stream.write(json.dumps(rec, ensure_ascii=False) + "\n")

    valid = len(tracking_errors)
    # 逐布局分解（含成功率 Wilson）
    for layout, st in layout_stats.items():
        d = st["episodes"]
        st["failure_denominator"] = d
        st["success_rate"] = (st["success"] / d) if d else None
        st["success_wilson95"] = list(wilson(st["success"], d))

    metrics = {
        "split": split,
        "layouts": sorted({s["layout"] for s in scenarios}),
        "episodes": denom,
        "valid_episodes": valid,
        "failure_denominator": denom,
        "success": success,
        "success_rate": (success / denom) if denom else None,
        "success_wilson95": list(wilson(success, denom)),
        "failure_modes": {m: rate_block(mode_counts[m], denom)
                          for m in FAILURE_MODES},
        "termination_reasons": reasons,
        "layout_breakdown": layout_stats,
        "mean_tracking_error": (sum(tracking_errors) / valid) if valid else None,
        "mean_min_obstacle": (sum(min_obstacles) / valid) if valid else None,
        "episodes_raw": records,
    }
    with open(os.path.join(out_dir, "metrics.json"), "w",
              encoding="utf-8") as stream:
        json.dump(metrics, stream, ensure_ascii=False, indent=2)
        stream.write("\n")
    return metrics


# ---------------------------------------------------------------- 训练（臂 B）
def train(env, agent, buffer, scenarios, args, run_dir):
    """在 train 布局轮换上以更大预算重训 DQN；返回训练摘要。"""
    decay = max(1, int(args.episodes * args.max_steps * 0.5))
    total_steps = 0
    update_steps = 0
    best_score = None
    best_model_path = os.path.join(run_dir, "best_model.pt")
    os.makedirs(run_dir, exist_ok=True)
    layouts_seen = set()
    for ep in range(args.episodes):
        scn = scenarios[ep % len(scenarios)]
        apply_scenario(env, scn)
        layouts_seen.add(scn["layout"])
        state = env.reset()
        if not env.reset_ok:
            continue
        done = False
        final_info = {}
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
            if done:
                break
        if best_score is None or ep_reward > best_score:
            best_score = ep_reward
            agent.save(best_model_path)
        if (ep + 1) % 50 == 0:
            print("  [训练] ep%03d reason=%s reward=%.2f total_steps=%d"
                  % (ep + 1, final_info.get("done_reason"), ep_reward,
                     total_steps), flush=True)
    agent.save(os.path.join(run_dir, "last_model.pt"))
    return {"episodes": args.episodes, "total_steps": total_steps,
            "update_steps": update_steps, "best_model": best_model_path,
            "train_layouts": sorted(layouts_seen)}


# ---------------------------------------------------------------- 模型解析
def resolve_existing_model(explicit):
    """解析臂 A 复用模型：显式路径 > car_n5 默认 > 扫描 training_runs/*。"""
    if explicit:
        if os.path.exists(explicit):
            return os.path.abspath(explicit), "explicit"
        return None, "explicit_not_found"
    if os.path.exists(DEFAULT_MODEL):
        return DEFAULT_MODEL, "default_car_n5"
    found = glob.glob(os.path.join(WS, "training_runs/*/best_model.pt"))
    if not found:
        return None, "none_found"
    latest = max(found, key=os.path.getmtime)
    return latest, "fallback_scan_latest"


def _arm_eval(env, agent, sets, max_steps, out_root, tag):
    """在 train/validation/test 三集上评估一个臂，返回 metrics 与泛化 gap。"""
    res = {}
    for split in ("train", "validation", "test"):
        res[split] = greedy_eval(env, agent, sets[split], max_steps,
                                 os.path.join(out_root, tag, split), split)
    train_sr = res["train"]["success_rate"]
    test_sr = res["test"]["success_rate"]
    gap = None if (train_sr is None or test_sr is None) else (train_sr - test_sr)
    res["generalization_gap"] = {
        "definition": "train 布局成功率 − test 布局成功率",
        "train_success_rate": train_sr,
        "test_success_rate": test_sr,
        "point": gap,
    }
    return res


# ---------------------------------------------------------------- CLI
def parse_args():
    p = argparse.ArgumentParser(
        description="阶段3 D2 地面 DQN 未见布局独立复评（复用 + 重训两臂）")
    p.add_argument("--mode", choices=["reuse", "retrain", "both"], default="both")
    p.add_argument("--episodes", type=int, default=800, help="臂 B 重训预算")
    p.add_argument("--val-n", type=int, default=20, help="val 布局每布局场景数")
    p.add_argument("--test-n", type=int, default=30, help="test 布局每布局场景数")
    p.add_argument("--train-n", type=int, default=20,
                   help="train 布局每布局评估场景数（用于泛化 gap）")
    p.add_argument("--max-steps", type=int, default=150)
    p.add_argument("--seed", type=int, default=101)
    p.add_argument("--out", default="experiments/d/d2_ground_reeval.json")
    p.add_argument("--existing-model", default="",
                   help="臂 A 复用的模型路径（默认 training_runs/car_n5/best_model.pt）")
    p.add_argument("--run-dir", default="training_runs/car_n5_d2_reeval",
                   help="臂 B 重训输出目录")
    # 与 n5 一致的超参
    p.add_argument("--batch-size", type=int, default=64)
    p.add_argument("--target-update-steps", type=int, default=500)
    p.add_argument("--train-start-steps", type=int, default=1000)
    p.add_argument("--learning-rate", type=float, default=1e-4)
    p.add_argument("--gamma", type=float, default=0.99)
    p.add_argument("--hidden-dims", default="128,128")
    gui = p.add_mutually_exclusive_group()
    gui.add_argument("--gui", action="store_true", help="启动 Gazebo GUI（默认）")
    gui.add_argument("--headless", action="store_true", help="无头运行")
    p.add_argument("--plan-only", action="store_true",
                   help="仅打印离线场景规划与待执行命令，不启动仿真")
    p.add_argument("--self-test", action="store_true",
                   help="仅运行 Wilson 对拍自检并退出")
    return p.parse_args()


def self_test():
    """Wilson 对拍（复用本文件实现），k=5,n=10 → (0.2366,0.7634)。"""
    ok = True
    print("[self-test] Wilson 95% 对拍（容差 1e-3，口径同 d2_stat_report）")
    cases = [(5, 10, (0.2366, 0.7634), "both"),
             (0, 20, (None, 0.1611), "upper"),
             (20, 20, (0.8389, None), "lower")]
    for k, n, exp, mode in cases:
        lo, hi = wilson(k, n)
        if mode == "both":
            passed = abs(lo - exp[0]) < 1e-3 and abs(hi - exp[1]) < 1e-3
        elif mode == "upper":
            passed = abs(hi - exp[1]) < 1e-3
        else:
            passed = abs(lo - exp[0]) < 1e-3
        ok = ok and passed
        print("  k=%d n=%d -> actual=(%.6f, %.6f) expected=%s  %s"
              % (k, n, lo, hi, exp, "PASS" if passed else "FAIL"))
    print("[self-test] 总结果：%s" % ("PASS" if ok else "FAIL"))
    return 0 if ok else 1


def main():
    args = parse_args()
    if args.self_test:
        return self_test()

    args.hidden_dims = [int(x) for x in args.hidden_dims.split(",") if x]
    base, sets, eval_seed = build_plan(args)
    summary = plan_summary(base, sets, eval_seed)

    # ---- 离线规划模式：不启动 Gazebo ----
    if args.plan_only:
        print("[D2] 场景规划（离线，未启动仿真）")
        for name in ("train", "validation", "test"):
            s = summary["sets"][name]
            per_layout = {}
            for scn in sets[name]:
                per_layout[scn["layout"]] = per_layout.get(scn["layout"], 0) + 1
            print("  %-11s 场景数=%d sha256=%s 每布局场景数=%s"
                  % (name, s["episodes"], s["sha256"][:12], per_layout))
        print("[D2] n5 基础场景集 sha256=%s"
              % summary["n5_scenario_split_sha256"][:12])
        print("[D2] 布局隔离断言：PASS（扩展集 train/val/test 布局互不相交）")
        print("[D2] 待执行命令示例（由主流程在 AGC 批跑结束后运行）：")
        print("  python3 scripts/d2_ground_reeval.py --mode both --episodes 800 \\")
        print("      --val-n 20 --test-n 30 --train-n 20 --max-steps 150 \\")
        print("      --seed 101 --out experiments/d/d2_ground_reeval.json   # 默认 --gui")
        print("  # 无头变体：追加 --headless")
        print("  # 仅臂 A：--mode reuse --existing-model training_runs/car_n5/best_model.pt")
        return 0

    # ---- 需要仿真：延迟导入 ROS 相关模块 ----
    try:
        from dqn_agent import DQNAgent
        from replay_buffer import PrioritizedReplayBuffer
        from ground_env import GroundEnv
    except Exception as exc:  # 依赖缺失时清晰报错
        print("[D2] 错误：无法导入地面仿真依赖（%s）。请确认已 source ROS/工作区环境。"
              % exc, flush=True)
        return 2

    out_path = os.path.join(WS, args.out)
    out_dir = os.path.dirname(out_path)
    os.makedirs(out_dir, exist_ok=True)
    run_dir = os.path.join(WS, args.run_dir)

    random.seed(args.seed)
    try:
        import torch
        torch.manual_seed(args.seed)
    except Exception:
        pass

    print("[D2] mode=%s | 布局=%s | train=%d val=%d test=%d | eval_seed=%d"
          % (args.mode, LAYOUT_SPLIT, len(sets["train"]),
             len(sets["validation"]), len(sets["test"]), eval_seed), flush=True)

    sim = log = None
    arms = {}
    try:
        sim, log = start_sim(not args.headless,
                             os.path.join(out_dir, "ground_sim.log"))
        if not wait_service("/ground_gazebo/set_model_state"):
            print("[D2] 错误：地面仿真服务 /ground_gazebo/set_model_state 不可用"
                  "（超时）。请确认 Gazebo/rosmaster 已就绪后重试。", flush=True)
            return 1

        env = GroundEnv()
        env.max_episode_steps = args.max_steps

        def make_agent():
            return DQNAgent(env.observation_space_dim, env.n_actions,
                            lr=args.learning_rate, gamma=args.gamma,
                            hidden_dims=args.hidden_dims)

        def build_report():
            """组装报告字典（臂A/臂B结果 + 场景集摘要 + 未决边界）。"""
            return {
                "created_at": datetime.now().isoformat(),
                "stage": "D2",
                "component": "ground_dqn_unseen_layout_reeval",
                "mode": args.mode,
                "seed": args.seed,
                "eval_seed": eval_seed,
                "episodes": args.episodes,
                "max_steps": args.max_steps,
                "gui": (not args.headless),
                "scenario_split_sha256": {
                    "expanded": {k: summary["sets"][k]["sha256"] for k in summary["sets"]},
                    "n5_base": summary["n5_scenario_split_sha256"],
                },
                "layout_split": LAYOUT_SPLIT,
                "scenario_plan": summary["sets"],
                "sample_sizes": {k: summary["sets"][k]["episodes"]
                                 for k in summary["sets"]},
                "arms": arms,
                "summary": {
                    "reuse_test_success_rate":
                        arms.get("reuse", {}).get("test", {}).get("success_rate"),
                    "retrain_test_success_rate":
                        arms.get("retrain", {}).get("test", {}).get("success_rate"),
                    "reuse_generalization_gap":
                        arms.get("reuse", {}).get("generalization_gap", {}).get("point"),
                    "retrain_generalization_gap":
                        arms.get("retrain", {}).get("generalization_gap", {}).get("point"),
                },
                "undecided_boundaries": [
                    "本报告为单次贪心评估的独立复评（扩大样本），非多随机种子重复；"
                    "点估计仍可能受评估分支随机性影响。",
                    "泛化 gap 用 train 布局（训练所见）成功率与 test 布局成功率之差"
                    "作描述性指标，未给差值置信区间（两集非配对）。",
                    "reset 失败计入失败分母，与 n5 口径一致。",
                    "Wilson 口径与 src/air_ground_core/scripts/d2_stat_report.py 一致"
                    "（Z95=1.959963984540054）。",
                ],
                "notes": [
                    "test 布局 %s 在训练/验证中均未出现（按布局隔离）。"
                    % sorted({s["layout"] for s in sets["test"]}),
                    "两臂使用完全相同的 test 场景集（同一 eval_seed 生成）。",
                    "障碍布局经 apply_scenario 注入 env.poles，无需切换 world 文件。",
                ],
            }

        def dump_report(path):
            """增量落盘：臂A完成即可先写 partial，降低长跑中断丢数据的风险。"""
            with open(path, "w", encoding="utf-8") as stream:
                json.dump(build_report(), stream, ensure_ascii=False, indent=2)
                stream.write("\n")
            print("[D2] 报告已写: %s" % path, flush=True)

        # ---- 臂 A：复用现有模型 ----
        if args.mode in ("reuse", "both"):
            model_path, source = resolve_existing_model(args.existing_model)
            if model_path is None:
                print("[D2] 错误：未找到可复用的模型（source=%s）；"
                      "请用 --existing-model 指定。" % source, flush=True)
                return 1
            print("[D2][臂A] 复用模型: %s (source=%s)" % (model_path, source),
                  flush=True)
            agent = make_agent()
            agent.load(model_path)
            arms["reuse"] = _arm_eval(env, agent, sets, args.max_steps,
                                      out_dir, "reuse")
            arms["reuse"]["model"] = model_path
            arms["reuse"]["model_source"] = source
            arms["reuse"]["model_sha256"] = _sha256(model_path)
            print("[D2][臂A] train=%s val=%s test=%s gap=%s"
                  % (arms["reuse"]["train"]["success_rate"],
                     arms["reuse"]["validation"]["success_rate"],
                     arms["reuse"]["test"]["success_rate"],
                     arms["reuse"]["generalization_gap"]["point"]), flush=True)

            dump_report(out_path + ".partial.json")

        # ---- 臂 B：更大预算重训 ----
        if args.mode in ("retrain", "both"):
            print("[D2][臂B] 在 train 布局上重训 %d episodes" % args.episodes,
                  flush=True)
            agent = make_agent()
            buffer = PrioritizedReplayBuffer(BUFFER_CAPACITY, alpha=PER_ALPHA,
                                             beta=PER_BETA)
            train_summary = train(env, agent, buffer, base["train"], args,
                                  run_dir)
            best = os.path.join(run_dir, "best_model.pt")
            agent.load(best)
            arms["retrain"] = _arm_eval(env, agent, sets, args.max_steps,
                                        out_dir, "retrain")
            arms["retrain"]["train_summary"] = train_summary
            arms["retrain"]["model"] = best
            arms["retrain"]["model_sha256"] = _sha256(best)
            print("[D2][臂B] train=%s val=%s test=%s gap=%s"
                  % (arms["retrain"]["train"]["success_rate"],
                     arms["retrain"]["validation"]["success_rate"],
                     arms["retrain"]["test"]["success_rate"],
                     arms["retrain"]["generalization_gap"]["point"]), flush=True)
    finally:
        if sim is not None:
            stop_sim(sim, log)

    dump_report(out_path)
    return 0


if __name__ == "__main__":
    sys.exit(main())
