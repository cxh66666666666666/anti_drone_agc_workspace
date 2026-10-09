#!/usr/bin/env python3
"""run_m6_scenarios.py —— M6 空地协同任务场景「单命令运行器」。

依次运行 M6 四类固定场景，每类 repeat 个相互独立的回合（每回合一个全新「启动→退出」全循环，
确保从场景初值 car_start 起算）：
  1) 静态障碍 static_obstacles : config/scenario_flat.yaml           期望 TRACKING_SUCCESS
  2) 目标移动 moving_target    : config/scenario_linear_target.yaml  期望 TRACKING_SUCCESS
  3) 观测丢失 obs_loss         : config/scenario_obs_loss.yaml       期望 MISSION_FAILED/TARGET_LOST
                                 （air_fusion 源 + run 期感知丢失注入）
  4) 不可达   unreachable      : config/scenario_unreachable.yaml    期望 MISSION_FAILED/PLAN_UNREACHABLE

每次 attempt 解析终态与 reason_code，生成逐回合报告（含 run 目录 = 回放日志）与 M0 口径
汇总，写入 experiments/agc_runs/m6_scenario_report.json。

用法：
    python3 scripts/run_m6_scenarios.py [--repeat 2] [--run-timeout 150] [--only key,...]

注：本脚本是离线编排工具（非实时链路），按 AGENTS.md 对离线工具的规定用 Python 实现；
实时协同节点仍为 C++。运行仿真按用户要求带 Gazebo GUI（非无头）。
"""

import argparse
import json
import os
import re
import subprocess
import sys
import time

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
# 本脚本位于 <workspace>/src/air_ground_core/scripts；编排脚本 start_agc_collab.sh 在 <workspace>/scripts
WORKSPACE = os.path.abspath(os.path.join(SCRIPT_DIR, "..", "..", ".."))
START_SCRIPT = os.path.join(WORKSPACE, "scripts", "start_agc_collab.sh")
CONFIG_DIR = os.path.join(WORKSPACE, "src", "air_ground_core", "config")
RUNS_ROOT = os.path.join(WORKSPACE, "experiments", "agc_runs")
REPORT_PATH = os.path.join(RUNS_ROOT, "m6_scenario_report.json")

STATE_NAME = {
    0: "IDLE", 1: "AIR_SEARCH", 2: "TARGET_ACQUIRED", 3: "GROUND_APPROACH",
    4: "TRACKING_SUCCESS", 5: "MISSION_FAILED", 6: "ABORTED",
    7: "INTERCEPT_CONFIRMED", 8: "INFRA_FAILURE",
}
CATEGORY_BY_STATE = {4: "tracking_success", 5: "mission_failure", 6: "aborted", 8: "infra_failure"}

# M6 四类固定场景（kind: nominal=名义任务；fault=设计内故障/失败场景）
SCENARIOS = [
    {
        "key": "static_obstacles",
        "name": "静态障碍",
        "scenario": "scenario_flat.yaml",
        "args": ["--target-source", "gt_adapter", "--obstacle-source", "scenario"],
        "kind": "nominal",
        "expect_state": 4,
        "expect_reason": "",
        "desc": "静态目标 + 两个静态障碍，车绕过障碍抵近并持续跟踪",
    },
    {
        "key": "moving_target",
        "name": "目标移动",
        "scenario": "scenario_linear_target.yaml",
        "args": ["--target-source", "gt_adapter", "--obstacle-source", "scenario"],
        "kind": "nominal",
        "expect_state": 4,
        "expect_reason": "",
        "desc": "目标 0.6 m/s 匀速往返（< 车上限 1.0 m/s），车跟随重规划并持续跟踪",
    },
    {
        "key": "obs_loss",
        "name": "观测丢失",
        "scenario": "scenario_obs_loss.yaml",
        "args": ["--target-source", "air_fusion", "--obstacle-source", "scenario",
                 "--inject", "loss", "--inject-delay", "6"],
        "kind": "fault",
        "expect_state": 5,
        "expect_reason": "TARGET_LOST",
        "desc": "融合源先获得目标，start 后 6s 全通道丢失 -> 停车 -> TARGET_LOST",
    },
    {
        "key": "unreachable",
        "name": "不可达/拦截失败",
        "scenario": "scenario_unreachable.yaml",
        "args": ["--target-source", "gt_adapter", "--obstacle-source", "scenario"],
        "kind": "fault",
        "expect_state": 5,
        "expect_reason": "PLAN_UNREACHABLE",
        "desc": "目标贴近 pole_2，安全接近点落于规划阻塞栅格 -> PLAN_UNREACHABLE",
    },
]


def load_json(path):
    try:
        with open(path) as f:
            return json.load(f)
    except (OSError, ValueError):
        return None


def parse_jsonl(path):
    out = []
    try:
        with open(path) as f:
            for line in f:
                line = line.strip()
                if line:
                    out.append(json.loads(line))
    except (OSError, ValueError):
        pass
    return out


def run_scenario(sc, repeat, run_timeout, suite_dir):
    """运行一个场景的批跑，返回 (batch_dir, returncode)。"""
    scenario_path = os.path.join(CONFIG_DIR, sc["scenario"])
    prefix = "_m6_%s_" % sc["key"]
    # 每个回合用一个全新「启动→退出」全循环（--cycles N --repeat 1）：保证回合间相互独立。
    # 原因：单次启动内多次 attempt 之间，ground_drive_adapter 会在任务非运行期持续把积分位姿
    # 写回车体，覆盖 scenario_reset 的 car_start（M1 权威批次亦如此），使后续回合并非从场景
    # 初值开始。用全新栈（世界 spawn + 启动复位）可确保每个回合都从 car_start 起算。
    # 直接执行编排脚本（走 shebang）以使其进程名/子 shell 名可被脚本自身残留核查正确排除。
    cmd = [START_SCRIPT, "--gui", "--cycles", str(repeat), "--repeat", "1",
           "--run-timeout", str(run_timeout), "--scenario", scenario_path,
           "--batch-prefix", prefix] + sc["args"]
    log_path = os.path.join(suite_dir, "%s.log" % sc["key"])
    print("[M6] 运行场景 %s：%s" % (sc["key"], " ".join(cmd)), flush=True)
    env = dict(os.environ)
    env.setdefault("DISPLAY", ":0")
    with open(log_path, "wb") as logf:
        proc = subprocess.run(cmd, cwd=WORKSPACE, env=env, stdout=logf,
                              stderr=subprocess.STDOUT)
    # ROS/roslaunch 输出可能含非 UTF-8 字节，读取日志时忽略非法字节
    txt = ""
    try:
        with open(log_path, "r", errors="ignore") as f:
            txt = f.read()
    except OSError:
        pass
    batch_dir = None
    m = re.search(r"batch_dir=(\S+)", txt)
    if m:
        batch_dir = m.group(1)
    return batch_dir, proc.returncode


def collect_episodes(sc, batch_dir):
    """从批跑目录读取逐 attempt 记录，并补充 run 元数据。"""
    episodes = []
    if not batch_dir or not os.path.isdir(batch_dir):
        return episodes
    for rec in parse_jsonl(os.path.join(batch_dir, "records.jsonl")):
        if rec.get("type") != "attempt":
            continue
        fs = rec.get("final_state")
        reason = rec.get("reason", "") or ""
        run_dir = rec.get("run_dir", "") or ""
        ep = {
            "cycle": rec.get("cycle"), "attempt": rec.get("attempt"),
            "reset_ok": rec.get("reset_ok"), "start_ok": rec.get("start_ok"),
            "final_state": fs,
            "final_state_name": STATE_NAME.get(fs, "UNKNOWN" if fs is not None else None),
            "reason": reason,
            "inject": rec.get("inject", "none"),
            "run_dir": run_dir,
            "meta": None, "summary_final_state": None,
        }
        meta = load_json(os.path.join(run_dir, "meta.json")) if run_dir else None
        if meta:
            ep["meta"] = {k: meta.get(k) for k in
                          ("run_id", "contract_version", "scenario_hash", "agc_params_hash",
                           "target_source", "obstacle_source")}
        summary = load_json(os.path.join(run_dir, "summary.json")) if run_dir else None
        if summary:
            ep["summary_final_state"] = summary.get("final_state")
        # 与期望结局比对
        ok = (fs == sc["expect_state"])
        if sc["expect_reason"]:
            ok = ok and (reason == sc["expect_reason"])
        ep["matches_expected"] = bool(ok)
        episodes.append(ep)
    return episodes


def scenario_stats(sc, episodes):
    attempted = len(episodes)
    counts = {"tracking_success": 0, "mission_failure": 0, "aborted": 0, "infra_failure": 0}
    for ep in episodes:
        cat = CATEGORY_BY_STATE.get(ep["final_state"])
        if cat:
            counts[cat] += 1
    valid_task = counts["tracking_success"] + counts["mission_failure"]
    return {
        "key": sc["key"], "name": sc["name"], "kind": sc["kind"], "desc": sc["desc"],
        "scenario": sc["scenario"], "expect_state": sc["expect_state"],
        "expect_reason": sc["expect_reason"],
        "attempted_runs": attempted,
        "valid_task_runs": valid_task,
        "tracking_success_runs": counts["tracking_success"],
        "mission_failure_runs": counts["mission_failure"],
        "aborted_runs": counts["aborted"],
        "infra_failure_runs": counts["infra_failure"],
        "matched_expected": sum(1 for ep in episodes if ep["matches_expected"]),
        "conform": (attempted > 0 and all(ep["matches_expected"] for ep in episodes)),
        "episodes": episodes,
    }


def main():
    ap = argparse.ArgumentParser(description="M6 空地协同任务场景单命令运行器")
    ap.add_argument("--repeat", type=int, default=2,
                    help="每场景回合数（每回合一个全新栈；默认 2）")
    ap.add_argument("--run-timeout", type=int, default=150, help="单 attempt 等待终态秒数")
    ap.add_argument("--only", default="", help="只运行指定场景 key（逗号分隔）")
    args = ap.parse_args()

    scenarios = SCENARIOS
    if args.only:
        keys = {k.strip() for k in args.only.split(",") if k.strip()}
        scenarios = [s for s in SCENARIOS if s["key"] in keys]

    suite_id = time.strftime("%Y%m%dT%H%M%S", time.gmtime())
    suite_dir = os.path.join(RUNS_ROOT, "_m6_suite_%s" % suite_id)
    os.makedirs(suite_dir, exist_ok=True)
    print("[M6] 套件目录：%s（repeat=%d）" % (suite_dir, args.repeat))

    results = []
    for sc in scenarios:
        batch_dir, rc = run_scenario(sc, args.repeat, args.run_timeout, suite_dir)
        episodes = collect_episodes(sc, batch_dir)
        st = scenario_stats(sc, episodes)
        st["batch_dir"] = batch_dir
        st["batch_returncode"] = rc
        batch = load_json(os.path.join(batch_dir, "batch.json")) if batch_dir else None
        st["any_leftover"] = batch.get("any_leftover") if isinstance(batch, dict) else None
        # 现场接口核查结果（按 cycle；不可达场景 /agc/ground/path 依设计无消息，path 频率项会 FAIL）
        st["audit_ok"] = ([c.get("audit_ok") for c in batch.get("cycles", [])]
                          if isinstance(batch, dict) else None)
        results.append(st)
        print("[M6] %-16s attempted=%d 期望命中=%d/%d conform=%s batch=%s"
              % (sc["key"], st["attempted_runs"], st["matched_expected"],
                 st["attempted_runs"], st["conform"], batch_dir))

    # M0 口径汇总（全场景）
    agg = {"attempted_runs": 0, "valid_task_runs": 0, "tracking_success_runs": 0,
           "mission_failure_runs": 0, "aborted_runs": 0, "infra_failure_runs": 0}
    for st in results:
        for k in agg:
            agg[k] += st.get(k, 0)
    eq_valid = agg["valid_task_runs"] == agg["tracking_success_runs"] + agg["mission_failure_runs"]
    eq_attempted = (agg["attempted_runs"] == agg["valid_task_runs"] +
                    agg["infra_failure_runs"] + agg["aborted_runs"])

    # 名义任务成功率：只统计 nominal 场景（fault 场景是设计内失败/注入，按契约 §8.4 排除）
    nom = [st for st in results if st["kind"] == "nominal"]
    nom_success = sum(st["tracking_success_runs"] for st in nom)
    nom_valid = sum(st["valid_task_runs"] for st in nom)

    report = {
        "generated_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "suite_dir": suite_dir,
        "repeat": args.repeat,
        "run_timeout_s": args.run_timeout,
        "scenarios": results,
        "aggregate": agg,
        "equations": {"valid_task": "PASS" if eq_valid else "FAIL",
                      "attempted": "PASS" if eq_attempted else "FAIL"},
        "nominal_task_completion_rate": {
            "num": nom_success, "den": nom_valid,
            "value": (float(nom_success) / nom_valid) if nom_valid else None,
            "note": "仅名义场景（静态障碍/目标移动）；fault 场景不计入名义成功率分母",
        },
        "all_scenarios_conform": all(st["conform"] for st in results) if results else False,
        "any_leftover": any(st["any_leftover"] for st in results),
    }
    os.makedirs(RUNS_ROOT, exist_ok=True)
    with open(REPORT_PATH, "w") as f:
        json.dump(report, f, ensure_ascii=False, indent=2)
    print("[M6] 报告已写入：%s" % REPORT_PATH)
    print("[M6] equations: valid_task=%s attempted=%s；全场景 conform=%s"
          % (report["equations"]["valid_task"], report["equations"]["attempted"],
             report["all_scenarios_conform"]))
    if not report["all_scenarios_conform"]:
        sys.exit(2)


if __name__ == "__main__":
    main()