#!/usr/bin/env python3
"""verify_m6_scenarios.py —— M6 空地协同任务场景离线核验。

读取 run_m6_scenarios.py 生成的 m6_scenario_report.json，逐项核对：
  A 场景覆盖与逐回合结局（状态/原因与期望一致）
  B 每回合元数据与契约版本（agc-m0-v1.2、场景/参数哈希、目标源）
  C 回放日志齐全（state_events / tracking_frames / planner / drive / target_trajectory / topics）
  D M0 结果口径汇总（两条恒等式、名义任务完成率、无 INTERCEPT_CONFIRMED）
  E 单一启动与干净退出（any_leftover=false）

用法：
    python3 verify_m6_scenarios.py [report.json]
输出：experiments/agc_runs/verify_m6_scenarios.json，并在 stdout 打印 PASS/FAIL。
"""

import glob
import json
import os
import sys

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
# 本脚本位于 <workspace>/src/air_ground_core/scripts
WORKSPACE = os.path.abspath(os.path.join(SCRIPT_DIR, "..", "..", ".."))
RUNS_ROOT = os.path.join(WORKSPACE, "experiments", "agc_runs")
REPORT_PATH = os.path.join(RUNS_ROOT, "m6_scenario_report.json")
OUT_PATH = os.path.join(RUNS_ROOT, "verify_m6_scenarios.json")

CONTRACT_VERSION = "agc-m0-v1.2"
REPLAY_FILES = ["state_events.jsonl", "planner_log.jsonl",
                "drive_log.jsonl", "target_trajectory.jsonl"]
# 抵近跟踪帧日志仅在 run 进入 GROUND_APPROACH 时产生（不可达场景不进入，故为条件项）
TRACKING_FILE = "tracking_frames.jsonl"
TOPIC_FILES = ["_agc_task_state.jsonl", "_agc_ground_cmd_vel.jsonl", "_agc_ground_odom.jsonl"]
FUSION_FILES = ["perception_log.jsonl", "fusion_log.jsonl"]
EXPECT_TARGET_SOURCE = {"obs_loss": "air_fusion"}  # 其余默认 gt_adapter
PATH_TOPIC = "/agc/ground/path"
# 正确失败：无可行路径且任务按预期结算 PLAN_UNREACHABLE（供审计条件豁免判断）
NOPATH_FINAL_STATE = 5
NOPATH_REASON = "PLAN_UNREACHABLE"


def audit_path_failures(batch_dir):
    """读 batch_dir 的 audit_interfaces_cycle*.json，按话题拆分 FAIL 项。

    返回 (非 path 的 FAIL 项, /agc/ground/path 的 FAIL 项, 说明)。
    path 项即「通用路径频率审计」，用于按任务状态做条件豁免判断。
    """
    if not batch_dir or not os.path.isdir(batch_dir):
        return (["<无批次目录>"], [], "batch_dir 不可用: %s" % batch_dir)
    files = sorted(glob.glob(os.path.join(batch_dir, "audit_interfaces_cycle*.json")))
    if not files:
        return (["<无审计文件>"], [], "无 audit_interfaces_cycle*.json")
    non_path, path = [], []
    for fp in files:
        try:
            with open(fp) as f:
                data = json.load(f)
        except (OSError, ValueError):
            non_path.append("%s:读取失败" % os.path.basename(fp))
            continue
        for c in data.get("checks", []):
            if c.get("status") != "FAIL":
                continue
            nm = c.get("name", "")
            item = "%s %s" % (os.path.basename(fp), nm)
            (path if PATH_TOPIC in nm else non_path).append(item)
    return (non_path, path,
            "%d 个审计文件；path FAIL=%d，非 path FAIL=%d" % (len(files), len(path), len(non_path)))


def entered_approach(run_dir):
    """读 state_events.jsonl，判断 run 是否进入过 GROUND_APPROACH。"""
    path = os.path.join(run_dir, "state_events.jsonl")
    try:
        with open(path, errors="ignore") as f:
            for line in f:
                if '"GROUND_APPROACH"' in line:
                    return True
    except OSError:
        pass
    return False


def first_drive_pose(run_dir):
    """读 drive_log.jsonl 首行，返回车实际起点 (x, y)；读取失败返回 None。"""
    path = os.path.join(run_dir, "drive_log.jsonl")
    try:
        with open(path, errors="ignore") as f:
            for line in f:
                line = line.strip()
                if line:
                    r = json.loads(line)
                    p = r.get("pose") or {}
                    return (float(p.get("x", 0.0)), float(p.get("y", 0.0)))
    except (OSError, ValueError):
        pass
    return None


def check(checks, cid, ok, detail):
    checks.append({"id": cid, "pass": bool(ok), "detail": detail})


def main():
    report_path = sys.argv[1] if len(sys.argv) > 1 else REPORT_PATH
    if not os.path.isfile(report_path):
        print("找不到报告：%s（请先运行 run_m6_scenarios.py）" % report_path)
        sys.exit(1)
    with open(report_path) as f:
        rep = json.load(f)

    checks = []
    scenarios = rep.get("scenarios", [])
    keys = [s["key"] for s in scenarios]
    check(checks, "A0", len(scenarios) >= 4 and set(keys) >=
          {"static_obstacles", "moving_target", "obs_loss", "unreachable"},
          "覆盖场景=%s（要求 4 类）" % keys)

    # A/B/C：逐场景、逐回合
    for s in scenarios:
        k = s["key"]
        check(checks, "A1.%s" % k, s.get("attempted_runs", 0) >= 1,
              "attempted=%d" % s.get("attempted_runs", 0))
        check(checks, "A2.%s" % k, bool(s.get("conform")),
              "期望 state=%s reason=%s；命中 %d/%d；实际=%s"
              % (s.get("expect_state"), s.get("expect_reason") or "(任意)",
                 s.get("matched_expected"), s.get("attempted_runs"),
                 [(e.get("final_state"), e.get("reason")) for e in s.get("episodes", [])]))
        for ep in s.get("episodes", []):
            tag = "%s.c%s.a%s" % (k, ep.get("cycle"), ep.get("attempt"))
            rd = ep.get("run_dir", "")
            check(checks, "A3.%s" % tag, bool(rd) and os.path.isdir(rd), "run_dir=%s" % rd)
            # 回合独立性：每个回合的车实际起点应为场景 car_start（-4,-3）
            p0 = first_drive_pose(rd) if (rd and os.path.isdir(rd)) else None
            check(checks, "A4.%s" % tag,
                  p0 is not None and abs(p0[0] + 4.0) <= 0.30 and abs(p0[1] + 3.0) <= 0.30,
                  "车实际起点=%s（期望 (-4,-3)±0.30，验证回合独立）" % (p0,))
            meta = ep.get("meta") or {}
            check(checks, "B1.%s" % tag, meta.get("contract_version") == CONTRACT_VERSION,
                  "contract_version=%s" % meta.get("contract_version"))
            check(checks, "B2.%s" % tag,
                  bool(meta.get("scenario_hash")) and bool(meta.get("agc_params_hash")),
                  "scenario_hash=%s agc_params_hash=%s"
                  % (meta.get("scenario_hash"), meta.get("agc_params_hash")))
            exp_src = EXPECT_TARGET_SOURCE.get(k, "gt_adapter")
            check(checks, "B3.%s" % tag, meta.get("target_source") == exp_src,
                  "target_source=%s（期望 %s）" % (meta.get("target_source"), exp_src))
            if rd and os.path.isdir(rd):
                missing = [f for f in REPLAY_FILES if not os.path.isfile(os.path.join(rd, f))]
                # 进入抵近阶段才要求 tracking_frames.jsonl（不可达场景不进入，依设计不产生）
                if entered_approach(rd) and not os.path.isfile(os.path.join(rd, TRACKING_FILE)):
                    missing.append(TRACKING_FILE)
                check(checks, "C1.%s" % tag, not missing, "缺回放日志=%s" % (missing or "无"))
                tdir = os.path.join(rd, "topics")
                tmiss = [f for f in TOPIC_FILES if not os.path.isfile(os.path.join(tdir, f))]
                check(checks, "C2.%s" % tag, not tmiss, "缺话题日志=%s" % (tmiss or "无"))
                if exp_src == "air_fusion":
                    fmiss = [f for f in FUSION_FILES if not os.path.isfile(os.path.join(rd, f))]
                    check(checks, "C3.%s" % tag, not fmiss, "缺感知/融合日志=%s" % (fmiss or "无"))

    # D：M0 口径
    agg = rep.get("aggregate", {})
    eq = rep.get("equations", {})
    check(checks, "D1", eq.get("valid_task") == "PASS",
          "valid_task=%s (%d=%d+%d)" % (eq.get("valid_task"), agg.get("valid_task_runs", 0),
                                        agg.get("tracking_success_runs", 0),
                                        agg.get("mission_failure_runs", 0)))
    check(checks, "D2", eq.get("attempted") == "PASS",
          "attempted=%s (%d=%d+%d+%d)" % (eq.get("attempted"), agg.get("attempted_runs", 0),
                                          agg.get("valid_task_runs", 0),
                                          agg.get("infra_failure_runs", 0),
                                          agg.get("aborted_runs", 0)))
    rate = rep.get("nominal_task_completion_rate", {})
    check(checks, "D3", rate.get("value") == 1.0 and rate.get("den", 0) > 0,
          "名义任务完成率=%s (%s/%s)" % (rate.get("value"), rate.get("num"), rate.get("den")))
    all_names = [(e.get("final_state"), e.get("final_state_name"))
                 for s in scenarios for e in s.get("episodes", [])]
    check(checks, "D4", all(fs != 7 for fs, _ in all_names) and
          all(nm != "INTERCEPT_CONFIRMED" for _, nm in all_names),
          "无 INTERCEPT_CONFIRMED（阶段 A）")

    # E：干净退出 + 现场接口审计（按任务状态判断）
    for s in scenarios:
        check(checks, "E1.%s" % s["key"], s.get("any_leftover") is False,
              "any_leftover=%s" % s.get("any_leftover"))
        # 按任务状态判断的审计：拆分非 path 的 FAIL 与 /agc/ground/path 的 FAIL。
        # 不可达场景 /agc/ground/path 依设计无消息（无可达路径），仅当本场景「全部回合按预期
        # 正确失败（MISSION_FAILED/PLAN_UNREACHABLE）」时才条件豁免该 path 项；其余任何异常无路径
        # （名义场景缺 path、或失败原因不是 PLAN_UNREACHABLE）都判 FAIL，不得放行。
        non_path, path, adetail = audit_path_failures(s.get("batch_dir"))
        eps = s.get("episodes", [])
        correct_nopath = bool(eps) and all(
            ep.get("final_state") == NOPATH_FINAL_STATE and ep.get("reason") == NOPATH_REASON
            for ep in eps)
        if not non_path and not path:
            check(checks, "E2.%s" % s["key"], True, "接口审计全项通过；%s" % adetail)
        elif not non_path and path and correct_nopath:
            check(checks, "E2.%s" % s["key"], True,
                  "仅 %s 频率/样本缺失且全部回合 PLAN_UNREACHABLE（无可行路径且正确失败），"
                  "条件豁免；%s" % (PATH_TOPIC, adetail))
        else:
            check(checks, "E2.%s" % s["key"], False,
                  "非 path FAIL=%s；path FAIL=%s；correct_nopath=%s；%s"
                  % (non_path or "无", path or "无", correct_nopath, adetail))

    # F：设计内失败/注入场景的语义
    loss = next((s for s in scenarios if s["key"] == "obs_loss"), None)
    if loss:
        inj = [e.get("inject", "") for e in loss.get("episodes", [])]
        check(checks, "F1", all("loss" in i for i in inj), "obs_loss 注入标记=%s" % inj)
    unr = next((s for s in scenarios if s["key"] == "unreachable"), None)
    if unr:
        rs = [e.get("reason") for e in unr.get("episodes", [])]
        check(checks, "F2", all(r == "PLAN_UNREACHABLE" for r in rs), "unreachable 原因=%s" % rs)

    npass = sum(1 for c in checks if c["pass"])
    nfail = len(checks) - npass
    out = {"report": report_path, "contract_version": CONTRACT_VERSION,
           "checks": checks, "PASS": npass, "FAIL": nfail,
           "result": "PASS" if nfail == 0 else "FAIL"}
    with open(OUT_PATH, "w") as f:
        json.dump(out, f, ensure_ascii=False, indent=2)

    for c in checks:
        print("%s  %-16s %s" % ("PASS" if c["pass"] else "FAIL", c["id"], c["detail"]))
    print("-" * 72)
    print("汇总: PASS=%d FAIL=%d" % (npass, nfail))
    print("verify_m6_scenarios.json 已写入: %s" % OUT_PATH)
    sys.exit(0 if nfail == 0 else 1)


if __name__ == "__main__":
    main()