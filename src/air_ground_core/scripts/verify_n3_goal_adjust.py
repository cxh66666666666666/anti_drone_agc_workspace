#!/usr/bin/env python3
"""verify_n3_goal_adjust.py —— N3 安全接近点/目标格可行化离线核验。

背景：原实现把安全接近点直接映射为栅格；当该格中心被膨胀图阻塞
（格中心净空 < dilate_m = max(inflation_m, protection_m)）时，astar 直接判
astar_no_path，误报 PLAN_UNREACHABLE。N3 在 goal_adjust_max_m 半径内就近取
可行格（不降低 protection 阈值），从而区分「可达但栅格落点错误」与「确实无路」。

两组场景核验：
  A 可达贴障（scenario_goal_adjust）：目标贴 pole_2，安全接近点格被阻塞但邻域有可行格
      -> 规划成功且 goal_adjusted=true、发布 path、终态 TRACKING_SUCCESS、最小净空 ≥ protection
  B 确实无路（scenario_truly_blocked）：目标被四道墙围住
      -> 目标段规划持续失败 reason∈{astar_no_path, goal_cell_blocked_no_feasible}、
         终态 PLAN_UNREACHABLE、目标段不发 path、未进入跟踪

用法：
    python3 verify_n3_goal_adjust.py [--adjust <批次>] [--blocked <批次>]
输出：experiments/agc_runs/verify_n3_goal_adjust.json，任一 FAIL 退出码 1。
"""
import argparse
import glob
import json
import os
import sys

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
WORKSPACE = os.path.abspath(os.path.join(SCRIPT_DIR, "..", "..", ".."))
RUNS_ROOT = os.path.join(WORKSPACE, "experiments", "agc_runs")
CONFIG = os.path.join(WORKSPACE, "src", "air_ground_core", "config", "agc_params.yaml")
OUT_PATH = os.path.join(RUNS_ROOT, "verify_n3_goal_adjust.json")

STATE_TRACKING_SUCCESS = 4
STATE_PLAN_UNREACHABLE = 5
# 目标段起始时刻：路口场景在首个非零目标到达前有一次初始规划（t≈0.2s），
# 之后 t>0.5s 的规划均针对被围目标。
TARGET_SEG_T = 0.5


def load_jsonl(path):
    out = []
    try:
        with open(path, errors="ignore") as f:
            for line in f:
                line = line.strip()
                if line:
                    out.append(json.loads(line))
    except OSError:
        pass
    return out


def load_json(path):
    try:
        with open(path) as f:
            return json.load(f)
    except (OSError, ValueError):
        return None


def run_of_batch(batch_dir):
    b = load_json(os.path.join(batch_dir, "batch.json"))
    if not isinstance(b, dict):
        return None, None
    for c in b.get("cycles", []):
        for a in c.get("attempts", []):
            return a.get("run_dir"), b
    return None, b


def check(checks, cid, ok, detail):
    checks.append({"id": cid, "pass": bool(ok), "detail": detail})


def params(checks):
    """读取参数并核对保护阈值未被降低（N3 不放松 protection）。"""
    try:
        import yaml
        with open(CONFIG) as f:
            doc = yaml.safe_load(f) or {}
        clr = doc["agc"]["clearance"]
        pl = doc["agc"]["planner"]
        protection = float(clr["protection_m"])
        inflation = float(pl["inflation_m"])
        adjust = float(pl["goal_adjust_max_m"])
        res = float(pl["grid_resolution_m"])
        dilate = max(inflation, protection)
        check(checks, "P0", abs(protection - 0.80) <= 1e-9 and dilate >= protection - 1e-9,
              "protection=%.2f inflation=%.2f dilate=max=%.2f adjust_max=%.2f res=%.2f"
              % (protection, inflation, dilate, adjust, res))
        return {"protection_m": protection, "inflation_m": inflation,
                "dilate_m": dilate, "adjust_max_m": adjust, "res_m": res}
    except Exception as exc:  # noqa: BLE001
        check(checks, "P0", False, "读取 agc_params 失败: %s" % exc)
        return None


def planner_rows(run_dir):
    return load_jsonl(os.path.join(run_dir, "planner_log.jsonl"))


def path_rows(run_dir):
    # agc_run_logger 把 '/' 替换为 '_'，故 /agc/ground/path -> _agc_ground_path.jsonl
    return load_jsonl(os.path.join(run_dir, "topics", "_agc_ground_path.jsonl"))


def final_state(run_dir):
    s = load_json(os.path.join(run_dir, "summary.json")) or {}
    return s.get("final_state"), s.get("final_state_name")


def terminal_reason(run_dir):
    """state_events.jsonl 中终态迁移的 reason（state=5 MISSION_FAILED 的 reason）。"""
    reason = None
    for e in load_jsonl(os.path.join(run_dir, "state_events.jsonl")):
        if e.get("state") == STATE_PLAN_UNREACHABLE or e.get("to") == "MISSION_FAILED":
            reason = e.get("reason") or reason
    return reason


def analyze_reachable(checks, batch_dir, P):
    """Group A：可达但安全接近点格被阻塞，应就近取可行格并成功跟踪。"""
    if not batch_dir or not os.path.isdir(batch_dir):
        check(checks, "A1", False, "批次目录不可用: %s" % batch_dir)
        return
    run_dir, batch = run_of_batch(batch_dir)
    run_dir = run_dir or ""
    check(checks, "A1",
          isinstance(batch, dict) and batch.get("any_leftover") is False,
          "any_leftover=%s" % (batch.get("any_leftover") if isinstance(batch, dict) else None))

    rows = [r for r in planner_rows(run_dir) if r.get("trigger") != "invalid"]
    succ = [r for r in rows if r.get("success")]
    adj = [r for r in succ if r.get("goal_adjusted")]
    # goal_adjusted=true 说明原始目标格确实被膨胀图阻塞，N3 走了就近可行格分支
    check(checks, "A2", bool(succ) and len(adj) == len(succ),
          "成功规划=%d，其中 goal_adjusted=true=%d（期望全部命中可行化分支）"
          % (len(succ), len(adj)))
    no_feas = [r for r in rows if r.get("failure_reason") == "goal_cell_blocked_no_feasible"]
    check(checks, "A3", not no_feas,
          "goal_cell_blocked_no_feasible 失败数=%d（可达场景不应出现）" % len(no_feas))

    paths = path_rows(run_dir)
    check(checks, "A4", len(paths) > 1, "/agc/ground/path 发布条数=%d（期望 >1）" % len(paths))

    st, name = final_state(run_dir)
    check(checks, "A5", st == STATE_TRACKING_SUCCESS,
          "终态=%s(%s)（期望 %d TRACKING_SUCCESS）" % (st, name, STATE_TRACKING_SUCCESS))

    tf = load_jsonl(os.path.join(run_dir, "tracking_frames.jsonl"))
    clr = [f.get("clearance_m") for f in tf if isinstance(f.get("clearance_m"), (int, float))]
    prot = P["protection_m"] if P else 0.80
    check(checks, "A6", bool(clr) and min(clr) >= prot - 1e-9,
          "跟踪帧=%d 最小净空=%.3f（保护阈值 %.2f）"
          % (len(tf), min(clr) if clr else -1, prot))


def analyze_blocked(checks, batch_dir):
    """Group B：目标被墙围住确实无路，应持续失败并判 PLAN_UNREACHABLE。"""
    if not batch_dir or not os.path.isdir(batch_dir):
        check(checks, "B1", False, "批次目录不可用: %s" % batch_dir)
        return
    run_dir, batch = run_of_batch(batch_dir)
    run_dir = run_dir or ""
    check(checks, "B1",
          isinstance(batch, dict) and batch.get("any_leftover") is False,
          "any_leftover=%s" % (batch.get("any_leftover") if isinstance(batch, dict) else None))

    rows = [r for r in planner_rows(run_dir) if r.get("trigger") != "invalid"]
    seg = [r for r in rows if (r.get("t_sim") or 0.0) > TARGET_SEG_T]
    legal = {"astar_no_path", "goal_cell_blocked_no_feasible"}
    all_fail = bool(seg) and all(not r.get("success") for r in seg)
    reasons = sorted({r.get("failure_reason") for r in seg})
    check(checks, "B2", all_fail and all(x in legal for x in reasons),
          "目标段规划=%d 全失败=%s reason=%s（期望 ∈ %s）"
          % (len(seg), all_fail, reasons, sorted(legal)))

    st, name = final_state(run_dir)
    reason = terminal_reason(run_dir)
    check(checks, "B3", st == STATE_PLAN_UNREACHABLE and reason == "PLAN_UNREACHABLE",
          "终态=%s(%s) reason=%s（期望 state=%d MISSION_FAILED / reason=PLAN_UNREACHABLE）"
          % (st, name, reason, STATE_PLAN_UNREACHABLE))

    late = [p for p in path_rows(run_dir) if (p.get("t_sim") or 0.0) > TARGET_SEG_T]
    check(checks, "B4", not late,
          "目标段 path 发布条数=%d（期望 0，确实无路不发路径）" % len(late))

    tf = load_jsonl(os.path.join(run_dir, "tracking_frames.jsonl"))
    check(checks, "B5", not tf, "tracking_frames 条数=%d（期望 0，未进入跟踪）" % len(tf))


def main():
    ap = argparse.ArgumentParser(description="N3 目标格可行化核验")
    ap.add_argument("--adjust", default="", help="Group A 可达贴障批次目录")
    ap.add_argument("--blocked", default="", help="Group B 确实无路批次目录")
    args = ap.parse_args()

    def pick(prefix):
        c = sorted(glob.glob(os.path.join(RUNS_ROOT, "_n3_%s_*" % prefix)))
        c = [x for x in c if os.path.isfile(os.path.join(x, "batch.json"))]
        return c[-1] if c else ""

    checks = []
    P = params(checks)
    adj_batch = args.adjust or pick("goal_adjust")
    blk_batch = args.blocked or pick("truly_blocked")
    analyze_reachable(checks, adj_batch, P)
    analyze_blocked(checks, blk_batch)

    npass = sum(1 for c in checks if c["pass"])
    nfail = len(checks) - npass
    out = {"contract_version": "agc-m0-v1.2",
           "batches": {"goal_adjust": adj_batch, "truly_blocked": blk_batch},
           "params": P, "checks": checks, "PASS": npass, "FAIL": nfail,
           "result": "PASS" if nfail == 0 else "FAIL"}
    with open(OUT_PATH, "w") as f:
        json.dump(out, f, ensure_ascii=False, indent=2)
    for c in checks:
        print("%s  %-4s %s" % ("PASS" if c["pass"] else "FAIL", c["id"], c["detail"]))
    print("-" * 72)
    print("汇总: PASS=%d FAIL=%d" % (npass, nfail))
    print("verify_n3_goal_adjust.json 已写入: %s" % OUT_PATH)
    sys.exit(0 if nfail == 0 else 1)


if __name__ == "__main__":
    main()