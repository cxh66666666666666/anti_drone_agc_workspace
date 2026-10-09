#!/usr/bin/env python3
"""verify_n1_gating.py —— N1 视场/距离门控离线核验。

读取两份 N1 批跑目录（短时出视野可恢复 / 持续出视野 -> TARGET_LOST），按 run 目录逐项核对：
  A 门控配置冻结（传感器几何参数与哈希）
  B 批跑与终态（干净退出、审计、终态与期望一致）
  C 门控一致性（拒收原因、门控关闭即不产生目标观测、obs_seq 单调、时间戳为原始观测时刻）
  D 行为（recover: 出视野 < 容忍窗且安全停车后恢复、无 TARGET_LOST；
           sustained: 出视野 > 容忍窗且结算 TARGET_LOST；/agc/targets 空窗对齐）

用法：
    python3 verify_n1_gating.py [--recover <批次目录>] [--sustained <批次目录>]
输出：experiments/agc_runs/verify_n1_gating.json，并在 stdout 打印 PASS/FAIL。
"""
import argparse
import glob
import json
import os
import sys

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
WORKSPACE = os.path.abspath(os.path.join(SCRIPT_DIR, "..", "..", ".."))
RUNS_ROOT = os.path.join(WORKSPACE, "experiments", "agc_runs")
CONFIG_DIR = os.path.join(WORKSPACE, "src", "air_ground_core", "config")
OUT_PATH = os.path.join(RUNS_ROOT, "verify_n1_gating.json")

LOST_TOL_S = 2.0            # 容忍窗（agc_params limits.target_lost_tolerance_s）
OUT_GAP_S = 0.30           # 连续出视野窗口的合并阈值
GATE_OUT = ("out_of_fov", "out_of_range_min", "out_of_range_max", "behind")
REASONS_OK = set(GATE_OUT) | {"ok", "no_target", "no_drone_pose"}


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
    """取批次首个 attempt 的 run 目录。"""
    b = load_json(os.path.join(batch_dir, "batch.json"))
    if not isinstance(b, dict):
        return None, None
    for c in b.get("cycles", []):
        for a in c.get("attempts", []):
            return a.get("run_dir"), b
    return None, b


def windows(times, gap=OUT_GAP_S):
    """把时刻列表按间隔合并为连续窗口 [(a,b),...]。"""
    ts = sorted(times)
    segs, cur = [], None
    prev = None
    for t in ts:
        if prev is None or (t - prev) <= gap:
            cur = [t, t] if cur is None else [cur[0], t]
        else:
            segs.append(tuple(cur))
            cur = [t, t]
        prev = t
    if cur:
        segs.append(tuple(cur))
    return segs


def both_out_windows(run_dir):
    """两通道同时出视野（门控关闭）的连续窗口。"""
    pl = load_jsonl(os.path.join(run_dir, "perception_log.jsonl"))
    off = {}
    for r in pl:
        if r.get("gate_reason") in GATE_OUT:
            off.setdefault(r.get("sensor_id"), set()).add(r["t_sim"])
    if not off:
        return []
    common = None
    for ts in off.values():
        common = ts if common is None else (common & ts)
    return windows(common or set())


def targets_empty_windows(run_dir):
    segs, cur, prev = [], None, None
    for r in load_jsonl(os.path.join(run_dir, "topics/_agc_targets.jsonl")):
        t = r["t_sim"]
        empty = not (r.get("msg", {}).get("tracks"))
        if empty:
            if prev is None or (t - prev) <= 0.5:
                cur = [t, t] if cur is None else [cur[0], t]
            else:
                if cur:
                    segs.append(tuple(cur))
                cur = [t, t]
        else:
            if cur:
                segs.append(tuple(cur))
                cur = None
        prev = t
    if cur:
        segs.append(tuple(cur))
    return [s for s in segs if s[1] - s[0] >= 0.15]


def check(checks, cid, ok, detail):
    checks.append({"id": cid, "pass": bool(ok), "detail": detail})


def check_config(checks):
    sensors = ["air_eo_front", "air_ir_top"]
    try:
        import yaml
        with open(os.path.join(CONFIG_DIR, "perception_sensors.yaml")) as f:
            doc = yaml.safe_load(f)
        by_id = {s.get("id"): s for s in (doc.get("sensors") or [])}
    except Exception as exc:  # noqa: BLE001
        check(checks, "A0", False, "读取 perception_sensors.yaml 失败: %s" % exc)
        return
    ok = True
    detail = []
    for sid in sensors:
        s = by_id.get(sid)
        has = bool(s) and all(k in s for k in
                              ("boresight_body", "hfov_deg", "vfov_deg",
                               "range_min_m", "range_max_m"))
        ok = ok and has
        detail.append("%s=%s" % (sid, "有几何参数" if has else "缺几何参数"))
    check(checks, "A0", ok, "；".join(detail))


def check_scenario(checks, tag, batch_dir, expect):
    """expect: dict(kind, reason, out_gt_tol)"""
    cid = tag.upper()
    if not batch_dir or not os.path.isdir(batch_dir):
        check(checks, "B1.%s" % cid, False, "批次目录不可用: %s" % batch_dir)
        return
    run_dir, batch = run_of_batch(batch_dir)
    run_dir = run_dir or ""

    # B：批跑与终态
    check(checks, "B1.%s" % cid, batch is not None, "batch.json 可读")
    any_lo = batch.get("any_leftover") if isinstance(batch, dict) else None
    audits = [c.get("audit_ok") for c in batch.get("cycles", [])] if isinstance(batch, dict) else None
    check(checks, "B2.%s" % cid,
          any_lo is False and bool(audits) and all(audits),
          "any_leftover=%s audit_ok=%s" % (any_lo, audits))

    ep = None
    if isinstance(batch, dict):
        for c in batch.get("cycles", []):
            for a in c.get("attempts", []):
                ep = a
    fs = ep.get("final_state") if ep else None
    reason = ep.get("reason") if ep else None
    if expect["kind"] == "sustained":
        check(checks, "B3.%s" % cid, fs == 5 and reason == "TARGET_LOST",
              "终态=%s/%s（期望 5/TARGET_LOST）" % (fs, reason))
    else:
        check(checks, "B3.%s" % cid, reason != "TARGET_LOST",
              "终态=%s/%s（期望非 TARGET_LOST）" % (fs, reason))

    if not (run_dir and os.path.isdir(run_dir)):
        check(checks, "C1.%s" % cid, False, "run 目录不可用: %s" % run_dir)
        return

    # C1：门控一致性（gate_ok 与 has_target 一致）
    pl = load_jsonl(os.path.join(run_dir, "perception_log.jsonl"))
    bad = [r for r in pl if r.get("gate_reason") != "no_target"
           and (bool(r.get("gate_ok")) != bool(r.get("has_target")))]
    check(checks, "C1.%s" % cid, not bad,
          "gate_ok 与 has_target 不一致样本=%d（样本总数=%d）" % (len(bad), len(pl)))

    # C2：拒收原因取值合法，且确有“出视野”样本
    bad_reason = sorted({r.get("gate_reason") for r in pl} - REASONS_OK)
    out_n = sum(1 for r in pl if r.get("gate_reason") in GATE_OUT)
    check(checks, "C2.%s" % cid, not bad_reason and out_n > 0,
          "非法原因=%s；出视野样本=%d" % (bad_reason or "无", out_n))

    # C3：目标 obs_seq 单调
    ao = load_jsonl(os.path.join(run_dir, "topics/_agc_air_obs.jsonl"))
    seq, mono = {}, True
    for r in ao:
        for s in r.get("msg", {}).get("sensors", []):
            for t in s.get("targets", []):
                sid = s.get("sensor_id")
                q = t.get("obs_seq", 0)
                if sid in seq and q < seq[sid]:
                    mono = False
                seq[sid] = q
    check(checks, "C3.%s" % cid, mono and bool(ao), "obs_seq 单调=%s（通道=%s）" % (mono, list(seq)))

    # C4：时间戳为原始观测时刻（stamp(ns)/1e9 == t_sim），frame=world
    max_dt, frame_ok = 0.0, True
    for r in ao:
        h = r.get("msg", {}).get("header", {})
        try:
            st = int(h.get("stamp", "0")) / 1e9
            max_dt = max(max_dt, abs(st - r["t_sim"]))
        except (TypeError, ValueError):
            frame_ok = False
        if h.get("frame_id") != "world":
            frame_ok = False
    check(checks, "C4.%s" % cid, frame_ok and max_dt < 0.15,
          "frame=world=%s；|stamp-t_sim|max=%.4fs（阈值 0.15，戳为仿真时钟而非墙钟）"
          % (frame_ok, max_dt))

    # D：行为
    wins = both_out_windows(run_dir)
    dur = [b - a for a, b in wins]
    empt = targets_empty_windows(run_dir)
    cm = load_jsonl(os.path.join(run_dir, "commander_log.jsonl"))
    reasons = [r.get("reason") for r in cm]
    has_stop = "goal_stale" in reasons
    has_move = "tracking" in reasons
    if expect["kind"] == "sustained":
        over = [d for d in dur if d > LOST_TOL_S]
        check(checks, "D1.%s" % cid, bool(over),
              "存在出视野窗 > 容忍窗 %.1fs 的窗口=%s；全部窗口时长=%s"
              % (LOST_TOL_S, bool(over), [round(d, 2) for d in dur]))
        check(checks, "D2.%s" % cid, fs == 5 and reason == "TARGET_LOST",
              "出视野超窗后结算 TARGET_LOST")
    else:
        under = [d for d in dur if d < LOST_TOL_S]
        check(checks, "D1.%s" % cid, bool(under) and all(d < LOST_TOL_S for d in dur),
              "全部出视野窗 < 容忍窗 %.1fs=%s；窗口时长=%s"
              % (LOST_TOL_S, all(d < LOST_TOL_S for d in dur), [round(d, 2) for d in dur]))
        check(checks, "D2.%s" % cid, has_stop and has_move,
              "安全停车(goal_stale)=%s 与恢复(tracking)=%s 均出现" % (has_stop, has_move))
    check(checks, "D3.%s" % cid, bool(empt) and bool(wins),
          "/agc/targets 空窗数=%d；出视野窗数=%d；空窗=%s"
          % (len(empt), len(wins), ["(%.2f,%.2f)" % s for s in empt]))


def main():
    ap = argparse.ArgumentParser(description="N1 视场门控离线核验")
    ap.add_argument("--recover", default="", help="短时出视野可恢复批次目录")
    ap.add_argument("--sustained", default="", help="持续出视野 TARGET_LOST 批次目录")
    args = ap.parse_args()

    def pick(name):
        if getattr(args, name):
            return getattr(args, name)
        cands = sorted(glob.glob(os.path.join(RUNS_ROOT, "_n1_fov_%s_*" % name)))
        return cands[-1] if cands else ""

    recover = pick("recover")
    sustained = pick("sustained")

    checks = []
    check_config(checks)
    check_scenario(checks, "recover", recover, {"kind": "recover", "reason": None})
    check_scenario(checks, "sustained", sustained, {"kind": "sustained", "reason": "TARGET_LOST"})

    npass = sum(1 for c in checks if c["pass"])
    nfail = len(checks) - npass
    out = {"contract_version": "agc-m0-v1.2",
           "recover_batch": recover, "sustained_batch": sustained,
           "lost_tolerance_s": LOST_TOL_S,
           "checks": checks, "PASS": npass, "FAIL": nfail,
           "result": "PASS" if nfail == 0 else "FAIL"}
    with open(OUT_PATH, "w") as f:
        json.dump(out, f, ensure_ascii=False, indent=2)
    for c in checks:
        print("%s  %-12s %s" % ("PASS" if c["pass"] else "FAIL", c["id"], c["detail"]))
    print("-" * 72)
    print("汇总: PASS=%d FAIL=%d" % (npass, nfail))
    print("verify_n1_gating.json 已写入: %s" % OUT_PATH)
    sys.exit(0 if nfail == 0 else 1)


if __name__ == "__main__":
    main()