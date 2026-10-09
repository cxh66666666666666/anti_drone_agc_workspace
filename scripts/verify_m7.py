#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""M7 单体性能与三维避障边界 —— 离线核验（Python）。

核对 experiments/m7 下的权威证据：
  A 交付完整性（无人机三集、地面 val/test、AGC LQR、AGC 地面）
  B 固定场景集（manifest/场景哈希与场景数）
  C 三维动作可用性（竖直动作率>0 且不破坏基础跟踪）
  D 安全触发与失败率口径
  E 失败分母口径
输出 verify_m7.json；任一 FAIL 退出码 1。

用法:
  python3 scripts/verify_m7.py --root experiments/m7
"""

import argparse
import json
import os

CHECKS = []


def check(cid, ok, detail=""):
    CHECKS.append({"id": cid, "status": "PASS" if ok else "FAIL",
                   "detail": detail})
    return ok


def load_json(path):
    try:
        with open(path, encoding="utf-8") as stream:
            return json.load(stream)
    except (OSError, ValueError):
        return None


def latest_dir(root, kind):
    base = os.path.join(root, kind)
    if not os.path.isdir(base):
        return None
    subs = sorted(d for d in os.listdir(base)
                  if os.path.isdir(os.path.join(base, d)))
    return os.path.join(base, subs[-1]) if subs else None


def check_drone(root):
    run = latest_dir(root, "drone")
    check("A1.1", run is not None, "drone run dir: %s" % run)
    if not run:
        return
    for split in ("train", "validation", "test"):
        d = os.path.join(run, split)
        m = load_json(os.path.join(d, "metrics.json"))
        meta = load_json(os.path.join(d, "run_meta.json"))
        has_ep = os.path.isfile(os.path.join(d, "episodes.jsonl"))
        has_traj = os.path.isdir(os.path.join(d, "trajectories"))
        check("A1.%s" % split, bool(m) and has_ep and has_traj,
              "metrics=%s episodes=%s trajectories=%s" % (bool(m), has_ep,
                                                          has_traj))
        if not m:
            continue
        # 失败分母口径：分母=场景数；valid + reset_failure == episodes
        reset_fail = round(m.get("reset_failure_rate", 0.0) * m["episodes"])
        check("E.%s" % split,
              abs(m.get("failure_denominator", -1) - m["episodes"]) < 1e-6
              and m["valid_episodes"] + reset_fail == m["episodes"],
              "denom=%s episodes=%s valid=%s reset_fail=%s"
              % (m.get("failure_denominator"), m["episodes"],
                 m["valid_episodes"], reset_fail))
        # 安全触发/失败率在 [0,1]
        rates = [m["success_rate"], m["collision_rate"], m["timeout_rate"],
                 m["shield_infeasible_rate"], m["reset_failure_rate"]]
        check("D.%s" % split, all(0.0 <= r <= 1.0 for r in rates),
              "rates=%s" % [round(r, 3) for r in rates])
        # 固定场景：run_meta 记录 manifest 哈希与场景 id
        if meta:
            check("B.%s" % split,
                  bool(meta.get("manifest_sha256"))
                  and len(meta.get("scenario_ids", [])) == m["episodes"],
                  "manifest_sha=%s ids=%d"
                  % (str(meta.get("manifest_sha256"))[:12],
                     len(meta.get("scenario_ids", []))))
        # 三维动作可用性
        check("C.%s.rate" % split, m.get("vertical_action_rate", 0.0) > 0.0,
              "vertical_action_rate=%.4f" % m.get("vertical_action_rate", 0.0))

    # 跨集：从 episodes.jsonl 重算竖直动作可用性与"不破坏基础跟踪"判据
    # 判据：竖直动作回合终态未发散(diverged/low_altitude)，且误差峰值≤发散阈值10m
    def _jsonl(path):
        rows = []
        try:
            with open(path, encoding="utf-8") as stream:
                for line in stream:
                    line = line.strip()
                    if line:
                        rows.append(json.loads(line))
        except (OSError, ValueError):
            pass
        return rows

    any_avail = False
    v_eps = 0
    v_ok = 0
    for split in ("train", "validation", "test"):
        rows = _jsonl(os.path.join(run, split, "episodes.jsonl"))
        for r in rows:
            if not r.get("reset_ok") or r.get("vertical_action_count", 0) <= 0:
                continue
            v_eps += 1
            if (r.get("done_reason") not in ("diverged", "low_altitude")
                    and r.get("max_tracking_error", 1e9) <= 10.0):
                v_ok += 1
        if rows and any(r.get("vertical_action_count", 0) > 0 for r in rows):
            any_avail = True
    check("C.avail", any_avail, "任一 split 出现竖直动作")
    check("C.preserve", v_eps > 0 and v_ok == v_eps,
          "竖直回合=%d 未破坏跟踪=%d" % (v_eps, v_ok))


def check_ground(root):
    gr = load_json(os.path.join(root, "ground", "ground_report.json"))
    check("A3.ground_report", bool(gr), "ground_report.json")
    if not gr:
        return
    check("B3.split_sha", bool(gr.get("scenario_split_sha256")),
          "sha=%s" % str(gr.get("scenario_split_sha256"))[:12])
    for split in ("validation", "test"):
        m = gr.get(split) or {}
        metrics_file = os.path.join(root, "ground", split, "metrics.json")
        has_traj = os.path.isdir(os.path.join(root, "ground", split,
                                              "trajectories"))
        check("A3.%s" % split, bool(m) and os.path.isfile(metrics_file)
              and has_traj, "metrics=%s traj=%s" % (bool(m), has_traj))
        denom = m.get("failure_denominator")
        reset_fail = round(m.get("reset_failure_rate", 0.0) * denom) if denom else -1
        check("E3.%s" % split,
              denom is not None and abs(denom - m.get("episodes", -1)) < 1e-6
              and m.get("valid_episodes", -1) + reset_fail == m.get("episodes"),
              "denom=%s episodes=%s valid=%s reset_fail=%s"
              % (denom, m.get("episodes"), m.get("valid_episodes"),
                 reset_fail))


def check_agc(root):
    lqr = load_json(os.path.join(root, "agc", "lqr",
                                 "lqr_tracking_report.json"))
    check("A4.lqr_report", bool(lqr), "lqr_tracking_report.json")
    if lqr:
        for name, e in lqr.get("trajectories", {}).items():
            check("A4.%s" % name, e.get("n", 0) > 0,
                  "n=%s mean=%s rms=%s" % (e.get("n"), e.get("mean_m"),
                                           e.get("rms_m")))
    agc = load_json(os.path.join(root, "agc", "ground",
                                 "agc_ground_report.json"))
    check("A5.agc_ground_report", bool(agc), "agc_ground_report.json")
    if agc:
        runs = agc.get("runs", [])
        check("A5.runs", len(runs) >= 1, "runs=%d" % len(runs))
        for r in runs:
            check("A5.tracking.%s" % r.get("run_dir"),
                  r.get("tracking_frames", 0) > 0
                  and r.get("tracking_error", {}).get("n", 0) > 0,
                  "frames=%s err_n=%s min_clear=%s"
                  % (r.get("tracking_frames"),
                     r.get("tracking_error", {}).get("n"),
                     r.get("min_clearance_m")))


def main():
    parser = argparse.ArgumentParser(description="M7 离线核验")
    parser.add_argument("--root", default="experiments/m7")
    parser.add_argument("--out", default=None)
    args = parser.parse_args()
    root = os.path.abspath(args.root)

    check_drone(root)
    check_ground(root)
    check_agc(root)

    passed = sum(1 for c in CHECKS if c["status"] == "PASS")
    failed = sum(1 for c in CHECKS if c["status"] == "FAIL")
    result = {"checks": CHECKS, "summary": {"pass": passed, "fail": failed}}
    out = args.out or os.path.join(root, "verify_m7.json")
    with open(out, "w", encoding="utf-8") as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2)
        stream.write("\n")
    for c in CHECKS:
        print("[%s] %s %s" % (c["status"], c["id"], c["detail"]))
    print("== PASS=%d FAIL=%d ==" % (passed, failed))
    raise SystemExit(1 if failed else 0)


if __name__ == "__main__":
    main()