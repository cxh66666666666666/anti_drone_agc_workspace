#!/usr/bin/env python3
"""verify_n2_follow.py —— N2 无人机跟随移动目标（安全距离带）离线核验。

读取 N2 三份批跑目录（静止 / 匀速 / 失联+恢复），逐项核对：
  A 跟随参数冻结与 setpoint 唯一发布者
  B 安全带行为（带内悬停 / 过远跟近 / 过近退离；模式与距离一致；无持续摆动）
  C 高度保持与参考跳变限幅
  D 失联冻结（不追旧目标）与恢复跟随
  E 干净退出

用法：
    python3 verify_n2_follow.py [--static <批次>] [--linear <批次>] [--loss <批次>]
输出：experiments/agc_runs/verify_n2_follow.json，并在 stdout 打印 PASS/FAIL。
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
OUT_PATH = os.path.join(RUNS_ROOT, "verify_n2_follow.json")

DIST_TOL = 0.5      # 模式/距离一致的额外容差（跟踪滞后）
ALT_TOL = 0.30      # 高度保持容差


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


def follow_params(checks):
    try:
        import yaml
        with open(CONFIG) as f:
            doc = yaml.safe_load(f) or {}
        p = doc["agc"]["air"]["follow"]
    except Exception as exc:  # noqa: BLE001
        check(checks, "A0", False, "读取 acg_params follow 段失败: %s" % exc)
        return {}
    ok = all(k in p for k in ("safe_min_m", "safe_max_m", "hysteresis_m", "altitude_m",
                              "ref_max_age_s", "confidence_min", "max_step_m")) and \
        p["safe_min_m"] < p["safe_max_m"]
    check(checks, "A0", ok, "follow 参数=%s" % p)
    return p


def authority_info(run_dir):
    """从 air_control_authority.jsonl 取 setpoint 实际发布者与控制权核查（M5 台账，N2 复用）。"""
    recs = load_jsonl(os.path.join(run_dir, "air_control_authority.jsonl"))
    lqr = [r for r in recs
           if r.get("active_controller") == "lqr_tracker_node"
           and "lqr_tracker_node" in (r.get("setpoint_publishers") or [])]
    if not lqr:
        return None
    return {"active": lqr[-1].get("active_controller"),
            "max_gap": max(r.get("setpoint_gap_s", 0.0) for r in lqr),
            "conflict": any(r.get("control_conflict") for r in recs),
            "pubs": lqr[-1].get("setpoint_publishers")}


def analyze(checks, tag, batch_dir, P, kind):
    cid = tag.upper()
    if not batch_dir or not os.path.isdir(batch_dir):
        check(checks, "B1.%s" % cid, False, "批次目录不可用: %s" % batch_dir)
        return
    run_dir, batch = run_of_batch(batch_dir)
    run_dir = run_dir or ""
    fl = load_jsonl(os.path.join(run_dir, "air_follow_log.jsonl"))
    check(checks, "B1.%s" % cid, bool(fl), "air_follow_log 条数=%d" % len(fl))
    check(checks, "A1.%s" % cid,
          isinstance(batch, dict) and batch.get("any_leftover") is False,
          "any_leftover=%s" % (batch.get("any_leftover") if isinstance(batch, dict) else None))

    # setpoint 唯一实际发布者 + 无控制空窗（复用 M5 控制权台账）
    auth = authority_info(run_dir)
    gap_max = 0.2  # /agc/air/setpoint_gap_max_s
    check(checks, "A2.%s" % cid,
          bool(auth) and auth["active"] == "lqr_tracker_node" and not auth["conflict"]
          and auth["max_gap"] <= gap_max,
          "实际控制器=%s 控制冲突=%s 最大空窗=%.3fs（期望 lqr_tracker_node/False/≤%.2f）"
          % (auth and auth["active"], auth and auth["conflict"],
             auth["max_gap"] if auth else -1, gap_max))
    nodes = ((load_json(os.path.join(run_dir, "summary.json")) or {}).get("nodes") or [])
    check(checks, "A3.%s" % cid, "/air_follow" in nodes,
          "air_follow 在节点清单=%s" % ("/air_follow" in nodes))
    if not fl:
        return

    smin, smax = P["safe_min_m"], P["safe_max_m"]
    hyst, alt = P["hysteresis_m"], P["altitude_m"]
    valid = [r for r in fl if r.get("valid")]
    modes = [r.get("band_mode") for r in fl]

    # 模式与距离一致（valid 行）
    bad_mode = [r for r in valid if
                (r["band_mode"] == "approach" and r["distance_m"] < smax - hyst - DIST_TOL) or
                (r["band_mode"] == "retreat" and r["distance_m"] > smin + hyst + DIST_TOL) or
                (r["band_mode"] == "hold_in_band" and
                 not (smin - DIST_TOL <= r["distance_m"] <= smax + DIST_TOL))]
    check(checks, "B2.%s" % cid, not bad_mode,
          "模式/距离不一致样本=%d（valid=%d）" % (len(bad_mode), len(valid)))

    # 高度保持
    bad_alt = [r for r in valid if abs(r["drone_z"] - alt) > ALT_TOL]
    zs = [r["drone_z"] for r in valid]
    check(checks, "C1.%s" % cid, bool(valid) and not bad_alt,
          "valid 高度 min/max=%.2f/%.2f（期望 %.2f±%.2f）"
          % (min(zs), max(zs), alt, ALT_TOL) if zs else "无 valid 样本")

    # 无 approach<->retreat 抖动
    seq = [m for m in modes if m in ("approach", "retreat", "hold_in_band")]
    chatter = sum(1 for a, b in zip(seq, seq[1:])
                  if {a, b} == {"approach", "retreat"})
    check(checks, "B3.%s" % cid, chatter == 0, "approach/retreat 抖动次数=%d" % chatter)

    # 参考跳变限幅：相邻参考位移 <= max_step + eps
    maxref = 0.0
    prev = None
    for r in fl:
        if prev is not None:
            maxref = max(maxref, ((r["ref_x"] - prev["ref_x"]) ** 2 +
                                  (r["ref_y"] - prev["ref_y"]) ** 2) ** 0.5)
        prev = r
    check(checks, "C2.%s" % cid, maxref <= P["max_step_m"] + 1e-6,
          "相邻参考最大位移=%.3f（限幅 %.2f）" % (maxref, P["max_step_m"]))

    # 行为覆盖
    if kind == "static":
        lastv = valid[-1]["distance_m"] if valid else -1
        check(checks, "D1.%s" % cid, smin - DIST_TOL <= lastv <= smax + DIST_TOL,
              "带内悬停：末个 valid 距离=%.2f（期望 [%.2f,%.2f]）" % (lastv, smin, smax))
    elif kind == "linear":
        has_hold = "hold_in_band" in modes
        has_appr = "approach" in modes
        check(checks, "D1.%s" % cid, has_hold and has_appr,
              "匀速/转向：hold_in_band=%s approach=%s" % (has_hold, has_appr))
    elif kind == "loss":
        # 失联窗口：valid=false 且目标年龄超限，任务仍运行（hold_no_target）
        loss = [r for r in fl if (not r.get("valid")) and r.get("band_mode") == "hold_no_target"]
        check(checks, "D2.%s" % cid, bool(loss),
              "失联冻结样本=%d（hold_no_target）" % len(loss))
        # 失联期间无人机水平位移小（不追旧目标）
        if loss:
            t0, t1 = loss[0]["t_sim"], loss[-1]["t_sim"]
            win = [r for r in fl if t0 <= r["t_sim"] <= t1]
            dx = max(r["drone_x"] for r in win) - min(r["drone_x"] for r in win)
            dy = max(r["drone_y"] for r in win) - min(r["drone_y"] for r in win)
            check(checks, "D3.%s" % cid, max(dx, dy) <= 1.0,
                  "失联窗(%.1f~%.1f)无人机位移=(%.2f,%.2f)（期望≤1.0）" % (t0, t1, dx, dy))
        # 恢复：失联后再次出现 valid 样本
        if loss:
            t_last_loss = loss[-1]["t_sim"]
            recov = [r for r in fl if r["t_sim"] > t_last_loss and r.get("valid")]
            check(checks, "D4.%s" % cid, bool(recov),
                  "失联后恢复跟随样本=%d" % len(recov))


def main():
    ap = argparse.ArgumentParser(description="N2 跟随核验")
    for k in ("static", "linear", "loss"):
        ap.add_argument("--%s" % k, default="")
    args = ap.parse_args()

    def pick(name):
        v = getattr(args, name)
        if v:
            return v
        c = sorted(glob.glob(os.path.join(RUNS_ROOT, "_n2_%s_*" % name)))
        c = [x for x in c if os.path.isfile(os.path.join(x, "batch.json"))]
        return c[-1] if c else ""

    checks = []
    P = follow_params(checks)
    if P:
        analyze(checks, "static", pick("static"), P, "static")
        analyze(checks, "linear", pick("linear"), P, "linear")
        analyze(checks, "loss", pick("loss"), P, "loss")

    npass = sum(1 for c in checks if c["pass"])
    nfail = len(checks) - npass
    out = {"contract_version": "agc-m0-v1.2",
           "batches": {k: pick(k) for k in ("static", "linear", "loss")},
           "checks": checks, "PASS": npass, "FAIL": nfail,
           "result": "PASS" if nfail == 0 else "FAIL"}
    with open(OUT_PATH, "w") as f:
        json.dump(out, f, ensure_ascii=False, indent=2)
    for c in checks:
        print("%s  %-12s %s" % ("PASS" if c["pass"] else "FAIL", c["id"], c["detail"]))
    print("-" * 72)
    print("汇总: PASS=%d FAIL=%d" % (npass, nfail))
    print("verify_n2_follow.json 已写入: %s" % OUT_PATH)
    sys.exit(0 if nfail == 0 else 1)


if __name__ == "__main__":
    main()