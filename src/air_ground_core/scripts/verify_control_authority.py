#!/usr/bin/env python3
"""verify_control_authority.py —— M5 控制权/交接/安全状态离线核验（离线工具，非实时链路）。

用法：
    python3 verify_control_authority.py --run <run_dir> --mode nominal|loss|abort|conflict
可选：
    --out <json_path>   默认 <run_dir>/verify_control_authority.json

逐项核查（PASS/FAIL/SKIP）：
  A 唯一控制源：地面 /agc/ground/cmd_vel 与 /agc/task/state 唯一发布者符合契约；
    地面写入权台账（control_authority.jsonl）无外部 set_model_state 写入；
    空中控制权时间线（air_control_authority.jsonl）任务运行期无空窗冲突；
    任务状态中的 air_controller/ground_controller 与阶段一致（唯一实际控制者）。
  B 交接：handover_detected、交接空窗 ≤ setpoint_gap_max_s、mavros OFFBOARD、
    控制者由 takeoff_controller 持久移交 lqr_tracker_node。
  C 安全状态：终态后地面零速停稳（读取紧随其后的 run 目录驻留窗）、空中 hold 验证通过；
    失联场景停车在容忍窗内；abort 场景 OPERATOR_ABORT；冲突场景 CONTROL_CONFLICT
    且台账记录 external_pose_write。
"""
import argparse
import json
import math
import os
import sys

TERMINAL = {4, 5, 6, 8}
STATE_NAME = {0: "IDLE", 1: "AIR_SEARCH", 2: "TARGET_ACQUIRED", 3: "GROUND_APPROACH",
              4: "TRACKING_SUCCESS", 5: "MISSION_FAILED", 6: "ABORTED", 8: "INFRA_FAILURE"}
DEFAULT_GAP_MAX = 0.2
DEFAULT_LOST_TOL = 2.0
STOP_VX_MAX = 0.02
STOP_DISP_MAX = 0.10
DWELL_MAX_S = 5.0


def load_jsonl(path):
    out = []
    if not os.path.isfile(path):
        return out
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                out.append(json.loads(line))
            except Exception:
                pass
    return out


def load_json(path):
    if not os.path.isfile(path):
        return None
    try:
        with open(path) as f:
            return json.load(f)
    except Exception:
        return None


def find_post_run(run_dir, t_end):
    """按 t_sim 连续性找到紧随其后的 run 目录（终态后 logger 立即新开）。"""
    parent = os.path.dirname(run_dir)
    cands = []
    try:
        names = os.listdir(parent)
    except OSError:
        return None
    for d in names:
        if d.startswith("_") or d.startswith("."):
            continue
        full = os.path.join(parent, d)
        if not os.path.isdir(full):
            continue
        meta = load_json(os.path.join(full, "meta.json"))
        if not meta:
            continue
        ts = meta.get("t_sim_start")
        if ts is None:
            continue
        if ts >= t_end - 0.5:
            cands.append((ts, full))
    cands.sort()
    return cands[0][1] if cands else None


def main():
    ap = argparse.ArgumentParser(description="M5 控制权/交接/安全状态离线核验")
    ap.add_argument("--run", required=True, help="run 目录")
    ap.add_argument("--mode", required=True,
                    choices=["nominal", "loss", "abort", "conflict", "timeout", "handover"])
    ap.add_argument("--out", default="", help="输出 JSON 路径（默认 <run>/verify_control_authority.json）")
    args = ap.parse_args()

    run = os.path.abspath(args.run)
    checks = []

    def add(name, status, detail=""):
        checks.append({"name": name, "status": status, "detail": str(detail)})

    summary = load_json(os.path.join(run, "summary.json"))
    handover = load_json(os.path.join(run, "air_handover.json"))
    if summary is None:
        add("summary.json 可读", "FAIL", "缺失或非法")
        _finish(run, args.out, checks)
        return

    pubs = summary.get("publishers") or {}
    final_state = summary.get("final_state")
    final_reason = summary.get("final_reason_code")
    t_end = summary.get("t_sim_end")

    # ---------------- A 唯一控制源 ----------------
    cmdp = pubs.get("/agc/ground/cmd_vel")
    add("A1 地面命令唯一源 /agc/ground/cmd_vel",
        "PASS" if cmdp == ["/ground_commander"] else "FAIL",
        "publishers=%s expected=['/ground_commander']" % cmdp)
    tsp = pubs.get("/agc/task/state")
    add("A2 任务状态唯一源 /agc/task/state",
        "PASS" if tsp == ["/task_manager"] else "FAIL",
        "publishers=%s expected=['/task_manager']" % tsp)

    ledger = load_jsonl(os.path.join(run, "control_authority.jsonl"))
    if not ledger:
        add("A3 地面写入权台账存在", "FAIL", "control_authority.jsonl 缺失或为空")
    else:
        conflicts = [e for e in ledger if e.get("control_conflict")]
        if args.mode == "conflict":
            ext = [e for e in conflicts if "external_pose_write" in (e.get("conflict_reason") or "")]
            add("A3 地面写入权冲突已记录", "PASS" if ext else "FAIL",
                "冲突条数=%d，外部写入条数=%d" % (len(conflicts), len(ext)))
        else:
            add("A3 地面运行期无写入权冲突", "PASS" if not conflicts else "FAIL",
                "冲突条数=%d 明细=%s" % (
                    len(conflicts),
                    (conflicts[0].get("conflict_reason") if conflicts else "无")))

    air_led = load_jsonl(os.path.join(run, "air_control_authority.jsonl"))
    air_run = [e for e in air_led if e.get("task_running")]
    if not air_led:
        add("A4 空中控制权时间线", "SKIP", "air_control_authority.jsonl 缺失（旧 run）")
    elif args.mode == "handover":
        bad = [e for e in air_run if e.get("control_conflict")]
        add("A4 空中运行期控制权空窗被记录", "PASS" if bad else "FAIL",
            "运行期记录=%d 冲突=%d（handover 注入模式：空窗为预期）" % (len(air_run), len(bad)))
    else:
        bad = [e for e in air_run if e.get("control_conflict")]
        add("A4 空中运行期无控制权空窗冲突", "PASS" if not bad else "FAIL",
            "运行期记录=%d 冲突=%d" % (len(air_run), len(bad)))

    # 阶段-控制器字段一致性
    st = load_jsonl(os.path.join(run, "topics/_agc_task_state.jsonl"))
    gc_bad = 0
    ac_bad = 0
    for rec in st:
        m = rec.get("msg") or {}
        s = m.get("state")
        gc = m.get("ground_controller")
        ac = m.get("air_controller")
        want_gc = "ground_commander" if s in (3, 4) else "none"
        if gc != want_gc:
            gc_bad += 1
        if s in (1, 2, 3, 4) and ac != "lqr_tracker_node":
            ac_bad += 1
    add("A5 阶段唯一控制者字段一致", "PASS" if (gc_bad == 0 and ac_bad == 0) else "FAIL",
        "ground_controller 不一致=%d air_controller 不一致=%d (样本=%d)"
        % (gc_bad, ac_bad, len(st)))

    # ---------------- B 空中交接 ----------------
    if handover is None:
        add("B1 交接报告存在", "FAIL", "air_handover.json 缺失")
    else:
        add("B1 交接已检测", "PASS" if handover.get("handover_detected") else "FAIL",
            "handover_detected=%s" % handover.get("handover_detected"))
        gap = handover.get("handover_gap_s")
        gmax = handover.get("setpoint_gap_max_s_param", DEFAULT_GAP_MAX)
        ok_gap = (gap is not None) and (gap <= gmax)
        add("B2 交接无 setpoint 空窗", "PASS" if ok_gap else "FAIL",
            "handover_gap_s=%s ≤ %s" % (gap, gmax))
        if args.mode == "handover":
            add("B3 交接期 OFFBOARD", "SKIP", "handover 注入模式：交接后 PX4 退出 OFFBOARD")
        else:
            add("B3 交接期 OFFBOARD", "PASS" if handover.get("mavros_mode") == "OFFBOARD" else "FAIL",
                "mavros_mode=%s" % handover.get("mavros_mode"))
        owner = handover.get("active_controller")
        add("B4 空中控制者持久为 lqr_tracker_node",
            "PASS" if owner == "lqr_tracker_node" else "FAIL",
            "active_controller=%s" % owner)

    # B5（N4）：交接后 setpoint advertiser 不残留起飞控制器（release_control 后 shutdown）
    if not air_led:
        add("B5 交接后 advertiser 不残留起飞控制器", "SKIP", "无 air_control_authority.jsonl")
    else:
        post = [e for e in air_run if e.get("handover_detected")]
        if not post:
            add("B5 交接后 advertiser 不残留起飞控制器", "SKIP", "无交接后运行期记录")
        else:
            last_pubs = post[-1].get("setpoint_publishers") or []
            resid = "drone_launch_controller" in last_pubs
            add("B5 交接后 advertiser 不残留起飞控制器", "PASS" if not resid else "FAIL",
                "交接后最后发布者=%s" % last_pubs)

    # ---------------- C 安全状态 ----------------
    add("C1 终态成立", "PASS" if final_state in TERMINAL else "FAIL",
        "final_state=%s(%s) reason=%s" % (final_state, STATE_NAME.get(final_state), final_reason))

    if handover is None:
        pass
    elif args.mode == "handover":
        add("C2 空中 hold 验证通过", "SKIP", "handover 注入模式：LQR 被终止，hold 不适用")
    else:
        hold_ok = bool(handover.get("hold_triggered")) and bool(handover.get("hold_ok"))
        add("C2 空中 hold 验证通过", "PASS" if hold_ok else "FAIL",
            "hold_triggered=%s hold_ok=%s max_dev=%s" % (
                handover.get("hold_triggered"), handover.get("hold_ok"),
                handover.get("hold_max_deviation_m")))

    if args.mode == "loss":
        tol = DEFAULT_LOST_TOL
        targets = load_jsonl(os.path.join(run, "topics/_agc_targets.jsonl"))
        cmd = load_jsonl(os.path.join(run, "topics/_agc_ground_cmd_vel.jsonl"))
        t_last = None
        for rec in targets:
            tr = ((rec.get("msg") or {}).get("tracks")) or []
            if tr:
                t_last = rec.get("t_sim")
        if t_last is None:
            add("C3 失联场景判别", "FAIL", "run 内无有效目标样本（非失联场景？）")
        else:
            tail = [r for r in cmd if r.get("t_sim", 0) >= t_last + tol]
            maxvx = max([abs(((r.get("msg") or {}).get("twist", {}).get("linear", {}).get("x", 0)))
                         for r in tail] or [0.0])
            add("C3 失联后地面停车", "PASS" if (final_reason == "TARGET_LOST" and maxvx <= STOP_VX_MAX) else "FAIL",
                "reason=%s t_last_target=%.2f 尾段样本=%d max|vx|=%.4f"
                % (final_reason, t_last, len(tail), maxvx))

    # 终态后地面停稳：读取紧随其后的 run 目录驻留窗（终态→下一次复位之间）
    post = find_post_run(run, t_end) if t_end is not None else None
    if post is None:
        add("C4 终态后地面零速停稳", "SKIP", "未找到后续 run 目录，无法取终态后驻留窗")
    else:
        resets = load_jsonl(os.path.join(post, "reset_log.jsonl"))
        rt = [r.get("t_sim") for r in resets if r.get("t_sim") is not None and r.get("t_sim") > t_end]
        t_bound = min(rt) if rt else (t_end + DWELL_MAX_S)
        t_bound = min(t_bound, t_end + DWELL_MAX_S)
        pcmd = load_jsonl(os.path.join(post, "topics/_agc_ground_cmd_vel.jsonl"))
        win = [r for r in pcmd if t_end < r.get("t_sim", 0) <= t_bound]
        pdrv = load_jsonl(os.path.join(post, "drive_log.jsonl"))
        dwin = [r for r in pdrv if t_end < r.get("t_sim", 0) <= t_bound]
        maxvx = max([abs(((r.get("msg") or {}).get("twist", {}).get("linear", {}).get("x", 0)))
                     for r in win] or [0.0])
        disp = 0.0
        if len(dwin) >= 2:
            x0, y0 = dwin[0]["pose"]["x"], dwin[0]["pose"]["y"]
            disp = max(math.hypot(r["pose"]["x"] - x0, r["pose"]["y"] - y0) for r in dwin)
        enough = len(win) >= 3 and len(dwin) >= 2
        ok = enough and maxvx <= STOP_VX_MAX and disp <= STOP_DISP_MAX
        add("C4 终态后地面零速停稳", "PASS" if ok else ("SKIP" if not enough else "FAIL"),
            "窗口[%.2f,%.2f] cmd=%d drive=%d max|vx|=%.4f 位移=%.4f m"
            % (t_end, t_bound, len(win), len(dwin), maxvx, disp))

    # ---------------- 模式专项 ----------------
    if args.mode == "nominal":
        add("M 名义成功终态", "PASS" if final_state == 4 else "FAIL",
            "final_state=%s" % final_state)
    elif args.mode == "abort":
        add("M 人工急停终态", "PASS" if (final_state == 6 and final_reason == "OPERATOR_ABORT") else "FAIL",
            "final_state=%s reason=%s" % (final_state, final_reason))
    elif args.mode == "conflict":
        add("M 控制冲突终态", "PASS" if (final_state == 8 and final_reason == "CONTROL_CONFLICT") else "FAIL",
            "final_state=%s reason=%s" % (final_state, final_reason))
        drv = load_jsonl(os.path.join(run, "drive_log.jsonl"))
        ext = [r for r in drv if r.get("external_write_count", 0) > 0 or r.get("pose_authority_ok") is False]
        add("M 外部写车位姿被 ground_drive_adapter 检出", "PASS" if ext else "FAIL",
            "drive_log 命中行=%d" % len(ext))
    elif args.mode == "timeout":
        reason_ok = final_reason in ("MISSION_TIMEOUT", "SEARCH_TIMEOUT", "APPROACH_TIMEOUT")
        add("M 超时终态", "PASS" if (final_state == 5 and reason_ok) else "FAIL",
            "final_state=%s reason=%s" % (final_state, final_reason))
    elif args.mode == "handover":
        gmax = (handover or {}).get("setpoint_gap_max_s_param", DEFAULT_GAP_MAX)
        confl = [e for e in air_run if e.get("control_conflict")]
        gaps = [e.get("setpoint_gap_s") for e in confl]
        big = [g for g in gaps if g is not None and g > gmax]
        add("M 交接后控制空窗被记录", "PASS" if (confl and big) else "FAIL",
            "control_conflict 条数=%d 空窗>%.2fs 条数=%d" % (len(confl), gmax, len(big)))
        owner = (handover or {}).get("active_controller")
        add("M 交接后实际控制源为 lqr_tracker_node", "PASS" if owner == "lqr_tracker_node" else "FAIL",
            "active_controller=%s" % owner)

    _finish(run, args.out, checks)


def _finish(run, out, checks):
    n_fail = sum(1 for c in checks if c["status"] == "FAIL")
    n_pass = sum(1 for c in checks if c["status"] == "PASS")
    n_skip = sum(1 for c in checks if c["status"] == "SKIP")
    passed = n_fail == 0
    result = {"run": run, "checks": checks, "passed": passed,
              "summary": {"pass": n_pass, "fail": n_fail, "skip": n_skip}}
    print("=== M5 控制权核验: %s ===" % os.path.basename(run))
    for c in checks:
        print("%-4s %-38s %s" % (c["status"], c["name"], c["detail"]))
    print("汇总: PASS=%d FAIL=%d SKIP=%d -> %s" % (n_pass, n_fail, n_skip, "PASS" if passed else "FAIL"))
    out_path = out or os.path.join(run, "verify_control_authority.json")
    try:
        with open(out_path, "w") as f:
            json.dump(result, f, ensure_ascii=False, indent=2)
        print("已写入: %s" % out_path)
    except OSError as exc:
        print("写结果失败: %s" % exc)
    if not passed:
        sys.exit(1)


if __name__ == "__main__":
    main()