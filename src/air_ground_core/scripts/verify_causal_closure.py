#!/usr/bin/env python3
"""verify_causal_closure.py —— M3 因果闭环离线核验（纯文件后处理，不连 ROS）。

用法：
    python3 verify_causal_closure.py \
        --control-run   <静态基线 run 目录> \
        --treatment-run <目标运动 run 目录> \
        --noinfo-run    <无信息安全停车 run 目录> \
        [--control-scenario <scenario_flat.yaml>] \
        [--treatment-scenario <scenario_linear_target.yaml>] \
        [--out <输出 JSON 路径>]

分组核验（A 受控对比 / B meta 可复算 / C 空中输入 / D 规划更新 /
E 执行闭环 / F 无信息安全停车），打印 PASS/FAIL 表并写结果 JSON。
任一检查 FAIL 时退出码为 1，全 PASS 为 0。
文件缺失/为空/字段缺失一律记 FAIL 并给出原因，不抛异常。
"""

import argparse
import json
import math
import os
import statistics
import sys
import time

import yaml

# 契约阈值（秒）
GOAL_MAX_AGE_S = 1.0
PATH_MAX_AGE_S = 2.0
ODOM_MAX_AGE_S = 0.5

# 状态名 → 状态码（summary/state_events 回退解析用）
STATE_NAME_TO_CODE = {
    "IDLE": 0, "AIR_SEARCH": 1, "TARGET_ACQUIRED": 2,
    "GROUND_APPROACH": 3, "TRACKING_SUCCESS": 4, "MISSION_FAILED": 5,
    "ABORTED": 6, "INTERCEPT_CONFIRMED": 7, "INFRA_FAILURE": 8,
}

# 受控对比需深度相等的场景键集合
SCENARIO_KEYS = ["car_start", "target_start", "drone_start", "obstacles", "map", "thresholds"]
# planner_log 每行必需字段
PLANNER_FIELDS = ["trigger", "goal", "car_pose", "map_version_hash",
                  "plan_time_ms", "n_waypoints", "success", "failure_reason"]
# 对照 run 允许的 planner trigger：静态目标不应出现 displacement/map_version；
# invalid 表示该拍输入无效（未发布路径），属正常取值。
CONTROL_TRIGGERS = {"first", "period", "infeasible", "invalid"}

DEFAULT_OUT = "/home/ubuntu/anti_drone_agc_workspace/experiments/agc_runs/m3_causal_closure.json"

_CACHE = {}   # 同一路径只解析一次


# ---------------------------------------------------------------- 基础工具
def _num(v):
    """宽松转 float；不可转返回 None。"""
    if isinstance(v, bool):
        return None
    try:
        if isinstance(v, str) and v.strip() == "":
            return None
        return float(v)
    except (TypeError, ValueError):
        return None


def read_jsonl(path):
    """读 JSONL，(records, error)。缺失/空/坏行均容错，不抛异常。"""
    key = os.path.abspath(path)
    if key in _CACHE:
        return _CACHE[key]
    if not os.path.isfile(path):
        res = (None, "文件缺失: %s" % path)
    else:
        recs, bad = [], 0
        try:
            with open(path, "r") as fh:
                for line in fh:
                    line = line.strip()
                    if not line:
                        continue
                    try:
                        recs.append(json.loads(line))
                    except ValueError:
                        bad += 1
        except OSError as exc:
            res = (None, "读取失败: %s" % exc)
            _CACHE[key] = res
            return res
        if not recs and bad:
            res = ([], "%d 行解析失败" % bad)
        elif not recs:
            res = ([], "空文件")
        elif bad:
            res = (recs, "%d 行解析失败" % bad)
        else:
            res = (recs, None)
    _CACHE[key] = res
    return res


def load_json(path):
    """读单个 JSON，返回 (obj, error)。"""
    key = os.path.abspath(path)
    if key in _CACHE:
        return _CACHE[key]
    if not os.path.isfile(path):
        res = (None, "文件缺失: %s" % path)
    else:
        try:
            with open(path, "r") as fh:
                res = (json.load(fh), None)
        except (OSError, ValueError) as exc:
            res = (None, "读取失败: %s" % exc)
    _CACHE[key] = res
    return res


def load_yaml(path):
    if not os.path.isfile(path):
        return None, "文件缺失: %s" % path
    try:
        with open(path, "r") as fh:
            return yaml.safe_load(fh), None
    except Exception as exc:                       # yaml 解析错误也要容错
        return None, "解析失败: %s" % exc


def envelope(samples):
    """位置样本 (t,x,y) 的欧氏包络 sqrt((maxx-minx)^2+(maxy-miny)^2)。"""
    if not samples:
        return None
    xs = [s[1] for s in samples]
    ys = [s[2] for s in samples]
    return math.hypot(max(xs) - min(xs), max(ys) - min(ys))


def nearest_by_time(samples, t):
    """在 (t,x,y) 样本中取时间最近者；t 为 None 或无有效样本返回 None。"""
    if t is None:
        return None
    best, bd = None, None
    for s in samples:
        if s[0] is None:
            continue
        d = abs(s[0] - t)
        if bd is None or d < bd:
            bd, best = d, s
    return best


# ---------------------------------------------------------------- 数据抽取
def target_samples(run):
    """目标地面位置样本 (t,x,y)：优先 targets 非空 tracks，退回 target_trajectory。"""
    recs, err = read_jsonl(os.path.join(run, "topics", "_agc_targets.jsonl"))
    out = []
    for r in recs or []:
        t = _num(r.get("t_sim"))
        for tr in ((r.get("msg") or {}).get("tracks") or []):
            pos = (((tr or {}).get("pose") or {}).get("pose") or {}).get("position") or {}
            x, y = _num(pos.get("x")), _num(pos.get("y"))
            if x is not None and y is not None:
                out.append((t, x, y))
    if out:
        return out, "topics/_agc_targets.jsonl", None
    recs2, err2 = read_jsonl(os.path.join(run, "target_trajectory.jsonl"))
    out2 = []
    for r in recs2 or []:
        x, y = _num(r.get("x")), _num(r.get("y"))
        if x is not None and y is not None:
            out2.append((_num(r.get("t_sim")), x, y))
    if out2:
        return out2, "target_trajectory.jsonl", None
    return [], None, (err or err2 or "无目标样本")


def goal_samples(run):
    """goal 位置样本 (t,x,y)：优先 /agc/ground/goal，退回 planner_log 的 goal 字段。"""
    recs, err = read_jsonl(os.path.join(run, "topics", "_agc_ground_goal.jsonl"))
    out = []
    for r in recs or []:
        pos = (((r.get("msg") or {}).get("pose") or {}).get("position")) or {}
        x, y = _num(pos.get("x")), _num(pos.get("y"))
        if x is not None and y is not None:
            out.append((_num(r.get("t_sim")), x, y))
    if out:
        return out, "topics/_agc_ground_goal.jsonl", None
    recs2, err2 = read_jsonl(os.path.join(run, "planner_log.jsonl"))
    out2 = []
    for r in recs2 or []:
        g = r.get("goal")
        if isinstance(g, (list, tuple)) and len(g) >= 2:
            x, y = _num(g[0]), _num(g[1])
            if x is not None and y is not None:
                out2.append((_num(r.get("t_sim")), x, y))
    if out2:
        return out2, "planner_log.jsonl(goal)", None
    return [], None, (err or err2 or "无 goal 样本")


def odom_samples(run):
    recs, err = read_jsonl(os.path.join(run, "topics", "_agc_ground_odom.jsonl"))
    out = []
    for r in recs or []:
        pos = (((r.get("msg") or {}).get("pose") or {}).get("pose") or {}).get("position") or {}
        x, y = _num(pos.get("x")), _num(pos.get("y"))
        if x is not None and y is not None:
            out.append((_num(r.get("t_sim")), x, y))
    return out, err


def cmd_vel_samples(run):
    recs, err = read_jsonl(os.path.join(run, "topics", "_agc_ground_cmd_vel.jsonl"))
    out = []
    for r in recs or []:
        lin = (((r.get("msg") or {}).get("twist") or {}).get("linear")) or {}
        v = _num(lin.get("x"))
        if v is not None:
            out.append((_num(r.get("t_sim")), v))
    return out, err


def task_states(run):
    recs, err = read_jsonl(os.path.join(run, "topics", "_agc_task_state.jsonl"))
    out = []
    for r in recs or []:
        m = r.get("msg") or {}
        st = m.get("state")
        try:
            st = int(st)
        except (TypeError, ValueError):
            st = None
        out.append((_num(r.get("t_sim")), st, m.get("state_name"), m.get("reason_code")))
    return out, err


def state_window(states, code):
    """某状态码出现的时间窗 [first, last]；无则 None。"""
    ts = [t for (t, st, _n, _r) in states if st == code and t is not None]
    return (min(ts), max(ts)) if ts else None


def task_window(run):
    """任务活动窗口 [t_first_non_idle, t_last]：任务状态非 IDLE 的首/末时刻；无则 (None, None)。

    目标/goal 的因果核查只在任务窗口内评估：run 目录还包含任务开始前的 IDLE 段，
    那一段的 goal 不构成「任务期内地面规划随之更新」的证据。
    """
    states, _err = task_states(run)
    ts = [t for (t, st, _n, _r) in states
          if t is not None and st is not None and st != 0]
    return (min(ts), max(ts)) if ts else (None, None)


def reset_settle_exclusions(run, settle_s=2.0):
    """scenario_reset 复位后 settle_s 秒的排除窗：复位按设计把目标写回 target_start（瞬移），
    该瞬移与「空中目标运动」无关，不应计入 goal 跟随目标的判据。"""
    recs, _err = read_jsonl(os.path.join(run, "reset_log.jsonl"))
    out = []
    for r in recs or []:
        t = _num(r.get("t_sim"))
        if t is not None:
            out.append((t, t + settle_s))
    return out


def in_task_window(samples, win):
    """把 (t,x,y) 样本裁剪到任务窗口；win=(None,None) 时不裁剪。"""
    t0, t1 = win
    if t0 is None or t1 is None:
        return list(samples)
    return [s for s in samples if s[0] is not None and t0 <= s[0] <= t1]


def in_any_exclusion(t, excls):
    return any(a <= t <= b for (a, b) in excls)


def planner_records(run):
    return read_jsonl(os.path.join(run, "planner_log.jsonl"))


def meta_of(run):
    return load_json(os.path.join(run, "meta.json"))


# ---------------------------------------------------------------- 主流程
def main():
    parser = argparse.ArgumentParser(
        description="M3 因果闭环离线核验（纯文件后处理，不连 ROS）")
    parser.add_argument("--control-run", required=True, help="静态基线 run 目录")
    parser.add_argument("--treatment-run", required=True, help="目标运动 run 目录")
    parser.add_argument("--noinfo-run", required=True, help="无信息安全停车 run 目录")
    pkg_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    parser.add_argument("--control-scenario",
                        default=os.path.join(pkg_root, "config", "scenario_flat.yaml"))
    parser.add_argument("--treatment-scenario",
                        default=os.path.join(pkg_root, "config", "scenario_linear_target.yaml"))
    parser.add_argument("--out", default=DEFAULT_OUT, help="输出 JSON 路径")
    args = parser.parse_args()

    ctrl = os.path.abspath(os.path.expanduser(args.control_run))
    trt = os.path.abspath(os.path.expanduser(args.treatment_run))
    nfo = os.path.abspath(os.path.expanduser(args.noinfo_run))

    checks = []

    def add(group, name, ok, value, threshold, detail=""):
        checks.append({"group": group, "name": name, "pass": bool(ok),
                       "value": value, "threshold": threshold, "detail": str(detail)})

    def add_check(group, name, threshold, fn):
        """执行单条检查，任何异常都转成 FAIL，不向上抛。"""
        try:
            ok, value, detail = fn()
        except Exception as exc:                   # 兜底：绝不因单条检查崩溃
            ok, value, detail = False, "-", "检查异常: %s: %s" % (type(exc).__name__, exc)
        add(group, name, ok, value, threshold, detail)

    # 场景文件（分组 A 用）
    ctrl_scn, ctrl_scn_err = load_yaml(args.control_scenario)
    trt_scn, trt_scn_err = load_yaml(args.treatment_scenario)

    # ============================================================ 分组 A
    def _a1():
        if ctrl_scn is None or trt_scn is None:
            return False, "-", "场景不可用: control=%s treatment=%s" % (
                ctrl_scn_err or "ok", trt_scn_err or "ok")
        sub_c = {k: ctrl_scn.get(k) for k in SCENARIO_KEYS}
        sub_t = {k: trt_scn.get(k) for k in SCENARIO_KEYS}
        diff = [k for k in SCENARIO_KEYS if sub_c[k] != sub_t[k]]
        val = "差异键=%s" % (diff or "无")
        return (not diff), val, "比较键=%s" % ",".join(SCENARIO_KEYS)

    add_check("A 受控对比·仅目标不同", "A1 两场景键集合深度相等",
              "差异键为空", _a1)

    def _a2():
        if trt_scn is None:
            return False, "-", "处理场景不可用: %s" % (trt_scn_err or "ok")
        tm = (trt_scn.get("target_motion") or {})
        mode = tm.get("mode")
        spd = _num(tm.get("speed_mps"))
        ok = (mode == "linear") and (spd is not None) and (0.0 < spd <= 1.0)
        return ok, "mode=%s speed=%s" % (mode, spd), "mode=linear 且 0<speed≤1.0"

    add_check("A 受控对比·仅目标不同", "A2 处理场景目标为 linear 且速度在 (0,1]",
              "mode=linear, 0<speed≤1.0", _a2)

    # ============================================================ 分组 B
    def _b3():
        parts, ok = [], True
        for label, run in [("control", ctrl), ("treatment", trt), ("noinfo", nfo)]:
            meta, err = meta_of(run)
            if meta is None:
                ok = False
                parts.append("%s: %s" % (label, err))
                continue
            ph = meta.get("agc_params_hash")
            sh = meta.get("scenario_hash")
            good = bool(ph) and bool(sh)
            ok = ok and good
            parts.append("%s: params=%s scenario=%s" % (
                label, (ph or "<空>")[:8], (sh or "<空>")[:8]))
        return ok, "3 run", "；".join(parts)

    add_check("B meta 可复算", "B3 三 run 的 agc_params_hash/scenario_hash 非空",
              "均非空", _b3)

    def _b4():
        m_c, e_c = meta_of(ctrl)
        m_t, e_t = meta_of(trt)
        if m_c is None or m_t is None:
            return False, "-", "meta 不可用: control=%s treatment=%s" % (
                e_c or "ok", e_t or "ok")
        h_c = m_c.get("agc_params_hash")
        h_t = m_t.get("agc_params_hash")
        ok = bool(h_c) and bool(h_t) and (h_c == h_t)
        return ok, "ctl=%s trt=%s" % ((h_c or "<空>")[:8], (h_t or "<空>")[:8]), "两值相等"

    add_check("B meta 可复算", "B4 对照与处理 agc_params_hash 相等",
              "相等", _b4)

    # ============================================================ 分组 C
    def _c5():
        s, src, err = target_samples(trt)
        s = in_task_window(s, task_window(trt))
        env = envelope(s)
        if env is None:
            return False, "-", "无目标样本: %s" % (err or "空")
        return (env >= 1.0), "%.3f m" % env, "≥1.0 m (%s, n=%d)" % (src, len(s))

    add_check("C 空中输入", "C5 处理 run 目标位置变化范围 ≥1.0 m",
              "≥1.0 m", _c5)

    def _c6():
        s, src, err = target_samples(ctrl)
        s = in_task_window(s, task_window(ctrl))
        env = envelope(s)
        if env is None:
            return False, "-", "无目标样本: %s" % (err or "空")
        return (env <= 0.1), "%.3f m" % env, "≤0.1 m (%s, n=%d)" % (src, len(s))

    add_check("C 空中输入", "C6 对照 run 目标位置变化范围 ≤0.1 m",
              "≤0.1 m", _c6)

    # ============================================================ 分组 D
    def _d7():
        win = task_window(trt)
        excls = reset_settle_exclusions(trt)
        g, gsrc, gerr = goal_samples(trt)
        tgt, tsrc, terr = target_samples(trt)
        g = [s for s in in_task_window(g, win) if not in_any_exclusion(s[0], excls)]
        tgt = in_task_window(tgt, win)
        if not g:
            return False, "-", "无 goal 样本: %s" % (gerr or "空")
        if not tgt:
            return False, "-", "无目标样本: %s" % (terr or "空")
        dists = []
        for (t, gx, gy) in g:
            best = nearest_by_time(tgt, t)
            if best is None:
                continue
            dists.append(math.hypot(gx - best[1], gy - best[2]))
        if not dists:
            return False, "-", "无法对齐（goal 无有效时间戳）"
        med = statistics.median(dists)
        mx = max(dists)
        ok = (med <= 0.5) and (mx <= 1.5)
        return ok, "med=%.3f max=%.3f" % (med, mx), \
            "中位≤0.5 且 最大≤1.5 m (%s↔%s, n=%d, 任务窗=[%s,%s], 复位排除=%d)" % (
                gsrc, tsrc, len(dists), win[0], win[1], len(excls))

    add_check("D 规划随之更新", "D7 处理 run 的 goal 贴合目标(中位≤0.5/最大≤1.5)",
              "中位≤0.5, 最大≤1.5 m", _d7)

    def _d8():
        g, src, err = goal_samples(trt)
        g = in_task_window(g, task_window(trt))
        env = envelope(g)
        if env is None:
            return False, "-", "无 goal 样本: %s" % (err or "空")
        return (env >= 1.0), "%.3f m" % env, "≥1.0 m (%s, n=%d)" % (src, len(g))

    add_check("D 规划随之更新", "D8 处理 run 的 goal 位置变化范围 ≥1.0 m",
              "≥1.0 m", _d8)

    def _d9():
        g, src, err = goal_samples(ctrl)
        g = in_task_window(g, task_window(ctrl))
        env = envelope(g)
        if env is None:
            return False, "-", "无 goal 样本: %s" % (err or "空")
        return (env <= 0.2), "%.3f m" % env, "≤0.2 m (%s, n=%d)" % (src, len(g))

    add_check("D 规划随之更新", "D9 对照 run 的 goal 位置变化范围 ≤0.2 m",
              "≤0.2 m", _d9)

    def _d10():
        recs, err = planner_records(trt)
        if recs is None:
            return False, "-", err
        if not recs:
            return False, "0 行", err or "空文件"
        bad_rows = []
        for i, r in enumerate(recs):
            # trigger=invalid 是「输入无效、未发布路径」的短记录，只含部分字段，不参与字段齐全判定
            if r.get("trigger") == "invalid":
                continue
            miss = [f for f in PLANNER_FIELDS if f not in r]
            if miss:
                bad_rows.append((i, r.get("trigger"), miss))
        ok = not bad_rows
        val = "完整 %d/%d 行" % (len(recs) - len(bad_rows), len(recs))
        if bad_rows:
            i, trig, miss = bad_rows[0]
            det = "有 %d 行缺字段；首例 行%d trigger=%s 缺=%s" % (
                len(bad_rows), i, trig, ",".join(miss))
        else:
            det = "每行含 %s" % ",".join(PLANNER_FIELDS)
        return ok, val, det

    add_check("D 规划随之更新", "D10 处理 run 的 planner_log 各字段齐全",
              "每行 8 字段齐全", _d10)

    def _d11():
        recs, err = planner_records(trt)
        if recs is None:
            return False, "-", err
        if not recs:
            return False, "0 行", err or "空文件"
        n = sum(1 for r in recs if r.get("trigger") == "displacement")
        return (n >= 1), "displacement=%d" % n, "≥1 次"

    add_check("D 规划随之更新", "D11 处理 run 存在 trigger=displacement",
              "≥1 次", _d11)

    def _d12():
        recs, err = planner_records(ctrl)
        if recs is None:
            return False, "-", err
        if not recs:
            return False, "0 行", err or "空文件"
        used = set()
        for r in recs:
            if "trigger" in r:
                used.add(r.get("trigger"))
        extra = sorted(x for x in used if x not in CONTROL_TRIGGERS)
        n_disp = sum(1 for r in recs if r.get("trigger") == "displacement")
        ok = (not extra) and (n_disp == 0)
        return ok, "trigger=%s displacement=%d" % (sorted(used, key=str), n_disp), \
            "仅允许 %s 且 displacement=0；越界=%s" % (sorted(CONTROL_TRIGGERS), extra or "无")

    add_check("D 规划随之更新", "D12 对照 run 无 displacement 且 trigger 合法",
              "无越界 trigger 且 displacement=0", _d12)

    # ============================================================ 分组 E
    def _e13():
        od, err = odom_samples(trt)
        if not od:
            return False, "-", "无 odom 样本: %s" % (err or "空")
        x0, y0 = od[0][1], od[0][2]
        mx = max(math.hypot(x - x0, y - y0) for (_t, x, y) in od)
        return (mx >= 3.0), "%.3f m" % mx, "≥3.0 m (n=%d)" % len(od)

    add_check("E 执行闭环", "E13 处理 run 车相对首个 odom 位移 ≥3.0 m",
              "≥3.0 m", _e13)

    def _e14():
        sts, _serr = task_states(trt)
        win = state_window(sts, 3)
        if win is None:
            return False, "-", "无 state==3(GROUND_APPROACH) 样本"
        cmd, cerr = cmd_vel_samples(trt)
        inwin = [v for (t, v) in cmd if t is not None and win[0] <= t <= win[1]]
        if not inwin:
            return False, "0 样本", "窗口 [%.2f,%.2f] 内无 cmd_vel: %s" % (
                win[0], win[1], cerr or "空")
        n = sum(1 for v in inwin if v > 0.05)
        mx = max(inwin)
        return (n >= 1), "max_vx=%.3f" % mx, \
            "窗[%.2f,%.2f] 内 %d/%d 样本 vx>0.05" % (win[0], win[1], n, len(inwin))

    add_check("E 执行闭环", "E14 GROUND_APPROACH 窗内存在 cmd_vel>0.05",
              "≥1 样本 >0.05", _e14)

    def _e15():
        sts, _ = task_states(trt)
        win = state_window(sts, 3)
        if win is None:
            return False, "-", "无 state==3 时间窗"
        od, _ = odom_samples(trt)
        inwin = sorted(s for s in od if s[0] is not None and win[0] <= s[0] <= win[1])
        if len(inwin) < 2:
            return False, "n=%d" % len(inwin), "窗内 odom 样本不足"
        g, _, _ = goal_samples(trt)
        cosines = []
        for i in range(len(inwin) - 1):
            t0, x0, y0 = inwin[i]
            _t1, x1, y1 = inwin[i + 1]
            dx, dy = x1 - x0, y1 - y0
            n = math.hypot(dx, dy)
            if n < 1e-9:
                continue
            best = nearest_by_time(g, t0)
            if best is None:
                continue
            vx, vy = best[1] - x0, best[2] - y0
            m = math.hypot(vx, vy)
            if m < 1e-9:
                continue
            cosines.append((dx * vx + dy * vy) / (n * m))
        if not cosines:
            return False, "-", "无有效相邻位移对"
        avg = sum(cosines) / len(cosines)
        return (avg > 0.0), "avg_cos=%.3f" % avg, \
            "相邻位移朝向 goal 的平均余弦 >0 (n=%d)" % len(cosines)

    add_check("E 执行闭环", "E15 GROUND_APPROACH 内位移朝向 goal(平均cos>0)",
              "平均cos>0", _e15)

    # ============================================================ 分组 F
    def _f16():
        recs, err = read_jsonl(os.path.join(nfo, "topics", "_agc_targets.jsonl"))
        if recs is None:
            return False, "-", err
        if not recs:
            return False, "0 行", err or "空文件"
        n_empty = sum(1 for r in recs if not ((r.get("msg") or {}).get("tracks")))
        t_nonempty = [s[0] for s in target_samples(nfo)[0] if s[0] is not None]
        t_last = max(t_nonempty) if t_nonempty else None
        ok = (n_empty >= 1) and (t_last is not None)
        return ok, "空tracks=%d t_last=%s" % (n_empty, t_last), \
            "存在空 tracks 样本 且 存在末次非空目标时刻"

    add_check("F 无信息·安全停车", "F16 无信息 run 有空 tracks 样本且 t_last 存在",
              "空tracks≥1 且 t_last 存在", _f16)

    def _f17():
        _recs, err = read_jsonl(os.path.join(nfo, "topics", "_agc_targets.jsonl"))
        t_nonempty = [s[0] for s in target_samples(nfo)[0] if s[0] is not None]
        if not t_nonempty:
            return False, "-", "无 t_last（目标样本缺失: %s）" % (err or "无")
        t_stop = max(t_nonempty) + GOAL_MAX_AGE_S + 0.1
        cmd, cerr = cmd_vel_samples(nfo)
        after = [v for (t, v) in cmd if t is not None and t >= t_stop]
        if not after:
            return False, "0 样本", "t_stop=%.3f 之后无 cmd_vel: %s" % (t_stop, cerr or "空")
        mx = max(abs(v) for v in after)
        return (mx <= 0.02), "max|vx|=%.4f" % mx, \
            "t_stop=%.3f 后 %d 样本 |vx|≤0.02" % (t_stop, len(after))

    add_check("F 无信息·安全停车", "F17 无信息 run t_stop 后 |cmd_vel.linear.x|≤0.02",
              "≤0.02", _f17)

    def _f18():
        t_nonempty = [s[0] for s in target_samples(nfo)[0] if s[0] is not None]
        if not t_nonempty:
            return False, "-", "无 t_last"
        t_stop = max(t_nonempty) + GOAL_MAX_AGE_S + 0.1
        od, oerr = odom_samples(nfo)
        after = sorted(s for s in od if s[0] is not None and s[0] >= t_stop)
        if not after:
            return False, "0 样本", "t_stop=%.3f 之后无 odom: %s" % (t_stop, oerr or "空")
        bx, by = after[0][1], after[0][2]
        mx = max(math.hypot(x - bx, y - by) for (_t, x, y) in after)
        return (mx <= 0.10), "max_disp=%.4f" % mx, \
            "t_stop=%.3f 后相对基准位移 ≤0.10 m (n=%d)" % (t_stop, len(after))

    add_check("F 无信息·安全停车", "F18 无信息 run t_stop 后车位移 ≤0.10 m",
              "≤0.10 m", _f18)

    def _f19():
        summ, serr = load_json(os.path.join(nfo, "summary.json"))
        state = name = reason = None
        src = None
        if summ is not None:
            state = summ.get("final_state")
            name = summ.get("final_state_name")
            reason = summ.get("final_reason_code")
            if state is not None or name or reason:
                src = "summary.json"
        if src is None:
            sts, terr = task_states(nfo)
            if sts:
                _t, state, name, reason = sts[-1]
                src = "topics/_agc_task_state.jsonl"
            else:
                ev, eerr = read_jsonl(os.path.join(nfo, "state_events.jsonl"))
                if ev:
                    last = ev[-1]
                    name = last.get("to") or last.get("event") or last.get("state_name")
                    reason = last.get("reason")
                    state = last.get("state")
                    if state is None and name in STATE_NAME_TO_CODE:
                        state = STATE_NAME_TO_CODE[name]
                    src = "state_events.jsonl"
                else:
                    return False, "-", "summary/state 均不可用: %s" % (
                        serr or terr or eerr or "空")
        try:
            state_i = int(state) if state is not None else None
        except (TypeError, ValueError):
            state_i = None
        if state_i is None and name in STATE_NAME_TO_CODE:
            state_i = STATE_NAME_TO_CODE[name]
        ok = ((state_i == 5) or (name == "MISSION_FAILED")) and (reason == "TARGET_LOST")
        return ok, "state=%s name=%s reason=%s" % (state_i, name, reason), \
            "来源=%s；需 (state==5 或 MISSION_FAILED) 且 reason==TARGET_LOST" % src

    add_check("F 无信息·安全停车", "F19 无信息 run 终态 MISSION_FAILED / TARGET_LOST",
              "state==5 且 reason==TARGET_LOST", _f19)

    # ============================================================ 输出
    n_fail = sum(1 for c in checks if not c["pass"])
    passed = (n_fail == 0)

    result = {
        "generated_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "control_run": ctrl,
        "treatment_run": trt,
        "noinfo_run": nfo,
        "scenarios": {"control": args.control_scenario,
                      "treatment": args.treatment_scenario},
        "thresholds": {"goal_max_age_s": GOAL_MAX_AGE_S,
                       "path_max_age_s": PATH_MAX_AGE_S,
                       "odom_max_age_s": ODOM_MAX_AGE_S},
        "checks": checks,
        "summary": {"pass": len(checks) - n_fail, "fail": n_fail, "total": len(checks)},
        "passed": passed,
    }

    print("=== M3 因果闭环核验（离线，不连 ROS）===")
    print("%-8s %-46s %-26s %s" % ("结果", "检查项", "实测值/阈值", "说明"))
    print("-" * 130)
    cur_group = None
    for c in checks:
        if c["group"] != cur_group:
            cur_group = c["group"]
            print("-- %s --" % cur_group)
        print("%-8s %-46s %-26s %s" % (
            "[PASS]" if c["pass"] else "[FAIL]", c["name"],
            "%s / %s" % (c["value"], c["threshold"]), c["detail"]))
    print("-" * 130)
    print("汇总: PASS=%d FAIL=%d TOTAL=%d" % (
        len(checks) - n_fail, n_fail, len(checks)))
    print("CAUSAL_CLOSURE: %s" % ("PASS" if passed else "FAIL"))

    out_path = os.path.abspath(os.path.expanduser(args.out))
    try:
        parent = os.path.dirname(out_path)
        if parent:
            os.makedirs(parent, exist_ok=True)
        with open(out_path, "w") as fh:
            json.dump(result, fh, ensure_ascii=False, indent=2)
        print("结果已写入: %s" % out_path)
    except OSError as exc:
        print("写结果 JSON 失败: %s" % exc)

    sys.exit(0 if passed else 1)


if __name__ == "__main__":
    main()