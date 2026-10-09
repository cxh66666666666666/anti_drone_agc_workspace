#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""verify_a0_perception.py —— 阶段3 A0「观测模型与真值集」离线核验（不依赖 ROS）。

读 A0 manifest、各场景 run 的 perception_log.jsonl 与感知配置，逐项核验并输出
experiments/a0/verify_a0_perception.json（每项：指标、实测、配置、容差、n、PASS/FAIL）。
不把未通过项标 PASS；未达容差如实记 FAIL 或标注「已量化边界/未决」。

用法：
    python3 verify_a0_perception.py [--manifest ...] [--runs-json ...] [--out ...]
        [--nominal-run DIR] [--occlusion-run DIR] [--fault-run DIR] [--maneuver-run DIR]
        [--replay-run1 DIR] [--replay-run2 DIR]
runs-json（默认 experiments/a0/smoke_index.json）提供 runs 映射与 replay_run_dirs。
"""

import argparse
import hashlib
import json
import math
import os
import subprocess
import sys

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
WORKSPACE = os.path.abspath(os.path.join(SCRIPT_DIR, "..", "..", ".."))
A0_DIR = os.path.join(WORKSPACE, "experiments", "a0")
DEFAULT_MANIFEST = os.path.join(A0_DIR, "manifest_a0.json")
DEFAULT_RUNS = os.path.join(A0_DIR, "smoke_index.json")
DEFAULT_OUT = os.path.join(A0_DIR, "verify_a0_perception.json")

OBS_HZ = 20.0
LEGAL_REASONS = {"ok", "no_target", "no_drone_pose", "out_of_range_min",
                 "out_of_range_max", "behind", "out_of_fov", "occluded"}
CORR_TOL = 0.08          # 相关系数容差（n≈3000 时标准误≈0.018，取 ~4σ）
LAT_TOL = 1e-9


def load_json(p):
    try:
        with open(p, encoding="utf-8") as f:
            return json.load(f)
    except (OSError, ValueError):
        return None


def load_jsonl(p):
    out = []
    try:
        with open(p, encoding="utf-8", errors="ignore") as f:
            for line in f:
                line = line.strip()
                if line:
                    out.append(json.loads(line))
    except OSError:
        pass
    return out


def sha256_file(p):
    if not os.path.isfile(p):
        return None
    h = hashlib.sha256()
    with open(p, "rb") as f:
        for c in iter(lambda: f.read(65536), b""):
            h.update(c)
    return h.hexdigest()


def norm_run(rd):
    """把 run 目录规范为绝对路径（smoke_index 里可能只存 run_id）。"""
    if not rd:
        return ""
    if os.path.isabs(rd):
        return rd
    cand = os.path.join(WORKSPACE, "experiments", "agc_runs", rd)
    if os.path.isdir(cand):
        return cand
    rel = os.path.join(WORKSPACE, rd)
    return rel if os.path.isdir(rel) else cand


def plog(run_dir):
    return load_jsonl(os.path.join(run_dir, "perception_log.jsonl"))


def sensors_by_id(sensors_file):
    import yaml
    with open(sensors_file, encoding="utf-8") as f:
        doc = yaml.safe_load(f)
    return {s.get("id"): s for s in (doc.get("sensors") or [])}, (doc.get("perception_model") or {})


def pearson(xs, ys):
    n = len(xs)
    if n < 3:
        return None
    mx = sum(xs) / n
    my = sum(ys) / n
    sxy = sum((a - mx) * (b - my) for a, b in zip(xs, ys))
    sxx = sum((a - mx) ** 2 for a in xs)
    syy = sum((b - my) ** 2 for b in ys)
    if sxx <= 0 or syy <= 0:
        return None
    return sxy / math.sqrt(sxx * syy)


class Checks:
    def __init__(self):
        self.items = []

    def add(self, cid, name, ok, measured, expected, tol, n, detail):
        self.items.append({"id": cid, "name": name, "pass": bool(ok),
                           "measured": measured, "expected": expected,
                           "tolerance": tol, "n": n, "detail": detail})


def bucket(t):
    return int(math.floor(t * OBS_HZ + 0.5))


# ---------------------------------------------------------------- 各项核验
def check_gating(ck, nominal_pl):
    """1) 门控一致性。"""
    gate = [r for r in nominal_pl if "gate_reason" in r]
    drop = [r for r in nominal_pl if r.get("dropped")]
    bad_pair = [r for r in gate if not r.get("is_false_alarm")
                and bool(r.get("gate_ok")) != bool(r.get("has_target"))]
    bad_reason = sorted({r.get("gate_reason") for r in gate} - LEGAL_REASONS)
    bad_drop_has = [r for r in drop if r.get("has_target")]
    ok = (not bad_pair and not bad_reason and not bad_drop_has and len(nominal_pl) > 0)
    ck.add("1", "门控一致性 gate_ok==has_target；gate_reason 合法；丢包不产生观测",
           ok, {"bad_pair": len(bad_pair), "bad_reason": bad_reason, "bad_drop_has_target": len(bad_drop_has)},
           {"bad_pair": 0, "bad_reason": [], "bad_drop_has_target": 0}, "严格 0",
           len(nominal_pl), "非法原因=%s" % (bad_reason or "无"))


def check_occlusion(ck, occ_pl, nominal_pl):
    """2) 遮挡：检出正确性（2a）与段连续+恢复（2b）。"""
    occ = [r for r in occ_pl if r.get("gate_reason") == "occluded"]
    occ_ok = all((not r.get("has_target")) and r.get("occluder") for r in occ)
    nom_occ = [r for r in nominal_pl if r.get("gate_reason") == "occluded"]
    det_ok = len(occ) > 0 and occ_ok and len(nom_occ) == 0
    ck.add("2a", "遮挡检出（occluded 段 occluder 非空且 has_target=false；默认配置无 occluded）", det_ok,
           {"occluded_n": len(occ), "has_target_false_and_occluder": occ_ok, "nominal_occluded_n": len(nom_occ)},
           {"occluded_n": ">0", "nominal_occluded_n": 0}, "严格", len(occ_pl),
           "nominal 无 occluded 佐证默认 occlusion.enabled=false 生效")
    # 段连续（按通道合并，允许小间隙）+ 之后恢复
    segs = []
    for sid in {r["sensor_id"] for r in occ}:
        ts = sorted(r["t_sim"] for r in occ if r["sensor_id"] == sid)
        a = b = None
        for t in ts:
            if a is None:
                a = b = t
            elif t - b <= 0.12:
                b = t
            else:
                segs.append(b - a); a = b = t
        if a is not None:
            segs.append(b - a)
    contiguous = bool(segs) and max(segs) >= 1.0
    # 恢复判据：存在某个 occluded 事件之后的 has_target=true 观测（遮挡→解除→重新获得）
    events = sorted(((r["t_sim"], r.get("gate_reason") == "occluded", bool(r.get("has_target")))
                     for r in occ_pl), key=lambda e: e[0])
    seen_occ, recovery = False, 0
    for _t, is_occ, ht in events:
        if is_occ:
            seen_occ = True
        elif seen_occ and ht:
            recovery += 1
    recov_ok = recovery > 0
    ck.add("2b", "遮挡段连续（存在≥1s 连续段）且之后恢复（has_target=true）", contiguous and recov_ok,
           {"n_segments": len(segs), "max_segment_s": round(max(segs), 2) if segs else 0.0,
            "recovery_after_occlusion": recovery,
            "has_target_total": sum(1 for r in occ_pl if r.get("has_target"))},
           {"contiguous_max_seg": ">=1s", "recovery_after_occlusion": ">0"},
           "严格", len(occ_pl),
           "a0_occlusion 为瞬态穿越遮挡（目标 y 跨 0 时被挡约 2~3s，|y| 大时解除并重新获得观测）")


def check_latency(ck, pls):
    """3) 延迟：t_send_s - t_sim == latency_s（逐条）。"""
    n, bad, maxd = 0, 0, 0.0
    for pl in pls:
        for r in pl:
            if "latency_s" in r and "t_send_s" in r and "t_sim" in r:
                n += 1
                d = abs((r["t_send_s"] - r["t_sim"]) - r["latency_s"])
                maxd = max(maxd, d)
                if d > LAT_TOL:
                    bad += 1
    ck.add("3", "延迟 t_send_s-t_sim==latency_s", bad == 0 and n > 0, {"max_abs_diff": maxd, "bad": bad},
           {"max_abs_diff": 0.0, "bad": 0}, "≤%.0e" % LAT_TOL, n, "全 A0 场景样本")


def check_drop_burst(ck, fault_pl, sensors):
    """4) 丢包/成簇（a0_fault）：独立丢包率≈drop_prob；成簇游程≈drop_burst_mean_s；双通道同向。"""
    by_sid = {}
    for r in fault_pl:
        by_sid.setdefault(r["sensor_id"], []).append(r)
    # 独立丢包率
    ind_meas, ind_exp = {}, {}
    for sid, rows in by_sid.items():
        ind_n = sum(1 for r in rows if r.get("drop_source") == "independent")
        tot = len(rows)
        ind_meas[sid] = (ind_n / tot) if tot else 0.0
        # 期望：按每拍 drop_prob 求均值（各拍抽样独立）
        probs = [r.get("drop_prob", 0.0) for r in rows]
        ind_exp[sid] = (sum(probs) / len(probs)) if probs else 0.0
    # 成簇游程：以 burst_state（两态状态机）计，连续拍数/obs_hz = 状态持续时长
    burst_runs = {}
    for sid, rows in by_sid.items():
        rows = sorted(rows, key=lambda r: r["t_sim"])
        runs, cur = [], 0
        for r in rows:
            if r.get("burst_state"):
                cur += 1
            else:
                if cur:
                    runs.append(cur)
                cur = 0
        if cur:
            runs.append(cur)
        burst_runs[sid] = [c / OBS_HZ for c in runs]
    # drop_group 同向：同一 bucket 两通道 burst_state 一致（共享状态机）
    sids = sorted(by_sid)
    same, diff = 0, 0
    if len(sids) == 2:
        m0 = {bucket(r["t_sim"]): bool(r.get("burst_state")) for r in by_sid[sids[0]]}
        m1 = {bucket(r["t_sim"]): bool(r.get("burst_state")) for r in by_sid[sids[1]]}
        for b in set(m0) & set(m1):
            if m0[b] == m1[b]:
                same += 1
            else:
                diff += 1
    # 判定
    rate_ok = all(abs(ind_meas[s] - ind_exp[s]) < 0.05 for s in by_sid)
    # 共享 drop_group 的两态机由「文件序首个拥有该组的通道」参数驱动（每拍只更新一次）
    owner = {}
    for sid in sensors:
        g = sensors[sid].get("drop_group", "")
        if g and g not in owner:
            owner[g] = sensors[sid].get("drop_burst_mean_s", 0.0)
    burst_ok, burst_detail = True, {}
    for sid, durs in burst_runs.items():
        cfg = sensors.get(sid, {}).get("drop_burst_mean_s", 0.0)
        g = sensors.get(sid, {}).get("drop_group", "")
        exp_mean = owner.get(g, cfg) if g else cfg
        nr = len(durs)
        # 小样本容差：burst 时长近似指数分布，均值标准误≈exp/sqrt(n)，取 2σ
        tol = max(0.15, 2.0 * exp_mean / math.sqrt(max(nr, 1)))
        if not durs:
            burst_detail[sid] = {"n_runs": 0, "meas_mean": None, "cfg": cfg, "exp_group_mean": exp_mean}
            continue
        mn = sum(durs) / nr
        burst_detail[sid] = {"n_runs": nr, "meas_mean": round(mn, 4), "cfg": cfg,
                             "exp_group_mean": exp_mean, "tol": round(tol, 4)}
        if abs(mn - exp_mean) > tol:
            burst_ok = False
    grp_ok = (diff == 0 and same > 0)
    ok = rate_ok and burst_ok and grp_ok and len(fault_pl) > 0
    ck.add("4", "丢包/成簇（独立率、成簇游程、drop_group 同向）", ok,
           {"ind_rate": {k: round(v, 4) for k, v in ind_meas.items()},
            "ind_expected": {k: round(v, 4) for k, v in ind_exp.items()},
            "burst": burst_detail, "group_same": same, "group_diff": diff,
            "tol_basis": "成簇游程均值容差：游程时长近似指数分布，均值标准误≈exp_group_mean/√n_runs，"
                         "取 2σ=max(0.15, 2·exp_group_mean/√n_runs)；下限 0.15s 覆盖整拍量化(1/obs_hz=0.05s)；"
                         "独立丢包容差 ±0.05（率的标准误<0.01）；组同向为严格 0 差异"},
           {"ind_rate≈drop_prob": True, "burst_mean≈drop_burst_mean_s": True, "group_diff": 0},
           "独立率±0.05；游程 2σ/√n（下限0.15）；组一致严格", len(fault_pl),
           "drop_group 同向一致比例=%.3f" % (same / (same + diff) if (same + diff) else 0.0))


def check_false_alarm(ck, pl_by_sensors):
    """5) 虚警（FA>0 场景）：实测率≈false_alarm_rate_hz；truth_id null；fa_pos 远离真值。"""
    meas_rate, exp_rate, fa_n, tot, tid_bad, dual, far_ok, far_n = {}, {}, 0, 0, 0, 0, 0, 0
    for sid, (pl, cfg) in pl_by_sensors.items():
        fa = [r for r in pl if r.get("is_false_alarm")]
        n = len(pl)
        fa_n += len(fa); tot += n
        exp = cfg.get("false_alarm_rate_hz", 0.0) / OBS_HZ  # 每拍概率
        meas_rate[sid] = (len(fa) / n) if n else 0.0
        exp_rate[sid] = exp
        for r in fa:
            # 同一拍/通道可能既有真实目标观测又叠加虚警：此时 truth_id 由真实目标给出属正常；
            # 严格约束是「无真实目标观测的虚警」truth_id 必须为 null。
            if not r.get("has_target"):
                if r.get("truth_id") is not None:
                    tid_bad += 1
            else:
                dual += 1
            fa_pos, truth = r.get("fa_pos"), r.get("target_truth")
            if fa_pos and truth:
                d = math.dist(fa_pos, truth)
                far_n += 1
                if d > 5.0:
                    far_ok += 1
    rate_ok = all(abs(meas_rate[s] - exp_rate[s]) < 0.02 for s in meas_rate)
    ok = rate_ok and tid_bad == 0 and (far_n == 0 or far_ok == far_n) and tot > 0
    ck.add("5", "虚警率≈false_alarm_rate_hz；无目标虚警 truth_id=null；fa_pos 远离真值", ok,
           {"meas_rate": {k: round(v, 4) for k, v in meas_rate.items()},
            "expected_rate": {k: round(v, 4) for k, v in exp_rate.items()},
            "fa_n": fa_n, "no_target_fa_truth_id_not_null": tid_bad, "fa_with_target_dual": dual,
            "far": far_ok, "far_n": far_n},
           {"meas_rate≈rate/hz": True, "no_target_fa_truth_id_not_null": 0, "all_far_gt_5m": True},
           "率±0.02；严格 0；严格", tot, "FA 场景样本（虚警可叠加在真实目标观测之上）")


def _err(rows):
    """返回 [(bucket, ex, ey)]（仅 has_target 且 outlier_bias_m==0）。"""
    out = []
    for r in rows:
        if r.get("has_target") and r.get("outlier_bias_m", 0) == 0 \
                and r.get("target_truth") and r.get("target_obs"):
            t, o = r["target_truth"], r["target_obs"]
            out.append((bucket(r["t_sim"]), o[0] - t[0], o[1] - t[1]))
    return out


def check_noise_corr(ck, pl, sensors):
    """6) 相关噪声（a0_replay）：通道内 corr_xy、通道间 corr_rho 乘积、AR lag-1、偏置。"""
    by_sid = {}
    for r in pl:
        by_sid.setdefault(r["sensor_id"], []).append(r)
    sids = sorted(by_sid)
    # 通道内 corr(e_x,e_y)
    within = {}
    for sid in sids:
        e = _err(by_sid[sid])
        c = pearson([x for _, x, _ in e], [y for _, _, y in e])
        within[sid] = c
    # 通道间 corr(e_x^f, e_x^i)
    cross_x = cross_y = None
    if len(sids) == 2:
        m0 = {b: (x, y) for b, x, y in _err(by_sid[sids[0]])}
        m1 = {b: (x, y) for b, x, y in _err(by_sid[sids[1]])}
        common = sorted(set(m0) & set(m1))
        cross_x = pearson([m0[b][0] for b in common], [m1[b][0] for b in common])
        cross_y = pearson([m0[b][1] for b in common], [m1[b][1] for b in common])
    # lag-1 自相关（bucket 差 1），x/y 各算
    lag1, lag1y = {}, {}
    for sid in sids:
        for src, dst in ((0, lag1), (1, lag1y)):
            e = dict((b, tup[src]) for b, tup in ((b, (x, y)) for b, x, y in _err(by_sid[sid])))
            xs, ys = [], []
            for b in e:
                if (b + 1) in e:
                    xs.append(e[b]); ys.append(e[b + 1])
            dst[sid] = pearson(xs, ys)
    # 偏置：均值标准误需按 AR(1) 自相关膨胀（有效样本 n_eff=n(1-a)/(1+a)）
    bias_meas, bias_cfg, bias_tol = {}, {}, {}
    for sid in sids:
        e = _err(by_sid[sid])
        xs = [x for _, x, _ in e]; ys = [y for _, _, y in e]
        n = len(xs)
        cfg = sensors.get(sid, {}).get("bias_m", [0.0, 0.0, 0.0])
        mxs = sum(xs) / n if n else 0.0; mys = sum(ys) / n if n else 0.0
        sx = math.sqrt(sum((a - mxs) ** 2 for a in xs) / (n - 1)) if n > 1 else 0.0
        sy = math.sqrt(sum((a - mys) ** 2 for a in ys) / (n - 1)) if n > 1 else 0.0

        def infl(a):
            a = min(max(a or 0.0, -0.9), 0.95)
            return math.sqrt((1.0 + a) / (1.0 - a))
        infx = infl(lag1.get(sid)); infy = infl(lag1y.get(sid))
        bias_meas[sid] = [round(mxs, 5), round(mys, 5)]
        bias_cfg[sid] = [cfg[0], cfg[1]]
        bias_tol[sid] = [round(3 * sx / math.sqrt(n) * infx, 4) if n else 0.0,
                         round(3 * sy / math.sqrt(n) * infy, 4) if n else 0.0]
    # 判定
    checks_ok = []
    within_exp = {}
    for sid in sids:
        cxy = sensors.get(sid, {}).get("corr_xy", 0.0)
        rho = sensors.get(sid, {}).get("corr_rho", 0.0)
        # 公共模态按独立 gx/gy 叠加，会稀释通道内 x-y 相关：期望 = (1-corr_rho^2)*corr_xy
        exp = (1.0 - rho * rho) * cxy
        within_exp[sid] = exp
        checks_ok.append(within[sid] is not None and abs(within[sid] - exp) <= CORR_TOL)
    if len(sids) == 2:
        exp_rho = sensors[sids[0]].get("corr_rho", 0.0) * sensors[sids[1]].get("corr_rho", 0.0)
        checks_ok.append(cross_x is not None and abs(cross_x - exp_rho) <= CORR_TOL)
        checks_ok.append(cross_y is not None and abs(cross_y - exp_rho) <= CORR_TOL)
    for sid in sids:
        tau = sensors.get(sid, {}).get("noise_tau_s", 0.0)
        exp_a = math.exp(-(1.0 / OBS_HZ) / tau) if tau > 0 else 0.0
        checks_ok.append(lag1[sid] is not None and abs(lag1[sid] - exp_a) <= CORR_TOL)
    for sid in sids:
        bm = bias_meas[sid]; bc = bias_cfg[sid]; bt = bias_tol[sid]
        checks_ok.append(abs(bm[0] - bc[0]) <= bt[0] and abs(bm[1] - bc[1]) <= bt[1])
    n = sum(len(_err(by_sid[s])) for s in sids)
    exp_rho_pair = (sensors[sids[0]].get("corr_rho", 0.0) * sensors[sids[1]].get("corr_rho", 0.0)) if len(sids) == 2 else None
    ck.add("6", "相关噪声（corr_xy/通道间 rho 乘积/lag-1 自相关/偏置）", all(checks_ok) and n > 0,
           {"within_corr_xy": {k: (round(v, 4) if v is not None else None) for k, v in within.items()},
            "cross_x": round(cross_x, 4) if cross_x is not None else None,
            "cross_y": round(cross_y, 4) if cross_y is not None else None,
            "lag1": {k: (round(v, 4) if v is not None else None) for k, v in lag1.items()},
            "bias_meas": bias_meas, "bias_cfg": bias_cfg, "bias_tol_3sigma": bias_tol},
           {"within_corr_xy": {k: sensors[k].get("corr_xy") for k in sids},
            "within_corr_xy_model_expected": {k: round(v, 4) for k, v in within_exp.items()},
            "cross_x": exp_rho_pair, "cross_y": exp_rho_pair,
            "lag1": {k: round(math.exp(-(1.0 / OBS_HZ) / sensors[k].get("noise_tau_s", 1.0)), 4)
                     if sensors[k].get("noise_tau_s", 0) > 0 else 0.0 for k in sids},
            "bias": {k: sensors[k].get("bias_m", [0, 0, 0])[:2] for k in sids}},
           "相关系数±%.2f；偏置±3σ/√n" % CORR_TOL, n, "a0_replay 运行")


def check_truth_leak(ck, run_dirs):
    """7) truth_id 不泄漏：msg 无 truth 字段；run 目录（除 perception_log.jsonl）不含 truth_id。"""
    msg_dir = os.path.join(WORKSPACE, "src", "air_ground_core", "msg")
    msg_hits = []
    if os.path.isdir(msg_dir):
        for fn in os.listdir(msg_dir):
            if fn.endswith(".msg"):
                with open(os.path.join(msg_dir, fn), errors="ignore") as f:
                    if "truth" in f.read():
                        msg_hits.append(fn)
    file_hits = []
    for rd in run_dirs:
        for root, _dirs, files in os.walk(rd):
            for fn in files:
                if fn == "perception_log.jsonl":
                    continue
                p = os.path.join(root, fn)
                try:
                    with open(p, errors="ignore") as f:
                        if "truth_id" in f.read():
                            file_hits.append(os.path.relpath(p, WORKSPACE))
                except OSError:
                    pass
    ok = not msg_hits and not file_hits
    ck.add("7", "truth_id 不泄漏（msg 无 truth 字段；run 目录仅评估日志含 truth_id）", ok,
           {"msg_truth_hits": msg_hits, "run_file_hits": file_hits},
           {"msg_truth_hits": [], "run_file_hits": []}, "严格 0",
           len(run_dirs), "扫描 msg/*.msg 与 %d 个 run 目录" % len(run_dirs))


def check_replay(ck, rd1, rd2):
    """8) 重放一致性：配对 bucket 上 |Δobs-Δtruth|max ≤ 1e-9（观测模型确定性判据）。"""
    def load(rd):
        d = {}
        for r in plog(rd):
            d[(r["sensor_id"], bucket(r["t_sim"]))] = r
        return d
    A, B = load(rd1), load(rd2)
    M = sorted(set(A) & set(B))
    both = [m for m in M if A[m].get("has_target") and B[m].get("has_target")]
    maxres, bad = 0.0, 0
    for m in both:
        for i in range(3):
            res = abs((B[m]["target_obs"][i] - A[m]["target_obs"][i])
                      - (B[m]["target_truth"][i] - A[m]["target_truth"][i]))
            maxres = max(maxres, res)
            if res > 1e-9:
                bad += 1
    te = [m for m in both if A[m]["target_truth"] == B[m]["target_truth"]]
    te_eq = sum(1 for m in te if A[m]["target_obs"] == B[m]["target_obs"])
    ok = (maxres <= 1e-9 and len(both) > 0)
    ck.add("8", "重放一致性 |Δobs-Δtruth|max ≤ 1e-9（观测=真值+可复现噪声）", ok,
           {"n_matched_buckets": len(M), "n_both_target": len(both),
            "max_residual": maxres, "n_residual_gt_1e-9": bad,
            "truth_equal_buckets": len(te), "truth_equal_and_obs_equal": te_eq},
           {"max_residual": 0.0, "n_residual_gt_1e-9": 0},
           "≤1e-9", len(both), "truth 相等 bucket 上 obs 严格一致 %d/%d" % (te_eq, len(te)))


def check_freeze_and_testset(ck):
    """9) a0_freeze --check PASS；冻结测试集 198 哈希匹配；eval_protocol_v1 语义未改。"""
    # a0_freeze --check
    fr = subprocess.run([sys.executable, os.path.join(SCRIPT_DIR, "a0_freeze.py"), "--check"],
                        capture_output=True, text=True)
    freeze_ok = (fr.returncode == 0 and "PASS" in fr.stdout)
    # 测试集哈希
    man = load_json(os.path.join(WORKSPACE, "experiments", "d", "manifest_test_v1.json"))
    bad = 0; n = 0
    if man:
        for e in man["entries"]:
            for k, hk in (("scenario_file", "scenario_hash"), ("sensors_file", "sensors_hash"),
                          ("world_file", "world_hash")):
                n += 1
                if sha256_file(os.path.join(WORKSPACE, e[k])) != e[hk]:
                    bad += 1
    # eval_protocol_v1 未被改动
    proto = os.path.join(WORKSPACE, "src", "air_ground_core", "config", "eval_protocol_v1.yaml")
    schema = os.path.join(WORKSPACE, "src", "air_ground_core", "config", "eval_manifest.schema.json")
    frozen = load_json(os.path.join(WORKSPACE, "experiments", "d", "protocol_v1_hashes.json")) or {}
    ff = frozen.get("files") or {}
    proto_ok = (sha256_file(proto) == ff.get("src/air_ground_core/config/eval_protocol_v1.yaml"))
    schema_ok = (sha256_file(schema) == ff.get("src/air_ground_core/config/eval_manifest.schema.json"))
    ok = freeze_ok and bad == 0 and proto_ok and schema_ok
    ck.add("9", "a0_freeze --check PASS；冻结测试集哈希匹配；eval_protocol_v1 未改", ok,
           {"a0_freeze_check": freeze_ok, "testset_mismatch": bad, "testset_checked": n,
            "eval_protocol_v1_unchanged": proto_ok, "schema_unchanged": schema_ok},
           {"a0_freeze_check": True, "testset_mismatch": 0, "eval_protocol_v1_unchanged": True},
           "严格", n, "测试集 198 = 66×3")


def main():
    ap = argparse.ArgumentParser(description="阶段3 A0 观测模型离线核验")
    ap.add_argument("--manifest", default=DEFAULT_MANIFEST)
    ap.add_argument("--runs-json", default=DEFAULT_RUNS)
    ap.add_argument("--out", default=DEFAULT_OUT)
    ap.add_argument("--nominal-run", default="")
    ap.add_argument("--occlusion-run", default="")
    ap.add_argument("--fault-run", default="")
    ap.add_argument("--maneuver-run", default="")
    ap.add_argument("--replay-run1", default="")
    ap.add_argument("--replay-run2", default="")
    args = ap.parse_args()

    man = load_json(args.manifest) or {}
    entries = {e["scenario_id"]: e for e in man.get("entries", [])}
    runs = load_json(args.runs_json) or {}
    rm = runs.get("runs", {})
    replay = runs.get("replay_run_dirs", [])

    def resolve(cli, sid, fallback=None):
        if cli:
            return norm_run(cli)
        if sid in rm:
            return norm_run(rm[sid])
        return fallback

    rd_nominal = resolve(args.nominal_run, "a0_nominal")
    rd_occ = resolve(args.occlusion_run, "a0_occlusion")
    rd_fault = resolve(args.fault_run, "a0_fault")
    rd_man = resolve(args.maneuver_run, "a0_maneuver")
    rd_r1 = norm_run(args.replay_run1) or (norm_run(replay[0]) if len(replay) > 0 else "")
    rd_r2 = norm_run(args.replay_run2) or (norm_run(replay[1]) if len(replay) > 1 else "")

    def sfile(sid):
        return os.path.join(WORKSPACE, entries[sid]["sensors_file"]) if sid in entries else ""

    def sens(sid):
        try:
            return sensors_by_id(sfile(sid))[0]
        except Exception:  # noqa: BLE001
            return {}

    ck = Checks()
    nominal_pl = plog(rd_nominal) if rd_nominal else []
    occ_pl = plog(rd_occ) if rd_occ else []
    fault_pl = plog(rd_fault) if rd_fault else []
    replay_pl = plog(rd_r1) if rd_r1 else []

    check_gating(ck, nominal_pl)
    check_occlusion(ck, occ_pl, nominal_pl)
    check_latency(ck, [nominal_pl, occ_pl, fault_pl, replay_pl,
                       plog(rd_man) if rd_man else [], plog(rd_r2) if rd_r2 else []])
    check_drop_burst(ck, fault_pl, sens("a0_fault"))
    # 虚警：a0_replay + a0_fault
    fa_map = {}
    if rd_r1:
        cfg_all = sens("a0_replay")
        by_sid = {}
        for r in replay_pl:
            by_sid.setdefault(r["sensor_id"], []).append(r)
        for sid, rows in by_sid.items():
            fa_map[sid] = (rows, cfg_all.get(sid, {}))
    check_false_alarm(ck, fa_map)
    check_noise_corr(ck, replay_pl, sens("a0_replay"))
    check_truth_leak(ck, [d for d in [rd_nominal, rd_occ, rd_fault, rd_man, rd_r1, rd_r2] if d])
    if rd_r1 and rd_r2:
        check_replay(ck, rd_r1, rd_r2)
    check_freeze_and_testset(ck)

    npass = sum(1 for c in ck.items if c["pass"])
    nfail = len(ck.items) - npass
    out = {
        "contract_version": "agc-m0-v1.2",
        "protocol": "a0_observation_model_v1",
        "manifest": os.path.relpath(args.manifest, WORKSPACE),
        "runs": {"a0_nominal": rd_nominal, "a0_occlusion": rd_occ, "a0_fault": rd_fault,
                 "a0_maneuver": rd_man, "a0_replay": [rd_r1, rd_r2]},
        "obs_hz": OBS_HZ,
        "checks": ck.items,
        "PASS": npass, "FAIL": nfail,
        "overall": "PASS" if nfail == 0 else "FAIL",
    }
    with open(args.out, "w", encoding="utf-8") as f:
        json.dump(out, f, ensure_ascii=False, indent=2)
    print("=== A0 观测模型核验 ===")
    for c in ck.items:
        print("  [%s] %-4s %s" % ("PASS" if c["pass"] else "FAIL", c["id"], c["name"]))
        if not c["pass"]:
            print("        实测=%s" % json.dumps(c["measured"], ensure_ascii=False)[:300])
    print("OUT: %s" % args.out)
    print("RESULT: %s (PASS=%d FAIL=%d)" % (out["overall"], npass, nfail))
    sys.exit(0 if nfail == 0 else 1)


if __name__ == "__main__":
    main()