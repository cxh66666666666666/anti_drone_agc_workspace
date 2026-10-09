#!/usr/bin/env python3
"""d2_stat_report.py —— 阶段3 D2「独立测试与统计报告管线」（离线 Python，无 ROS 依赖）。

职责：只读 D1 产出的批跑索引（experiments/d/batch_index_*.jsonl）与 D0 冻结的 manifest，
按 eval_protocol_v1 口径计算并落盘统计报告：
  - 总体与逐 family：attempted/valid_task/tracking_success/mission_failure/aborted/infra_failure；
    成功率点估计 + Wilson 95% 区间；失败模式占比（按归一化 reason 与 category）+ Wilson 95%；
    infra_failure_rate / abort_rate（分母 attempted）。
  - 三类分母恒等式核对（逐 family 与总体 PASS/FAIL）。
  - 预注册对照的配对效应量：风险差点估计 + bootstrap 95% 区间（固定随机种子，可复现）。
  - 判定：以效应量 95% 区间与 min_meaningful_difference_pp 的关系给出 evidence_sufficient，
    不使用“两组区间是否重叠”判定。
  - 泛化 gap：AGC 无训练集，标注 not_applicable（占位，留待地面 DQN 部分）。
  - 每个指标附原始 run 清单（run_id 列表，按 family 分列）。

只读输入；不改动任何 D0/D1 冻结对象。容忍批跑未完成/部分数据：空 family 或缺失配对时
输出 insufficient_data 而不崩溃。

用法：
    python3 src/air_ground_core/scripts/d2_stat_report.py \
        [--index experiments/d/batch_index_*.jsonl ...] \
        [--manifest experiments/d/manifest_test_v1.json ...] \
        [--out experiments/d/d2_report.json] \
        [--mde-pp 10] [--bootstrap 10000] [--seed 20261007] [--no-md] [--self-test]
"""

import argparse
import glob
import hashlib
import json
import os
import random
import sys
import time

# ---------------------------------------------------------------- 路径与常量
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
WORKSPACE = os.path.abspath(os.path.join(SCRIPT_DIR, "..", "..", ".."))
EXP_D = os.path.join(WORKSPACE, "experiments", "d")
DOC_D2 = os.path.join(WORKSPACE, "docs", "项目任务日志", "阶段3", "D", "D2.md")
PROTOCOL_HASHES = os.path.join(EXP_D, "protocol_v1_hashes.json")

DEFAULT_MANIFESTS = [
    os.path.join(EXP_D, "manifest_test_v1.json"),
    os.path.join(EXP_D, "manifest_ablation_fixed_vs_randomized_v1.json"),
    os.path.join(EXP_D, "manifest_ablation_remove_tracking_mechanism_v1.json"),
]

# 终态 → 统计分类（eval_protocol_v1.outcomes.category_by_state）
CATEGORY_BY_STATE = {4: "tracking_success", 5: "mission_failure", 6: "aborted", 8: "infra_failure"}
CATEGORIES = ("tracking_success", "mission_failure", "aborted", "infra_failure")
VALID_TASK = ("tracking_success", "mission_failure")

Z95 = 1.959963984540054
MD_BEGIN = "<!-- D2-STATS:BEGIN -->"
MD_END = "<!-- D2-STATS:END -->"


def now_iso():
    return time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())


def sha256_file(path):
    try:
        with open(path, "rb") as f:
            return hashlib.sha256(f.read()).hexdigest()
    except OSError:
        return None


# ---------------------------------------------------------------- 统计原语
def wilson(k, n, z=Z95):
    """Wilson score 95% 区间；n==0 返回 (None, None)。"""
    if n <= 0:
        return (None, None)
    p = k / n
    denom = 1.0 + z * z / n
    center = (p + z * z / (2.0 * n)) / denom
    half = z * ((p * (1.0 - p) / n + z * z / (4.0 * n * n)) ** 0.5) / denom
    return (max(0.0, center - half), min(1.0, center + half))


def percentile(sorted_vals, q):
    """线性插值分位数（不依赖 numpy）。sorted_vals 需已升序。"""
    if not sorted_vals:
        return None
    if len(sorted_vals) == 1:
        return sorted_vals[0]
    idx = q * (len(sorted_vals) - 1)
    lo = int(idx)
    hi = min(lo + 1, len(sorted_vals) - 1)
    frac = idx - lo
    return sorted_vals[lo] + frac * (sorted_vals[hi] - sorted_vals[lo])


def boot_ci(samples):
    """对 bootstrap 样本列表取 95% 百分位区间。"""
    s = sorted(v for v in samples if v is not None)
    if not s:
        return (None, None)
    return (percentile(s, 0.025), percentile(s, 0.975))


def norm_reason(r):
    """归一化 reason：去首尾空白与成对引号（兼容 "''" 这类抓取噪声）。"""
    if r is None:
        return "NONE"
    s = str(r).strip()
    while len(s) >= 2 and s[0] in "'\"" and s[-1] in "'\"":
        s = s[1:-1].strip()
    return s if s else "NONE"


def rate_block(k, n):
    """点估计 + Wilson 95% 区间 + n，供各率复用。"""
    lo, hi = wilson(k, n)
    return {
        "n": n,
        "count": k,
        "point": (k / n) if n > 0 else None,
        "ci95": [lo, hi],
    }


# ---------------------------------------------------------------- 读入
def load_index(paths):
    """读入一个或多个批跑索引 jsonl；返回全部原始行（含 _source）。"""
    rows = []
    files_meta = []
    for path in paths:
        cnt = 0
        try:
            with open(path, "r", encoding="utf-8") as f:
                for line in f:
                    line = line.strip()
                    if not line:
                        continue
                    try:
                        obj = json.loads(line)
                    except ValueError:
                        continue
                    obj["_source"] = path
                    rows.append(obj)
                    cnt += 1
        except OSError:
            files_meta.append({"path": path, "exists": False, "rows": 0})
            continue
        files_meta.append({
            "path": path, "exists": True, "sha256": sha256_file(path), "rows": cnt,
        })
    return rows, files_meta


def dedupe_final(rows):
    """同一 scenario_id 取最后一次（按 ts，其次 attempt_index）为最终结果；保留全部尝试计数。"""
    by_scn = {}
    for r in rows:
        scn = r.get("scenario_id")
        if not scn:
            continue
        by_scn.setdefault(scn, []).append(r)
    finals = {}
    attempts = {}
    for scn, lst in by_scn.items():
        attempts[scn] = len(lst)

        def key(r):
            return (str(r.get("ts") or ""), int(r.get("attempt_index") or 0))
        finals[scn] = max(lst, key=key)
    return finals, attempts


def load_manifest_map(paths):
    """加载 manifest，构建 scenario_id -> entry 映射，并记录文件哈希。"""
    m = {}
    files_meta = []
    for path in paths:
        try:
            with open(path, "r", encoding="utf-8") as f:
                data = json.load(f)
        except (OSError, ValueError):
            files_meta.append({"path": path, "exists": False, "entries": 0})
            continue
        entries = data.get("entries") or []
        for e in entries:
            scn = e.get("scenario_id")
            if scn:
                m[scn] = e
        files_meta.append({
            "path": path, "exists": True, "sha256": sha256_file(path), "entries": len(entries),
        })
    return m, files_meta


# ---------------------------------------------------------------- 分类汇总
def category_of(rec):
    cat = rec.get("category")
    if cat in CATEGORIES:
        return cat
    st = rec.get("final_state")
    if isinstance(st, int) and st in CATEGORY_BY_STATE:
        return CATEGORY_BY_STATE[st]
    return "unknown"


def run_id_of(rec):
    meta = rec.get("meta") or {}
    return meta.get("run_id") or rec.get("run_id")


def family_of(rec, manifest_map):
    """family 以索引为准，缺失时回落到 manifest。"""
    fam = rec.get("family")
    if fam:
        return fam
    e = manifest_map.get(rec.get("scenario_id")) or {}
    return e.get("family") or "unknown"


def summarize_group(records):
    """对一组最终结果记录做分类汇总 + 各率 + Wilson + 失败模式。"""
    cats = {c: 0 for c in CATEGORIES}
    reasons = {}
    run_ids = {c: [] for c in CATEGORIES}
    run_ids["unknown"] = []
    for rec in records:
        cat = category_of(rec)
        if cat in cats:
            cats[cat] += 1
            run_ids[cat].append(run_id_of(rec))
        else:
            run_ids["unknown"].append(run_id_of(rec))
        if cat != "tracking_success":
            rs = norm_reason(rec.get("reason"))
            reasons.setdefault(rs, {"count": 0, "run_ids": []})
            reasons[rs]["count"] += 1
            reasons[rs]["run_ids"].append(run_id_of(rec))

    attempted = len(records)
    valid_task = cats["tracking_success"] + cats["mission_failure"]
    infra = cats["infra_failure"]
    aborted = cats["aborted"]

    failure_modes = {
        "denominator": "attempted",
        "by_reason": {
            rs: {"count": d["count"], **rate_block(d["count"], attempted),
                 "run_ids": d["run_ids"]}
            for rs, d in sorted(reasons.items())
        },
        "by_category": {
            c: {"count": cats[c], **rate_block(cats[c], attempted), "run_ids": run_ids[c]}
            for c in ("mission_failure", "aborted", "infra_failure")
        },
    }

    return {
        "attempted": attempted,
        "valid_task": valid_task,
        "tracking_success": cats["tracking_success"],
        "mission_failure": cats["mission_failure"],
        "aborted": aborted,
        "infra_failure": infra,
        "tracking_success_rate": rate_block(cats["tracking_success"], valid_task),
        "infra_failure_rate": rate_block(infra, attempted),
        "abort_rate": rate_block(aborted, attempted),
        "failure_modes": failure_modes,
        "run_ids": {"all": [run_id_of(r) for r in records], "by_category": run_ids},
    }


def denominator_equations(s):
    """核对两条三类分母恒等式。"""
    eq1_lhs = s["attempted"]
    eq1_rhs = s["valid_task"] + s["infra_failure"] + s["aborted"]
    eq2_lhs = s["valid_task"]
    eq2_rhs = s["tracking_success"] + s["mission_failure"]
    return {
        "attempted_eq_valid_plus_infra_plus_aborted": {
            "lhs": eq1_lhs, "rhs": eq1_rhs, "pass": eq1_lhs == eq1_rhs,
        },
        "valid_task_eq_success_plus_mission_failure": {
            "lhs": eq2_lhs, "rhs": eq2_rhs, "pass": eq2_lhs == eq2_rhs,
        },
        "pass": (eq1_lhs == eq1_rhs) and (eq2_lhs == eq2_rhs),
    }


# ---------------------------------------------------------------- 配对/效应量
def paired_risk_difference(pairs, n_boot, rng):
    """配对风险差：pairs = [(a_success, b_success)]，RD = mean(a) - mean(b)，bootstrap 95%。"""
    if not pairs:
        return None
    a = [1 if p[0] else 0 for p in pairs]
    b = [1 if p[1] else 0 for p in pairs]
    point = (sum(a) / len(a)) - (sum(b) / len(b))
    m = len(pairs)
    samples = []
    for _ in range(n_boot):
        idxs = [rng.randrange(m) for _ in range(m)]
        sa = sum(a[i] for i in idxs) / m
        sb = sum(b[i] for i in idxs) / m
        samples.append(sa - sb)
    lo, hi = boot_ci(samples)
    return {"point": point, "ci95": [lo, hi], "n_pairs": m}


def unpaired_risk_difference(a_success, b_success, n_boot, rng):
    """非配对风险差：RD = mean(a) - mean(b)，各自独立重采样，bootstrap 95%。"""
    if not a_success or not b_success:
        return None
    na, nb = len(a_success), len(b_success)
    point = (sum(a_success) / na) - (sum(b_success) / nb)
    samples = []
    for _ in range(n_boot):
        sa = sum(a_success[rng.randrange(na)] for _ in range(na)) / na
        sb = sum(b_success[rng.randrange(nb)] for _ in range(nb)) / nb
        samples.append(sa - sb)
    lo, hi = boot_ci(samples)
    return {"point": point, "ci95": [lo, hi], "n_arm_a": na, "n_arm_b": nb}


def decide(effect, mde):
    """以效应量 95% 区间与 ±MDE 比较判定证据是否充分（非区间重叠法）。"""
    if effect is None:
        return {
            "evidence_sufficient": False,
            "reason": "insufficient_data",
            "mde_pp": mde * 100.0,
        }
    lo, hi = effect["ci95"]
    sufficient = (lo is not None and hi is not None) and (lo > mde or hi < -mde)
    if lo is None:
        reason = "insufficient_data"
    elif sufficient:
        reason = "ci_entirely_outside_mde"
    else:
        reason = "ci_overlaps_mde_band"
    return {
        "evidence_sufficient": bool(sufficient),
        "ci95": [lo, hi],
        "mde_pp": mde * 100.0,
        "decision_rule": "effect CI vs ±MDE（区间整体 > MDE 或整体 < -MDE 才充分）；不以两组区间是否重叠判定",
        "reason": reason,
    }


def build_comparisons(finals, manifest_map, n_boot, seed, mde):
    """构建预注册对照 C1/C2/C3。"""
    # 分组：nominal(gt_adapter)、ablation_track_mech(air_fusion)、ablation_fixed
    nominal = {}
    fusion = {}
    fixed = {}
    for scn, rec in finals.items():
        fam = family_of(rec, manifest_map)
        if fam == "ablation_track_mech":
            fusion[scn] = rec
        elif fam == "ablation_fixed":
            fixed[scn] = rec
        elif fam == "nominal":
            nominal[scn] = rec

    def succ(rec):
        return category_of(rec) == "tracking_success"

    def rid(rec):
        return run_id_of(rec)

    # ---- C1/C3：严格按 pair_id 配对（a=air_fusion 臂, b=gt_adapter 配对子集）
    pairs = []
    pair_detail = []
    missing_pair_ids = []
    matched_base_ids = []
    for scn, rec in sorted(fusion.items()):
        me = manifest_map.get(scn) or {}
        pid = me.get("pair_id")
        if not pid:                       # 缺 pair_id 视为缺臂
            missing_pair_ids.append(scn)
            continue
        base = nominal.get(pid)
        if base is None:                  # pair_id 指向的 nominal 结果缺失
            missing_pair_ids.append(pid)
            continue
        pairs.append((succ(rec), succ(base)))  # (a=air_fusion, b=gt_adapter)
        matched_base_ids.append(pid)
        pair_detail.append({"pair_id": pid, "a_run": rid(rec), "b_run": rid(base),
                            "a_success": succ(rec), "b_success": succ(base)})

    rng = random.Random(seed)
    eff_c1 = paired_risk_difference(pairs, n_boot, rng)  # RD = p(air_fusion) - p(gt_adapter)
    comparisons = []
    gt_ids = sorted(nominal.keys())
    fus_ids = sorted(fusion.keys())
    gt_run_ids = [rid(nominal[s]) for s in gt_ids]
    fus_run_ids = [rid(fusion[s]) for s in fus_ids]
    # arm_b 仅取“配对上”的 nominal 子集（严格配对，n 与 ablation_track_mech 对齐）
    matched_gt_run_ids = [rid(nominal[p]) for p in matched_base_ids]

    def comp_entry(cid, arm_a, arm_b, effect, extra, insufficient=None, pairs=None):
        insuff = (effect is None) if insufficient is None else insufficient
        entry = {
            "id": cid,
            "kind": "paired" if (effect and "n_pairs" in effect) else "unpaired",
            "arm_a": arm_a,
            "arm_b": arm_b,
            "metric": "tracking_success_rate",
            "risk_difference_definition": "p(arm_a) - p(arm_b)",
            "risk_difference": (effect or {}).get("point"),
            "ci95": (effect or {}).get("ci95"),
            "insufficient_data": insuff,
            "raw_run_ids": {"arm_a": arm_a.get("run_ids"), "arm_b": arm_b.get("run_ids"),
                            "pairs": pairs or []},
        }
        for k in ("n_pairs", "n_arm_a", "n_arm_b"):
            if effect and k in effect:
                entry[k] = effect[k]
        entry.update(extra)
        entry["decision"] = decide(effect, mde)
        if insuff:                        # 数据不全时判定一律不充分
            entry["decision"]["evidence_sufficient"] = False
            entry["decision"]["reason"] = "insufficient_data"
        return entry

    # C1 与 C3 是同一组臂（协议：C1 a=B_fusion,b=B_gt；C3 a=without_tracking_mechanism(air_fusion),
    # b=full_stack(gt_adapter)），arm_a/arm_b 与风险差同向。
    insuff = (eff_c1 is None) or (len(missing_pair_ids) > 0)
    arm_fusion = {"label": "ablation_track_mech (air_fusion)", "run_ids": fus_run_ids,
                  "rate": rate_block(sum(1 if succ(fusion[s]) else 0 for s in fus_ids), len(fus_ids))}
    arm_gt_paired = {"label": "test nominal paired subset (gt_adapter)", "run_ids": matched_gt_run_ids,
                     "rate": rate_block(sum(1 if succ(nominal[p]) else 0 for p in matched_base_ids),
                                        len(matched_base_ids))}
    c1 = comp_entry("C1", dict(arm_fusion, label="B_fusion / " + arm_fusion["label"]),
                    dict(arm_gt_paired, label="B_gt / " + arm_gt_paired["label"]),
                    eff_c1, {"paired_by": "pair_id", "missing_pair_ids": missing_pair_ids,
                             "hypothesis": "多源门控与融合相对真值直供的成功率差"},
                    insufficient=insuff, pairs=pair_detail)
    c3 = comp_entry("C3", dict(arm_fusion, label="without_tracking_mechanism / " + arm_fusion["label"]),
                    dict(arm_gt_paired, label="full_stack / " + arm_gt_paired["label"]),
                    eff_c1, {"paired_by": "pair_id", "missing_pair_ids": missing_pair_ids,
                             "hypothesis": "去掉关键跟踪机制后的成功率下降",
                             "note": "与 C1 共用同一臂对，arm_a/arm_b 与风险差同向"},
                    insufficient=insuff, pairs=pair_detail)
    comparisons += [c1, c3]

    # ---- C2：固定 vs 随机化（a=randomized/test nominal 族, b=fixed/ablation_fixed），非配对
    fx_ids = sorted(fixed.keys())
    a_fixed = [1 if succ(fixed[s]) else 0 for s in fx_ids]
    b_rand = [1 if succ(nominal[s]) else 0 for s in gt_ids]
    eff_c2 = unpaired_risk_difference(b_rand, a_fixed, n_boot, rng)  # RD = p_randomized - p_fixed
    c2 = comp_entry("C2",
                    {"label": "randomized / test nominal", "run_ids": gt_run_ids,
                     "rate": rate_block(sum(b_rand), len(b_rand))},
                    {"label": "fixed / ablation_fixed", "run_ids": [rid(fixed[s]) for s in fx_ids],
                     "rate": rate_block(sum(a_fixed), len(a_fixed))},
                    eff_c2, {"paired_by": None, "hypothesis": "随机化场景分布相对固定场景的成功率差"})
    comparisons.append(c2)

    # ---- perception_fault_v2：修正版，单独成条目，不与 v1 合并
    pf_v2 = {s: r for s, r in finals.items()
             if family_of(r, manifest_map) == "perception_fault_v2"}
    pf_v1 = {s: r for s, r in finals.items()
             if family_of(r, manifest_map) == "perception_fault"}
    if pf_v2:
        v2_ids = sorted(pf_v2.keys())
        v1_ids = sorted(pf_v1.keys())
        v2_succ = [1 if succ(pf_v2[s]) else 0 for s in v2_ids]
        v1_succ = [1 if succ(pf_v1[s]) else 0 for s in v1_ids]
        eff_v2 = unpaired_risk_difference(v2_succ, v1_succ, n_boot, rng)  # RD = p_v2 - p_v1
        cpf = comp_entry("C_perception_fault_v2",
                         {"label": "perception_fault_v2 (修正版)", "run_ids": [rid(pf_v2[s]) for s in v2_ids],
                          "rate": rate_block(sum(v2_succ), len(v2_succ))},
                         {"label": "perception_fault (v1)", "run_ids": [rid(pf_v1[s]) for s in v1_ids],
                          "rate": rate_block(sum(v1_succ), len(v1_succ))},
                         eff_v2, {"paired_by": None, "hypothesis": "perception_fault 修正版相对 v1 的成功率差",
                                  "note": "perception_fault_v2 单独成族/条目，不与 v1 合并"})
        comparisons.append(cpf)
    return comparisons


# ---------------------------------------------------------------- 报告
def build_report(args):
    index_paths = []
    for pat in args.index:
        hits = sorted(glob.glob(pat)) if any(c in pat for c in "*?[") else [pat]
        index_paths.extend(hits)
    index_paths = sorted(set(index_paths))

    rows, index_files = load_index(index_paths)
    finals, attempts = dedupe_final(rows)
    manifest_map, manifest_files = load_manifest_map(args.manifest)

    # 逐 family + 总体
    fam_groups = {}
    for rec in finals.values():
        fam = family_of(rec, manifest_map)
        fam_groups.setdefault(fam, []).append(rec)

    per_family = {}
    denom_eq = {}
    for fam in sorted(fam_groups):
        s = summarize_group(fam_groups[fam])
        per_family[fam] = s
        denom_eq[fam] = denominator_equations(s)

    overall = summarize_group(list(finals.values()))
    denom_eq["OVERALL"] = denominator_equations(overall)

    # 尝试计数审计
    attempt_audit = {
        "rows_total": len(rows),
        "scenarios_total": len(finals),
        "retried_scenarios": sorted([s for s, n in attempts.items() if n > 1]),
        "attempts_per_scenario": {s: attempts[s] for s in sorted(attempts)},
    }

    mde = args.mde_pp / 100.0
    comparisons = build_comparisons(finals, manifest_map, args.bootstrap, args.seed, mde)

    # 泛化 gap 占位
    generalization_gap = {
        "status": "not_applicable",
        "note": "AGC 无训练集；泛化 gap(train_rate - test_rate) 由地面 DQN 部分给出，此处留字段占位。",
        "value": None,
    }

    try:
        with open(PROTOCOL_HASHES, "r", encoding="utf-8") as f:
            protocol_hashes = json.load(f)
    except (OSError, ValueError):
        protocol_hashes = None

    report = {
        "report_version": "d2-report-v1",
        "generated_at": now_iso(),
        "protocol": "eval_protocol_v1",
        "params": {"mde_pp": args.mde_pp, "bootstrap": args.bootstrap, "seed": args.seed},
        "index_sources": index_files,
        "manifest_sources": manifest_files,
        "hashes": {
            "protocol": (protocol_hashes or {}).get("combined_hash"),
            "protocol_frozen_at": (protocol_hashes or {}).get("frozen_at"),
            "protocol_files": (protocol_hashes or {}).get("files"),
            "n_index_rows": len(rows),
        },
        "attempt_audit": attempt_audit,
        "overall": overall,
        "per_family": per_family,
        "denominator_equations": denom_eq,
        "comparisons": comparisons,
        "generalization_gap": generalization_gap,
        "raw_run_ids": {fam: per_family[fam]["run_ids"] for fam in per_family},
        "raw_run_ids_overall": overall["run_ids"],
        "notes": [
            "成功率主指标分母为 valid_task；infra_failure_rate/abort_rate 分母为 attempted。",
            "同一 scenario_id 多次尝试取最后一次为最终结果，全部尝试计数保留于 attempt_audit（不静默抹去）。",
            "证据充分性判定使用效应量 95% 区间与 ±MDE 比较，不使用“两组区间是否重叠”。",
        ],
    }
    return report


# ---------------------------------------------------------------- 打印
def _f(v):
    return "None" if v is None else "%.4f" % v


def print_report(report):
    p = print
    p("=" * 78)
    p("D2 统计报告  generated_at=%s  seed=%s  bootstrap=%s  mde_pp=%s"
      % (report["generated_at"], report["params"]["seed"], report["params"]["bootstrap"], report["params"]["mde_pp"]))
    p("index: %s" % ", ".join("%s(%d rows)" % (f["path"].split("/")[-1], f.get("rows", 0))
                              for f in report["index_sources"]))
    p("-" * 78)

    def show(title, s):
        p("[%s] attempted=%d valid_task=%d success=%d mission_failure=%d aborted=%d infra=%d"
          % (title, s["attempted"], s["valid_task"], s["tracking_success"],
             s["mission_failure"], s["aborted"], s["infra_failure"]))
        r = s["tracking_success_rate"]
        p("    tracking_success_rate=%s n=%d ci95=[%s, %s]"
          % (_f(r["point"]), r["n"], _f(r["ci95"][0]), _f(r["ci95"][1])))
        ir = s["infra_failure_rate"]
        ar = s["abort_rate"]
        p("    infra_failure_rate=%s ci95=[%s, %s]  abort_rate=%s ci95=[%s, %s]"
          % (_f(ir["point"]), _f(ir["ci95"][0]), _f(ir["ci95"][1]),
             _f(ar["point"]), _f(ar["ci95"][0]), _f(ar["ci95"][1])))
        for rs, d in s["failure_modes"]["by_reason"].items():
            p("    reason %-20s count=%d p=%s ci95=[%s, %s]"
              % (rs, d["count"], _f(d["point"]), _f(d["ci95"][0]), _f(d["ci95"][1])))

    show("OVERALL", report["overall"])
    for fam in sorted(report["per_family"]):
        show(fam, report["per_family"][fam])

    p("-" * 78)
    for key in sorted(report["denominator_equations"]):
        eq = report["denominator_equations"][key]
        p("denominator[%s]: identity1 PASS=%s  identity2 PASS=%s  => %s"
          % (key, eq["attempted_eq_valid_plus_infra_plus_aborted"]["pass"],
             eq["valid_task_eq_success_plus_mission_failure"]["pass"],
             "PASS" if eq["pass"] else "FAIL"))

    p("-" * 78)
    for c in report["comparisons"]:
        d = c["decision"]
        n_txt = c.get("n_pairs") or (str(c.get("n_arm_a")) + "/" + str(c.get("n_arm_b")))
        if c["insufficient_data"]:
            p("comparison %s: insufficient_data (arm_a=%d, arm_b=%d) evidence_sufficient=%s"
              % (c["id"], len(c["raw_run_ids"]["arm_a"]), len(c["raw_run_ids"]["arm_b"]), d["evidence_sufficient"]))
        else:
            p("comparison %s [%s]: arm_a=%s(n=%d) arm_b=%s(n=%d) RD=%s ci95=[%s, %s] n=%s evidence_sufficient=%s (%s)"
              % (c["id"], c["kind"], c["arm_a"]["label"], len(c["raw_run_ids"]["arm_a"]),
                 c["arm_b"]["label"], len(c["raw_run_ids"]["arm_b"]), _f(c["risk_difference"]),
                 _f(c["ci95"][0]), _f(c["ci95"][1]), n_txt, d["evidence_sufficient"], d["reason"]))
            if c.get("missing_pair_ids"):
                p("    missing_pair_ids=%s（该对照记 insufficient_data）" % c["missing_pair_ids"])
    p("generalization_gap: %s (%s)" % (report["generalization_gap"]["status"], report["generalization_gap"]["note"]))
    p("=" * 78)


# ---------------------------------------------------------------- Markdown
def render_md(report):
    L = []
    L.append("## 统计结果")
    L.append("")
    L.append("%s" % MD_BEGIN)
    L.append("")
    L.append("- 生成时间：`%s`；协议：`eval_protocol_v1`（combined_hash=`%s`）"
             % (report["generated_at"], report["hashes"]["protocol"]))
    L.append("- 参数：seed=`%s`，bootstrap=`%s`，mde_pp=`%s`"
             % (report["params"]["seed"], report["params"]["bootstrap"], report["params"]["mde_pp"]))
    L.append("- 输入索引：%s" % "；".join("`%s`(%d 行)" % (f["path"], f.get("rows", 0))
                                          for f in report["index_sources"]))
    L.append("- 尝试审计：索引总行数=%d，去重场景数=%d，重试场景=%s"
             % (report["attempt_audit"]["rows_total"], report["attempt_audit"]["scenarios_total"],
                report["attempt_audit"]["retried_scenarios"] or "无"))
    L.append("")

    L.append("### 成功率（点估计 + Wilson 95%）与分类计数")
    L.append("")
    L.append("| 分组 | attempted | valid_task | tracking_success | mission_failure | aborted | infra_failure | 成功率 | Wilson 95% |")
    L.append("| --- | --- | --- | --- | --- | --- | --- | --- | --- |")

    def row(name, s):
        r = s["tracking_success_rate"]
        return "| %s | %d | %d | %d | %d | %d | %d | %s | [%s, %s] |" % (
            name, s["attempted"], s["valid_task"], s["tracking_success"], s["mission_failure"],
            s["aborted"], s["infra_failure"], _f(r["point"]), _f(r["ci95"][0]), _f(r["ci95"][1]))

    L.append(row("OVERALL", report["overall"]))
    for fam in sorted(report["per_family"]):
        L.append(row(fam, report["per_family"][fam]))
    L.append("")

    L.append("### 次要率与失败模式（分母 attempted）")
    L.append("")
    L.append("| 分组 | infra_failure_rate | abort_rate | 失败原因(count, p, Wilson95) |")
    L.append("| --- | --- | --- | --- |")

    def fm_cell(s):
        parts = []
        for rs, d in s["failure_modes"]["by_reason"].items():
            parts.append("%s: %d, %s [%s, %s]" % (rs, d["count"], _f(d["point"]),
                                                  _f(d["ci95"][0]), _f(d["ci95"][1])))
        return "; ".join(parts) if parts else "无"

    def row2(name, s):
        return "| %s | %s [%s, %s] | %s [%s, %s] | %s |" % (
            name, _f(s["infra_failure_rate"]["point"]), _f(s["infra_failure_rate"]["ci95"][0]),
            _f(s["infra_failure_rate"]["ci95"][1]), _f(s["abort_rate"]["point"]),
            _f(s["abort_rate"]["ci95"][0]), _f(s["abort_rate"]["ci95"][1]), fm_cell(s))

    L.append(row2("OVERALL", report["overall"]))
    for fam in sorted(report["per_family"]):
        L.append(row2(fam, report["per_family"][fam]))
    L.append("")

    L.append("### 三类分母恒等式核对")
    L.append("")
    L.append("| 分组 | attempted=valid+infra+aborted | valid=success+mission_failure | 结论 |")
    L.append("| --- | --- | --- | --- |")
    for key in sorted(report["denominator_equations"]):
        eq = report["denominator_equations"][key]
        L.append("| %s | %s | %s | %s |" % (
            key, "PASS" if eq["attempted_eq_valid_plus_infra_plus_aborted"]["pass"] else "FAIL",
            "PASS" if eq["valid_task_eq_success_plus_mission_failure"]["pass"] else "FAIL",
            "**PASS**" if eq["pass"] else "**FAIL**"))
    L.append("")

    L.append("### 预注册对照效应量（风险差 + bootstrap 95%）")
    L.append("")
    L.append("| 对照 | 类型 | arm_a | arm_b | 风险差 | bootstrap 95% | 证据充分 | 判定依据 |")
    L.append("| --- | --- | --- | --- | --- | --- | --- | --- |")
    for c in report["comparisons"]:
        d = c["decision"]
        a = "%s(n=%d)" % (c["arm_a"]["label"], len(c["raw_run_ids"]["arm_a"]))
        b = "%s(n=%d)" % (c["arm_b"]["label"], len(c["raw_run_ids"]["arm_b"]))
        if c["insufficient_data"]:
            miss = c.get("missing_pair_ids") or []
            L.append("| %s | %s | %s | %s | insufficient_data | - | 否 | 缺臂或缺配对%s |"
                     % (c["id"], c["kind"], a, b,
                        ("，缺失 pair_id=%s" % miss) if miss else ""))
        else:
            L.append("| %s | %s | %s | %s | %s | [%s, %s] | %s | %s |" % (
                c["id"], c["kind"], a, b, _f(c["risk_difference"]),
                _f(c["ci95"][0]), _f(c["ci95"][1]),
                "是" if d["evidence_sufficient"] else "否", d["reason"]))
    L.append("")
    L.append("> 判定规则：效应量 95%% 区间整体 > +MDE 或整体 < -MDE 才算证据充分；**不以“两组区间是否重叠”判定**。"
             "MDE=%s pp。" % report["params"]["mde_pp"])
    L.append("")
    L.append("### 泛化 gap")
    L.append("")
    L.append("- `%s`：%s" % (report["generalization_gap"]["status"], report["generalization_gap"]["note"]))
    L.append("")

    L.append("### 原始 run 清单（可审计）")
    L.append("")
    for fam in sorted(report["raw_run_ids"]):
        s = report["raw_run_ids"][fam]["by_category"]
        L.append("- **%s**：all=%s；tracking_success=%s；mission_failure=%s；aborted=%s；infra_failure=%s"
                 % (fam, report["raw_run_ids"][fam]["all"], s.get("tracking_success"),
                    s.get("mission_failure"), s.get("aborted"), s.get("infra_failure")))
    L.append("- **overall**：all=%s" % report["raw_run_ids_overall"]["all"])
    # C1/C3 配对关系（pair_id -> 两臂 run）
    for c in report["comparisons"]:
        if c["id"] in ("C1", "C3") and c["raw_run_ids"].get("pairs"):
            L.append("- **%s 配对**：%s" % (c["id"], "；".join(
                "%s[a=%s,b=%s]" % (p["pair_id"], p["a_run"], p["b_run"])
                for p in c["raw_run_ids"]["pairs"])))
    L.append("")
    L.append(MD_END)
    return "\n".join(L)


def write_md(report):
    section = render_md(report)
    if os.path.exists(DOC_D2):
        with open(DOC_D2, "r", encoding="utf-8") as f:
            content = f.read()
        b = content.rfind(MD_BEGIN)
        e = content.find(MD_END, b) if b >= 0 else -1
        if b >= 0 and e >= 0:
            head = content[:b].rstrip("\n")
            tail = content[e + len(MD_END):]
            if head.endswith("## 统计结果"):
                head = head[: -len("## 统计结果")].rstrip("\n")
            new = head + "\n\n" + section + tail
        else:
            content = content.rstrip("\n")
            new = content + "\n\n" + section + "\n"
    else:
        os.makedirs(os.path.dirname(DOC_D2), exist_ok=True)
        new = "# D2：独立测试与统计报告管线\n\n> 本文由 `src/air_ground_core/scripts/d2_stat_report.py` 生成/更新；"
        new += "统计段落位于文内 `D2-STATS` 标记块之间，重跑只替换该区间，不覆盖 D0/D1 内容。\n\n"
        new += section + "\n"
    with open(DOC_D2, "w", encoding="utf-8") as f:
        f.write(new)
    return DOC_D2


# ---------------------------------------------------------------- 自检
def self_test():
    """Wilson 正确性对拍 + bootstrap 可复现证据；全部打印期望/实际。"""
    ok = True
    print("[self-test] Wilson 95% 对拍（容差 1e-3）")
    cases = [
        (5, 10, (0.2366, 0.7634), "both"),
        (0, 20, (None, 0.1611), "upper"),
        (20, 20, (0.8389, None), "lower"),
    ]
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

    print("[self-test] bootstrap 配对效应量可复现（合成小数据，固定 seed，不入库）")
    # 合成 8 对（a=机制臂, b=基线臂），确定性构造
    synth = [(1, 1), (1, 0), (0, 1), (0, 0), (1, 1), (0, 0), (1, 0), (0, 1)]
    for s in (20261007, 20261007):
        rng = random.Random(s)
        eff = paired_risk_difference(synth, 10000, rng)
        print("  seed=%d -> RD=%.6f ci95=[%.6f, %.6f]"
              % (s, eff["point"], eff["ci95"][0], eff["ci95"][1]))
    r1 = paired_risk_difference(synth, 10000, random.Random(20261007))
    r2 = paired_risk_difference(synth, 10000, random.Random(20261007))
    same = r1 == r2
    ok = ok and same
    print("  两次同 seed 结果一致：%s" % ("PASS" if same else "FAIL"))
    print("[self-test] 总结果：%s" % ("PASS" if ok else "FAIL"))
    return 0 if ok else 1


# ---------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser(description="阶段3 D2 独立测试与统计报告管线（离线）")
    ap.add_argument("--index", action="append", default=None,
                    help="批跑索引 jsonl，可多次；默认全部 experiments/d/batch_index_*.jsonl")
    ap.add_argument("--manifest", action="append", default=None,
                    help="manifest json，可多次；默认 test + 两套消融")
    ap.add_argument("--out", default=os.path.join(EXP_D, "d2_report.json"))
    ap.add_argument("--mde-pp", type=float, default=10.0, help="最小有意义差异（百分点）")
    ap.add_argument("--bootstrap", type=int, default=10000, help="bootstrap 重采样次数")
    ap.add_argument("--seed", type=int, default=20261007, help="bootstrap 随机种子")
    ap.add_argument("--no-md", action="store_true", help="不写 D2.md")
    ap.add_argument("--self-test", action="store_true", help="运行 Wilson/bootstrap 自检并退出")
    args = ap.parse_args()

    if args.self_test:
        sys.exit(self_test())

    if args.index is None:
        args.index = [os.path.join(EXP_D, "batch_index_*.jsonl")]
    if args.manifest is None:
        args.manifest = DEFAULT_MANIFESTS

    report = build_report(args)
    print_report(report)

    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    with open(args.out, "w", encoding="utf-8") as f:
        json.dump(report, f, ensure_ascii=False, indent=2)
    print("已写出：%s" % args.out)

    if not args.no_md:
        path = write_md(report)
        print("已更新：%s" % path)


if __name__ == "__main__":
    main()
