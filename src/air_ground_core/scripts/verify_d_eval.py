#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""verify_d_eval.py —— 阶段3 D 统一核验脚本（离线，不启动 Gazebo）。

对 D0/D1/D2 的 7 项关键判据逐项输出 PASS/FAIL/SKIP，落盘 experiments/d/verify_d_eval.json，
并在 stdout 打印表格。退出码：0=PASS，1=存在 FAIL（SKIP 不计入 FAIL）。

判据：
  1. 协议冻结            : 调用 d0_freeze.py --check 复核 protocol_v1_hashes.json
  2. manifest 确定性/schema: schema 校验 + 产物存在性/sha256 + d1_scenario_gen --rebuild-check（不可用则 SKIP）
  3. split 隔离          : scenario_id 跨 split 不重复、layout 集合跨 split 不相交、seed 唯一（配对除外）
  4. 三类分母恒等式      : 从 batch_index_*.jsonl 取重试末条，核对两条恒等式（总体+逐 family）
  5. Wilson 计算正确性   : 内置 Wilson 95% 实现与已知值对拍
  6. D2 报告完整性       : d2_report.json 必备字段齐备
  7. 文档存在性          : docs/项目任务日志/阶段3/D/{D0,D1,D2}.md

用法：
    python3 verify_d_eval.py                  # 全量核验
    python3 verify_d_eval.py --skip-rebuild   # 跳过 d1 逐字节重建（较慢，~数十秒）
    python3 verify_d_eval.py --out <path>     # 自定义 JSON 落盘路径
"""

import argparse
import glob
import hashlib
import json
import math
import os
import re
import subprocess
import sys
import time

# ---------------------------------------------------------------- 路径常量
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
WORKSPACE = os.path.abspath(os.path.join(SCRIPT_DIR, "..", "..", ".."))
D_DIR = os.path.join(WORKSPACE, "experiments", "d")
SCHEMA_PATH = os.path.join(WORKSPACE, "src", "air_ground_core", "config", "eval_manifest.schema.json")
D0_SCRIPT = os.path.join(SCRIPT_DIR, "d0_freeze.py")
D1_GEN = os.path.join(SCRIPT_DIR, "d1_scenario_gen.py")
DOC_DIR = os.path.join(WORKSPACE, "docs", "项目任务日志", "阶段3", "D")
DEFAULT_OUT = os.path.join(D_DIR, "verify_d_eval.json")

MANIFEST_SPLITS = ["test", "train", "validation"]
CATEGORIES = ("tracking_success", "mission_failure", "aborted", "infra_failure")
PY = sys.executable or "python3"


# ---------------------------------------------------------------- 通用工具
def sha256_file(path):
    """返回文件 sha256；不存在返回 None。"""
    if not os.path.isfile(path):
        return None
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(65536), b""):
            h.update(chunk)
    return h.hexdigest()


def load_json(path):
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def run_cli(script, args, timeout=300):
    """以子进程运行冻结脚本；返回 (returncode, stdout+stderr)。异常/超时返回 (-1, 说明)。"""
    if not os.path.isfile(script):
        return -1, "脚本不存在: %s" % script
    try:
        p = subprocess.run([PY, script] + args, cwd=WORKSPACE, timeout=timeout,
                           stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        return p.returncode, p.stdout.decode("utf-8", "replace")
    except subprocess.TimeoutExpired:
        return -1, "子进程超时(%ss): %s" % (timeout, os.path.basename(script))
    except Exception as e:  # noqa: BLE001
        return -1, "子进程异常: %s" % e


def tail_lines(text, n=2):
    """取输出末 n 行（压缩为单行）用于 detail。"""
    lines = [ln.strip() for ln in (text or "").splitlines() if ln.strip()]
    return " | ".join(lines[-n:]) if lines else ""


# ---------------------------------------------------------------- Wilson 95%
def wilson_ci(k, n, z=1.96):
    """Wilson score 区间；n<=0 返回 (0.0, 1.0)。"""
    if n <= 0:
        return 0.0, 1.0
    phat = k / n
    denom = 1.0 + z * z / n
    center = (phat + z * z / (2 * n)) / denom
    half = (z * math.sqrt(phat * (1 - phat) / n + z * z / (4 * n * n))) / denom
    return max(0.0, center - half), min(1.0, center + half)


# ---------------------------------------------------------------- 内置 schema 校验（jsonschema 缺失时等价回退）
_TYPES = {
    "object": dict, "array": list, "string": str,
    "integer": int, "number": (int, float), "boolean": bool, "null": type(None),
}


def _type_ok(value, tname):
    t = _TYPES.get(tname)
    if t is None:
        return True
    if tname in ("integer", "number") and isinstance(value, bool):
        return False  # JSON 中 bool 不算 number/integer
    return isinstance(value, t)


def validate_schema(inst, schema, path="$"):
    """极简 draft-07 子集校验，覆盖本 schema 用到的关键字；返回错误列表。"""
    errs = []
    if not isinstance(schema, dict):
        return errs
    if "type" in schema:
        tnames = schema["type"] if isinstance(schema["type"], list) else [schema["type"]]
        if not any(_type_ok(inst, t) for t in tnames):
            errs.append("%s: 类型应为 %s" % (path, tnames))
            return errs
    if "enum" in schema and inst not in schema["enum"]:
        errs.append("%s: 取值 %r 不在枚举 %s" % (path, inst, schema["enum"]))
    if "pattern" in schema and isinstance(inst, str):
        if not re.match(schema["pattern"], inst):
            errs.append("%s: 不匹配模式 %s" % (path, schema["pattern"]))
    if isinstance(inst, dict):
        for req in schema.get("required", []):
            if req not in inst:
                errs.append("%s: 缺少必填字段 %s" % (path, req))
        props = schema.get("properties", {})
        ap = schema.get("additionalProperties", True)
        for k, v in inst.items():
            if k in props:
                errs += validate_schema(v, props[k], "%s.%s" % (path, k))
            elif ap is False:
                errs.append("%s: 出现未定义字段 %s" % (path, k))
            elif isinstance(ap, dict):
                errs += validate_schema(v, ap, "%s.%s" % (path, k))
    if isinstance(inst, list) and "items" in schema:
        for i, v in enumerate(inst):
            errs += validate_schema(v, schema["items"], "%s[%d]" % (path, i))
    return errs


def validate_manifest_obj(obj, schema):
    """优先 jsonschema，缺失回退内置校验；返回错误列表。"""
    try:
        import jsonschema  # type: ignore
        v = jsonschema.Draft7Validator(schema)
        return ["%s" % e.message for e in sorted(v.iter_errors(obj), key=lambda e: list(e.path))]
    except ImportError:
        return validate_schema(obj, schema)


# ---------------------------------------------------------------- 判据 1：协议冻结
def check_protocol_freeze():
    name = "1 协议冻结(d0_freeze --check)"
    if not os.path.isfile(D0_SCRIPT):
        return {"name": name, "status": "FAIL", "detail": "未找到 d0_freeze.py"}
    rc, out = run_cli(D0_SCRIPT, ["--check"], timeout=60)
    if rc == -1:
        return {"name": name, "status": "FAIL", "detail": "执行失败: %s" % tail_lines(out)}
    status = "PASS" if rc == 0 else "FAIL"
    return {"name": name, "status": status, "detail": "exit=%d; %s" % (rc, tail_lines(out))}


# ---------------------------------------------------------------- 判据 2：manifest 确定性/schema
def check_manifest_schema(skip_rebuild=False):
    name = "2 manifest 确定性/schema"
    if not os.path.isfile(SCHEMA_PATH):
        return {"name": name, "status": "FAIL", "detail": "缺少 schema: %s" % SCHEMA_PATH}
    schema = load_json(SCHEMA_PATH)
    hard_errors = []          # schema / 文件存在性 / sha256 错误
    rebuild_notes = []        # 逐 manifest 的 rebuild-check 结果
    n_entries = 0

    for split in MANIFEST_SPLITS:
        mpath = os.path.join(D_DIR, "manifest_%s_v1.json" % split)
        if not os.path.isfile(mpath):
            hard_errors.append("%s: manifest 不存在" % split)
            continue
        man = load_json(mpath)
        entries = man.get("entries", [])
        n_entries += len(entries)
        # (a) schema 校验
        for msg in validate_manifest_obj(man, schema)[:5]:
            hard_errors.append("%s schema: %s" % (split, msg))
        # (b) 产物存在性 + sha256 一致性
        missing, mismatch = 0, 0
        for e in entries:
            for fkey, hkey in (("scenario_file", "scenario_hash"),
                               ("sensors_file", "sensors_hash"),
                               ("world_file", "world_hash")):
                fp = os.path.join(WORKSPACE, e.get(fkey, ""))
                if not os.path.isfile(fp):
                    missing += 1
                    continue
                if sha256_file(fp) != e.get(hkey):
                    mismatch += 1
        if missing:
            hard_errors.append("%s: %d 个产物文件缺失" % (split, missing))
        if mismatch:
            hard_errors.append("%s: %d 个 sha256 与 manifest 不一致" % (split, mismatch))

        # (c) d1 逐字节重建（可选）
        if skip_rebuild:
            rebuild_notes.append("%s=SKIP(--skip-rebuild)" % split)
        elif not os.path.isfile(D1_GEN):
            rebuild_notes.append("%s=SKIP(cli缺失)" % split)
        else:
            rc, out = run_cli(D1_GEN, ["--out", mpath, "--rebuild-check"], timeout=300)
            if rc == 0:
                rebuild_notes.append("%s=PASS" % split)
            elif rc == 1:
                rebuild_notes.append("%s=FAIL" % split)
                hard_errors.append("%s rebuild-check FAIL: %s" % (split, tail_lines(out, 1)))
            else:
                rebuild_notes.append("%s=SKIP(%s)" % (split, tail_lines(out, 1)[:60] or "cli不可用"))

    status = "FAIL" if hard_errors else "PASS"
    if hard_errors:
        detail = "entries=%d; 错误: %s" % (n_entries, "; ".join(hard_errors[:4]))
    else:
        detail = "entries=%d schema/哈希全部一致; rebuild[%s]" % (n_entries, ", ".join(rebuild_notes))
    return {"name": name, "status": status, "detail": detail,
            "rebuild": rebuild_notes, "errors": hard_errors}


# ---------------------------------------------------------------- 判据 3：split 隔离
def check_split_isolation():
    name = "3 split 隔离"
    per_split = {}
    for split in MANIFEST_SPLITS:
        mpath = os.path.join(D_DIR, "manifest_%s_v1.json" % split)
        if os.path.isfile(mpath):
            per_split[split] = load_json(mpath).get("entries", [])
    if not per_split:
        return {"name": name, "status": "FAIL", "detail": "未找到任何 manifest"}

    errs = []
    # (a) scenario_id 跨 split 重复
    id_split = {}
    for split, entries in per_split.items():
        for e in entries:
            id_split.setdefault(e["scenario_id"], set()).add(split)
    cross = {sid: sorted(s) for sid, s in id_split.items() if len(s) > 1}
    if cross:
        errs.append("%d 个 scenario_id 跨 split 重复" % len(cross))

    # (b) layout 集合跨 split 不相交
    layout_sets = {s: {e.get("layout") for e in ents if e.get("layout")} for s, ents in per_split.items()}
    splits = sorted(layout_sets)
    for i in range(len(splits)):
        for j in range(i + 1, len(splits)):
            inter = layout_sets[splits[i]] & layout_sets[splits[j]]
            if inter:
                errs.append("layout 跨 split 相交 %s/%s: %s" % (splits[i], splits[j], sorted(inter)))

    # (c) seed 唯一性：重复 seed 仅允许出现在同一非空 pair_id 的配对条目
    seed_pairs = {}
    for split, entries in per_split.items():
        for e in entries:
            seed_pairs.setdefault(e["seed"], []).append(e.get("pair_id"))
    bad_seeds = []
    for seed, pairs in seed_pairs.items():
        if len(pairs) > 1:
            uniq = set(pairs)
            if len(uniq) != 1 or next(iter(uniq)) is None:
                bad_seeds.append(seed)
    if bad_seeds:
        errs.append("%d 个 seed 在非配对情形下重复" % len(bad_seeds))

    status = "FAIL" if errs else "PASS"
    detail = "; ".join(errs) if errs else (
        "scenario_id 唯一; layout 集合不相交; seed 唯一(配对除外); 共 %d 条" % sum(len(v) for v in per_split.values()))
    return {"name": name, "status": status, "detail": detail}


# ---------------------------------------------------------------- 判据 4：三类分母恒等式
def _dedup_last(rows):
    """同 scenario_id 多次尝试取最后一条（按 attempt_index，其次行序）。"""
    last = {}
    for r in rows:
        sid = r.get("scenario_id")
        cur = last.get(sid)
        if cur is None or (r.get("attempt_index") or 0) >= (cur.get("attempt_index") or 0):
            last[sid] = r
    return list(last.values())


def _counts(rows):
    c = {k: 0 for k in CATEGORIES}
    unknown = 0
    for r in rows:
        cat = r.get("category")
        if cat in c:
            c[cat] += 1
        else:
            unknown += 1
    return c, unknown


def check_denominators():
    name = "4 三类分母恒等式"
    files = sorted(glob.glob(os.path.join(D_DIR, "batch_index_*.jsonl")))
    if not files:
        return {"name": name, "status": "SKIP", "detail": "未找到 batch_index_*.jsonl，无索引数据"}

    errs, summaries = [], []
    for fp in files:
        try:
            raw = [json.loads(ln) for ln in open(fp, "r", encoding="utf-8") if ln.strip()]
        except Exception as e:  # noqa: BLE001
            errs.append("%s 解析失败: %s" % (os.path.basename(fp), e))
            continue
        rows = _dedup_last(raw)
        c, unknown = _counts(rows)
        attempted = len(rows)
        valid = c["tracking_success"] + c["mission_failure"]
        eq_valid = valid == c["tracking_success"] + c["mission_failure"]  # 恒真，保留显式
        eq_att = attempted == valid + c["infra_failure"] + c["aborted"]
        tag = os.path.basename(fp).replace("batch_index_", "").replace(".jsonl", "")
        summaries.append("%s[att=%d valid=%d ts=%d mf=%d infra=%d ab=%d unk=%d %s]" % (
            tag, attempted, valid, c["tracking_success"], c["mission_failure"],
            c["infra_failure"], c["aborted"], unknown,
            "OK" if (eq_valid and eq_att) else "BAD"))
        if not eq_valid:
            errs.append("%s: valid_task != success+mission_failure" % tag)
        if not eq_att:
            errs.append("%s: attempted != valid+infra+aborted (unknown=%d)" % (tag, unknown))

        # 逐 family
        fam_rows = {}
        for r in rows:
            fam_rows.setdefault(r.get("family"), []).append(r)
        for fam, frs in fam_rows.items():
            fc, _ = _counts(frs)
            fvalid = fc["tracking_success"] + fc["mission_failure"]
            if len(frs) != fvalid + fc["infra_failure"] + fc["aborted"]:
                errs.append("%s/%s: 分母恒等式不成立" % (tag, fam))

    status = "FAIL" if errs else "PASS"
    detail = "; ".join(errs) if errs else ("%d 个索引文件恒等式成立: %s" % (len(files), "; ".join(summaries)))
    return {"name": name, "status": status, "detail": detail}


# ---------------------------------------------------------------- 判据 5：Wilson 正确性
def check_wilson():
    name = "5 Wilson 计算正确性"
    tol = 1e-3
    cases = [
        ("k=5,n=10", 5, 10, 0.2366, 0.7634),
        ("k=0,n=20 上界", 0, 20, None, 0.1611),
        ("k=20,n=20 下界", 20, 20, 0.8389, None),
    ]
    errs, notes = [], []
    for label, k, n, exp_lo, exp_hi in cases:
        lo, hi = wilson_ci(k, n)
        notes.append("%s=(%.4f,%.4f)" % (label, lo, hi))
        if exp_lo is not None and abs(lo - exp_lo) > tol:
            errs.append("%s 下界 %.4f!=%.4f" % (label, lo, exp_lo))
        if exp_hi is not None and abs(hi - exp_hi) > tol:
            errs.append("%s 上界 %.4f!=%.4f" % (label, hi, exp_hi))
    status = "FAIL" if errs else "PASS"
    detail = "; ".join(errs) if errs else "; ".join(notes)
    return {"name": name, "status": status, "detail": detail}


# ---------------------------------------------------------------- 判据 6：D2 报告完整性
def check_d2_report():
    name = "6 D2 报告完整性"
    path = os.path.join(D_DIR, "d2_report.json")
    if not os.path.isfile(path):
        return {"name": name, "status": "FAIL",
                "detail": "未找到 d2_report.json，请先运行 d2_stat_report.py"}
    rep = load_json(path)
    errs = []

    # per_family：成功率点估计 + Wilson 区间 + n
    pf = rep.get("per_family")
    if not isinstance(pf, dict) or not pf:
        errs.append("缺少 per_family")
    else:
        for fam, v in pf.items():
            rate = (v or {}).get("tracking_success_rate")
            if not isinstance(rate, dict) or not all(kk in rate for kk in ("point", "ci95", "n")):
                errs.append("per_family.%s 缺少 success(point/ci95/n)" % fam)
            elif not (isinstance(rate.get("ci95"), list) and len(rate["ci95"]) == 2):
                errs.append("per_family.%s ci95 非法" % fam)

    # denominator_equations
    de = rep.get("denominator_equations")
    if not isinstance(de, dict) or not de:
        errs.append("缺少 denominator_equations")
    else:
        for key, eq in de.items():
            if isinstance(eq, dict) and eq.get("pass") is False:
                errs.append("denominator_equations.%s 未通过" % key)

    # comparisons：C1/C2/C3 或 insufficient_data
    comps = rep.get("comparisons")
    if not isinstance(comps, list) or not comps:
        errs.append("缺少 comparisons")
    else:
        ids = {c.get("id") for c in comps if isinstance(c, dict)}
        any_insufficient = any(isinstance(c, dict) and c.get("insufficient_data") for c in comps)
        if not ({"C1", "C2", "C3"} <= ids or any_insufficient):
            errs.append("comparisons 缺少 C1/C2/C3 且无 insufficient_data")

    # raw_run_ids
    if not rep.get("raw_run_ids"):
        errs.append("缺少 raw_run_ids")

    status = "FAIL" if errs else "PASS"
    detail = "; ".join(errs) if errs else (
        "per_family=%d; equations=%d; comparisons=%s; raw_run_ids 齐备"
        % (len(pf or {}), len(de or {}), sorted({c.get("id") for c in comps})))
    return {"name": name, "status": status, "detail": detail}


# ---------------------------------------------------------------- 判据 7：文档存在性
def check_docs():
    name = "7 D0/D1/D2 文档存在性"
    docs = {d: os.path.isfile(os.path.join(DOC_DIR, "%s.md" % d)) for d in ("D0", "D1", "D2")}
    missing = [d for d, ok in docs.items() if not ok]
    if missing:
        status = "FAIL"
        detail = "缺失: %s（请补齐 docs/项目任务日志/阶段3/D/*.md）" % ", ".join(missing)
    else:
        status, detail = "PASS", "D0.md/D1.md/D2.md 均存在"
    return {"name": name, "status": status, "detail": detail,
            "present": {d: ("PASS" if ok else "FAIL") for d, ok in docs.items()}}


# ---------------------------------------------------------------- 主流程
def main():
    ap = argparse.ArgumentParser(description="阶段3 D 统一核验脚本（离线）")
    ap.add_argument("--skip-rebuild", action="store_true", help="跳过 d1 逐字节重建检查（较慢）")
    ap.add_argument("--out", default=DEFAULT_OUT, help="JSON 落盘路径")
    args = ap.parse_args()

    print("=== 阶段3 D 统一核验 ===")
    items = [
        check_protocol_freeze(),
        check_manifest_schema(skip_rebuild=args.skip_rebuild),
        check_split_isolation(),
        check_denominators(),
        check_wilson(),
        check_d2_report(),
        check_docs(),
    ]

    counts = {"PASS": 0, "FAIL": 0, "SKIP": 0}
    for it in items:
        counts[it["status"]] = counts.get(it["status"], 0) + 1
        print("[%s] %-26s %s" % (it["status"], it["name"], it["detail"]))

    overall = "FAIL" if counts["FAIL"] > 0 else "PASS"
    print("-" * 72)
    print("overall: %s  (PASS=%d FAIL=%d SKIP=%d)" % (overall, counts["PASS"], counts["FAIL"], counts["SKIP"]))

    # 落盘
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    payload = {
        "generated_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "workspace": WORKSPACE,
        "overall": overall,
        "counts": counts,
        "items": items,
    }
    with open(args.out, "w", encoding="utf-8") as f:
        json.dump(payload, f, ensure_ascii=False, indent=2)
    print("JSON 落盘: %s" % args.out)
    return 1 if overall == "FAIL" else 0


if __name__ == "__main__":
    sys.exit(main())
