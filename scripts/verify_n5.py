#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""N5 证据 —— 离线核验（纯 stdlib，不跑仿真）。

核对 experiments/n5 下的权威报告：
  A 低矮屏障上越 vs 水平绕行（drone/n5_barrier_report.json）
  B 实时 LQR 竖直参考（agc/lqr_vertical/lqr_vertical_report.json）
  C 地面布局扩展与隔离（ground/n5_ground_report.json）
  D 全程与稳态窗口（agc/lqr/n5_lqr_window_report.json）
缺失文件记 FAIL（可先跑）；仅在 M7 诊断对比处允许 SKIP。
输出 verify_n5.json；任一 FAIL 退出码 1。

用法:
  python3 scripts/verify_n5.py --root experiments/n5
"""

import argparse
import json
import os

CHECKS = []


def check(cid, ok, detail=""):
    CHECKS.append({"id": cid, "status": "PASS" if ok else "FAIL",
                   "detail": detail})
    return ok


def skip(cid, detail=""):
    # 显式跳过：不计入 FAIL
    CHECKS.append({"id": cid, "status": "SKIP", "detail": detail})
    return True


def load_json(path):
    try:
        with open(path, encoding="utf-8") as stream:
            return json.load(stream)
    except (OSError, ValueError):
        return None


def _num(x):
    # 是否为数值（排除 bool）
    return isinstance(x, (int, float)) and not isinstance(x, bool)


def _rates_ok(d):
    # 标准失败率口径字段均在 [0,1]
    keys = ("success_rate", "collision_rate", "timeout_rate",
            "reset_failure_rate")
    vals = [d[k] for k in keys if k in d]
    return bool(vals) and all(_num(v) and 0.0 <= v <= 1.0 for v in vals)


def check_a(root):
    # N5-A 低矮屏障上越 vs 水平绕行
    p = os.path.join(root, "drone", "n5_barrier_report.json")
    rep = load_json(p)
    check("A.report", bool(rep), "drone/n5_barrier_report.json")
    if not rep:
        return
    top_keys = ("created_at", "model", "model_sha256", "groups",
                "manifest_sha256", "thresholds", "comparison")
    check("A.keys", all(k in rep for k in top_keys),
          "缺失=%s" % [k for k in top_keys if k not in rep])
    check("A.model_sha", bool(rep.get("model_sha256")), "model_sha256")
    check("A.thresholds", bool(rep.get("thresholds")), "thresholds 非空")
    msha = rep.get("manifest_sha256") or {}
    check("A.manifest_sha", bool(msha) and all(bool(v) for v in msha.values()),
          "manifest_sha256=%s" % list(msha.keys()))

    groups = rep.get("groups") or {}
    agg_keys = ("success_rate", "collision_rate", "timeout_rate",
                "failure_denominator", "vertical_action_rate", "min_clearance")
    for name in ("low_barrier", "tall_barrier"):
        g = groups.get(name) or {}
        agg = g.get("aggregates") or {}
        need = ("episodes", "overfly_episodes", "detour_episodes", "barrier_top_m",
                "manifest")
        check("A.%s.schema" % name,
              all(k in g for k in need) and all(k in agg for k in agg_keys),
              "缺失=%s" % [k for k in need if k not in g])
        eps = g.get("episodes", 0)
        check("A.%s.episodes" % name,
              _num(eps) and eps > 0
              and abs(agg.get("failure_denominator", -1) - eps) < 1e-6,
              "episodes=%s denom=%s" % (eps, agg.get("failure_denominator")))

    low = groups.get("low_barrier") or {}
    tall = groups.get("tall_barrier") or {}
    check("A.low_barrier.below_altitude",
          _num(low.get("barrier_top_m")) and low["barrier_top_m"] < 2.0
          and low.get("overfly_episodes", 0) > 0,
          "top=%s overfly=%s" % (low.get("barrier_top_m"),
                                 low.get("overfly_episodes")))
    check("A.tall_barrier.above_altitude",
          _num(tall.get("barrier_top_m")) and tall["barrier_top_m"] >= 2.0
          and tall.get("overfly_episodes", -1) == 0
          and tall.get("detour_episodes", 0) > 0,
          "top=%s overfly=%s detour=%s"
          % (tall.get("barrier_top_m"), tall.get("overfly_episodes"),
             tall.get("detour_episodes")))
    check("A.overfly_low_gt_tall",
          low.get("overfly_episodes", -1) > tall.get("overfly_episodes", -1),
          "low=%s tall=%s" % (low.get("overfly_episodes"),
                              tall.get("overfly_episodes")))


def check_b(root):
    # N5-B 实时 LQR 竖直参考
    p = os.path.join(root, "agc", "lqr_vertical", "lqr_vertical_report.json")
    rep = load_json(p)
    check("B.report", bool(rep), "agc/lqr_vertical/lqr_vertical_report.json")
    if not rep:
        return
    top_keys = ("z_offset_m", "topics", "phases", "metrics", "checks", "pass")
    check("B.keys", all(k in rep for k in top_keys),
          "缺失=%s" % [k for k in top_keys if k not in rep])
    check("B.pass", rep.get("pass") is True, "pass=%s" % rep.get("pass"))

    ck = rep.get("checks") or {}
    main = ("ref_z_changed_by_offset", "drone_followed_offset",
            "recovered_after_clear")
    check("B.main_checks", all(ck.get(k) is True for k in main),
          "%s timeout_zeroing=%s"
          % ({k: ck.get(k) for k in main}, ck.get("timeout_zeroing_observed")))

    zoff = rep.get("z_offset_m")
    check("B.z_offset_positive", _num(zoff) and zoff > 0, "z_offset_m=%s" % zoff)
    shift = ck.get("ref_z_shift_m")
    check("B.ref_shift_matches_offset",
          _num(zoff) and _num(shift) and abs(shift - zoff) <= 0.20,
          "ref_z_shift_m=%s z_offset_m=%s" % (shift, zoff))
    topics = rep.get("topics") or {}
    check("B.topics", len(topics) >= 3, "topics=%d" % len(topics))


def check_c(root):
    # N5-C 地面布局扩展与隔离
    p = os.path.join(root, "ground", "n5_ground_report.json")
    rep = load_json(p)
    check("C.report", bool(rep), "ground/n5_ground_report.json")
    if not rep:
        return
    top_keys = ("scenario_split_sha256", "layout_split", "layouts_poles",
                "scenario_splits", "train", "validation", "test", "notes")
    check("C.keys", all(k in rep for k in top_keys),
          "缺失=%s" % [k for k in top_keys if k not in rep])
    check("C.split_sha", bool(rep.get("scenario_split_sha256")),
          "scenario_split_sha256")

    ls = rep.get("layout_split") or {}
    check("C.layout_split_values",
          bool(ls) and all(v in ("train", "validation", "test")
                           for v in ls.values()),
          "layout_split=%s" % ls)
    # 由 layout_split 反推各 split 的布局集合
    by_split = {}
    for lay, sp in ls.items():
        by_split.setdefault(sp, []).append(lay)
    train_l = set(by_split.get("train", []))
    val_l = set(by_split.get("validation", []))
    test_l = set(by_split.get("test", []))
    check("C.isolation",
          bool(test_l) and not (test_l & (train_l | val_l)),
          "test=%s vs train∪val=%s" % (sorted(test_l),
                                        sorted(train_l | val_l)))

    ss = rep.get("scenario_splits") or {}
    check("C.scenario_splits",
          all(bool(ss.get(k)) for k in ("train", "validation", "test")),
          "counts=%s" % {k: len(ss.get(k, [])) for k in
                         ("train", "validation", "test")})

    for split, layouts in (("validation", val_l), ("test", test_l)):
        m = rep.get(split) or {}
        eps = m.get("episodes")
        check("C.%s.denom" % split,
              _num(eps) and abs(m.get("failure_denominator", -1) - eps) < 1e-6,
              "episodes=%s denom=%s" % (eps, m.get("failure_denominator")))
        check("C.%s.rates" % split, _rates_ok(m), "rates 在 [0,1]")
        lb = m.get("layout_breakdown") or {}
        check("C.%s.breakdown" % split, set(lb.keys()) == layouts,
              "breakdown=%s split_layouts=%s" % (sorted(lb.keys()),
                                                 sorted(layouts)))

    check("C.notes", bool(rep.get("notes")), "notes 非空")

    # 诊断：若存在 M7 模型结果则对比，否则 SKIP（不算 FAIL）
    diag_p = os.path.join(root, "ground", "carm7", "n5_ground_evalonly.json")
    if not os.path.isfile(diag_p):
        skip("C.m7_diag", "carm7/n5_ground_evalonly.json 不存在，跳过 M7 对比")
    else:
        diag = load_json(diag_p)
        d_res = (diag or {}).get("results") or {}
        ok = bool(diag) and any(k in d_res for k in ("validation", "test"))
        detail = "M7 产物不可用"
        if ok:
            # 对比 test 上成功率/碰撞率（M7 vs N5）
            d_test = d_res.get("test") or {}
            n_test = rep.get("test") or {}
            detail = "test M7(succ=%s, coll=%s) vs N5(succ=%s, coll=%s)" % (
                d_test.get("success_rate"), d_test.get("collision_rate"),
                n_test.get("success_rate"), n_test.get("collision_rate"))
        check("C.m7_diag", ok, detail)


def check_d(root):
    # N5-D 全程与稳态窗口
    p = os.path.join(root, "agc", "lqr", "n5_lqr_window_report.json")
    rep = load_json(p)
    check("D.report", bool(rep), "agc/lqr/n5_lqr_window_report.json")
    if not rep:
        return
    top_keys = ("topic", "startup_exclude_s", "tail_s", "window_boundary_rule",
                "trajectory_matrix", "trajectories")
    check("D.keys", all(k in rep for k in top_keys),
          "缺失=%s" % [k for k in top_keys if k not in rep])
    check("D.window_rule", bool(rep.get("window_boundary_rule")),
          "window_boundary_rule 非空")

    trajs = rep.get("trajectories") or {}
    want = {"linear", "circular", "fixed_horizontal", "figure_eight"}
    check("D.trajectory_names",
          want.issubset(set(trajs.keys())) or len(trajs) >= 4,
          "trajectories=%s" % sorted(trajs.keys()))

    stat_keys = ("mean_m", "rms_m", "max_m", "p95_m")
    for name, e in trajs.items():
        w = e.get("windows") or {}
        ok = True
        for wn in ("full", "post_startup", "steady_tail"):
            x = w.get(wn) or {}
            if not (_num(x.get("n")) and x["n"] > 0
                    and all(_num(x.get(k)) for k in stat_keys)):
                ok = False
        check("D.%s.windows" % name, ok, "windows 完整且 n>0")
        full = w.get("full") or {}
        ps = w.get("post_startup") or {}
        check("D.%s.startup_excluded" % name,
              _num(ps.get("n")) and _num(full.get("n")) and ps["n"] <= full["n"]
              and _num(ps.get("max_m")) and _num(full.get("max_m"))
              and ps["max_m"] <= full["max_m"] + 1e-6,
              "post_n=%s<=full_n=%s post_max=%s<=full_max=%s"
              % (ps.get("n"), full.get("n"), ps.get("max_m"),
                 full.get("max_m")))


def main():
    parser = argparse.ArgumentParser(description="N5 离线核验")
    parser.add_argument("--root", default="experiments/n5")
    parser.add_argument("--out", default=None)
    args = parser.parse_args()
    root = os.path.abspath(args.root)

    check_a(root)
    check_b(root)
    check_c(root)
    check_d(root)

    passed = sum(1 for c in CHECKS if c["status"] == "PASS")
    failed = sum(1 for c in CHECKS if c["status"] == "FAIL")
    skipped = sum(1 for c in CHECKS if c["status"] == "SKIP")
    result = {"checks": CHECKS,
              "summary": {"pass": passed, "fail": failed, "skip": skipped}}
    out = args.out or os.path.join(root, "verify_n5.json")
    with open(out, "w", encoding="utf-8") as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2)
        stream.write("\n")
    for c in CHECKS:
        print("[%s] %s %s" % (c["status"], c["id"], c["detail"]))
    print("== PASS=%d FAIL=%d SKIP=%d ==" % (passed, failed, skipped))
    raise SystemExit(1 if failed else 0)


if __name__ == "__main__":
    main()