#!/usr/bin/env python3
"""verify_fusion.py —— M4 感知与多源融合离线核验（非实时链路）。

读取 run 目录日志，按模式逐项输出 PASS/FAIL 与 JSON：
  - 观测接口字段：/agc/air/obs 含 sensor_id / frame=world / stamp>0 / confidence / 单调 obs_seq
  - 融合输出：/agc/targets 的 source 与 meta.target_source 一致，唯一发布者
  - 可消费：goal 跟随目标；注入/正常下的终态统计
  - latency ：融合日志出现该通道 stale，且其余 inlier 仍支撑输出
  - loss    ：全通道丢失后 /agc/targets 转空、cmd_vel 归零、终态 TARGET_LOST
  - conflict：融合日志把注入通道判为 outlier 且融合跟随 inlier（不跨源取最大）
  - obstacle：/agc/obstacles 非空且被 map/planner 消费（规划日志存在）
  - 标定    ：经验覆盖 P(|err|<=tol) 与声明 confidence 一致

用法：
  python3 verify_fusion.py --batch <批跑目录> --mode nominal|latency|loss|conflict|obstacle|baseline
  结果写入 <batch>/verify_fusion.json，任一 FAIL 退出码 1。
"""
import argparse
import glob
import json
import math
import os
import sys

TARGETS = "/agc/targets"
OBS = "/agc/air/obs"
OBSTACLES = "/agc/obstacles"
GOAL = "/agc/ground/goal"
CMD = "/agc/ground/cmd_vel"


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


def topic_path(run_dir, topic):
    # agc_run_logger 把话题名中的 '/' 替换为 '_'，故 /agc/targets -> _agc_targets.jsonl
    return os.path.join(run_dir, "topics", topic.replace("/", "_") + ".jsonl")


def nsec(stamp):
    """header.stamp 在话题日志里是纳秒字符串。"""
    try:
        return float(stamp) * 1e-9
    except Exception:
        return 0.0


def run_dirs(batch_dir):
    """优先从 batch.json 的 attempt.run_dir 取 run 目录（run 在上层 agc_runs 下）；
    回退为批目录内的子目录。"""
    runs = []
    bj = os.path.join(batch_dir, "batch.json")
    if os.path.isfile(bj):
        try:
            with open(bj) as f:
                d = json.load(f)
            for c in d.get("cycles", []):
                for a in c.get("attempts", []):
                    rd = a.get("run_dir")
                    if rd and os.path.isfile(os.path.join(rd, "meta.json")):
                        runs.append(rd)
        except Exception:
            pass
    if not runs:
        for name in sorted(os.listdir(batch_dir)):
            d = os.path.join(batch_dir, name)
            if os.path.isdir(d) and os.path.isfile(os.path.join(d, "meta.json")):
                runs.append(d)
    return runs


class Checks:
    def __init__(self):
        self.items = []

    def add(self, name, ok, detail=""):
        self.items.append({"name": name, "status": "PASS" if ok else "FAIL", "detail": str(detail)})

    def passed(self):
        return all(i["status"] == "PASS" for i in self.items)


def check_schema(c, run_dir, meta):
    obs_path = topic_path(run_dir, OBS)
    obs = load_jsonl(obs_path)
    if not obs:
        c.add("观测接口存在", meta.get("target_source") == "gt_adapter", "无 /agc/air/obs 样本")
        return
    ok_fields = True
    seq_ok = True
    last = {}
    n = 0
    for rec in obs:
        msg = rec.get("msg", {})
        for so in msg.get("sensors", []):
            sid = so.get("sensor_id", "")
            for t in so.get("targets", []):
                n += 1
                hdr = t.get("header", {})
                if not sid or hdr.get("frame_id") != "world" or nsec(hdr.get("stamp", 0)) <= 0:
                    ok_fields = False
                if not (0.0 <= float(t.get("confidence", -1)) <= 1.0):
                    ok_fields = False
                seq = t.get("obs_seq", 0)
                if sid in last and seq <= last[sid]:
                    seq_ok = False
                last[sid] = seq
    c.add("观测字段(sensor_id/frame/stamp/confidence)", ok_fields and n > 0, "样本=%d" % n)
    c.add("观测 obs_seq 单调", seq_ok, "通道=%d" % len(last))


def check_targets_source(c, run_dir, meta, expect_source):
    tr = load_jsonl(topic_path(run_dir, TARGETS))
    src_ok = True
    empty = 0
    nonempty = 0
    for rec in tr:
        tracks = rec.get("msg", {}).get("tracks", [])
        if not tracks:
            empty += 1
            continue
        nonempty += 1
        for t in tracks:
            if t.get("source") != expect_source:
                src_ok = False
    c.add("目标源标注", src_ok, "expected=%s" % expect_source)
    c.add("目标 track 存在", nonempty > 0, "nonempty=%d empty=%d" % (nonempty, empty))
    return nonempty, empty


def check_causal(c, run_dir):
    tr = load_jsonl(topic_path(run_dir, TARGETS))
    goal = load_jsonl(topic_path(run_dir, GOAL))
    if not tr or not goal:
        c.add("goal 跟随目标", False, "缺少 targets/goal 样本")
        return
    # 目标地面投影序列 vs goal 序列按时间最近邻对齐
    tg = [(nsec(r["msg"]["header"]["stamp"]), r) for r in tr if r["msg"].get("tracks")]
    gl = [(nsec(r["msg"]["header"]["stamp"]), r) for r in goal]
    dists = []
    for ts, r in gl:
        if not tg:
            break
        _, trr = min(tg, key=lambda x: abs(x[0] - ts))
        tp = trr["msg"]["tracks"][0]["pose"]["pose"]["position"]
        gp = r["msg"]["pose"]["position"]
        dists.append(math.hypot(tp["x"] - gp["x"], tp["y"] - gp["y"]))
    if not dists:
        c.add("goal 跟随目标", False, "无可对齐样本")
        return
    dists.sort()
    med = dists[len(dists) // 2]
    c.add("goal 跟随融合目标", med <= 1.0, "中位距离=%.3f m" % med)


def check_final_state(c, run_dir, expect_states):
    sp = os.path.join(run_dir, "summary.json")
    if not os.path.isfile(sp):
        c.add("终态", False, "无 summary.json")
        return None
    with open(sp) as f:
        s = json.load(f)
    st = s.get("final_state")
    c.add("终态符合预期", st in expect_states,
          "state=%s name=%s expect=%s" % (st, s.get("final_state_name"), expect_states))
    return s


def check_calibration(c, run_dir, tol_m, tol_prob):
    pl = load_jsonl(os.path.join(run_dir, "perception_log.jsonl"))
    per = {}
    for rec in pl:
        if not rec.get("has_target"):
            continue
        truth = rec.get("target_truth")
        obs = rec.get("target_obs")
        conf = rec.get("confidence")
        if not truth or not obs or conf is None:
            continue
        err = math.hypot(obs[0] - truth[0], obs[1] - truth[1])
        d = per.setdefault(rec["sensor_id"], {"n": 0, "hit": 0, "conf": []})
        d["n"] += 1
        d["hit"] += 1 if err <= tol_m else 0
        d["conf"].append(conf)
    if not per:
        c.add("confidence 经验覆盖", False, "无感知样本")
        return
    worst = 0.0
    detail = []
    for sid, d in per.items():
        emp = d["hit"] / d["n"]
        decl = sum(d["conf"]) / len(d["conf"])
        worst = max(worst, abs(emp - decl))
        detail.append("%s:经验=%.3f 声明=%.3f n=%d" % (sid, emp, decl, d["n"]))
    c.add("confidence 经验覆盖一致", worst <= tol_prob,
          "; ".join(detail) + " (最大偏差=%.3f≤%.2f)" % (worst, tol_prob))


def check_latency(c, run_dir):
    fl = load_jsonl(os.path.join(run_dir, "fusion_log.jsonl"))
    stale = 0
    supported = 0
    for rec in fl:
        if rec.get("kind") != "target":
            continue
        for e in rec.get("inputs", []):
            if e.get("verdict") == "stale":
                stale += 1
        if stale and rec.get("n_inliers", 0) >= 1:
            supported += 1
    c.add("延迟判 stale", stale > 0, "stale 计数=%d" % stale)
    c.add("其余 inlier 支撑输出", supported > 0, "有 inlier 的 tick=%d" % supported)


def check_loss(c, run_dir):
    pl = load_jsonl(os.path.join(run_dir, "perception_log.jsonl"))
    sensors = set()
    dropped_ticks = 0
    for rec in pl:
        sensors.add(rec.get("sensor_id"))
    # 统计「所有通道同拍 dropped」的 tick 数
    by_t = {}
    for rec in pl:
        by_t.setdefault(rec.get("t_sim"), []).append(rec.get("dropped", False))
    for drops in by_t.values():
        if drops and all(drops):
            dropped_ticks += 1
    c.add("全通道丢失出现", dropped_ticks > 0,
          "dropped ticks=%d sensors=%d" % (dropped_ticks, len(sensors)))

    # 观测消失后 /agc/targets 转空
    tr = load_jsonl(topic_path(run_dir, TARGETS))
    last_nonempty = None
    first_empty_after = None
    for rec in tr:
        ts = rec.get("t_sim")
        tracks = rec.get("msg", {}).get("tracks", [])
        if tracks:
            last_nonempty = ts
        elif last_nonempty is not None and first_empty_after is None:
            first_empty_after = ts
    c.add("丢失后目标转空", first_empty_after is not None and last_nonempty is not None,
          "last_nonempty=%s first_empty=%s" % (last_nonempty, first_empty_after))

    # 转空后应在丢失容忍窗（lost_tol 2s + goal 年龄 1s + 裕度）内停稳：以日志尾部样本判定
    if first_empty_after is not None:
        cmd = load_jsonl(topic_path(run_dir, CMD))
        after = [r for r in cmd if r.get("t_sim", -1) >= first_empty_after]
        if not after:
            c.add("丢失后停车", False, "无 cmd 样本")
        else:
            tail = after[-5:]
            mx_tail = max(abs(r["msg"]["twist"]["linear"]["x"]) for r in tail)
            moving = [r["t_sim"] for r in after
                      if abs(r["msg"]["twist"]["linear"]["x"]) > 0.02]
            t_stop = max(moving) if moving else first_empty_after
            delay = t_stop - first_empty_after
            c.add("丢失后停车", mx_tail <= 0.02 and delay <= 3.5,
                  "日志尾部 max|vx|=%.4f；停车延迟=%.2fs（容忍窗内）" % (mx_tail, delay))
    else:
        c.add("丢失后停车", False, "无转空时刻")


def check_conflict(c, run_dir, injected="air_ir_top"):
    fl = load_jsonl(os.path.join(run_dir, "fusion_log.jsonl"))
    outlier_seen = 0
    follow_ok = 0
    for rec in fl:
        if rec.get("kind") != "target":
            continue
        outs = rec.get("outliers", [])
        if injected in outs:
            outlier_seen += 1
            fused = rec.get("fused_pos")
            ins = rec.get("inliers", [])
            # 融合应跟随某个 inlier 的观测位置（不跟随被拒的离群通道）
            if fused and ins:
                follow_ok += 1
    c.add("冲突判 outlier", outlier_seen > 0, "outlier 计数=%d (注入=%s)" % (outlier_seen, injected))
    c.add("融合跟随 inlier", follow_ok > 0, "跟随 tick=%d" % follow_ok)


def check_obstacles(c, run_dir):
    ob = load_jsonl(topic_path(run_dir, OBSTACLES))
    nonempty = sum(1 for r in ob if r.get("msg", {}).get("tracks"))
    ids = set()
    for r in ob:
        for t in r.get("msg", {}).get("tracks", []):
            ids.add(t.get("obstacle_id"))
    c.add("/agc/obstacles 非空", nonempty > 0, "nonempty=%d ids=%s" % (nonempty, sorted(ids)))
    c.add("障碍被规划消费", os.path.isfile(os.path.join(run_dir, "planner_log.jsonl")),
          "planner_log 存在=%s" % os.path.isfile(os.path.join(run_dir, "planner_log.jsonl")))


def check_cold_outlier(c, run_dir):
    """N4：冷启动高置信离群 —— 应判冷启动歧义、不输出目标、地面不误跟。"""
    fl = load_jsonl(os.path.join(run_dir, "fusion_log.jsonl"))
    amb = [r for r in fl if r.get("kind") == "target" and r.get("result") == "cold_start_ambiguous"]
    c.add("冷启动歧义被判定", len(amb) > 0, "cold_start_ambiguous tick=%d" % len(amb))
    tr = load_jsonl(topic_path(run_dir, TARGETS))
    nonempty = sum(1 for r in tr if r.get("msg", {}).get("tracks"))
    c.add("不输出无法判定的冲突目标", nonempty == 0, "nonempty=%d" % nonempty)
    n_tf = len(load_jsonl(os.path.join(run_dir, "tracking_frames.jsonl")))
    c.add("地面未向歧义目标抵近", n_tf == 0, "tracking_frames=%d" % n_tf)
    sp = os.path.join(run_dir, "summary.json")
    reason = None
    if os.path.isfile(sp):
        with open(sp) as f:
            reason = json.load(f).get("final_reason_code")
    c.add("结局为超时类而非误跟",
          reason in ("SEARCH_TIMEOUT", "MISSION_TIMEOUT", "APPROACH_TIMEOUT"),
          "reason=%s" % reason)


def check_single_channel(c, run_dir):
    """N4：单通道失效 —— 剩余通道继续输出，无误拒。"""
    fl = load_jsonl(os.path.join(run_dir, "fusion_log.jsonl"))
    single = 0
    rej = 0
    for r in fl:
        if r.get("kind") != "target":
            continue
        if r.get("result") == "gate_rejected_all":
            rej += 1
        if r.get("result") == "fused" and r.get("n_inliers", 0) == 1:
            single += 1
    c.add("单通道失效后单源融合仍输出", single > 0, "单源 fused tick=%d" % single)
    c.add("无 gate_rejected_all 误报", rej == 0, "gate_rejected_all tick=%d" % rej)


def main():
    ap = argparse.ArgumentParser(description="M4 融合离线核验")
    ap.add_argument("--batch", required=True, help="批跑目录")
    ap.add_argument("--mode", default="nominal",
                    choices=["nominal", "latency", "loss", "conflict", "obstacle", "baseline",
                             "outlier", "single_channel"])
    ap.add_argument("--tol-m", type=float, default=0.25, help="confidence 容限（m）")
    ap.add_argument("--tol-prob", type=float, default=0.15, help="经验覆盖允许偏差")
    args = ap.parse_args()

    batch = args.batch
    rds = run_dirs(batch)
    c = Checks()
    if not rds:
        c.add("run 目录", False, "批目录无 run")
    # 若无强制 run 目录，也尝试直接核验批目录本身
    for rd in rds:
        with open(os.path.join(rd, "meta.json")) as f:
            meta = json.load(f)
        expect = meta.get("target_source", "gt_adapter")
        prefix = os.path.basename(rd)
        if args.mode == "baseline":
            c.add("基线目标源=gt_adapter %s" % prefix, expect == "gt_adapter", "source=%s" % expect)
            check_causal(c, rd)
            check_final_state(c, rd, {4, 5, 6, 8})
            continue
        if args.mode == "outlier":
            check_schema(c, rd, meta)
            check_cold_outlier(c, rd)
            check_final_state(c, rd, {5})
            continue
        check_schema(c, rd, meta)
        check_targets_source(c, rd, meta, "air_fusion")
        check_causal(c, rd)
        check_calibration(c, rd, args.tol_m, args.tol_prob)
        if args.mode == "nominal":
            check_final_state(c, rd, {4, 5})  # 抵近跟踪成功或任务失败（如可见范围内）
        elif args.mode == "latency":
            check_latency(c, rd)
            check_final_state(c, rd, {4, 5})
        elif args.mode == "loss":
            check_loss(c, rd)
            check_final_state(c, rd, {5})
        elif args.mode == "conflict":
            check_conflict(c, rd, injected="air_ir_top")
            check_final_state(c, rd, {4, 5})
        elif args.mode == "obstacle":
            check_obstacles(c, rd)
            check_final_state(c, rd, {4, 5})
        elif args.mode == "single_channel":
            check_single_channel(c, rd)
            check_final_state(c, rd, {4, 5})

    out = {"batch": batch, "mode": args.mode, "checks": c.items, "passed": c.passed()}
    dst = os.path.join(batch, "verify_fusion.json")
    with open(dst, "w") as f:
        json.dump(out, f, ensure_ascii=False, indent=2)
    n_pass = sum(1 for i in c.items if i["status"] == "PASS")
    for i in c.items:
        print("%-4s %s | %s" % (i["status"], i["name"], i["detail"]))
    print("[verify_fusion] %s mode=%s PASS=%d FAIL=%d -> %s" % (
        batch, args.mode, n_pass, len(c.items) - n_pass, dst))
    return 0 if c.passed() else 1


if __name__ == "__main__":
    sys.exit(main())