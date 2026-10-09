#!/usr/bin/env python3
"""summarize_runs.py —— 汇总 agc run 日志并按契约口径校验（纯命令行，不依赖 ROS）。

用法：
    python3 summarize_runs.py [root]

root 缺省为 /home/ubuntu/anti_drone_agc_workspace/experiments/agc_runs
（即运行期 /agc/log/root 的默认值）。

输出：
- 分列 attempted / valid_task / tracking_success / mission_failure / aborted /
  infra_failure / invalid_logs；
- 校验两条契约恒等式并打印 PASS/FAIL；
- 任务成功率与基础设施失败/人工终止占全部尝试的比率；
- 四类事件计数占位（读各 run 的 state_events.jsonl）；
- 结果写入 <root>/aggregate.json。
"""

import argparse
import json
import os
import sys
import time

DEFAULT_ROOT = "/home/ubuntu/anti_drone_agc_workspace/experiments/agc_runs"

# 终态 -> 统计分类
CATEGORY_BY_STATE = {
    4: "tracking_success",
    5: "mission_failure",
    6: "aborted",
    8: "infra_failure",
}
VALID_CATEGORIES = ("tracking_success", "mission_failure")
COUNT_KEYS = ("tracking_success", "mission_failure", "aborted", "infra_failure")

# 四类事件占位计数（标签 -> state_events.jsonl 中查找的 token）
EVENT_LABELS = [
    ("真实接触/几何相交(碰撞)", "GROUND_COLLISION"),
    ("保护距离违例", "CLEARANCE_VIOLATION"),
    ("紧急距离触发", "EMERGENCY_TRIGGER"),
    ("过滤器触发", "FILTER_TRIGGER"),
]


def load_run(run_dir):
    """读取单个 run 目录，返回 (record, error)。error 非空表示无效日志。"""
    path = os.path.join(run_dir, "summary.json")
    if not os.path.isfile(path):
        return None, "缺少 summary.json"
    try:
        with open(path, "r") as f:
            s = json.load(f)
    except (ValueError, OSError):
        return None, "summary.json JSON 非法"
    if not isinstance(s, dict):
        return None, "summary.json 非对象"
    counting_pre = s.get("counting") if isinstance(s.get("counting"), dict) else {}
    final_state = s.get("final_state")
    # 「开目录但尚未开始尝试」的空 run（logger 的占位 summary）：不是无效日志，只是不计入分母
    if final_state is None and (s.get("phase") or counting_pre.get("attempted_delta") == 0):
        return {"dir": os.path.basename(run_dir.rstrip("/")),
                "run_id": s.get("run_id"),
                "final_state": None, "category": None,
                "attempted_delta": 0, "valid_task_delta": 0,
                "clean_exit": bool(s.get("clean_exit", False)),
                "no_attempt": True}, None
    if final_state is None:
        return None, "终态缺失"
    try:
        final_state = int(final_state)
    except (TypeError, ValueError):
        return None, "终态类型非法"

    counting = s.get("counting") if isinstance(s.get("counting"), dict) else {}
    category = counting.get("category") or CATEGORY_BY_STATE.get(final_state)
    if category not in CATEGORY_BY_STATE.values():
        return None, "终态无法归类"

    # 优先采用 run 日志自报的 delta，缺省时按分类推导（便于暴露不一致）
    attempted_delta = counting.get("attempted_delta")
    valid_delta = counting.get("valid_task_delta")
    if attempted_delta is None:
        attempted_delta = 1
    if valid_delta is None:
        valid_delta = 1 if category in VALID_CATEGORIES else 0

    return {
        "dir": os.path.basename(run_dir.rstrip("/")),
        "run_id": s.get("run_id"),
        "final_state": final_state,
        "category": category,
        "attempted_delta": int(attempted_delta),
        "valid_task_delta": int(valid_delta),
        "clean_exit": bool(s.get("clean_exit", False)),
        "no_attempt": False,
    }, None


def count_events(run_dirs):
    """统计各 run 的 state_events.jsonl 中四类 token 出现次数。"""
    counts = {token: 0 for _, token in EVENT_LABELS}
    missing = []
    for run_dir in run_dirs:
        path = os.path.join(run_dir, "state_events.jsonl")
        if not os.path.isfile(path):
            missing.append(os.path.basename(run_dir.rstrip("/")))
            continue
        try:
            with open(path, "r") as f:
                for line in f:
                    for token in counts:
                        # 每行对每个 token 至多计 1 次（一行可能同时含 event 与 reason_code）
                        if token in line:
                            counts[token] += 1
        except OSError:
            missing.append(os.path.basename(run_dir.rstrip("/")))
    return counts, missing


def main():
    parser = argparse.ArgumentParser(description="汇总 agc run 日志")
    parser.add_argument("root", nargs="?", default=DEFAULT_ROOT,
                        help="run 日志根目录（默认 %s）" % DEFAULT_ROOT)
    args = parser.parse_args()
    root = args.root

    if not os.path.isdir(root):
        print("根目录不存在：%s" % root)
        sys.exit(1)

    run_dirs = sorted(
        os.path.join(root, d) for d in os.listdir(root)
        # 跳过隐藏/下划线开头的目录（归档目录、临时目录），它们不是 run
        if os.path.isdir(os.path.join(root, d)) and not d.startswith(("_", "."))
    )

    agg = {k: 0 for k in COUNT_KEYS}
    attempted_runs = 0
    valid_task_runs = 0
    invalid_logs = []
    no_attempt_runs = []
    runs_total = 0

    for run_dir in run_dirs:
        record, err = load_run(run_dir)
        if err is not None:
            invalid_logs.append(os.path.basename(run_dir.rstrip("/")))
            continue
        if record.get("no_attempt"):
            no_attempt_runs.append(record["dir"])
            continue
        runs_total += 1
        attempted_runs += record["attempted_delta"]
        valid_task_runs += record["valid_task_delta"]
        agg[record["category"]] += 1

    eq_valid = (valid_task_runs == agg["tracking_success"] + agg["mission_failure"])
    eq_attempted = (
        attempted_runs
        == valid_task_runs + agg["infra_failure"] + agg["aborted"]
    )

    def rate(num, den):
        if den <= 0:
            return {"value": None, "num": num, "den": den}
        return {"value": float(num) / float(den), "num": num, "den": den}

    success_rate = rate(agg["tracking_success"], valid_task_runs)
    infra_rate = rate(agg["infra_failure"], attempted_runs)
    abort_rate = rate(agg["aborted"], attempted_runs)

    event_counts, event_missing = count_events(run_dirs)

    aggregate = {
        "root": root,
        "generated_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "runs_total": runs_total,
        "attempted_runs": attempted_runs,
        "valid_task_runs": valid_task_runs,
        "tracking_success_runs": agg["tracking_success"],
        "mission_failure_runs": agg["mission_failure"],
        "aborted_runs": agg["aborted"],
        "infra_failure_runs": agg["infra_failure"],
        "invalid_logs": invalid_logs,
        "no_attempt_runs": no_attempt_runs,
        "equations": {
            "valid_task": "PASS" if eq_valid else "FAIL",
            "attempted": "PASS" if eq_attempted else "FAIL",
        },
        "success_rate": success_rate,
        "infra_failure_rate": infra_rate,
        "abort_rate": abort_rate,
        "event_counts": event_counts,
        "event_files_missing": event_missing,
    }

    out_path = os.path.join(root, "aggregate.json")
    with open(out_path, "w") as f:
        json.dump(aggregate, f, ensure_ascii=False, indent=2)

    # ---------------------------------------------------------- 文本报表
    def fmt_rate(r, label):
        if r["value"] is None:
            return "%-34s: N/A (0/%d)" % (label, r["den"])
        return "%-34s: %.4f (%d/%d)" % (label, r["value"], r["num"], r["den"])

    print("=== AGC run 汇总 ===")
    print("root                                : %s" % root)
    print("有效 run 目录数 (runs_total)        : %d" % runs_total)
    print("attempted_runs                      : %d" % attempted_runs)
    print("valid_task_runs                     : %d" % valid_task_runs)
    print("tracking_success_runs               : %d" % agg["tracking_success"])
    print("mission_failure_runs                : %d" % agg["mission_failure"])
    print("aborted_runs                        : %d" % agg["aborted"])
    print("infra_failure_runs                  : %d" % agg["infra_failure"])
    print("invalid_logs                        : %d %s"
          % (len(invalid_logs), ("[" + ", ".join(invalid_logs) + "]") if invalid_logs else ""))
    print("no_attempt_runs (未开始尝试, 不计分母) : %d %s"
          % (len(no_attempt_runs),
             ("[" + ", ".join(no_attempt_runs) + "]") if no_attempt_runs else ""))
    print("EQUATION valid_task: %s" % ("PASS" if eq_valid else "FAIL"))
    print("EQUATION attempted: %s" % ("PASS" if eq_attempted else "FAIL"))
    print(fmt_rate(success_rate, "任务成功率 (success/valid_task)"))
    print(fmt_rate(infra_rate, "基础设施失败/全部尝试"))
    print(fmt_rate(abort_rate, "人工终止/全部尝试"))
    print("")
    print("四类事件计数（占位，来源 state_events.jsonl；缺失记 0）:")
    for label, token in EVENT_LABELS:
        print("  %-16s %-20s : %d" % (label, token, event_counts[token]))
    print("  state_events.jsonl 缺失 run 数    : %d %s"
          % (len(event_missing), ("[" + ", ".join(event_missing) + "]") if event_missing else ""))
    print("")
    print("aggregate.json 已写入: %s" % out_path)

    if not (eq_valid and eq_attempted):
        sys.exit(1)


if __name__ == "__main__":
    main()
