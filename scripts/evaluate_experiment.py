#!/usr/bin/env python3
"""
实验数据分析脚本 — 读取 experiment_logger_node.py 生成的 CSV，计算六项评价指标。

用法:
    python3 evaluate_experiment.py file1.csv [file2.csv ...]

仅依赖 Python 标准库，无需安装第三方包。
"""

import csv
import math
import sys
import os
import statistics
from collections import namedtuple


# ---------------------------------------------------------------------------
# 数据行结构（与 CSV 列一一对应）
# ---------------------------------------------------------------------------
ROW_FIELDS = [
    "timestamp",
    "pose_x", "pose_y", "pose_z",
    "pose_qx", "pose_qy", "pose_qz", "pose_qw",
    "offset_x", "offset_y", "offset_z",
    "action",
    "position_error",
    "adjusted_ref_x", "adjusted_ref_y", "adjusted_ref_z",
    "is_avoiding",
    "offset_magnitude_xy",
]
Row = namedtuple("Row", ROW_FIELDS)


# ---------------------------------------------------------------------------
# 工具函数
# ---------------------------------------------------------------------------

def _parse_float(val):
    """将 CSV 单元格字符串转为 float，空值或 'nan' 返回 NaN。"""
    if val is None or val.strip() == "":
        return float("nan")
    try:
        return float(val)
    except ValueError:
        return float("nan")


def load_csv(filepath):
    """读取 CSV 文件，返回 Row 列表。跳过表头行。"""
    rows = []
    with open(filepath, "r", newline="") as f:
        reader = csv.reader(f)
        header = next(reader, None)
        if header is None:
            print(f"[警告] {filepath} 为空文件，跳过。")
            return rows
        # 验证列数
        if len(header) != len(ROW_FIELDS):
            print(f"[警告] {filepath} 列数异常: 期望 {len(ROW_FIELDS)}, 实际 {len(header)}")
        for line in reader:
            if len(line) < len(ROW_FIELDS):
                continue  # 跳过残缺行
            values = [_parse_float(v) for v in line[:len(ROW_FIELDS)]]
            rows.append(Row(*values))
    return rows


def dist_3d(x1, y1, z1, x2, y2, z2):
    """3D 欧氏距离。"""
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2 + (z2 - z1) ** 2)


# ---------------------------------------------------------------------------
# 单项指标计算
# ---------------------------------------------------------------------------

def eval_avoidance_success(rows):
    """
    避障成功率判定。
    返回 (result_str, has_avoidance_flag).
    """
    avoidance_indices = [i for i, r in enumerate(rows) if r.is_avoiding == 1]
    if not avoidance_indices:
        return ("N/A (无避障)", False)

    # 找到最后一个避障片段的结束位置
    # 先找出所有连续的 avoidance segment
    segments = _find_avoidance_segments(rows)
    if not segments:
        return ("N/A (无避障)", False)

    last_end = segments[-1][1]  # 最后一个片段的结束索引（exclusive）

    # 避障结束后，检查 offset_magnitude_xy 是否稳定归零
    post_rows = rows[last_end:]
    if len(post_rows) < 3:
        # 数据在避障刚结束或还在进行中就截断了
        return ("FAIL (数据截断)", True)

    # 检查避障结束后最后几行 offset 是否稳定 < 0.05
    stable_zero = all(
        not math.isnan(r.offset_magnitude_xy) and r.offset_magnitude_xy < 0.05
        for r in post_rows
    )
    if stable_zero:
        return ("SUCCESS", True)
    else:
        return ("FAIL (offset 未归零)", True)


def _find_avoidance_segments(rows):
    """找出 is_avoiding == 1 的连续片段，返回 [(start_idx, end_idx), ...]，end 为 exclusive。"""
    segments = []
    i = 0
    n = len(rows)
    while i < n:
        if rows[i].is_avoiding == 1:
            start = i
            while i < n and rows[i].is_avoiding == 1:
                i += 1
            segments.append((start, i))
        else:
            i += 1
    return segments


def eval_min_obstacle_distance(rows):
    """
    估算最小障碍距离。
    返回 (dist_str, dist_val_or_nan).
    """
    segments = _find_avoidance_segments(rows)
    if not segments:
        return ("N/A (无避障)", float("nan"))

    max_offset = -1.0
    for start, end in segments:
        for i in range(start, end):
            v = rows[i].offset_magnitude_xy
            if not math.isnan(v) and v > max_offset:
                max_offset = v

    if max_offset < 0:
        return ("N/A (offset 数据不可用)", float("nan"))

    # 估算公式
    if max_offset >= 1.2:
        min_dist = 0.15
    else:
        min_dist = 0.35 * (1.0 - (max_offset - 0.3) / 0.9)

    # clamp 到 [0.05, 0.35]
    min_dist = max(0.05, min(0.35, min_dist))
    return (f"{min_dist:.2f} (估算值)", min_dist)


def eval_tracking_error(rows):
    """
    跟踪误差：基于 position_error 列。
    返回 dict 包含全程和避障期间的 max/avg，如果全 NaN 则分别标记 N/A。
    """
    all_pe = [r.position_error for r in rows if not math.isnan(r.position_error)]

    avoidance_pe = []
    for start, end in _find_avoidance_segments(rows):
        for i in range(start, end):
            v = rows[i].position_error
            if not math.isnan(v):
                avoidance_pe.append(v)

    result = {}

    if all_pe:
        result["max_overall"] = f"{max(all_pe):.3f}"
        result["avg_overall"] = f"{statistics.mean(all_pe):.3f}"
    else:
        result["max_overall"] = "N/A"
        result["avg_overall"] = "N/A"

    if avoidance_pe:
        result["max_avoid"] = f"{max(avoidance_pe):.3f}"
        result["avg_avoid"] = f"{statistics.mean(avoidance_pe):.3f}"
    else:
        result["max_avoid"] = "N/A"
        result["avg_avoid"] = "N/A"

    if not all_pe:
        result["note"] = "（position_error 数据不可用）"
    else:
        result["note"] = ""

    return result


def eval_path_length(rows):
    """基于 pose_x/y/z 计算累积 3D 路径长度。"""
    total = 0.0
    for i in range(1, len(rows)):
        r0, r1 = rows[i - 1], rows[i]
        # 跳过含 NaN 的位姿
        if any(math.isnan(v) for v in [r0.pose_x, r0.pose_y, r0.pose_z,
                                        r1.pose_x, r1.pose_y, r1.pose_z]):
            continue
        total += dist_3d(r0.pose_x, r0.pose_y, r0.pose_z,
                         r1.pose_x, r1.pose_y, r1.pose_z)
    return total


def eval_avoidance_duration(rows):
    """统计所有避障片段的持续时长（秒），基于 timestamp 差值。"""
    segments = _find_avoidance_segments(rows)
    total_duration = 0.0
    for start, end in segments:
        if end >= len(rows):
            end = len(rows) - 1
        t_start = rows[start].timestamp
        t_end = rows[end - 1].timestamp if end > start else t_start
        if not math.isnan(t_start) and not math.isnan(t_end):
            total_duration += (t_end - t_start)
    return total_duration


def eval_offset_jitter(rows):
    """计算避障期间 offset_magnitude_xy 的标准差。"""
    offsets = []
    for start, end in _find_avoidance_segments(rows):
        for i in range(start, end):
            v = rows[i].offset_magnitude_xy
            if not math.isnan(v):
                offsets.append(v)

    if len(offsets) < 2:
        return float("nan")

    return statistics.stdev(offsets)


# ---------------------------------------------------------------------------
# 汇总输出
# ---------------------------------------------------------------------------

def analyze_file(filepath):
    """分析单个 CSV，返回指标 dict。"""
    rows = load_csv(filepath)
    if not rows:
        return None  # 空文件

    # 避障成功率
    success_str, has_avoid = eval_avoidance_success(rows)

    # 最小障碍距离
    dist_str, _ = eval_min_obstacle_distance(rows)

    # 跟踪误差
    te = eval_tracking_error(rows)

    # 路径长度
    path_len = eval_path_length(rows)

    # 绕障耗时
    avoid_dur = eval_avoidance_duration(rows)

    # offset 抖动
    jitter = eval_offset_jitter(rows)

    return {
        "filename": os.path.basename(filepath),
        "avoidance_success": success_str,
        "min_dist": dist_str,
        "max_tracking_overall": te["max_overall"],
        "avg_tracking_overall": te["avg_overall"],
        "max_tracking_avoid": te["max_avoid"],
        "avg_tracking_avoid": te["avg_avoid"],
        "tracking_note": te["note"],
        "path_length": path_len,
        "avoid_duration": avoid_dur,
        "offset_jitter": jitter,
    }


def print_summary(results):
    """打印多文件汇总表格。"""
    if not results:
        print("无有效数据。")
        return

    # 列宽：指标名 30 字符，每个文件 28 字符
    metric_width = 32
    file_width = 28

    # 表头
    header_line = "=" * 100
    print(header_line)
    print("实验评价指标汇总")
    print(header_line)

    # 标题行
    title_row = "指标".ljust(metric_width)
    for r in results:
        title_row += r["filename"].ljust(file_width)
    print(title_row)
    print("-" * 100)

    # 指标行定义：(label, key, format_func)
    metrics = [
        ("避障成功率", "avoidance_success", lambda v: v),
        ("最小障碍距离 (m, 估算)", "min_dist", lambda v: v),
        ("最大跟踪误差 (m, 全程)", "max_tracking_overall", lambda v: v),
        ("平均跟踪误差 (m, 全程)", "avg_tracking_overall", lambda v: v),
        ("最大跟踪误差 (m, 避障期间)", "max_tracking_avoid", lambda v: v),
        ("平均跟踪误差 (m, 避障期间)", "avg_tracking_avoid", lambda v: v),
        ("路径长度 (m)", "path_length", lambda v: f"{v:.2f}" if isinstance(v, float) else v),
        ("绕障耗时 (s)", "avoid_duration", lambda v: f"{v:.2f}" if isinstance(v, float) else v),
        ("offset 抖动程度 (std)", "offset_jitter",
         lambda v: f"{v:.3f}" if isinstance(v, float) and not math.isnan(v) else "N/A"),
    ]

    for label, key, fmt in metrics:
        line = label.ljust(metric_width)
        for r in results:
            val = r.get(key, "N/A")
            line += fmt(val).ljust(file_width)
        print(line)

    print(header_line)

    # 打印 tracking note（如有）
    for r in results:
        note = r.get("tracking_note", "")
        if note:
            print(f"[备注] {r['filename']}: 跟踪误差 {note}")


# ---------------------------------------------------------------------------
# 入口
# ---------------------------------------------------------------------------

def main():
    if len(sys.argv) < 2:
        print("用法: python3 evaluate_experiment.py file1.csv [file2.csv ...]")
        sys.exit(1)

    results = []
    for filepath in sys.argv[1:]:
        if not os.path.isfile(filepath):
            print(f"[错误] 文件不存在: {filepath}")
            continue
        print(f"分析中: {filepath} ...")
        r = analyze_file(filepath)
        if r is None:
            print(f"  [警告] {filepath} 无有效数据，已跳过。")
            continue
        results.append(r)

    print_summary(results)


if __name__ == "__main__":
    main()
