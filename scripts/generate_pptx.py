#!/usr/bin/env python3
"""
@file: generate_pptx.py
@brief: 生成结课答辩PPT（数据驱动版 + 演示留白）
@date: 2026-07-30
"""

from pptx import Presentation
from pptx.util import Inches, Pt, Emu
from pptx.dml.color import RGBColor
from pptx.enum.text import PP_ALIGN, MSO_ANCHOR
from pptx.enum.shapes import MSO_SHAPE
import os
import csv
import json
from collections import Counter

# ── Midnight Executive 配色方案 ──
C_PRIMARY = RGBColor(0x1E, 0x27, 0x61)    # 海军蓝（主色）
C_ACCENT = RGBColor(0xCA, 0xDC, 0xFC)    # 冰蓝（辅助色，用于装饰线条/强调）
C_DARK = RGBColor(0x2C, 0x3E, 0x50)      # 深灰（正文）
C_GRAY = RGBColor(0x7F, 0x8C, 0x8D)      # 灰色（标注文字）
C_WHITE = RGBColor(0xFF, 0xFF, 0xFF)     # 白色（背景）
C_LIGHT_BG = RGBColor(0xF5, 0xF7, 0xFA)  # 浅灰（卡片背景）
C_GREEN = RGBColor(0x2E, 0xCC, 0x71)     # 翡翠绿（成功）
C_ORANGE = RGBColor(0xF3, 0x9C, 0x12)    # 琥珀色（警告）
C_RED = RGBColor(0xE7, 0x4C, 0x3C)       # 红色（危险）
C_DEMO_BG = RGBColor(0x2C, 0x3E, 0x50)   # 演示占位背景
C_LIGHT = RGBColor(0xD4, 0xE6, 0xF1)     # 浅蓝（边框）

# ── 尺寸 ──
SLIDE_W = Inches(13.333)
SLIDE_H = Inches(7.5)
MARGIN_L = Inches(0.9)
MARGIN_R = Inches(0.9)
CONTENT_W = SLIDE_W - MARGIN_L - MARGIN_R

prs = Presentation()
prs.slide_width = SLIDE_W
prs.slide_height = SLIDE_H


# ── 辅助函数 ──

def add_blank_slide():
    layout = prs.slide_layouts[6]
    slide = prs.slides.add_slide(layout)
    bg = slide.background
    fill = bg.fill
    fill.solid()
    fill.fore_color.rgb = C_WHITE
    return slide


def add_shape(slide, left, top, width, height, fill_color=None, line_color=None, line_width=None):
    shape = slide.shapes.add_shape(MSO_SHAPE.RECTANGLE, left, top, width, height)
    shape.line.fill.background()
    if fill_color:
        shape.fill.solid()
        shape.fill.fore_color.rgb = fill_color
    else:
        shape.fill.background()
    if line_color:
        shape.line.fill.solid()
        shape.line.color.rgb = line_color
        if line_width:
            shape.line.width = line_width
    return shape


def add_text_box(slide, left, top, width, height, text, font_size=18,
                 bold=False, color=C_DARK, alignment=PP_ALIGN.LEFT,
                 font_name="Microsoft YaHei"):
    txBox = slide.shapes.add_textbox(left, top, width, height)
    tf = txBox.text_frame
    tf.word_wrap = True
    p = tf.paragraphs[0]
    p.text = text
    p.font.size = Pt(font_size)
    p.font.bold = bold
    p.font.color.rgb = color
    p.font.name = font_name
    p.alignment = alignment
    return txBox


def add_bullet(tf, text, font_size=15, color=C_DARK, bold=False, level=0, font_name="Microsoft YaHei"):
    p = tf.add_paragraph()
    p.text = text
    p.font.size = Pt(font_size)
    p.font.color.rgb = color
    p.font.bold = bold
    p.font.name = font_name
    p.level = level
    p.space_before = Pt(3)
    p.space_after = Pt(1)
    return p


def add_page_number(slide, num, total=16):
    add_text_box(slide, SLIDE_W - Inches(1.2), SLIDE_H - Inches(0.5),
                 Inches(1.0), Inches(0.4),
                 f"{num} / {total}", font_size=9, color=C_GRAY, alignment=PP_ALIGN.RIGHT)


def add_top_bar(slide, color=C_PRIMARY):
    add_shape(slide, 0, 0, SLIDE_W, Inches(0.08), fill_color=color)


def add_bottom_line(slide):
    add_shape(slide, MARGIN_L, SLIDE_H - Inches(0.7), CONTENT_W, Pt(1.5), fill_color=C_PRIMARY)


def add_section_title(slide, title, subtitle=None):
    add_top_bar(slide)
    add_text_box(slide, MARGIN_L, Inches(0.3), CONTENT_W, Inches(0.7),
                 title, font_size=28, bold=True, color=C_PRIMARY)
    add_shape(slide, MARGIN_L, Inches(0.95), Inches(1.8), Pt(3), fill_color=C_ACCENT)
    if subtitle:
        add_text_box(slide, MARGIN_L, Inches(1.1), CONTENT_W, Inches(0.4),
                     subtitle, font_size=16, color=C_ACCENT)
    add_bottom_line(slide)


def add_card(slide, left, top, width, height, title, items, title_color=C_PRIMARY):
    shape = add_shape(slide, left, top, width, height, fill_color=C_LIGHT_BG, line_color=C_LIGHT, line_width=Pt(1))
    add_text_box(slide, left + Inches(0.2), top + Inches(0.1), width - Inches(0.4), Inches(0.4),
                 title, font_size=15, bold=True, color=title_color)
    txBox = add_text_box(slide, left + Inches(0.2), top + Inches(0.5),
                         width - Inches(0.4), height - Inches(0.6), "", font_size=13, color=C_DARK)
    tf = txBox.text_frame
    tf.word_wrap = True
    for i, item in enumerate(items):
        if i == 0:
            tf.paragraphs[0].text = item
            tf.paragraphs[0].font.size = Pt(13)
            tf.paragraphs[0].font.color.rgb = C_DARK
            tf.paragraphs[0].font.name = "Microsoft YaHei"
            tf.paragraphs[0].space_before = Pt(0)
        else:
            add_bullet(tf, item, font_size=13, color=C_DARK)
    return shape


def add_table(slide, left, top, col_widths, headers, rows, header_color=C_PRIMARY):
    """绘制表格（纯形状实现，不依赖pptx table）"""
    row_h = Inches(0.38)
    table_w = sum(col_widths)
    n_rows = len(rows) + 1  # +1 for header
    # Header
    x = left
    for ci, (cw, h) in enumerate(zip(col_widths, headers)):
        add_shape(slide, x, top, cw, row_h, fill_color=header_color, line_color=C_LIGHT, line_width=Pt(0.5))
        add_text_box(slide, x + Inches(0.06), top + Inches(0.03), cw - Inches(0.12), row_h - Inches(0.06),
                     h, font_size=10, bold=True, color=C_WHITE)
        x += cw
    # Data rows
    for ri, row in enumerate(rows):
        y = top + (ri + 1) * row_h
        bg = C_LIGHT_BG if ri % 2 == 0 else C_WHITE
        x = left
        for ci, (cw, cell) in enumerate(zip(col_widths, row)):
            is_bold = ci == 0
            tc = C_DARK
            if ri == len(rows) - 1 and cell.startswith("汇总"):
                tc = C_PRIMARY
            add_shape(slide, x, y, cw, row_h, fill_color=bg, line_color=C_LIGHT, line_width=Pt(0.5))
            add_text_box(slide, x + Inches(0.06), y + Inches(0.03), cw - Inches(0.12), row_h - Inches(0.06),
                         cell, font_size=10, bold=is_bold, color=tc)
            x += cw
    return n_rows * row_h


def add_stat_box(slide, left, top, width, label, value, color=C_PRIMARY, sub=""):
    """添加统计数字卡片"""
    add_shape(slide, left, top, width, Inches(1.0), fill_color=C_LIGHT_BG, line_color=C_LIGHT, line_width=Pt(1))
    add_text_box(slide, left + Inches(0.1), top + Inches(0.05), width - Inches(0.2), Inches(0.25),
                 label, font_size=10, color=C_GRAY)
    add_text_box(slide, left + Inches(0.1), top + Inches(0.3), width - Inches(0.2), Inches(0.4),
                 value, font_size=22, bold=True, color=color)
    if sub:
        add_text_box(slide, left + Inches(0.1), top + Inches(0.7), width - Inches(0.2), Inches(0.25),
                     sub, font_size=9, color=C_GRAY)


# ── 读取训练数据 ──
BASE = "/home/ubuntu/anti_drone_agc_workspace/training_runs"

def load_eval(path):
    rows = []
    try:
        with open(path) as f:
            reader = csv.DictReader(f)
            for r in reader:
                rows.append(r)
    except: pass
    return rows

def load_training(path):
    rows = []
    try:
        with open(path) as f:
            reader = csv.DictReader(f)
            for r in reader:
                rows.append(r)
    except: pass
    return rows

def load_json(path):
    try:
        with open(path) as f:
            return json.load(f)
    except: return {}

# 加载所有训练运行
runs = [
    ("dqn_20260729_203753_852395", "Run 1 (初始尝试)"),
    ("dqn_20260729_211357_108909", "Run 2 (快速测试)"),
    ("dqn_20260730_110905_340632", "Run 3 (中途中断)"),
    ("dqn_20260730_120942_331115", "Run 4 (有改进)"),
    ("dqn_20260730_150505_352023", "Run 5 (最佳结果)"),
]

run_data = {}
for rid, rname in runs:
    p = os.path.join(BASE, rid)
    ev = load_eval(os.path.join(p, "evaluation_log.csv"))
    tr = load_training(os.path.join(p, "training_log.csv"))
    st = load_json(os.path.join(p, "status.json"))
    run_data[rid] = {"eval": ev, "train": tr, "status": st, "name": rname}

# 最佳运行（Run 5）的统计
best_rid = "dqn_20260730_150505_352023"
best_tr = run_data[best_rid]["train"]
best_ev = run_data[best_rid]["eval"]
best_st = run_data[best_rid]["status"]

# 计算最佳运行的详细统计
total_episodes = len(best_tr)
success_count = sum(1 for r in best_tr if r["result"] == "success")
collision_count = sum(1 for r in best_tr if r["result"] == "failed")
success_rate = success_count / total_episodes * 100 if total_episodes else 0

# 阶段统计
early = [r for r in best_tr if int(r["episode"]) < 80]
mid = [r for r in best_tr if 80 <= int(r["episode"]) < 160]
late = [r for r in best_tr if int(r["episode"]) >= 160]

def phase_stats(phase):
    if not phase: return (0, 0, 0)
    s = sum(1 for r in phase if r["result"] == "success")
    f = sum(1 for r in phase if r["result"] == "failed")
    t = len(phase)
    return (s, f, t)

early_s, early_f, early_t = phase_stats(early)
mid_s, mid_f, mid_t = phase_stats(mid)
late_s, late_f, late_t = phase_stats(late)

# 评估阶段最佳
best_eval = max(best_ev, key=lambda r: float(r.get("success_rate", 0)))

# 最佳连续成功
best_streak = 0
current_streak = 0
for r in best_tr:
    if r["result"] == "success":
        current_streak += 1
        best_streak = max(best_streak, current_streak)
    else:
        current_streak = 0

# 平均指标
succ_eps = [r for r in best_tr if r["result"] == "success"]
fail_eps = [r for r in best_tr if r["result"] == "failed"]
avg_succ_reward = sum(float(r["total_reward"]) for r in succ_eps) / len(succ_eps) if succ_eps else 0
avg_succ_steps = sum(int(r["steps"]) for r in succ_eps) / len(succ_eps) if succ_eps else 0
avg_fail_steps = sum(int(r["steps"]) for r in fail_eps) / len(fail_eps) if fail_eps else 0
avg_pos_err = sum(float(r["final_pos_error"]) for r in succ_eps) / len(succ_eps) if succ_eps else 0
avg_fail_reward = sum(float(r["total_reward"]) for r in fail_eps) / len(fail_eps) if fail_eps else 0
avg_all_reward = sum(float(r["total_reward"]) for r in best_tr) / total_episodes if total_episodes else 0
avg_all_steps = sum(int(r["steps"]) for r in best_tr) / total_episodes if total_episodes else 0
avg_fail_pos_err = sum(float(r["final_pos_error"]) for r in fail_eps) / len(fail_eps) if fail_eps else 0

# 训练配置参数
best_config = load_json(os.path.join(BASE, best_rid, "config.json"))

# 动作分布（合并左升+右升为"上升"）
action_counts = [0, 0, 0, 0, 0]  # 0:沿路径, 1:左移, 2:右移, 3:左升, 4:右升
for r in best_tr:
    for a in range(5):
        action_counts[a] += int(r[f"action_{a}"])
total_actions = sum(action_counts)
merged_counts = [action_counts[0], action_counts[1], action_counts[2], action_counts[3] + action_counts[4]]
action_names = ["沿路径", "左移", "右移", "上升"]


# ═══════════════════════════════════════════════
# 第1页：封面
# ═══════════════════════════════════════════════
slide = add_blank_slide()
add_shape(slide, 0, 0, Inches(6.5), SLIDE_H, fill_color=C_PRIMARY)
add_shape(slide, Inches(6.5), 0, Inches(6.833), SLIDE_H, fill_color=C_WHITE)
add_shape(slide, Inches(6.5), 0, Pt(4), SLIDE_H, fill_color=C_ACCENT)

add_text_box(slide, Inches(0.8), Inches(1.5), Inches(5.2), Inches(1.0),
             "反无人机空地协同系统", font_size=40, bold=True, color=C_WHITE)
add_text_box(slide, Inches(0.8), Inches(2.6), Inches(5.2), Inches(0.5),
             "Anti-Drone Air-Ground Collaboration System",
             font_size=18, color=RGBColor(0xA0, 0xC4, 0xE0))
add_shape(slide, Inches(0.8), Inches(3.3), Inches(2.5), Pt(2), fill_color=C_ACCENT)
add_text_box(slide, Inches(0.8), Inches(3.7), Inches(5.2), Inches(0.4),
             "《人工智能实践》课程结课答辩", font_size=22, bold=True, color=C_WHITE)

info_items = [
    "小  组：第 12 组", "班  级：智科 1 班", "组  长：陈鑫豪",
    "组  员：万前进  何俊峰  李英楷", "指导教师：叶景贞", "答辩日期：2026 年 7 月 30 日",
]
txBox = add_text_box(slide, Inches(7.2), Inches(2.0), Inches(5.0), Inches(4.0),
                     "", font_size=16, color=C_DARK)
tf = txBox.text_frame; tf.word_wrap = True
for i, item in enumerate(info_items):
    if i == 0:
        tf.paragraphs[0].text = item
        tf.paragraphs[0].font.size = Pt(18); tf.paragraphs[0].font.bold = True
        tf.paragraphs[0].font.color.rgb = C_PRIMARY; tf.paragraphs[0].font.name = "Microsoft YaHei"
    else:
        p = tf.add_paragraph(); p.text = item; p.font.size = Pt(16)
        p.font.color.rgb = C_DARK; p.font.name = "Microsoft YaHei"; p.space_before = Pt(10)

add_shape(slide, 0, SLIDE_H - Inches(0.15), SLIDE_W, Inches(0.15), fill_color=C_ACCENT)
add_page_number(slide, 1)


# ═══════════════════════════════════════════════
# 第2页：目录
# ═══════════════════════════════════════════════
slide = add_blank_slide()
add_section_title(slide, "目  录", "CONTENTS")

toc = [
    ("01", "项目背景与意义", "反无人机需求 · 研究动机"),
    ("02", "系统架构", "四层架构 · 数据流 · 技术栈"),
    ("03", "核心技术方案", "LQR跟踪 · 感知系统 · RL建模"),
    ("04", "实验数据总览", "5次训练运行 · 关键指标对比"),
    ("05", "DQN训练结果", "最佳运行 · 阶段分析"),
    ("06", "动作分布与详细指标", "动作分布 · 平均指标 · 配置参数"),
    ("07", "训练过程分析", "成功率趋势 · 损失曲线 · 碰撞分析"),
    ("08", "训练分析摘要", "三阶段分析 · 关键发现"),
    ("09", "规则式避障实验", "场景实验 · 真实数据表"),
    ("10", "演示展示", "仿真截图 · 飞行视频"),
]
for i, (num, title, desc) in enumerate(toc):
    row = i // 2; col = i % 2
    left = MARGIN_L + col * (CONTENT_W / 2 + Inches(0.2))
    top = Inches(1.5) + row * Inches(1.1)
    circle = add_shape(slide, left, top, Inches(0.45), Inches(0.45), fill_color=C_PRIMARY)
    circle.text_frame.paragraphs[0].text = num
    circle.text_frame.paragraphs[0].font.size = Pt(14); circle.text_frame.paragraphs[0].font.bold = True
    circle.text_frame.paragraphs[0].font.color.rgb = C_WHITE; circle.text_frame.paragraphs[0].alignment = PP_ALIGN.CENTER
    circle.text_frame.paragraphs[0].font.name = "Microsoft YaHei"
    circle.text_frame.vertical_anchor = MSO_ANCHOR.MIDDLE
    add_text_box(slide, left + Inches(0.65), top, Inches(4.5), Inches(0.3),
                 title, font_size=17, bold=True, color=C_DARK)
    add_text_box(slide, left + Inches(0.65), top + Inches(0.3), Inches(4.5), Inches(0.25),
                 desc, font_size=11, color=C_GRAY)
add_page_number(slide, 2)


# ═══════════════════════════════════════════════
# 第3页：项目背景与意义（精简为1列，去掉国内外研究现状）
# ═══════════════════════════════════════════════
slide = add_blank_slide()
add_section_title(slide, "01  项目背景与意义")

# 项目背景
add_shape(slide, MARGIN_L, Inches(1.5), CONTENT_W, Inches(2.2), fill_color=C_LIGHT_BG, line_color=C_LIGHT, line_width=Pt(1))
add_text_box(slide, MARGIN_L + Inches(0.2), Inches(1.6), CONTENT_W - Inches(0.4), Inches(0.35),
             "项目背景", font_size=16, bold=True, color=C_PRIMARY)
bg_text = "无人机广泛用于巡检、物流、安防等场景\n黑飞无人机对敏感区域构成安全威胁\n传统反无人机方法依赖人工操作，效率低"
txBox = add_text_box(slide, MARGIN_L + Inches(0.2), Inches(2.1), CONTENT_W - Inches(0.4), Inches(1.5), "", font_size=13, color=C_DARK)
tf = txBox.text_frame; tf.word_wrap = True
for i, line in enumerate(bg_text.split("\n")):
    if i == 0:
        tf.paragraphs[0].text = f"• {line}"; tf.paragraphs[0].font.size = Pt(13)
        tf.paragraphs[0].font.color.rgb = C_DARK; tf.paragraphs[0].font.name = "Microsoft YaHei"
    else:
        add_bullet(tf, f"• {line}", font_size=13)

# 研究意义
add_shape(slide, MARGIN_L, Inches(3.9), CONTENT_W, Inches(2.0), fill_color=C_LIGHT_BG, line_color=C_LIGHT, line_width=Pt(1))
add_text_box(slide, MARGIN_L + Inches(0.2), Inches(4.0), CONTENT_W - Inches(0.4), Inches(0.35),
             "研究意义", font_size=16, bold=True, color=C_PRIMARY)
sig_text = "应用层面：提升反无人机系统的自动化程度\n技术层面：融合LQR控制与强化学习决策\n实践层面：构建完整的ROS仿真验证平台"
txBox2 = add_text_box(slide, MARGIN_L + Inches(0.2), Inches(4.5), CONTENT_W - Inches(0.4), Inches(1.3), "", font_size=13, color=C_DARK)
tf2 = txBox2.text_frame; tf2.word_wrap = True
for i, line in enumerate(sig_text.split("\n")):
    if i == 0:
        tf2.paragraphs[0].text = f"• {line}"; tf2.paragraphs[0].font.size = Pt(13)
        tf2.paragraphs[0].font.color.rgb = C_DARK; tf2.paragraphs[0].font.name = "Microsoft YaHei"
    else:
        add_bullet(tf2, f"• {line}", font_size=13)

add_page_number(slide, 3)


# ═══════════════════════════════════════════════
# 第4页：系统架构
# ═══════════════════════════════════════════════
slide = add_blank_slide()
add_section_title(slide, "02  系统架构设计")

layers = [
    ("感知层 (Perception)", "Velodyne LiDAR → perception_node → 12维扇区特征", C_PRIMARY),
    ("决策层 (Planning)", "rl_planner_node (DQN/规则式) → 避障偏移量", C_ACCENT),
    ("跟踪层 (Control)", "lqr_tracker_node → LQR控制器 → 位置设定点", RGBColor(0x17, 0xA5, 0x89)),
    ("执行层 (Execution)", "MAVROS ↔ PX4 SITL ↔ Gazebo 仿真环境", RGBColor(0xE6, 0x7E, 0x22)),
]
layer_h = Inches(1.0); start_y = Inches(1.5); layer_w = Inches(10.5); layer_left = MARGIN_L
for i, (name, desc, color) in enumerate(layers):
    y = start_y + i * (layer_h + Inches(0.25))
    add_shape(slide, layer_left, y, layer_w, layer_h, fill_color=color)
    add_text_box(slide, layer_left + Inches(0.3), y + Inches(0.1), Inches(4.0), Inches(0.4),
                 name, font_size=16, bold=True, color=C_WHITE)
    add_text_box(slide, layer_left + Inches(0.3), y + Inches(0.5), Inches(9.5), Inches(0.4),
                 desc, font_size=13, color=C_WHITE)
    if i < len(layers) - 1:
        add_shape(slide, layer_left + Inches(5.0), y + layer_h, Inches(0.5), Inches(0.08), fill_color=C_GRAY)

add_text_box(slide, MARGIN_L, Inches(6.5), Inches(5.0), Inches(0.3),
             "数据流", font_size=11, bold=True, color=C_GRAY, alignment=PP_ALIGN.CENTER)
add_text_box(slide, MARGIN_L, Inches(6.7), Inches(6.5), Inches(0.3),
             "/velodyne_points → /obstacle_features → /rl_planner/local_goal → /mavros/setpoint_position/local",
             font_size=11, color=C_GRAY)
add_shape(slide, Inches(7.5), Inches(6.5), Inches(4.5), Inches(0.7), fill_color=C_LIGHT_BG, line_color=C_LIGHT, line_width=Pt(1))
add_text_box(slide, Inches(7.7), Inches(6.55), Inches(4.1), Inches(0.25),
             "技术栈", font_size=12, bold=True, color=C_ACCENT)
add_text_box(slide, Inches(7.7), Inches(6.75), Inches(4.1), Inches(0.4),
             "Ubuntu+ROS  |  Gazebo+XTDrone  |  PX4+MAVROS  |  PyTorch",
             font_size=9, color=C_DARK)
add_page_number(slide, 4)


# ═══════════════════════════════════════════════
# 第5页：核心技术方案（每列精简至5-6个要点）
# ═══════════════════════════════════════════════
slide = add_blank_slide()
add_section_title(slide, "03  核心技术方案")
col_w = (CONTENT_W - Inches(0.4)) / 3

add_card(slide, MARGIN_L, Inches(1.5), col_w, Inches(4.8),
         "LQR 轨迹跟踪控制器",
         ["• 状态空间：位置误差 + 速度误差",
          "• 控制律：u = -K·x，Riccati方程求解",
          "• 轨迹：直线/圆形/8字形/悬停",
          "• 8字形：ω=0.1 rad/s, v=0.3 m/s",
          "• 发布 /mavros/setpoint_position/local",
          "• 误差监控：位置/速度实时反馈"],
         C_PRIMARY)

add_card(slide, MARGIN_L + col_w + Inches(0.2), Inches(1.5), col_w, Inches(4.8),
         "感知系统",
         ["• Velodyne VLP-16 激光雷达",
          "• 360° 环绕扫描，12个扇区",
          "• 每扇区输出最近障碍物距离",
          "• 话题：/obstacle_features",
          "• 消息类型：ObstacleFeatures",
          "• 最小量程 0.5m（过滤自身回波）"],
         C_ACCENT)

add_card(slide, MARGIN_L + 2 * (col_w + Inches(0.2)), Inches(1.5), col_w, Inches(4.8),
         "强化学习建模",
         ["• 状态空间（21维）：12扇区+目标方向+距离+速度+上一步动作",
          "• 动作空间（5个）：沿路径/左移/右移/左上升/右上升",
          "• 奖励：progress + success(+30) + collision(-30)",
          "  + proximity_penalty + step_penalty + smoothness",
          "• DQN：Q网络 + 目标网络 + ReplayBuffer",
          "• ε-greedy探索（1.0→0.05, 12000步）"],
         RGBColor(0x17, 0xA5, 0x89))
add_page_number(slide, 5)


# ═══════════════════════════════════════════════
# 第6页：实验数据总览（4列表格，去掉关键发现）
# ═══════════════════════════════════════════════
slide = add_blank_slide()
add_section_title(slide, "04  实验数据总览", "5次DQN训练运行 · 关键指标对比")

# 运行对比表（4列：运行名、episode数、最佳成功率、状态）
col_w_table = [Inches(1.8), Inches(2.5), Inches(2.5), Inches(2.0)]
headers = ["运行", "Episode", "最佳成功率", "状态"]
rows = []
for rid, rname in runs:
    st = run_data[rid]["status"]
    ev = run_data[rid]["eval"]
    eps = st.get("completed_episodes", 0)
    status = st.get("status", "?")
    best_sr = 0
    for e in ev:
        sr = float(e.get("success_rate", 0))
        if sr > best_sr:
            best_sr = sr
    short_name = rid.split("_")[-1]
    status_label = {"completed": "完成", "interrupted": "中断", "completed_without_updates": "无更新"}
    sl = status_label.get(status, status)
    rows.append([short_name, str(eps), f"{best_sr*100:.0f}%", sl])

add_table(slide, MARGIN_L, Inches(1.5), col_w_table, headers, rows)
add_page_number(slide, 6)


# ═══════════════════════════════════════════════
# 第7页：DQN训练结果（4个统计卡片+阶段表+动作分布，去掉底部平均指标对比）
# ═══════════════════════════════════════════════
slide = add_blank_slide()
add_section_title(slide, "05  DQN训练结果", "最佳运行（Run 5: 150505）详细分析")

# 顶部统计数字卡（4个，去掉"最佳评估成功率"）
stat_w = (CONTENT_W - Inches(0.3)) / 4
stats = [
    ("总Episode", str(total_episodes), "完成239个", C_PRIMARY),
    ("成功数", str(success_count), f"成功率 {success_rate:.0f}%", C_GREEN),
    ("总训练步数", f"{best_st.get('total_steps', 0):,}", "更新步数 20,893", C_ACCENT),
    ("最佳连续成功", f"{best_streak}", "连续成功", C_ORANGE),
]
for i, (label, value, sub, color) in enumerate(stats):
    left = MARGIN_L + i * (stat_w + Inches(0.1))
    add_stat_box(slide, left, Inches(1.5), stat_w, label, value, color, sub)

# 阶段分析
phase_top = Inches(2.8)
add_text_box(slide, MARGIN_L, phase_top, CONTENT_W, Inches(0.35),
             "分阶段训练表现", font_size=16, bold=True, color=C_PRIMARY)

ph_col = [Inches(3.5), Inches(2.0), Inches(2.0), Inches(2.0), Inches(2.0)]
ph_headers = ["阶段", "Episode", "成功", "失败", "成功率"]
ph_rows = [
    [f"早期 (0-79)", str(early_t), str(early_s), str(early_f),
     f"{early_s/early_t*100:.0f}%" if early_t else "0%"],
    [f"中期 (80-159)", str(mid_t), str(mid_s), str(mid_f),
     f"{mid_s/mid_t*100:.0f}%" if mid_t else "0%"],
    [f"后期 (160-238)", str(late_t), str(late_s), str(late_f),
     f"{late_s/late_t*100:.0f}%" if late_t else "0%"],
]
add_table(slide, MARGIN_L, phase_top + Inches(0.4), ph_col, ph_headers, ph_rows, C_PRIMARY)

add_page_number(slide, 7)


# ═══════════════════════════════════════════════
# 第8页：DQN 动作分布与详细指标（从第7页拆分）
# ═══════════════════════════════════════════════
slide = add_blank_slide()
add_section_title(slide, "06  DQN 动作分布与详细指标", "最佳运行（Run 5）动作分布、平均指标与训练配置")

# 左侧：动作分布表 + 平均指标表
left_w = Inches(5.8)
add_text_box(slide, MARGIN_L, Inches(1.5), left_w, Inches(0.35),
             "动作分布（全训练期）", font_size=16, bold=True, color=C_PRIMARY)
act_col = [Inches(1.8), Inches(1.8), Inches(1.8)]
act_headers = ["动作", "次数", "占比"]
act_rows = []
for a in range(4):
    cnt = merged_counts[a]
    pct = cnt / total_actions * 100 if total_actions else 0
    act_rows.append([action_names[a], f"{cnt:,}", f"{pct:.1f}%"])
add_table(slide, MARGIN_L, Inches(1.9), act_col, act_headers, act_rows, C_ACCENT)

# 平均指标表
avg_table_top = Inches(3.5)
add_text_box(slide, MARGIN_L, avg_table_top, left_w, Inches(0.35),
             "平均指标对比", font_size=16, bold=True, color=C_PRIMARY)
avg_col = [Inches(1.5), Inches(1.4), Inches(1.4), Inches(1.4)]
avg_headers = ["类别", "奖励", "步数", "位置误差"]
avg_rows = [
    ["成功", f"{avg_succ_reward:+.1f}", f"{avg_succ_steps:.0f}", f"{avg_pos_err:.2f}m"],
    ["失败", f"{avg_fail_reward:+.1f}", f"{avg_fail_steps:.0f}", f"{avg_fail_pos_err:.2f}m"],
    ["所有", f"{avg_all_reward:+.1f}", f"{avg_all_steps:.0f}", "-"],
]
add_table(slide, MARGIN_L, avg_table_top + Inches(0.4), avg_col, avg_headers, avg_rows, C_PRIMARY)

# 右侧：训练配置参数
right_left = MARGIN_L + left_w + Inches(0.3)
right_w = CONTENT_W - left_w - Inches(0.3)
add_text_box(slide, right_left, Inches(1.5), right_w, Inches(0.35),
             "训练配置参数", font_size=16, bold=True, color=C_PRIMARY)
cfg = best_config
cfg_items = [
    f"学习率 (learning_rate):  {cfg.get('learning_rate', 'N/A')}",
    f"折扣因子 (gamma):        {cfg.get('gamma', 'N/A')}",
    f"批次大小 (batch_size):   {cfg.get('batch_size', 'N/A')}",
    f"经验池容量 (buffer):     {cfg.get('buffer_capacity', 'N/A')}",
    f"ε-初始:                  {cfg.get('epsilon_start', 'N/A')}",
    f"ε-最终:                  {cfg.get('epsilon_end', 'N/A')}",
    f"ε-衰减步数:              {cfg.get('epsilon_decay_steps', 'N/A')}",
    f"目标网络更新步数:        {cfg.get('target_update_steps', 'N/A')}",
    f"训练起始步数:            {cfg.get('train_start_steps', 'N/A')}",
    f"Episode数:               {cfg.get('episodes', 'N/A')}",
    f"每Episode最大步数:       {cfg.get('max_steps', 'N/A')}",
    f"评估间隔:                {cfg.get('eval_interval', 'N/A')}",
    f"奖励函数版本:            {cfg.get('reward_version', 'N/A')}",
]
txBox = add_text_box(slide, right_left, Inches(1.9), right_w, Inches(5.0), "", font_size=11, color=C_DARK)
tf = txBox.text_frame; tf.word_wrap = True
for i, item in enumerate(cfg_items):
    if i == 0:
        tf.paragraphs[0].text = item; tf.paragraphs[0].font.size = Pt(11)
        tf.paragraphs[0].font.color.rgb = C_DARK; tf.paragraphs[0].font.name = "Microsoft YaHei"
        tf.paragraphs[0].space_before = Pt(0)
    else:
        add_bullet(tf, item, font_size=11)
add_page_number(slide, 8)


# ═══════════════════════════════════════════════
# 第9页：训练过程分析（每20episode聚合，12组，条形图）
# ═══════════════════════════════════════════════
slide = add_blank_slide()
add_section_title(slide, "07  训练过程分析", "Run 5 训练过程详细数据")

# 左侧：成功率变化趋势（每20个episode聚合，只显示前12组）
left_w8 = Inches(5.5)
add_text_box(slide, MARGIN_L, Inches(1.4), left_w8, Inches(0.35),
             "阶段性成功率", font_size=16, bold=True, color=C_PRIMARY)

# 聚合20个episode为一组，只显示前12组
window = 20
groups = []
for i in range(0, min(total_episodes, 240), window):
    chunk = best_tr[i:i+window]
    s = sum(1 for r in chunk if r["result"] == "success")
    groups.append((i, i+len(chunk)-1, s, len(chunk)))

# 柱状图
bar_top = Inches(1.7)
bar_h = Inches(0.22)
bar_gap = Inches(0.04)
bar_left = MARGIN_L
bar_max_w = left_w8 - Inches(0.5)
max_per_group = window

for gi, (start, end, succ, count) in enumerate(groups):
    if gi >= 12: break  # 最多显示12组
    y = bar_top + gi * (bar_h + bar_gap)
    lab = f"{start}-{end}"
    add_text_box(slide, bar_left, y, Inches(0.85), bar_h, lab, font_size=8, color=C_GRAY, alignment=PP_ALIGN.RIGHT)
    fail_w = bar_max_w * (count - succ) / max_per_group
    succ_w = bar_max_w * succ / max_per_group
    if succ_w > 0:
        add_shape(slide, bar_left + Inches(0.9), y, succ_w, bar_h, fill_color=C_GREEN)
    if fail_w > 0:
        add_shape(slide, bar_left + Inches(0.9) + succ_w, y, fail_w, bar_h, fill_color=C_RED)

# 图例（放在条形图下方）
bars_bottom = bar_top + 12 * (bar_h + bar_gap)
leg_y = bars_bottom + Inches(0.08)
add_shape(slide, bar_left + Inches(0.9), leg_y, Inches(0.3), Inches(0.18), fill_color=C_GREEN)
add_text_box(slide, bar_left + Inches(1.3), leg_y, Inches(0.8), Inches(0.18), "成功", font_size=9, color=C_GREEN)
add_shape(slide, bar_left + Inches(2.2), leg_y, Inches(0.3), Inches(0.18), fill_color=C_RED)
add_text_box(slide, bar_left + Inches(2.6), leg_y, Inches(0.8), Inches(0.18), "失败(碰撞)", font_size=9, color=C_RED)

# 右侧：评估指标趋势（表格上移至T=1.7"）
right_w8 = Inches(5.5)
right_left8 = MARGIN_L + left_w8 + Inches(0.3)
add_text_box(slide, right_left8, Inches(1.4), right_w8, Inches(0.35),
             "评估指标趋势", font_size=16, bold=True, color=C_PRIMARY)

# 评估结果表
ev_col = [Inches(1.2), Inches(1.2), Inches(1.2), Inches(1.2), Inches(1.0)]
ev_headers = ["评估", "成功率", "碰撞率", "平均奖励", "步骤"]
ev_rows = []
for e in best_ev:
    sr = f"{float(e['success_rate'])*100:.0f}%"
    cr = f"{float(e['collision_rate'])*100:.0f}%"
    mr = f"{float(e['mean_reward']):+.0f}"
    ms = f"{float(e['mean_steps']):.0f}"
    ev_rows.append([f"#{e['evaluation']}", sr, cr, mr, ms])
add_table(slide, right_left8, Inches(1.7), ev_col, ev_headers, ev_rows, C_ACCENT)

add_page_number(slide, 9)


# ═══════════════════════════════════════════════
# 第10页：训练分析摘要
# ═══════════════════════════════════════════════
slide = add_blank_slide()
add_section_title(slide, "08  训练分析摘要", "Run 5 三阶段训练过程分析与关键发现")

# 三阶段分析卡片
phase_card_w = (CONTENT_W - Inches(0.4)) / 3
phases_data = [
    ("早期（0-79 Episode）", "探索阶段", [
        "ε从1.0逐渐衰减，以随机探索为主",
        "碰撞率高，但已出现成功避障",
        "成功episode逐步增加",
        "奖励波动较大，策略未收敛",
    ], C_ORANGE),
    ("中期（80-159 Episode）", "策略形成阶段", [
        "ε降至约0.5，探索与利用平衡",
        "多组成功率超过50%",
        "策略趋于稳定，碰撞减少",
        "奖励曲线趋于平滑",
    ], C_PRIMARY),
    ("后期（160-238 Episode）", "策略成熟阶段", [
        "ε降至0.05，以利用为主",
        "连续多组保持高成功率",
        "最佳连续成功13次",
        "仍存在偶发碰撞（边界情况）",
    ], C_GREEN),
]
for i, (title, subtitle, items, color) in enumerate(phases_data):
    left = MARGIN_L + i * (phase_card_w + Inches(0.2))
    add_shape(slide, left, Inches(1.5), phase_card_w, Inches(3.0), fill_color=C_LIGHT_BG, line_color=color, line_width=Pt(2))
    add_shape(slide, left, Inches(1.5), phase_card_w, Inches(0.5), fill_color=color)
    add_text_box(slide, left + Inches(0.15), Inches(1.55), phase_card_w - Inches(0.3), Inches(0.25),
                 title, font_size=13, bold=True, color=C_WHITE)
    add_text_box(slide, left + Inches(0.15), Inches(1.8), phase_card_w - Inches(0.3), Inches(0.2),
                 subtitle, font_size=10, color=C_WHITE)
    txBox = add_text_box(slide, left + Inches(0.15), Inches(2.15), phase_card_w - Inches(0.3), Inches(2.2), "", font_size=11, color=C_DARK)
    tf = txBox.text_frame; tf.word_wrap = True
    for j, item in enumerate(items):
        if j == 0:
            tf.paragraphs[0].text = f"• {item}"; tf.paragraphs[0].font.size = Pt(11)
            tf.paragraphs[0].font.color.rgb = C_DARK; tf.paragraphs[0].font.name = "Microsoft YaHei"
            tf.paragraphs[0].space_before = Pt(0)
        else:
            add_bullet(tf, f"• {item}", font_size=11)

# 底部：关键发现
add_text_box(slide, MARGIN_L, Inches(4.8), CONTENT_W, Inches(0.35),
             "关键发现", font_size=16, bold=True, color=C_PRIMARY)
findings = [
    "① DQN策略从随机探索到稳定避障的收敛过程清晰，后期成功率明显优于早期",
    "② 连续成功表明模型已学到有效的避障策略，但偶发碰撞提示边界情况仍需优化",
    "③ 奖励函数设计合理：progress引导沿路径前进，collision惩罚有效抑制碰撞行为",
    "④ 经验回放机制使训练稳定，ε-greedy探索策略有效平衡了探索与利用",
]
txBox = add_text_box(slide, MARGIN_L, Inches(5.2), CONTENT_W, Inches(1.8), "", font_size=12, color=C_DARK)
tf = txBox.text_frame; tf.word_wrap = True
for i, item in enumerate(findings):
    if i == 0:
        tf.paragraphs[0].text = item; tf.paragraphs[0].font.size = Pt(12)
        tf.paragraphs[0].font.color.rgb = C_DARK; tf.paragraphs[0].font.name = "Microsoft YaHei"
        tf.paragraphs[0].space_before = Pt(0)
    else:
        add_bullet(tf, item, font_size=12)
add_page_number(slide, 10)


# ═══════════════════════════════════════════════
# 第11页：规则式避障实验（去掉评估指标列表）
# ═══════════════════════════════════════════════
slide = add_blank_slide()
add_section_title(slide, "09  规则式避障实验", "基于扇区距离的启发式避障基线")

# 场景
add_text_box(slide, MARGIN_L, Inches(1.4), CONTENT_W, Inches(0.35),
             "实验场景", font_size=16, bold=True, color=C_PRIMARY)
scenarios = [
    ("S0 无障碍", "基础跟踪测试"), ("S1 稀疏静态", "单障碍避障"),
    ("S2 密集静态", "多障碍连续避障"), ("S3 动态障碍", "移动障碍避障"),
]
for i, (sname, sdesc) in enumerate(scenarios):
    left = MARGIN_L + i * (CONTENT_W / 4 + Inches(0.1))
    w = CONTENT_W / 4 - Inches(0.1)
    add_shape(slide, left, Inches(1.85), w, Inches(0.8), fill_color=C_LIGHT_BG, line_color=C_LIGHT, line_width=Pt(1))
    add_text_box(slide, left + Inches(0.1), Inches(1.9), w - Inches(0.2), Inches(0.25),
                 sname, font_size=13, bold=True, color=C_PRIMARY)
    add_text_box(slide, left + Inches(0.1), Inches(2.2), w - Inches(0.2), Inches(0.3),
                 sdesc, font_size=11, color=C_GRAY)

# 实验数据表
add_text_box(slide, MARGIN_L, Inches(2.9), CONTENT_W, Inches(0.35),
             "规则式避障实验结果（2026-07-09）", font_size=16, bold=True, color=C_PRIMARY)
data_rows = [
    ("指标", "数值", "说明"),
    ("运行时长", "81.7 s", "单次实验持续"),
    ("记录条数", "818 条", "CSV记录总数"),
    ("避障激活占比", "35.7%", "is_avoiding=1 时间占比"),
    ("位置误差均值", "0.794 m", "中位0.676m，最大5.107m"),
    ("动作分布", "前进64.2% / 左移18.8% / 上升16.9%", "规则式决策分布"),
]
data_col = [Inches(1.8), Inches(2.5), Inches(6.5)]

# 手动绘制表头
hdr_top = Inches(3.35)
hdr_cols = [Inches(1.8), Inches(2.5), Inches(6.5)]
hdr_texts = ["指标", "数值", "说明"]
x = MARGIN_L
for cw, ht in zip(hdr_cols, hdr_texts):
    add_shape(slide, x, hdr_top, cw, Inches(0.38), fill_color=C_PRIMARY, line_color=C_LIGHT, line_width=Pt(0.5))
    add_text_box(slide, x + Inches(0.06), hdr_top + Inches(0.03), cw - Inches(0.12), Inches(0.32),
                 ht, font_size=10, bold=True, color=C_WHITE)
    x += cw

for ri, row in enumerate(data_rows[1:]):
    y = hdr_top + (ri + 1) * Inches(0.38)
    bg = C_LIGHT_BG if ri % 2 == 0 else C_WHITE
    x = MARGIN_L
    for ci, (cw, cell) in enumerate(zip(hdr_cols, row)):
        add_shape(slide, x, y, cw, Inches(0.38), fill_color=bg, line_color=C_LIGHT, line_width=Pt(0.5))
        add_text_box(slide, x + Inches(0.06), y + Inches(0.03), cw - Inches(0.12), Inches(0.32),
                     cell, font_size=10, bold=(ci == 0), color=C_DARK)
        x += cw

add_page_number(slide, 11)


# ═══════════════════════════════════════════════
# 第12页：演示展示（留白页，缩小占位框高度）
# ═══════════════════════════════════════════════
slide = add_blank_slide()
add_section_title(slide, "10  演示展示", "仿真运行截图 & 避障飞行视频")

# 左侧大图占位（高度从3.5"缩小到2.8"）
add_shape(slide, MARGIN_L, Inches(1.5), Inches(7.0), Inches(2.8),
          fill_color=C_DEMO_BG, line_color=None)
add_shape(slide, MARGIN_L + Inches(0.5), Inches(1.5), Inches(6.0), Pt(1), fill_color=C_ACCENT)
add_text_box(slide, MARGIN_L, Inches(2.2), Inches(7.0), Inches(0.5),
             "仿真飞行演示视频", font_size=22, bold=True, color=C_WHITE,
             alignment=PP_ALIGN.CENTER)
add_text_box(slide, MARGIN_L, Inches(2.8), Inches(7.0), Inches(0.4),
             "（待录制后插入）", font_size=14, color=C_GRAY, alignment=PP_ALIGN.CENTER)
add_text_box(slide, MARGIN_L, Inches(3.3), Inches(7.0), Inches(0.3),
             "推荐时长：30-60秒 · 展示LQR跟踪 + 避障全流程", font_size=11, color=C_GRAY, alignment=PP_ALIGN.CENTER)

# 右侧两个小图占位（高度缩小）
# 右1
add_shape(slide, MARGIN_L + Inches(7.3), Inches(1.5), Inches(3.8), Inches(1.3),
          fill_color=C_DEMO_BG, line_color=None)
add_text_box(slide, MARGIN_L + Inches(7.3), Inches(1.75), Inches(3.8), Inches(0.5),
             "Gazebo仿真截图", font_size=15, bold=True, color=C_WHITE, alignment=PP_ALIGN.CENTER)
add_text_box(slide, MARGIN_L + Inches(7.3), Inches(2.3), Inches(3.8), Inches(0.3),
             "（待插入）", font_size=11, color=C_GRAY, alignment=PP_ALIGN.CENTER)

# 右2
add_shape(slide, MARGIN_L + Inches(7.3), Inches(3.0), Inches(3.8), Inches(1.3),
          fill_color=C_DEMO_BG, line_color=None)
add_text_box(slide, MARGIN_L + Inches(7.3), Inches(3.25), Inches(3.8), Inches(0.5),
             "DQN训练曲线", font_size=15, bold=True, color=C_WHITE, alignment=PP_ALIGN.CENTER)
add_text_box(slide, MARGIN_L + Inches(7.3), Inches(3.8), Inches(3.8), Inches(0.3),
             "（待插入训练曲线图）", font_size=11, color=C_GRAY, alignment=PP_ALIGN.CENTER)

# 底部：演示要点
add_text_box(slide, MARGIN_L, Inches(4.6), CONTENT_W, Inches(0.35),
             "演示内容建议", font_size=16, bold=True, color=C_PRIMARY)
demo_tips = [
    "①  Gazebo仿真环境启动 → 无人机起飞 → LQR 8字形轨迹跟踪",
    "②  遇到障碍物 → 规则式避障规划器触发 → 绕行后回到轨迹",
    "③  DQN训练过程展示（可选）：训练曲线、成功率变化",
    "④  系统整体演示：感知 → 决策 → 跟踪 → 执行全链路闭环",
]
txBox = add_text_box(slide, MARGIN_L, Inches(5.0), CONTENT_W, Inches(1.5), "", font_size=13, color=C_DARK)
tf = txBox.text_frame; tf.word_wrap = True
for i, tip in enumerate(demo_tips):
    if i == 0:
        tf.paragraphs[0].text = tip; tf.paragraphs[0].font.size = Pt(13)
        tf.paragraphs[0].font.color.rgb = C_DARK; tf.paragraphs[0].font.name = "Microsoft YaHei"
        tf.paragraphs[0].space_before = Pt(0)
    else:
        add_bullet(tf, tip, font_size=13)
add_page_number(slide, 12)


# ═══════════════════════════════════════════════
# 第13页：演示展示（第二页，缩小占位框高度）
# ═══════════════════════════════════════════════
slide = add_blank_slide()
add_section_title(slide, "10  演示展示（续）", "系统运行截图 & 实验录制")

# 上排两个图占位（高度从2.5"缩小到2.0"）
for i, (title, desc) in enumerate([
    ("场景S1：稀疏静态障碍", "无人机在LQR跟踪中遇到单个障碍物，触发避障绕行"),
    ("场景S2：密集静态障碍", "多障碍连续避障，规则式规划器实时决策绕行方向"),
]):
    left = MARGIN_L + i * (CONTENT_W / 2 + Inches(0.1))
    w = CONTENT_W / 2 - Inches(0.1)
    add_shape(slide, left, Inches(1.5), w, Inches(2.0), fill_color=C_DEMO_BG, line_color=None)
    add_text_box(slide, left, Inches(1.85), w, Inches(0.5), title, font_size=17, bold=True, color=C_WHITE, alignment=PP_ALIGN.CENTER)
    add_text_box(slide, left, Inches(2.4), w, Inches(0.4), desc, font_size=11, color=C_GRAY, alignment=PP_ALIGN.CENTER)
    add_text_box(slide, left, Inches(2.9), w, Inches(0.3), "（待插入截图）", font_size=11, color=C_GRAY, alignment=PP_ALIGN.CENTER)

# 下排两个图占位（高度从2.5"缩小到2.0"）
for i, (title, desc) in enumerate([
    ("场景S3：动态障碍物", "移动障碍物场景，测试避障策略的实时响应能力"),
    ("实验数据可视化", "训练曲线、成功率/碰撞率变化、动作分布图"),
]):
    left = MARGIN_L + i * (CONTENT_W / 2 + Inches(0.1))
    w = CONTENT_W / 2 - Inches(0.1)
    add_shape(slide, left, Inches(3.8), w, Inches(2.0), fill_color=C_DEMO_BG, line_color=None)
    add_text_box(slide, left, Inches(4.15), w, Inches(0.5), title, font_size=17, bold=True, color=C_WHITE, alignment=PP_ALIGN.CENTER)
    add_text_box(slide, left, Inches(4.7), w, Inches(0.4), desc, font_size=11, color=C_GRAY, alignment=PP_ALIGN.CENTER)
    add_text_box(slide, left, Inches(5.2), w, Inches(0.3), "（待插入截图/图表）", font_size=11, color=C_GRAY, alignment=PP_ALIGN.CENTER)

add_page_number(slide, 13)


# ═══════════════════════════════════════════════
# 第14页：项目进展与分工（团队分工去掉描述，只保留姓名和角色）
# ═══════════════════════════════════════════════
slide = add_blank_slide()
add_section_title(slide, "11  项目进展与分工")

col_w3 = (CONTENT_W - Inches(0.3)) / 2
add_card(slide, MARGIN_L, Inches(1.5), col_w3, Inches(2.6),
         "已完成工作",
         ["✔ LQR轨迹跟踪器（直线/圆形/8字/悬停）",
          "✔ 感知节点 perception_node（12扇区）",
          "✔ 规则式避障规划器 rl_planner_node",
          "✔ 实验日志节点 experiment_logger_node",
          "✔ DQN智能体完整实现 + 训练管线",
          "✔ 5次DQN训练运行，最佳成功率100%",
          "✔ 启动脚本 start_sim.sh（多模式）",
          "✔ 实验数据记录与CSV导出"],
         C_GREEN)

add_card(slide, MARGIN_L + col_w3 + Inches(0.3), Inches(1.5), col_w3, Inches(2.6),
         "进行中工作",
         ["→ 多场景（S0-S3）系统化指标记录",
          "→ DQN训练调参优化（提升稳定性）",
          "→ 规则式 vs DQN 避障效果对比",
          "→ 实验数据可视化（训练曲线图）"],
         C_ORANGE)

# 团队分工（只保留姓名和角色，去掉描述文字）
add_text_box(slide, MARGIN_L, Inches(4.4), CONTENT_W, Inches(0.35),
             "团队分工", font_size=16, bold=True, color=C_PRIMARY)
members = [
    ("陈鑫豪", "组长/算法"),
    ("万前进", "实验测试"),
    ("何俊峰", "系统集成"),
    ("李英楷", "文档/PPT"),
]
member_w = (CONTENT_W - Inches(0.3)) / 4
for i, (name, role) in enumerate(members):
    left = MARGIN_L + i * (member_w + Inches(0.1))
    avatar = add_shape(slide, left + member_w / 2 - Inches(0.3), Inches(4.85),
                       Inches(0.6), Inches(0.6), fill_color=C_PRIMARY)
    avatar.text_frame.paragraphs[0].text = name[0]
    avatar.text_frame.paragraphs[0].font.size = Pt(20); avatar.text_frame.paragraphs[0].font.bold = True
    avatar.text_frame.paragraphs[0].font.color.rgb = C_WHITE; avatar.text_frame.paragraphs[0].alignment = PP_ALIGN.CENTER
    avatar.text_frame.paragraphs[0].font.name = "Microsoft YaHei"
    avatar.text_frame.vertical_anchor = MSO_ANCHOR.MIDDLE
    add_text_box(slide, left, Inches(5.55), member_w, Inches(0.3),
                 name, font_size=14, bold=True, color=C_DARK, alignment=PP_ALIGN.CENTER)
    add_text_box(slide, left, Inches(5.85), member_w, Inches(0.25),
                 role, font_size=11, color=C_ACCENT, alignment=PP_ALIGN.CENTER)
add_page_number(slide, 14)


# ═══════════════════════════════════════════════
# 第15页：总结与展望
# ═══════════════════════════════════════════════
slide = add_blank_slide()
add_section_title(slide, "12  总结与展望")

add_text_box(slide, MARGIN_L, Inches(1.4), CONTENT_W, Inches(0.35),
             "项目总结", font_size=16, bold=True, color=C_PRIMARY)
summary_items = [
    "① 构建了完整的ROS反无人机空地协同仿真系统（感知→决策→跟踪→执行）",
    "② 实现LQR轨迹跟踪控制器，支持直线/圆形/8字形/悬停多种轨迹",
    "③ 实现扇区特征感知节点和规则式避障规划器，具备实时避障能力",
    "④ 完整实现DQN智能体（Q网络+目标网络+经验回放），训练管线就绪",
    "⑤ 完成5次DQN训练实验，最佳运行在episode 120-200区间连续保持100%成功率",
    "⑥ 建立实验日志系统，支持CSV记录和实验数据统计分析",
]
txBox = add_text_box(slide, MARGIN_L, Inches(1.8), CONTENT_W, Inches(2.8), "", font_size=13, color=C_DARK)
tf = txBox.text_frame; tf.word_wrap = True
for i, item in enumerate(summary_items):
    if i == 0:
        tf.paragraphs[0].text = item; tf.paragraphs[0].font.size = Pt(13)
        tf.paragraphs[0].font.color.rgb = C_DARK; tf.paragraphs[0].font.name = "Microsoft YaHei"
        tf.paragraphs[0].space_before = Pt(0); tf.paragraphs[0].space_after = Pt(4)
    else:
        p = add_bullet(tf, item, font_size=13); p.space_after = Pt(4)

add_text_box(slide, MARGIN_L, Inches(4.4), CONTENT_W, Inches(0.35),
             "后续计划", font_size=16, bold=True, color=C_PRIMARY)
plan_items = [
    "① 完成DQN完整训练与评估，产出训练曲线和策略对比",
    "② 实现A*/RRT全局路径规划基线，与RL方法进行对比实验",
    "③ 在S0-S3四个场景下系统化评估，统计成功率/碰撞率等指标",
    "④ 优化奖励函数和网络结构，提升避障策略的鲁棒性",
    "⑤ 完善技术报告和实验文档，形成可复现的实验流程",
]
txBox2 = add_text_box(slide, MARGIN_L, Inches(4.8), CONTENT_W, Inches(2.2), "", font_size=13, color=C_DARK)
tf2 = txBox2.text_frame; tf2.word_wrap = True
for i, item in enumerate(plan_items):
    if i == 0:
        tf2.paragraphs[0].text = item; tf2.paragraphs[0].font.size = Pt(13)
        tf2.paragraphs[0].font.color.rgb = C_DARK; tf2.paragraphs[0].font.name = "Microsoft YaHei"
        tf2.paragraphs[0].space_before = Pt(0); tf2.paragraphs[0].space_after = Pt(4)
    else:
        p = add_bullet(tf2, item, font_size=13); p.space_after = Pt(4)
add_page_number(slide, 15)


# ═══════════════════════════════════════════════
# 第16页：Q&A
# ═══════════════════════════════════════════════
slide = add_blank_slide()
add_shape(slide, 0, 0, SLIDE_W, SLIDE_H, fill_color=C_PRIMARY)
add_shape(slide, 0, 0, SLIDE_W, Inches(0.1), fill_color=C_ACCENT)
add_text_box(slide, 0, Inches(2.5), SLIDE_W, Inches(1.0),
             "感谢各位老师聆听", font_size=40, bold=True, color=C_WHITE, alignment=PP_ALIGN.CENTER)
add_text_box(slide, 0, Inches(3.5), SLIDE_W, Inches(0.6),
             "敬请老师批评指正", font_size=24, color=RGBColor(0xA0, 0xC4, 0xE0), alignment=PP_ALIGN.CENTER)
add_shape(slide, Inches(5.5), Inches(4.3), Inches(2.3), Pt(2), fill_color=C_ACCENT)
add_text_box(slide, 0, Inches(4.7), SLIDE_W, Inches(0.4),
             "第 12 组 · 陈鑫豪 / 万前进 / 何俊峰 / 李英楷",
             font_size=16, color=RGBColor(0x85, 0xB0, 0xD0), alignment=PP_ALIGN.CENTER)
add_text_box(slide, 0, Inches(5.2), SLIDE_W, Inches(0.4),
             "指导老师：叶景贞",
             font_size=14, color=RGBColor(0x85, 0xB0, 0xD0), alignment=PP_ALIGN.CENTER)
add_shape(slide, 0, SLIDE_H - Inches(0.15), SLIDE_W, Inches(0.15), fill_color=C_ACCENT)
add_page_number(slide, 16)


# ── 保存 ──
output_dir = os.path.join(os.path.dirname(os.path.dirname(__file__)), "docs", "指导书")
os.makedirs(output_dir, exist_ok=True)
output_path = os.path.join(output_dir, "项目答辩PPT.pptx")
prs.save(output_path)
print(f"✅ PPT 已生成: {output_path}")
print(f"📄 共 {len(prs.slides)} 页")
print(f"📊 数据来源: 5次训练运行, {total_episodes}个episode, {best_st.get('total_steps',0):,}步训练数据")