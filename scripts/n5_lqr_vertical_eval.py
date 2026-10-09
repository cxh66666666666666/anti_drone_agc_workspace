#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""N5 垂直参考验证：运行时注入竖直偏移，验证 C++ 节点参考 z 与无人机跟随。

复用 m7_lqr_tracking_eval.py 的启动/环境/清理模式：以固定水平点轨迹
（fixed_horizontal）启动 `scripts/start_sim.sh lqr`，C++ `lqr_tracker_node`
订阅 `/rl_planner/local_offset` 并把 ref_point.position += offset（含 z）。

记录话题（已核对 lqr_tracker_node.cpp）:
  - /rl_planner/local_offset      geometry_msgs/Vector3Stamped  (注入)
  - /lqr_tracker/adjusted_ref     geometry_msgs/PoseStamped     (叠加偏移后的参考)
  - /lqr_tracker/ref_trajectory   nav_msgs/Path                 (原始轨迹, 不含偏移)
  - /mavros/local_position/pose   geometry_msgs/PoseStamped     (无人机实际位姿)
  - /lqr_tracker/position_error   std_msgs/Float64              (跟踪误差)

注意: /lqr_tracker/ref_trajectory 是原始轨迹（不含偏移），叠加 RL 偏移后的
参考发布在 /lqr_tracker/adjusted_ref，故"参考 z 偏移"以 adjusted_ref 为准。

用法:
  python3 scripts/n5_lqr_vertical_eval.py --headless --out-dir experiments/n5/agc/lqr_vertical
"""

import argparse
import json
import os
import signal
import subprocess
import time
from datetime import datetime

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
START_SIM = os.path.join(WS, "scripts/start_sim.sh")
INJECTOR = os.path.join(WS, "scripts/n5_lqr_vertical_inject.py")

RAW_REF_TOPIC = "/lqr_tracker/ref_trajectory"      # nav_msgs/Path（原始, 不含偏移）
ADJ_REF_TOPIC = "/lqr_tracker/adjusted_ref"        # geometry_msgs/PoseStamped（含偏移）
DRONE_TOPIC = "/mavros/local_position/pose"        # geometry_msgs/PoseStamped
ERR_TOPIC = "/lqr_tracker/position_error"          # std_msgs/Float64
OFFSET_TOPIC = "/rl_planner/local_offset"          # geometry_msgs/Vector3Stamped

# 采样器：订阅三个话题，把每条消息(墙钟时间 + 值)写入 JSONL
SAMPLER = r'''
import json, time
import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64

OUT = open(%(out)r, "w")
def rec(kind, v):
    OUT.write(json.dumps({"t": time.time(), "kind": kind, "v": float(v)}) + "\n")
    OUT.flush()

rospy.init_node("n5_vert_sampler", anonymous=True, disable_signals=True)
rospy.Subscriber("%(drone)s", PoseStamped, lambda m: rec("drone_z", m.pose.position.z))
rospy.Subscriber("%(adj)s", PoseStamped, lambda m: rec("ref_z", m.pose.position.z))
rospy.Subscriber("%(err)s", Float64, lambda m: rec("pos_err", m.data))
rospy.spin()
'''


def _env():
    env = dict(os.environ)
    env["ROS_LOG_DIR"] = "/tmp/ros_logs"
    return env


def _bash(cmd, timeout=None):
    return subprocess.run(["bash", "-c", cmd], cwd=WS, env=_env(),
                          capture_output=True, text=True, timeout=timeout)


def _source():
    return "source /opt/ros/noetic/setup.bash && source devel/setup.bash"


def wait_ready(topics, node="/lqr_tracker_node", max_wait=240):
    """等待 lqr_tracker_node 与所需话题均就绪（避免过早采样）。"""
    deadline = time.time() + max_wait
    while time.time() < deadline:
        nodes = _bash("%s && rosnode list" % _source()).stdout
        listed = _bash("%s && rostopic list" % _source()).stdout
        if node in nodes and all(t in listed for t in topics):
            return True
        time.sleep(3)
    return False


def _decimate(values, cap=400):
    """下采样序列，限制落盘体积但保留波形。"""
    if len(values) <= cap:
        return values
    step = len(values) / float(cap)
    return [values[min(len(values) - 1, int(i * step))] for i in range(cap)]


def stop_sim():
    """与 m7 一致的兜底清理（bracket 模式避免误杀自身）。"""
    _bash("pkill -9 -f '[l]qr_tracker_node'; "
          "pkill -9 -f '[d]rone_launch_controller'; pkill -9 -f '[g]zclient'; "
          "pkill -9 -f '[g]zserver'; pkill -9 -f '[p]x4'; "
          "pkill -9 -f '[m]avros'; pkill -9 -f '[r]osmaster'; "
          "pkill -9 -f '[r]osout'")
    time.sleep(6)


def _slice(samples, kind, t0, t1):
    """按墙钟窗口取某类采样值。"""
    return [s["v"] for s in samples if s["kind"] == kind and t0 <= s["t"] <= t1]


def _mean(values):
    return sum(values) / len(values) if values else None


def _phase_summary(name, t0, t1, samples):
    ref = _slice(samples, "ref_z", t0, t1)
    drone = _slice(samples, "drone_z", t0, t1)
    err = _slice(samples, "pos_err", t0, t1)
    return {
        "name": name, "t_start": t0, "t_end": t1, "duration_s": t1 - t0,
        "ref_z_mean": _mean(ref), "drone_z_mean": _mean(drone),
        "pos_err_mean": _mean(err),
        "n_ref": len(ref), "n_drone": len(drone), "n_err": len(err),
        # 下采样后的原始采样序列（用于复核波形）
        "ref_z_samples": _decimate(ref),
        "drone_z_samples": _decimate(drone),
        "pos_err_samples": _decimate(err),
    }


def main():
    parser = argparse.ArgumentParser(description="N5 LQR 垂直偏移参考验证")
    parser.add_argument("--gui", action="store_true")
    parser.add_argument("--headless", action="store_true", help="无头(默认)")
    parser.add_argument("--out-dir", default="experiments/n5/agc/lqr_vertical")
    parser.add_argument("--z", type=float, default=0.8, help="注入竖直偏移(米)")
    parser.add_argument("--altitude", type=float, default=3.0)
    args = parser.parse_args()

    out_dir = args.out_dir if os.path.isabs(args.out_dir) else os.path.join(WS, args.out_dir)
    os.makedirs(out_dir, exist_ok=True)
    gui = bool(args.gui) and not args.headless

    base_s, inject_s, recover_s, settle_s = 15.0, 15.0, 15.0, 5.0
    total = base_s + inject_s + recover_s
    samples_path = os.path.join(out_dir, "samples.jsonl")
    inject_record = os.path.join(out_dir, "inject.jsonl")
    if os.path.exists(samples_path):
        os.remove(samples_path)
    if os.path.exists(inject_record):
        os.remove(inject_record)

    report = {
        "created_at": datetime.now().isoformat(),
        "gui": gui, "altitude_m": args.altitude, "z_offset_m": args.z,
        "timing_s": {"baseline": base_s, "inject": inject_s, "recover": recover_s},
        "topics": {
            "offset_cmd": OFFSET_TOPIC + " (geometry_msgs/Vector3Stamped)",
            "adjusted_ref": ADJ_REF_TOPIC + " (geometry_msgs/PoseStamped, 含偏移)",
            "raw_ref_trajectory": RAW_REF_TOPIC + " (nav_msgs/Path, 原始不含偏移)",
            "drone_pose": DRONE_TOPIC + " (geometry_msgs/PoseStamped)",
            "position_error": ERR_TOPIC + " (std_msgs/Float64)",
        },
    }

    print("[N5] 启动仿真 fixed_horizontal altitude=%.1f ..." % args.altitude, flush=True)
    sim = subprocess.Popen(
        ["bash", START_SIM, "lqr", "--altitude", "%.2f" % args.altitude,
         "--trajectory", "fixed_horizontal",
         "--trajectory-duration", "40",
         "--gui" if gui else "--headless"],
        cwd=WS, env=_env(),
        stdout=open(os.path.join(out_dir, "sim.log"), "w"),
        stderr=subprocess.STDOUT, start_new_session=True)

    sampler = None
    injector = None
    try:
        if not wait_ready([ADJ_REF_TOPIC, DRONE_TOPIC, ERR_TOPIC]):
            report["error"] = "topics_not_ready"
        else:
            time.sleep(6)  # 越过起飞/LQR 起步瞬态，进入稳态

            code = SAMPLER % {"out": samples_path, "drone": DRONE_TOPIC,
                              "adj": ADJ_REF_TOPIC, "err": ERR_TOPIC}
            sampler_cmd = "%s && timeout %d python3 - <<'PY'\n%s\nPY" % (
                _source(), int(total + 10), code)
            sampler = subprocess.Popen(["bash", "-c", sampler_cmd], cwd=WS,
                                       env=_env(), stdout=subprocess.DEVNULL,
                                       stderr=subprocess.DEVNULL,
                                       start_new_session=True)

            time.sleep(3)  # 等采样器订阅就绪
            t_base = time.time()
            time.sleep(base_s)
            t_inject = time.time()

            # 后台注入竖直偏移（持续到自身 duration 结束）
            inject_cmd = ("%s && python3 %s --z %.4f --rate 20 --duration %.1f "
                          "--record %s --tag phase_inject" % (
                              _source(), INJECTOR, args.z, inject_s, inject_record))
            injector = subprocess.Popen(["bash", "-c", inject_cmd], cwd=WS,
                                        env=_env(), start_new_session=True,
                                        stdout=subprocess.DEVNULL,
                                        stderr=subprocess.DEVNULL)
            time.sleep(inject_s + 1.0)  # 等注入器自行停止(+0.5s超时清零)
            t_clear = time.time()
            time.sleep(recover_s)
            t_end = time.time()

        # 收敛采样器
        if sampler is not None:
            try:
                os.killpg(os.getpgid(sampler.pid), signal.SIGINT)
            except ProcessLookupError:
                pass
            sampler.wait(timeout=10)
    finally:
        if injector is not None and injector.poll() is None:
            try:
                os.killpg(os.getpgid(injector.pid), signal.SIGINT)
            except ProcessLookupError:
                pass
        try:
            os.killpg(os.getpgid(sim.pid), signal.SIGINT)
        except ProcessLookupError:
            pass
        stop_sim()

    if report.get("error"):
        _write(report, out_dir)
        print("[N5] 失败: %s" % report["error"])
        return

    # 解析采样并分相位统计
    samples = []
    if os.path.exists(samples_path):
        with open(samples_path, encoding="utf-8") as stream:
            for line in stream:
                line = line.strip()
                if line:
                    try:
                        samples.append(json.loads(line))
                    except json.JSONDecodeError:
                        pass

    baseline = _phase_summary("baseline", t_base, t_inject, samples)
    inject = _phase_summary("inject", t_inject + settle_s, t_clear - 1.0, samples)
    recover = _phase_summary("recover", t_clear + 3.0, t_end, samples)
    report["phases"] = [baseline, inject, recover]

    # 关键指标：偏移前后参考 z 与无人机 z 的变化量
    def _d(a, b):
        return None if (a is None or b is None) else a - b

    ref_shift = _d(inject["ref_z_mean"], baseline["ref_z_mean"])
    drone_shift = _d(inject["drone_z_mean"], baseline["drone_z_mean"])
    ref_recover_err = _d(recover["ref_z_mean"], baseline["ref_z_mean"])
    drone_recover_err = _d(recover["drone_z_mean"], baseline["drone_z_mean"])

    tol_ref, tol_drone, tol_rec = 0.20, 0.40, 0.30
    checks = {
        # 参考 z 变化 ≈ 注入量
        "ref_z_shift_m": ref_shift,
        "ref_z_shift_error_m": _d(ref_shift, args.z),
        "ref_z_changed_by_offset": (ref_shift is not None
                                    and abs(ref_shift - args.z) <= tol_ref),
        # 无人机实际 z 跟随该偏移
        "drone_z_shift_m": drone_shift,
        "drone_z_shift_error_m": _d(drone_shift, args.z),
        "drone_followed_offset": (drone_shift is not None
                                  and abs(drone_shift - args.z) <= tol_drone),
        # 清除后回归基线（节点 0.5s 超时清零）
        "recover_ref_z_err_m": ref_recover_err,
        "recover_drone_z_err_m": drone_recover_err,
        "recovered_after_clear": (ref_recover_err is not None and drone_recover_err is not None
                                  and abs(ref_recover_err) <= tol_rec
                                  and abs(drone_recover_err) <= tol_rec),
        "timeout_zeroing_observed": _timeout_observed(inject_record, t_clear),
    }
    report["metrics"] = {
        "baseline_ref_z_mean": baseline["ref_z_mean"],
        "baseline_drone_z_mean": baseline["drone_z_mean"],
        "inject_ref_z_mean": inject["ref_z_mean"],
        "inject_drone_z_mean": inject["drone_z_mean"],
        "recover_ref_z_mean": recover["ref_z_mean"],
        "recover_drone_z_mean": recover["drone_z_mean"],
    }
    report["checks"] = checks
    report["pass"] = bool(checks["ref_z_changed_by_offset"]
                          and checks["drone_followed_offset"]
                          and checks["recovered_after_clear"])

    _write(report, out_dir)
    print("[N5] ref_shift=%.3f(期望 %.3f) drone_shift=%s recovered=%s pass=%s"
          % (ref_shift if ref_shift is not None else float("nan"), args.z,
             "%.3f" % drone_shift if drone_shift is not None else "NA",
             checks["recovered_after_clear"], report["pass"]), flush=True)


def _timeout_observed(inject_record, t_clear):
    """注入结束事件后没有继续发布 => 节点须靠 0.5s 超时清零。"""
    if not os.path.exists(inject_record):
        return False
    events = []
    with open(inject_record, encoding="utf-8") as stream:
        for line in stream:
            try:
                events.append(json.loads(line))
            except json.JSONDecodeError:
                pass
    if not events or events[-1].get("event") != "inject_end":
        return False
    # 结束事件发生在清除边界附近（前后 2s）即视为观察到停止发布
    return abs(events[-1].get("wall_time", 0.0) - t_clear) <= 3.0


def _write(report, out_dir):
    path = os.path.join(out_dir, "lqr_vertical_report.json")
    with open(path, "w", encoding="utf-8") as stream:
        json.dump(report, stream, ensure_ascii=False, indent=2)
        stream.write("\n")
    print("[N5] 报告: %s" % path)


if __name__ == "__main__":
    main()