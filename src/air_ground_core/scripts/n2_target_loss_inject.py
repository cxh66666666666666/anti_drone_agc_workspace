#!/usr/bin/env python3
"""n2_target_loss_inject.py —— N2 目标失联/恢复注入（离线工具，非实时链路）。

在任务进入运行期（GROUND_APPROACH）后，按**仿真时钟**关闭 /agc/target/enabled 一段窗口再恢复：
- 关闭期间 gt_adapter / air_fusion 发空目标 -> air_follow 观测过期 -> 冻结保持点（不追旧目标）；
- 恢复后目标观测重新新鲜 -> air_follow 恢复跟随。
用于验证「失联不追旧目标」与「恢复跟随」。实时链路节点仍为 C++，本工具只改参数。

用法：
    python3 n2_target_loss_inject.py --off-after 6 --window 5 [--record <dir>]
"""
import argparse
import json
import os
import time

import rospy
from air_ground_core.msg import AgcTaskState


def main():
    ap = argparse.ArgumentParser(description="N2 目标失联/恢复注入")
    ap.add_argument("--off-after", type=float, default=6.0, help="进入 GROUND_APPROACH 后 N 秒关闭目标")
    ap.add_argument("--window", type=float, default=5.0, help="关闭持续 M 秒后恢复")
    ap.add_argument("--record", default="", help="注入事件写入 <dir>/injections.jsonl")
    args = ap.parse_args()

    rospy.init_node("n2_target_loss_inject", anonymous=True)
    state = {"s": None}
    rospy.Subscriber("/agc/task/state", AgcTaskState, lambda m: state.__setitem__("s", m.state))

    def log(ev):
        ev["t_wall"] = time.time()
        if args.record:
            os.makedirs(args.record, exist_ok=True)
            with open(os.path.join(args.record, "injections.jsonl"), "a") as f:
                f.write(json.dumps(ev, ensure_ascii=False) + "\n")
        print(json.dumps(ev, ensure_ascii=False))

    rospy.loginfo("n2_target_loss_inject: 等待任务进入 GROUND_APPROACH ...")
    r = rospy.Rate(20)
    while not rospy.is_shutdown() and state["s"] != 3:
        r.sleep()
    rospy.set_param("/agc/target/enabled", True)

    t0 = rospy.Time.now().to_sec()
    while not rospy.is_shutdown() and rospy.Time.now().to_sec() - t0 < args.off_after:
        r.sleep()
    rospy.set_param("/agc/target/enabled", False)
    log({"type": "target_loss", "action": "off", "window_s": args.window,
         "t_sim": rospy.Time.now().to_sec()})

    t1 = rospy.Time.now().to_sec()
    while not rospy.is_shutdown() and rospy.Time.now().to_sec() - t1 < args.window:
        r.sleep()
    rospy.set_param("/agc/target/enabled", True)
    log({"type": "target_loss", "action": "on", "t_sim": rospy.Time.now().to_sec()})


if __name__ == "__main__":
    main()