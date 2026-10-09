#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""N5 LQR 垂直参考：运行时向 /rl_planner/local_offset 注入竖直(+z)偏移。

lqr_tracker_node 订阅 /rl_planner/local_offset(geometry_msgs/Vector3Stamped)，
把 ref_point.position += offset（3D，含 z）。本注入器以固定频率持续发布，
保证节点不会因 >0.5s 无消息而清零；--duration 结束后停止发布，节点随即
触发 0.5s 超时清零，用于验证恢复行为。

用法:
  python3 scripts/n5_lqr_vertical_inject.py --z 0.8 --rate 20 --duration 15 \
      --record experiments/n5/agc/lqr_vertical/inject.jsonl --tag phase_inject
"""

import argparse
import json
import time
from datetime import datetime


def main():
    parser = argparse.ArgumentParser(description="N5 垂直偏移注入器")
    parser.add_argument("--z", type=float, default=0.8, help="竖直偏移(米)")
    parser.add_argument("--rate", type=float, default=20.0, help="发布频率(Hz)")
    parser.add_argument("--duration", type=float, default=15.0, help="注入时长(秒)")
    parser.add_argument("--record", default="", help="注入事件 JSONL 路径")
    parser.add_argument("--tag", default="", help="本次注入标记")
    args = parser.parse_args()

    import rospy
    from geometry_msgs.msg import Vector3Stamped

    rospy.init_node("n5_lqr_vertical_injector", anonymous=True, disable_signals=True)
    pub = rospy.Publisher("/rl_planner/local_offset", Vector3Stamped, queue_size=10)
    rate = rospy.Rate(args.rate)

    def emit(event, extra=None):
        """把一次注入相位事件追加到 JSONL（含墙钟时间）。"""
        if not args.record:
            return
        rec = {"wall_time": time.time(), "iso": datetime.now().isoformat(),
               "event": event, "tag": args.tag, "z": args.z, "rate": args.rate}
        if extra:
            rec.update(extra)
        with open(args.record, "a", encoding="utf-8") as stream:
            stream.write(json.dumps(rec, ensure_ascii=False) + "\n")

    # 等待订阅者上线，确保注入立即生效（lqr_tracker_node 已在运行）
    deadline = time.time() + 5.0
    while pub.get_num_connections() == 0 and time.time() < deadline and not rospy.is_shutdown():
        rate.sleep()

    msg = Vector3Stamped()
    msg.header.frame_id = "map"
    msg.vector.x = 0.0
    msg.vector.y = 0.0
    msg.vector.z = args.z
    emit("inject_start", {"connections": pub.get_num_connections()})

    published = 0
    end = time.time() + args.duration
    try:
        while time.time() < end and not rospy.is_shutdown():
            msg.header.stamp = rospy.Time.now()
            pub.publish(msg)
            published += 1
            rate.sleep()
    finally:
        emit("inject_end", {"published": published})
        print("[N5-inject] 注入结束 tag=%s z=%.3f 帧数=%d"
              % (args.tag, args.z, published), flush=True)


if __name__ == "__main__":
    main()