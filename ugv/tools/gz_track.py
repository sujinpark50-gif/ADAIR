# -*- coding: utf-8 -*-
"""ugv/tools/gz_track.py — Gazebo 안의 차량 자세·조향·바퀴 속도를 시뮬레이션 시각과 함께 CSV 로 기록한다.

속도 측정의 근거 자료다. PX4 텔레메트리나 배속 설정값이 아니라 Gazebo 물리 엔진이 발행한 위치와
메시지 머리말의 시뮬레이션 시각(header.stamp)만 쓴다. WSL(Gazebo 와 같은 컴퓨터)에서 실행한다.

    GZ_PARTITION=adair_ugv GZ_IP=127.0.0.1 python3 ugv/tools/gz_track.py --world adair_flat \
        --model adair_ugv_1 --out /mnt/c/.../logs/ugv/track.csv --duration 120

필요: python3 gz.transport13, gz.msgs10 (gz-harmonic 설치 시 함께 설치됨)
"""

import argparse
import csv
import math
import signal
import threading
import time

from gz.msgs10.model_pb2 import Model
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node


def yaw_roll_pitch(q):
    w, x, y, z = q.w, q.x, q.y, q.z
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return yaw, roll, pitch


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--world", default="adair_flat")
    ap.add_argument("--model", default="adair_ugv_1")
    ap.add_argument("--out", required=True)
    ap.add_argument("--duration", type=float, default=0.0, help="벽시계 초. 0 이면 Ctrl+C 까지")
    ap.add_argument("--hz", type=float, default=20.0, help="기록 간격 (시뮬레이션 시각 기준)")
    a = ap.parse_args()

    lock = threading.Lock()
    joints = {"steer": float("nan"), "wheel": float("nan")}
    last_t, count = [-1.0], [0]
    # 받는 즉시 쓴다 (도중에 종료돼도 그때까지의 기록이 남는다)
    f = open(a.out, "w", newline="")
    writer = csv.writer(f)
    writer.writerow(["sim_t", "x", "y", "z", "yaw", "roll", "pitch", "steer_rad", "wheel_rad_s", "wall_t"])
    step = 1.0 / a.hz

    def on_joints(msg: Model):
        st, wh = [], []
        for j in msg.joint:
            if j.name.endswith("_steering_joint"):
                st.append(j.axis1.position)
            elif j.name.startswith("wheel_rear"):
                wh.append(j.axis1.velocity)
        with lock:
            if st:
                joints["steer"] = sum(st) / len(st)
            if wh:
                joints["wheel"] = sum(wh) / len(wh)

    def on_pose(msg: Pose_V):
        t = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9
        if t - last_t[0] < step - 1e-9:
            return
        for p in msg.pose:
            if p.name == a.model:
                yaw, roll, pitch = yaw_roll_pitch(p.orientation)
                with lock:
                    writer.writerow([round(t, 4), round(p.position.x, 4), round(p.position.y, 4), round(p.position.z, 4),
                                     round(yaw, 5), round(roll, 5), round(pitch, 5),
                                     round(joints["steer"], 5), round(joints["wheel"], 4), round(time.time(), 3)])
                    count[0] += 1
                last_t[0] = t
                break

    node = Node()
    if not node.subscribe(Pose_V, f"/world/{a.world}/dynamic_pose/info", on_pose):
        raise SystemExit("pose 구독 실패")
    node.subscribe(Model, f"/world/{a.world}/model/{a.model}/joint_state", on_joints)

    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    t0 = time.time()
    try:
        while not a.duration or time.time() - t0 < a.duration:
            time.sleep(0.5)
            with lock:
                f.flush()
    except KeyboardInterrupt:
        pass
    with lock:
        f.close()
    print(f"{count[0]} rows -> {a.out}")


if __name__ == "__main__":
    main()
