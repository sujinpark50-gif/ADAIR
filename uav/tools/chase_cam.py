#!/usr/bin/env python3
"""Gazebo GUI 추적 카메라 — 드론 진행 방향 뒤쪽·위에서 드론을 바라보게 한다.

Gazebo 기본 follow 는 카메라 위치만 따라가고 방향을 드론 쪽으로 돌리지 않아 드론이 화면에서 빠진다.
이 스크립트는 /world/<world>/dynamic_pose/info 로 드론 위치를 받아 /gui/move_to/pose 로 카메라를 직접 놓는다.

PX4 컨테이너 안에서 실행 (gz Python 바인딩 필요):
  docker cp uav/tools/chase_cam.py px4:/tmp/ && docker exec -d px4 python3 /tmp/chase_cam.py
  중지: docker exec px4 pkill -f chase_cam.py
"""
import argparse
import math
import time

from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.gui_camera_pb2 import GUICamera
from gz.msgs10.pose_pb2 import Pose
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.stringmsg_pb2 import StringMsg
from gz.transport13 import Node

ap = argparse.ArgumentParser()
ap.add_argument("--world", default="kangwon")
ap.add_argument("--model", default="x500_0")
ap.add_argument("--back", type=float, default=10.0, help="드론 뒤쪽 거리 (m)")
ap.add_argument("--up", type=float, default=4.0, help="드론 위쪽 높이 (m)")
ap.add_argument("--hz", type=float, default=10.0)
ap.add_argument("--lead", type=float, default=0.9,
                help="GUI 카메라 지연 보정 초기값 (s). 실제 카메라 위치를 보고 자동 조정된다")
a = ap.parse_args()

node = Node()
state = {"pos": None, "cam": None}


def on_pose(msg):
    for p in msg.pose:
        if p.name == a.model:
            state["pos"] = (p.position.x, p.position.y, p.position.z)
            return


def on_cam(msg):
    state["cam"] = (msg.position.x, msg.position.y, msg.position.z)


node.subscribe(Pose_V, f"/world/{a.world}/dynamic_pose/info", on_pose)
node.subscribe(Pose, "/gui/camera/pose", on_cam)
off = StringMsg(); off.data = ""
node.request("/gui/follow", off, StringMsg, Boolean, 2000)   # 기본 follow 끄기

heading = None   # 진행 방향 단위벡터 (x, y). 정지 중에는 마지막 값 유지
prev = None
prev_t = None
vel = (0.0, 0.0, 0.0)   # 실제 시간 기준 속도 (m/s, 시뮬 배속 포함)
while True:
    time.sleep(1.0 / a.hz)
    pos = state["pos"]
    if pos is None:
        continue
    now = time.monotonic()
    if prev is not None and now > prev_t:
        dt = now - prev_t
        v = tuple((pos[k] - prev[k]) / dt for k in range(3))
        vel = tuple(vel[k] * 0.7 + v[k] * 0.3 for k in range(3))
    prev_t = now
    if prev is not None:
        dx, dy = pos[0] - prev[0], pos[1] - prev[1]
        d = math.hypot(dx, dy)
        if d > 0.3:   # 움직일 때만 방향 갱신 (저역통과로 흔들림 완화)
            nx, ny = dx / d, dy / d
            if heading is None:
                heading = (nx, ny)
            else:
                hx, hy = heading[0] * 0.8 + nx * 0.2, heading[1] * 0.8 + ny * 0.2
                n = math.hypot(hx, hy) or 1.0
                heading = (hx / n, hy / n)
    prev = pos
    hx, hy = heading or (0.0, 1.0)
    # 지연 보정 자동 조정: 실제 카메라가 원하는 자리(드론 뒤 back m)보다 앞서 있으면 lead 를 줄이고,
    # 뒤처져 있으면 늘린다. 지연은 속도·렌더링 부하에 따라 달라 고정값으로는 맞지 않는다.
    speed = math.hypot(vel[0], vel[1])
    if state["cam"] is not None and speed > 1.0:
        want = (pos[0] - hx * a.back, pos[1] - hy * a.back)
        ahead = (state["cam"][0] - want[0]) * hx + (state["cam"][1] - want[1]) * hy
        a.lead = min(2.0, max(0.0, a.lead - 0.15 * ahead / speed))
    # 애니메이션 지연 동안 드론이 갈 거리만큼 앞당겨 놓는다
    pos = tuple(pos[k] + vel[k] * a.lead for k in range(3))
    cx, cy, cz = pos[0] - hx * a.back, pos[1] - hy * a.back, pos[2] + a.up
    vx, vy, vz = pos[0] - cx, pos[1] - cy, pos[2] - cz
    yaw = math.atan2(vy, vx)
    pitch = math.atan2(-vz, math.hypot(vx, vy))   # 아래를 보면 +
    cy_, sy_ = math.cos(yaw / 2), math.sin(yaw / 2)
    cp_, sp_ = math.cos(pitch / 2), math.sin(pitch / 2)
    req = GUICamera()
    req.pose.position.x, req.pose.position.y, req.pose.position.z = cx, cy, cz
    req.pose.orientation.w = cp_ * cy_
    req.pose.orientation.x = -sp_ * sy_
    req.pose.orientation.y = sp_ * cy_
    req.pose.orientation.z = cp_ * sy_
    node.request("/gui/move_to/pose", req, GUICamera, Boolean, 500)
