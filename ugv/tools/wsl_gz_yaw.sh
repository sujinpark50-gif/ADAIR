#!/usr/bin/env bash
# Gazebo 안 모델의 실제 방위(북=0, 시계, 도): bash wsl_gz_yaw.sh <world> <model>
export GZ_PARTITION=adair_ugv GZ_IP=127.0.0.1
timeout 5 gz topic -e -n 1 -t "/world/$1/dynamic_pose/info" | grep -A14 "name: \"$2\"" | python3 -c "
import sys, math, re
t = sys.stdin.read()
o = t[t.find('orientation'):]
v = {k: float(re.search(k + r': ([-0-9.e]+)', o).group(1)) if re.search(k + r': ([-0-9.e]+)', o) else 0.0 for k in ('x', 'y', 'z', 'w')}
yaw = math.atan2(2 * (v['w'] * v['z'] + v['x'] * v['y']), 1 - 2 * (v['y'] ** 2 + v['z'] ** 2))
print(round((90 - math.degrees(yaw)) % 360, 1))"
