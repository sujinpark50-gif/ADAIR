# WSL: PX4 ulog 에서 추정 위치(vehicle_global_position)·실제 위치(_groundtruth)·조향 목표를 CSV 로: python3 wsl_ulog_gpos.py <ulg> <out.csv>
import csv, sys
from pyulog import ULog
u = ULog(sys.argv[1], ["vehicle_global_position", "vehicle_global_position_groundtruth", "vehicle_attitude",
                       "vehicle_attitude_groundtruth"])
d = {(m.name, m.multi_id): m.data for m in u.data_list}
est, gt = d.get(("vehicle_global_position", 0)), d.get(("vehicle_global_position_groundtruth", 0))
with open(sys.argv[2], "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["t_us", "src", "lat", "lon"])
    for src, m in (("est", est), ("gt", gt)):
        if m is None:
            continue
        for i in range(0, len(m["timestamp"]), 5):
            w.writerow([int(m["timestamp"][i]), src, repr(float(m["lat"][i])), repr(float(m["lon"][i]))])
print("ok", est is not None, gt is not None)
