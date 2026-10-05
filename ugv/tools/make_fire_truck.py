# ugv/tools/make_fire_truck.py — 소방차 Gazebo 모델 만들기 (px4-start.sh 가 띄울 때 부른다)
#
#   python3 ugv/tools/make_fire_truck.py <기반 model.sdf> <기반 모델 이름> <자원 id> <출력 폴더>
#
# PX4 rover 모델(model.sdf)을 그대로 복사하고 이름만 fire_truck 으로 바꾼 뒤, 마지막 </model> 앞에
# 경광등·방수포 링크(ugv/gazebo/fire_truck/fx_parts.sdf.in)를 넣는다.
#
# 왜 <include merge="true"> 로 감싸지 않나 (2026-10-03 WSL): 감싸면 바퀴 플러그인 일부가 /fire_truck_3/command/motor_speed
# 처럼 /model/ 없는 토픽을 구독해 PX4 명령(/model/fire_truck_3/command/motor_speed)을 못 받았다 → 한쪽 바퀴만 돌아
# 경로에서 100 m 넘게 벗어났다(OFF_ROUTE). 복사본은 기반 모델과 토픽 규칙이 같다.
# 기반 모델 안의 상대 경로(meshes/… 등)는 model://<기반>/… 로 바꾼다 — 복사본이 다른 폴더에 있어서.

import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
FX_DIR = os.path.normpath(os.path.join(HERE, "..", "gazebo", "fire_truck"))
ASSET_TAGS = ("uri", "albedo_map", "normal_map", "roughness_map", "metalness_map", "emissive_map")


def make(base_sdf: str, base_name: str, rid: str) -> tuple[str, str]:
    """(model.sdf 내용, 차체 링크 이름)."""
    sdf = open(base_sdf, encoding="utf-8").read()
    m = re.search(r"<model\s+name\s*=\s*(['\"])([^'\"]+)\1", sdf)
    if not m:
        raise SystemExit(f"{base_sdf}: <model name=…> 없음")
    sdf = sdf[:m.start()] + '<model name="fire_truck"' + sdf[m.end():]
    link = re.search(r"<link\s+name\s*=\s*(['\"])([^'\"]+)\1", sdf)
    base_link = link.group(2) if link else "base_link"

    def absolutize(mt):
        tag, val = mt.group(1), mt.group(2).strip()
        if "://" in val or val.startswith("/") or not val:
            return mt.group(0)
        return f"<{tag}>model://{base_name}/{val}</{tag}>"
    sdf = re.sub(r"<(%s)>([^<]*)</\1>" % "|".join(ASSET_TAGS), absolutize, sdf)

    # 바퀴 진행 방향 마찰(mu, fdir1 방향)만 올린다. 바퀴 반지름이 6.9 cm 라 mu 1 이면 2 cm 넘는 턱도 못 넘는다
    # (넘으려면 mu ≥ tan θ, cos θ = (r - 턱)/r). 도로 면이 겹치는 교차로에 4~5 cm 턱이 있어 평지에서 멈췄다
    # (FIRE8, 2026-10-05, 웨이포인트 26). mu 4 면 약 5 cm 까지. 옆 방향(mu2)은 그대로 — 제자리 회전이 미끄러져야 한다.
    sdf = re.sub(r"<mu>[^<]*</mu>", f"<mu>{os.getenv('FIRE_TRUCK_WHEEL_MU', '4.0')}</mu>", sdf)

    parts = open(os.path.join(FX_DIR, "fx_parts.sdf.in"), encoding="utf-8").read()
    parts = parts.replace("@BASE_LINK@", base_link).replace("@RID@", rid).replace("@FX@", FX_DIR)
    i = sdf.rindex("</model>")
    return sdf[:i] + parts + "  " + sdf[i:], base_link


if __name__ == "__main__":
    base_sdf, base_name, rid, out_dir = sys.argv[1:5]
    text, base_link = make(base_sdf, base_name, rid)
    os.makedirs(out_dir, exist_ok=True)
    open(os.path.join(out_dir, "model.sdf"), "w", encoding="utf-8").write(text)
    open(os.path.join(out_dir, "model.config"), "w", encoding="utf-8").write(
        '<?xml version="1.0"?>\n<model><name>fire_truck</name><version>1.0</version>'
        '<sdf version="1.9">model.sdf</sdf></model>\n')
    print(base_link)
