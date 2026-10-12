# -*- coding: utf-8 -*-
"""ugv/tools/make_vehicle_variant.py — 차량 모델 후보(변형) 생성기. 정식 모델 ugv/gazebo/models/<모델>/ 은 바꾸지 않는다.

결과: ugv/gazebo/variants/<변형>/<모델>/{model.sdf, model.config} + ugv/gazebo/variants/<변형>/MANIFEST.json
선택: px4-start.sh 에 UGV_MODEL_VARIANT=<변형> (또는 GZ_MODEL_OVERRIDE=<변형 폴더>) — 그 모델 이름이 변형 폴더에서 먼저 해석된다.

변형 (한 번에 한 가지만 바꾼다, 2026-10-12 차량 개조 검증 지시서 4~5절):
  base     원본 + 바퀴 4개 접촉 센서 + Contact 시스템 (물리 불변, 계측용)
  rwd      base − 앞바퀴 두 개의 속도 구동(JointController) → 앞바퀴 자유 회전 (조건부 비교 후보)
  susp_a0  base + 바퀴마다 차체↔바퀴(앞은 조향 링크) 수직 prismatic 스프링·감쇠 (K 130 kN/m, C 8 kN·s/m, ref −0.03 m, ±0.10 m, 링크 5 kg)
           = 시험 chain56 이 쓴 logs/ugv/stage5/jturn/fable_models/susp 와 내용 동일 (결과 재사용 조건)
  susp_a   susp_a0 + JointStatePublisher 에 서스펜션 관절 4개 추가 (물리 동일, 관절 위치 계측용)
  (소방차 adair_firetruck: base·susp_a 만. susp_a = 축별 K·C — 앞 711 kN/m·43 kN·s/m, 뒤 977 kN/m·59.1 kN·s/m, ref −0.028, ±0.10 m, 링크 10 kg.
   UGV 값 복사가 아니라 소방차 질량(앞 2,000·뒤 2,750 kg/바퀴 스프렁)에서 다시 계산한 계획 8.1 F-a. UGV 실측 유효 강성 비 0.64 를 가정하면
   유효 동특성이 UGV susp_a 실측(2.37 Hz, 감쇠비 0.72)과 비슷 — 소방차에서 다시 실측해야 함)
  susp_a_m susp_a 의 질량 보정판 — 차체 1500 → 1480 kg (관성 같은 비율)로 총질량 1,604 kg 를 원본과 맞춤. 원인 시험 전용 (지시서 0절)
해시는 줄바꿈 정규화 내용 해시 (fable_models 사본은 CRLF 로 저장돼 있어 바이트는 다르고 내용은 같다).
원본 생성: fable_make_models.py·fable_make_susp.py (2026-10-12, logs 아래) 를 옮긴 것. --check 는 재생성 결과와 저장본 sha1 비교.

사용: python -m ugv.tools.make_vehicle_variant [--variant base,rwd,susp_a0,susp_a] [--check]
"""
import argparse
import hashlib
import json
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
MODELS = ROOT / "ugv" / "gazebo" / "models"
VARIANTS = ROOT / "ugv" / "gazebo" / "variants"
WHEELS = ["wheel_rear_left_link", "wheel_rear_right_link", "wheel_front_left_link", "wheel_front_right_link"]
SUSP_POS = {"rear_left": (-1.35, 0.8), "rear_right": (-1.35, -0.8), "front_left": (1.35, 0.8), "front_right": (1.35, -0.8)}
SUSP_A = {"k": 130000.0, "c": 8000.0, "ref": -0.03, "travel": 0.10, "link_mass": 5.0}
# 차종별 susp_a: 바퀴 위치(SDF 실측)와 바퀴별 (K, C, ref)
MODEL_SUSP = {
    "adair_ugv": {"pos": SUSP_POS, "wheel": {w: (SUSP_A["k"], SUSP_A["c"], SUSP_A["ref"]) for w in SUSP_POS},
                  "travel": SUSP_A["travel"], "link_mass": SUSP_A["link_mass"]},
    "adair_firetruck": {"pos": {"rear_left": (-1.6, 1.0), "rear_right": (-1.6, -1.0), "front_left": (2.2, 1.0), "front_right": (2.2, -1.0)},
                        "wheel": {"rear_left": (977000.0, 59100.0, -0.028), "rear_right": (977000.0, 59100.0, -0.028),
                                  "front_left": (711000.0, 43000.0, -0.028), "front_right": (711000.0, 43000.0, -0.028)},
                        "travel": 0.10, "link_mass": 10.0},
}
MODEL_VARIANTS = {"adair_ugv": ["base", "rwd", "susp_a0", "susp_a", "susp_a_m"], "adair_firetruck": ["base", "susp_a"]}


def sha1(b: bytes) -> str:
    """내용 해시: 줄바꿈 정규화(CR 제거) 뒤 sha1. 저장소가 core.autocrlf 라 PC 마다 CRLF/LF 가 달라도 같은 값"""
    return hashlib.sha1(b.replace(b"\r", b"")).hexdigest()


def add_contact(s: str) -> str:
    for w in WHEELS:
        i = s.index(f'<link name="{w}">')
        j = s.index('</link>', i)
        blk = s[i:j]
        sensor = (f'\n      <sensor name="{w}_contact" type="contact"><always_on>1</always_on><update_rate>100</update_rate>'
                  f'<contact><collision>collision</collision></contact></sensor>\n    ')
        s = s[:i] + blk.replace('</collision>', '</collision>' + sensor, 1) + s[j:]
    return s.replace('  </model>', '    <plugin filename="gz-sim-contact-system" name="gz::sim::systems::Contact"/>\n  </model>')


def remove_front_drive(s: str) -> str:
    pat = re.compile(r'    <plugin filename="gz-sim-joint-controller-system" name="gz::sim::systems::JointController">\s*'
                     r'<joint_name>wheel_front_(left|right)_joint</joint_name>.*?</plugin>\n', re.S)
    s, n = pat.subn('', s)
    assert n == 2, f"앞바퀴 JointController {n}개 (2개여야 함)"
    return s


def add_suspension(s: str, cfg: dict) -> str:
    links = joints = ""
    travel, link_mass = cfg["travel"], cfg["link_mass"]
    for w, (px, py) in cfg["pos"].items():
        k, c, ref = cfg["wheel"][w]
        links += f'''
    <link name="susp_{w}_link">
      <pose relative_to="base_link">{px} {py} 0 0 0 0</pose>
      <inertial><mass>{link_mass}</mass><inertia><ixx>0.05</ixx><ixy>0</ixy><ixz>0</ixz><iyy>0.05</iyy><iyz>0</iyz><izz>0.05</izz></inertia></inertial>
    </link>'''
        joints += f'''
    <joint name="susp_{w}_joint" type="prismatic">
      <parent>base_link</parent>
      <child>susp_{w}_link</child>
      <axis>
        <xyz expressed_in="__model__">0 0 1</xyz>
        <limit><lower>-{travel:.2f}</lower><upper>{travel:.2f}</upper><effort>1e6</effort></limit>
        <dynamics><spring_stiffness>{k}</spring_stiffness><spring_reference>{ref}</spring_reference><damping>{c}</damping></dynamics>
      </axis>
    </joint>'''
    for w in ("rear_left", "rear_right"):
        s = s.replace(f'<joint name="wheel_{w}_joint" type="revolute">\n      <parent>base_link</parent>',
                      f'<joint name="wheel_{w}_joint" type="revolute">\n      <parent>susp_{w}_link</parent>')
    for w in ("front_left", "front_right"):
        s = s.replace(f'<joint name="wheel_{w}_steering_joint" type="revolute">\n      <parent>base_link</parent>',
                      f'<joint name="wheel_{w}_steering_joint" type="revolute">\n      <parent>susp_{w}_link</parent>')
    assert s.count('<parent>susp_') == 4, s.count('<parent>susp_')
    assert '    <!-- 관절 -->' in s
    return s.replace('    <!-- 관절 -->', links + '\n\n    <!-- 관절 -->' + joints)


def publish_susp_joints(s: str) -> str:
    anchor = '      <joint_name>wheel_front_right_steering_joint</joint_name>\n    </plugin>'
    assert s.count(anchor) == 1
    extra = "".join(f'      <joint_name>susp_{w}_joint</joint_name>\n' for w in SUSP_POS)
    return s.replace(anchor, '      <joint_name>wheel_front_right_steering_joint</joint_name>\n' + extra + '    </plugin>')


def scale_body_mass(s: str, new_mass: float) -> str:
    """base_link 질량을 new_mass 로, 관성 대각 성분은 같은 비율로 (무게중심 위치 그대로)."""
    i = s.index('<link name="base_link">')
    j = s.index("</inertial>", i)
    blk = s[i:j]
    m0 = float(re.search(r"<mass>([\d.]+)</mass>", blk).group(1))
    r = new_mass / m0
    blk2 = re.sub(r"<mass>[\d.]+</mass>", f"<mass>{new_mass}</mass>", blk, count=1)
    blk2 = re.sub(r"<(ixx|iyy|izz)>([\d.]+)</(?:ixx|iyy|izz)>", lambda m: f"<{m.group(1)}>{float(m.group(2)) * r:.4g}</{m.group(1)}>", blk2)
    return s[:i] + blk2 + s[j:]


def build(variant: str, model: str = "adair_ugv") -> str:
    sdf = (MODELS / model / "model.sdf").read_text(encoding="utf-8")
    base = add_contact(sdf)
    if variant == "base":
        return base
    if variant == "rwd":
        return remove_front_drive(base)
    if variant in ("susp_a0", "susp_a", "susp_a_m"):
        if variant not in MODEL_VARIANTS[model]:
            raise SystemExit(f"{model} 에는 변형 {variant} 없음")
        s = add_suspension(base, MODEL_SUSP[model])
        if variant == "susp_a0":
            return s
        s = publish_susp_joints(s)
        return s if variant == "susp_a" else scale_body_mass(s, 1480.0)
    raise SystemExit(f"변형 {variant} 없음")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--variant", default=None, help="쉼표로 (기본: 그 차종의 전부)")
    ap.add_argument("--model", default="adair_ugv")
    ap.add_argument("--check", action="store_true", help="저장본과 재생성 결과 sha1 비교만")
    a = ap.parse_args()
    bad = 0
    for v in (a.variant.split(",") if a.variant else MODEL_VARIANTS[a.model]):
        txt = build(v, a.model)
        d = VARIANTS / v / a.model
        out = d / "model.sdf"
        # Windows 에서도 LF 그대로 (해시 재현)
        new = txt.encode("utf-8")
        if a.check:
            old = out.read_bytes() if out.exists() else b""
            ok = sha1(old) == sha1(new)
            bad += not ok
            print(v, "일치" if ok else "불일치", sha1(new)[:16])
            continue
        d.mkdir(parents=True, exist_ok=True)
        out.write_bytes(new)
        (d / "model.config").write_bytes((MODELS / a.model / "model.config").read_bytes())
        man = {"variant": v, "model": a.model, "source_sha1": sha1((MODELS / a.model / "model.sdf").read_bytes()),
               "model_sdf_sha1": sha1(new), "params": ({k: (list(v2) if isinstance(v2, tuple) else v2) for k, v2 in MODEL_SUSP[a.model].items()}
                                                         if v.startswith("susp") else None),
               "generator": "ugv/tools/make_vehicle_variant.py"}
        (VARIANTS / v / ("MANIFEST.json" if a.model == "adair_ugv" else f"MANIFEST_{a.model}.json")).write_text(json.dumps(man, ensure_ascii=False, indent=1) + "\n", encoding="utf-8")
        print(v, len(new), "bytes", sha1(new)[:16])
    sys.exit(1 if bad else 0)


if __name__ == "__main__":
    main()
