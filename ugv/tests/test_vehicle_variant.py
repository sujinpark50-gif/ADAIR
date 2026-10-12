# -*- coding: utf-8 -*-
"""차량 후보 생성기 (ugv/tools/make_vehicle_variant.py) — 저장본 재현, 한 가지만 바꾸는지, 질량·관절·센서 구성."""
import re
import xml.etree.ElementTree as ET

import pytest

from ugv.tools import make_vehicle_variant as mv

VARIANTS = ["base", "rwd", "susp_a0", "susp_a"]


def _model(txt):
    return ET.fromstring(txt).find("model")


def _mass(m):
    return sum(float(l.find("inertial/mass").text) for l in m.findall("link") if l.find("inertial/mass") is not None)


@pytest.mark.parametrize("v", VARIANTS)
def test_saved_matches_regenerated(v):
    saved = (mv.VARIANTS / v / "adair_ugv" / "model.sdf").read_bytes()
    assert mv.sha1(saved) == mv.sha1(mv.build(v).encode("utf-8"))


def test_susp_a0_content_equals_chain56_model():
    # chain56 이 쓴 사본의 내용 해시 (logs/ugv/stage5/jturn/fable_models/susp, CRLF 저장) — 결과 재사용 조건
    assert mv.sha1(mv.build("susp_a0").encode("utf-8")).startswith("35fb49b6be916e78")


def test_base_only_adds_contact_sensors():
    orig = _model((mv.MODELS / "adair_ugv" / "model.sdf").read_text(encoding="utf-8"))
    m = _model(mv.build("base"))
    assert len(m.findall(".//sensor[@type='contact']")) == 4
    assert _mass(m) == pytest.approx(_mass(orig)) == pytest.approx(1604.0)
    assert [j.get("name") for j in m.findall("joint")] == [j.get("name") for j in orig.findall("joint")]


def test_rwd_removes_only_front_drive():
    base, rwd = mv.build("base"), mv.build("rwd")
    jc = lambda t: re.findall(r"JointController\">\s*<joint_name>(\w+)</joint_name>", t)
    assert sorted(set(jc(base)) - set(jc(rwd))) == ["wheel_front_left_joint", "wheel_front_right_joint"]
    assert "wheel_rear_left_joint" in jc(rwd) and "wheel_rear_right_joint" in jc(rwd)


@pytest.mark.parametrize("v", ["susp_a0", "susp_a"])
def test_suspension_structure(v):
    m = _model(mv.build(v))
    sj = [j for j in m.findall("joint") if j.get("type") == "prismatic"]
    assert sorted(j.get("name") for j in sj) == sorted(f"susp_{w}_joint" for w in mv.SUSP_POS)
    for j in sj:
        assert j.find("parent").text == "base_link"
        dyn = j.find("axis/dynamics")
        assert float(dyn.find("spring_stiffness").text) == 130000.0 and float(dyn.find("damping").text) == 8000.0
        lim = j.find("axis/limit")
        assert (float(lim.find("lower").text), float(lim.find("upper").text)) == (-0.10, 0.10)
    parents = {j.get("name"): j.find("parent").text for j in m.findall("joint")}
    for w in ("rear_left", "rear_right"):
        assert parents[f"wheel_{w}_joint"] == f"susp_{w}_link"
    for w in ("front_left", "front_right"):
        assert parents[f"wheel_{w}_steering_joint"] == f"susp_{w}_link"
        assert parents[f"wheel_{w}_joint"] == f"wheel_{w}_steering_link"
    assert _mass(m) == pytest.approx(1624.0)
    # 구동·조향 명령 구성은 base 와 같다
    jc = lambda t: re.findall(r"JointController\">\s*<joint_name>(\w+)</joint_name>", t)
    assert jc(mv.build(v)) == jc(mv.build("base"))


def test_susp_a_publishes_susp_joints_only_difference():
    a0, a = mv.build("susp_a0"), mv.build("susp_a")
    pub = lambda t: re.findall(r"<joint_name>(susp_\w+)</joint_name>", t.split("JointStatePublisher")[1])
    assert pub(a0) == [] and sorted(pub(a)) == sorted(f"susp_{w}_joint" for w in mv.SUSP_POS)
    assert a.replace("".join(f"      <joint_name>susp_{w}_joint</joint_name>\n" for w in mv.SUSP_POS), "") == a0
