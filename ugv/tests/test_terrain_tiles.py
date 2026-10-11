"""트윈 지형 타일 생성기 — 설정 기록·시험 전용 도로 가드 (2026-10-11 트윈 지도 정제 1단계)."""
import pytest

from ugv.tools import build_terrain_tiles as BT


def test_tiles_meta_records_params():
    p = BT.build_params()
    for k in ("junction_mode", "junc_level_sigma_m", "road_surface_sigma_m", "blend_m", "git_head", "extra_roads"):
        assert k in p
    assert p["junction_mode"] in ("terrain", "level")


def test_test_only_roads_rejected_in_twin_build(monkeypatch):
    class R:
        def __init__(self, test):
            self.attrs = {"test_only": True} if test else {}

    class Net:
        roads = {"1": R(False), "T1": R(True)}

    monkeypatch.setattr(BT, "RoadNetwork", lambda: Net())
    monkeypatch.delenv("ADAIR_ALLOW_TEST_ROADS", raising=False)
    with pytest.raises(SystemExit, match="시험 전용"):
        BT.build_all(only={"NONE"})
