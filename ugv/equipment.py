# ugv/equipment.py — 차량 장비 상태: 물탱크·펌프(소방차), 적재(UGV), 경광등, 도착 뒤 작업(activity)
#
# 작업(activity) 은 주행과 별개다. task 는 도착하면 COMPLETED(지금 계약 그대로)이고, 그 뒤의 작업
# (진압·짐 내리기) 동안 차는 state=WORKING 이라 READY 가 아니다 → 총괄은 반납하지 않고 점유를 유지한다
# (orchestrator/engine.py _maybe_release 가 READY 를 확인한 뒤에만 반납). 작업이 끝나면 READY.
# 출발 전 싣기(LOADING)는 task 진행 단계(progress.phase)로 보인다.
#
#   activity : IDLE | LOADING | SUPPRESSING | UNLOADING
#   water    : 남은 물(L) — 진압 중 pump_lps 로 줄어든다. 0 이면 진압 종료(EMPTY). 거점 노드 도착 시 가득
#   cargo    : {"name", "kg", "loaded"} | None

from dataclasses import dataclass, field


@dataclass
class Equipment:
    water_capacity_l: float = 0.0
    pump_lps: float = 0.0
    payload_kg: float = 0.0
    has_siren: bool = False

    water_l: float = 0.0
    cargo: dict | None = None
    siren_on: bool = False
    pump_on: bool = False
    activity: str = "IDLE"
    activity_task_id: str | None = None
    activity_started_s: float | None = None     # 시뮬레이션 초
    activity_until_s: float | None = None       # 정해진 끝 (싣기·내리기). 진압은 물이 바닥날 때까지라 None
    history: list = field(default_factory=list)  # 최근 작업 기록 (최대 10)

    @classmethod
    def for_type(cls, spec: dict) -> "Equipment":
        e = cls(water_capacity_l=spec.get("water_capacity_l", 0.0), pump_lps=spec.get("pump_lps", 0.0),
                payload_kg=spec.get("payload_kg", 0.0), has_siren=spec.get("siren", False))
        e.water_l = e.water_capacity_l
        return e

    # --- 판단 ---------------------------------------------------------------

    def can_carry(self, kg: float) -> str | None:
        """실을 수 없으면 이유, 있으면 None."""
        if self.payload_kg <= 0:
            return "이 차량은 짐을 싣지 않는다 (payload 0 kg)"
        if kg > self.payload_kg:
            return f"적재 한도 초과: {kg:.0f} kg > {self.payload_kg:.0f} kg"
        return None

    def can_suppress(self) -> str | None:
        if self.pump_lps <= 0:
            return "이 차량에는 펌프가 없다"
        if self.water_l <= 0:
            return "물탱크가 비었다 — 거점으로 돌아가 채워야 한다"
        return None

    # --- 작업 기록 -----------------------------------------------------------

    def begin(self, activity: str, task_id: str | None, now_s: float, until_s: float | None = None) -> None:
        self.activity, self.activity_task_id = activity, task_id
        self.activity_started_s, self.activity_until_s = now_s, until_s
        self.pump_on = activity == "SUPPRESSING"

    def end(self, now_s: float, result: str, **extra) -> dict:
        rec = {"activity": self.activity, "task_id": self.activity_task_id,
               "started_sim_s": None if self.activity_started_s is None else round(self.activity_started_s, 1),
               "ended_sim_s": round(now_s, 1), "result": result, **extra}
        self.history = (self.history + [rec])[-10:]
        self.activity, self.activity_task_id = "IDLE", None
        self.activity_started_s = self.activity_until_s = None
        self.pump_on = False
        return rec

    def refill(self) -> float:
        added = self.water_capacity_l - self.water_l
        self.water_l = self.water_capacity_l
        return added

    def to_dict(self) -> dict:
        return {
            "activity": self.activity,
            "activity_task_id": self.activity_task_id,
            "activity_started_sim_s": None if self.activity_started_s is None else round(self.activity_started_s, 1),
            "activity_until_sim_s": None if self.activity_until_s is None else round(self.activity_until_s, 1),
            "siren_on": self.siren_on,
            "pump_on": self.pump_on,
            "water_l": round(self.water_l, 1) if self.water_capacity_l else None,
            "water_capacity_l": self.water_capacity_l or None,
            "pump_lps": self.pump_lps or None,
            "payload_kg": self.payload_kg or None,
            "cargo": self.cargo,
            "last_activities": self.history[-3:],
        }
