# ugv/drivers/base.py — 이동 드라이버 추상 인터페이스
# 실제 PX4(px4.py)와 시뮬레이션(sim.py)이 같은 계약을 따른다.
# I/O 가 필요한 connect/goto 는 async, 값 조회는 최근 스냅샷을 돌려주는 동기 메서드.

from abc import ABC, abstractmethod


class MotionDriver(ABC):
    @abstractmethod
    async def connect(self) -> None:
        """기체/시뮬레이터에 연결하고 상태 갱신을 시작한다."""

    @abstractmethod
    def position(self) -> tuple[float, float]:
        """현재 위치 (lat, lon)."""

    @abstractmethod
    def battery(self) -> float:
        """배터리 잔량 (%)."""

    @abstractmethod
    def status(self) -> str:
        """현재 모드/상태 문자열."""

    @abstractmethod
    async def goto(self, waypoints: list[tuple[float, float]]) -> bool:
        """웨이포인트 (lat, lon) 순서대로 이동 시작.

        시작에 성공하면 True, 실패하면 False. 도착까지 기다리지 않으며
        진행은 progress() 로 확인한다.
        """

    @abstractmethod
    def progress(self) -> tuple[int, int]:
        """(통과한 웨이포인트 수, 전체 웨이포인트 수)."""
