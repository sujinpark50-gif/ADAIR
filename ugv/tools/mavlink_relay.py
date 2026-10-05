#!/usr/bin/env python3
# ugv/tools/mavlink_relay.py — PX4 SITL(127.0.0.1) ↔ 원격 UGV 서버 MAVLink UDP 중계 (표준 라이브러리만)
#
# px4-start.sh 는 PX4 에 원격 링크(-t UGV_HOST)를 직접 추가한다. 그게 안 될 때만 이것을 쓴다.
# PX4 SITL 의 기본 API 링크는 127.0.0.1:1454i 로만 보내므로, Gazebo/PX4 가 도는 기계(WSL)에서
# 이 스크립트가 그 포트를 받아 UGV 서버 기계의 같은 포트로 넘기고, 답장은 PX4 로 되돌려 준다.
#
#   python3 ugv/tools/mavlink_relay.py --host 192.168.0.23               # 14541 14542 14543
#   python3 ugv/tools/mavlink_relay.py --host 192.168.0.23 --ports 14541
#   python3 ugv/tools/mavlink_relay.py --selftest                        # 이 기계 안에서 왕복 확인
#
# 주의: 같은 기계에서 MAVSDK·QGC 가 1454i 를 이미 쓰고 있으면 bind 가 실패한다 (WSL 에는 둘 다 없어야 한다).

import argparse
import selectors
import socket
import threading
import time


class Relay:
    """포트 하나: PX4 쪽 소켓(127.0.0.1:port) + 원격 쪽 소켓(0.0.0.0:임의)."""

    def __init__(self, port: int, host: str, remote_port: int | None = None, bind_ip: str = "127.0.0.1"):
        self.port = port
        self.remote = (host, remote_port or port)
        self.local = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.local.bind((bind_ip, port))
        self.out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.out.bind(("0.0.0.0", 0))
        self.px4_addr = None            # PX4 가 보내는 주소 (127.0.0.1:1458i). 처음 받은 뒤부터 답장 가능
        self.up = self.down = 0         # 중계한 패킷 수

    def on_local(self):
        data, addr = self.local.recvfrom(65535)
        self.px4_addr = addr
        self.out.sendto(data, self.remote)
        self.up += 1

    def on_remote(self):
        data, _ = self.out.recvfrom(65535)
        if self.px4_addr:
            self.local.sendto(data, self.px4_addr)
            self.down += 1


def run(relays: list[Relay], stop: threading.Event | None = None, report_s: float = 10.0):
    sel = selectors.DefaultSelector()
    for r in relays:
        sel.register(r.local, selectors.EVENT_READ, r.on_local)
        sel.register(r.out, selectors.EVENT_READ, r.on_remote)
    last = time.monotonic()
    while not (stop and stop.is_set()):
        for key, _ in sel.select(timeout=0.5):
            try:
                key.data()
            except OSError as e:         # 원격이 아직 안 떠 있으면 ICMP unreachable 이 올라올 수 있다
                print(f"경고: {e}")
        if report_s and time.monotonic() - last > report_s:
            last = time.monotonic()
            print("  ".join(f"{r.port}: ↑{r.up} ↓{r.down}" for r in relays), flush=True)


def selftest() -> bool:
    """가짜 PX4(127.0.0.1) ↔ 중계 ↔ 가짜 UGV 서버(127.0.0.1:다른 포트) 왕복."""
    server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    server.bind(("127.0.0.1", 0))
    relay = Relay(0, "127.0.0.1", remote_port=server.getsockname()[1])
    relay.port = relay.local.getsockname()[1]
    stop = threading.Event()
    threading.Thread(target=run, args=([relay], stop, 0), daemon=True).start()

    px4 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    px4.bind(("127.0.0.1", 0))
    px4.settimeout(2)
    server.settimeout(2)
    px4.sendto(b"HEARTBEAT", ("127.0.0.1", relay.port))
    got, addr = server.recvfrom(100)
    server.sendto(b"COMMAND_LONG", addr)          # MAVSDK 처럼 보낸 주소로 답장
    back, _ = px4.recvfrom(100)
    stop.set()
    ok = got == b"HEARTBEAT" and back == b"COMMAND_LONG"
    print("selftest", "OK" if ok else "FAIL", got, back)
    return ok


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--host", help="UGV 서버 기계 IP (Mac)")
    ap.add_argument("--ports", type=int, nargs="+", default=[14541, 14542, 14543])
    ap.add_argument("--selftest", action="store_true")
    a = ap.parse_args()
    if a.selftest:
        raise SystemExit(0 if selftest() else 1)
    if not a.host:
        ap.error("--host 가 필요하다")
    relays = [Relay(p, a.host) for p in a.ports]
    print(f"중계 시작: 127.0.0.1:{a.ports} ↔ {a.host}:{a.ports}")
    run(relays)


if __name__ == "__main__":
    main()
