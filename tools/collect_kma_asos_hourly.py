#!/usr/bin/env python3
"""기상청 ASOS 시간자료(공공데이터포털 AsosHourlyInfoService)를 CSV 로 받는다.

2019-04-04 인제 산불 재현에서 총괄이 "그 시각까지 관측된 기상"으로 판단하도록 쓰는 입력 자료.
인증키는 DATA_GO_KR_SERVICE_KEY (공공데이터포털 일반 인증키). 값은 출력하지 않는다.

예)
  python tools/collect_kma_asos_hourly.py --env-file ../inje-wildfire-weather-analysis/.env \
      --stations 211,90,212,100 --start 2019040400 --end 2019040523 \
      --output data/weather/kma_asos_hourly_20190404_20190405.csv
"""

import argparse
import csv
import json
import os
import sys
import time
from pathlib import Path
from urllib.error import HTTPError, URLError
from urllib.parse import unquote, urlencode
from urllib.request import Request, urlopen

API_URL = "https://apis.data.go.kr/1360000/AsosHourlyInfoService/getWthrDataList"
# 판단에 쓰는 열을 앞쪽에 둔다. 나머지 응답 필드는 뒤에 그대로 보존한다.
PREFERRED = ["tm", "stnId", "stnNm", "ta", "ws", "wd", "hm", "rn", "pa", "ps", "td",
             "taQcflg", "wsQcflg", "wdQcflg", "hmQcflg"]


def load_env(path: Path) -> None:
    if not path.is_file():
        return
    for raw in path.read_text(encoding="utf-8-sig").splitlines():
        line = raw.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        k, v = (p.strip() for p in line.split("=", 1))
        if len(v) >= 2 and v[0] == v[-1] and v[0] in "\"'":
            v = v[1:-1]
        os.environ.setdefault(k, v)


def fetch(key: str, stn: str, start: str, end: str) -> list:
    rows, page, total = [], 1, None
    while total is None or len(rows) < total:
        q = {"ServiceKey": unquote(key), "pageNo": page, "numOfRows": 999, "dataType": "JSON",
             "dataCd": "ASOS", "dateCd": "HR", "startDt": start[:8], "startHh": start[8:10],
             "endDt": end[:8], "endHh": end[8:10], "stnIds": stn}
        req = Request(f"{API_URL}?{urlencode(q)}", headers={"User-Agent": "adair-orchestrator/1.0"})
        for attempt in range(3):
            try:
                with urlopen(req, timeout=30) as r:
                    payload = json.loads(r.read().decode("utf-8-sig"))
                break
            except (HTTPError, URLError, TimeoutError, json.JSONDecodeError) as e:
                if attempt == 2:
                    raise RuntimeError(f"{stn} 호출 실패: {type(e).__name__}") from e
                time.sleep(2 ** attempt)
        head = payload.get("response", {}).get("header", {})
        if str(head.get("resultCode")) != "00":
            raise RuntimeError(f"{stn} API 오류 {head.get('resultCode')}: {head.get('resultMsg')}")
        body = payload["response"]["body"]
        total = int(body.get("totalCount", 0))
        items = (body.get("items") or {}).get("item", [])
        items = [items] if isinstance(items, dict) else items
        if not items:
            break
        rows.extend(items)
        page += 1
    return rows


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--env-file", type=Path, default=Path(".env"))
    ap.add_argument("--stations", default="211")
    ap.add_argument("--start", required=True, help="YYYYMMDDHH")
    ap.add_argument("--end", required=True, help="YYYYMMDDHH")
    ap.add_argument("--output", type=Path, required=True)
    a = ap.parse_args()
    load_env(a.env_file)
    key = os.getenv("DATA_GO_KR_SERVICE_KEY", "").strip()
    if not key:
        print("DATA_GO_KR_SERVICE_KEY 가 없습니다.", file=sys.stderr)
        return 2
    rows = []
    for stn in a.stations.split(","):
        got = fetch(key, stn.strip(), a.start, a.end)
        print(f"지점 {stn}: {len(got)}건")
        rows.extend(got)
    rows = sorted({(r.get("stnId"), r.get("tm")): r for r in rows}.values(), key=lambda r: (r.get("stnId"), r.get("tm")))
    fields = [f for f in PREFERRED if any(f in r for r in rows)]
    fields += sorted({k for r in rows for k in r} - set(fields))
    a.output.parent.mkdir(parents=True, exist_ok=True)
    with a.output.open("w", encoding="utf-8-sig", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fields, extrasaction="ignore")
        w.writeheader()
        w.writerows(rows)
    print(f"저장: {len(rows)}건 -> {a.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
