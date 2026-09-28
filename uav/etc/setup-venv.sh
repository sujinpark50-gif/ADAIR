#!/bin/bash
# UAV Agent 실행 환경 구축 — repo 루트에 .venv 를 만든다.
#   ./uav/etc/setup-venv.sh
# 이후 실행:  source .venv/bin/activate
#
# venv 자체는 git 에 올리지 않는다(.gitignore). bin/activate 안에는 생성 당시의
# 절대경로가 박히고 lib/ 에는 플랫폼별 바이너리가 들어가서, 커밋해도 다른 머신에서 깨진다.
# 재현성은 requirements.txt 가 책임진다.
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/../.." && pwd)"   # uav/etc/ → repo 루트
VENV="$ROOT/.venv"

python3 -m venv "$VENV"
"$VENV/bin/pip" install --upgrade pip
"$VENV/bin/pip" install -r "$ROOT/requirements.txt"

echo
echo "완료. 활성화:  source .venv/bin/activate"
"$VENV/bin/python" -c "import mavsdk, fastapi; print('mavsdk', mavsdk.__version__)" 2>/dev/null \
  || echo "(mavsdk 버전 확인 실패 — import 는 Agent 기동 시 다시 검증됩니다)"
