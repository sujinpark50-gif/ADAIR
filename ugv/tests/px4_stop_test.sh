#!/usr/bin/env bash
# ugv/tests/px4_stop_test.sh — (WSL 안에서) px4-stop.sh 종료 절차 시험. Gazebo·PX4 없이 가짜 프로세스로.
#   bash ugv/tests/px4_stop_test.sh   → 마지막 줄 "PASS n/n" 또는 "FAIL ..."
# 시험 전용 파티션(adair_stoptest)·상태 폴더를 써서 실제 시험(adair_ugv)·다른 작업의 프로세스는 건드리지 않는다.
set -u
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
STOP="$REPO/ugv/tools/px4-stop.sh"
export STATE_DIR; STATE_DIR=$(mktemp -d /tmp/adair-stoptest.XXXX)
export GZ_PARTITION=adair_stoptest STOP_GRACE_S=1
pass=0; total=0; fails=""
check() { total=$((total + 1)); if eval "$2"; then pass=$((pass + 1)); else fails="$fails [$1]"; fi; }
st() { awk '{print $22}' "/proc/$1/stat"; }
alive() { [ -d "/proc/$1" ] && [ "$(awk '{print $3}' "/proc/$1/stat" 2>/dev/null)" != "Z" ]; }
fake() {  # $1 이름(argv0) $2 파티션 $3 TERM 무시(1)
  if [ "${3:-0}" = 1 ]; then
    GZ_PARTITION=$2 setsid bash -c "trap '' TERM; exec -a '$1' sleep 300" >/dev/null 2>&1 &
  else
    GZ_PARTITION=$2 setsid bash -c "exec -a '$1' sleep 300" >/dev/null 2>&1 &
  fi
  echo $!
}
rec() { printf '%s\t%s\t%s\t%s\t%s\n' "$1" "$2" "$3" "$GZ_PARTITION" "fake" >> "$STATE_DIR/procs.tsv"; }

# 1) 기록된 프로세스, TERM 에 종료
p1=$(fake "gz sim -r -s w1.sdf" adair_stoptest); sleep 0.3; rec gz "$p1" "$(st "$p1")"
# 2) 기록된 프로세스, TERM 무시 → KILL
p2=$(fake "/x/bin/px4 -i 7 -d" adair_stoptest 1); sleep 0.3; rec px4-7 "$p2" "$(st "$p2")"
# 3) 기록의 시작 시각이 다름 (PID 재사용) → 신호 보내지 않음
p3=$(fake "gz sim -r -s w3.sdf" otherpart); sleep 0.3; rec gz "$p3" "999"
# 4) 기록 밖, 우리 파티션 → 잔존으로 끔
p4=$(fake "gz sim -r -s w4.sdf" adair_stoptest); sleep 0.3
# 5) 기록 밖, 다른 파티션 → 그대로
p5=$(fake "gz sim -g" otherpart2); sleep 0.3

STOP_STRICT=1 bash "$STOP" > "$STATE_DIR/out.txt"; rc=$?
J=$(cat "$STATE_DIR/stop_last.json")
check "1 TERM 종료" "! alive $p1"
check "2 KILL 종료" "! alive $p2"
check "3 PID 재사용 보호" "alive $p3"
check "4 잔존 끔" "! alive $p4"
check "5 다른 파티션 유지" "alive $p5"
check "종료 코드 2(잔존)" "[ $rc = 2 ]"
check "강제 1" "echo '$J' | grep -q '\"forced\":1'"
check "잔존 1" "echo '$J' | grep -q '\"residual_own\":1'"
check "기록 파일 정리" "[ ! -f '$STATE_DIR/procs.tsv' ]"
check "다른 파티션 보고" "echo '$J' | grep -q '$p5:otherpart2'"
# 두 번째 호출: 깨끗함 → 0
STOP_STRICT=1 bash "$STOP" > /dev/null; rc2=$?
check "재호출 0" "[ $rc2 = 0 ]"
kill -KILL "$p3" "$p5" 2>/dev/null
rm -rf "$STATE_DIR"
if [ "$pass" = "$total" ]; then echo "PASS $pass/$total"; else echo "FAIL $pass/$total:$fails"; echo "$J"; exit 1; fi
