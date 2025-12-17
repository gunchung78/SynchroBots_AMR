#!/usr/bin/env bash
# forward_5s.sh — N초 동안 V m/s 전진 후 정지
# 사용: ./forward_5s.sh [토픽=/cmd_vel] [초=5] [Hz=10] [속도=0.2]

set -euo pipefail

TOPIC="${1:-/cmd_vel}"
DUR="${2:-3.92625}"
HZ="${3:-5}"
VX="${4:-0.0}"
VTH="${5:-0.5281}"

# 정지 메시지 (단일 YAML 객체)
STOP_MSG='{ linear: { x: 0.0, y: 0.0, z: 0.0 }, angular: { x: 0.0, y: 0.0, z: 0.0 } }'

# 종료시 무조건 정지
stop_robot() {
  rostopic pub -1 "$TOPIC" geometry_msgs/Twist "$STOP_MSG" >/dev/null || true
}
trap stop_robot EXIT

# ROS master 확인
if ! rosparam list >/dev/null 2>&1; then
  echo "ERROR: ROS master 연결 안 됨(roscore/bringup 확인)." >&2
  exit 1
fi

CMD_MSG="{ linear: { x: ${VX}, y: 0.0, z: 0.0 }, angular: { x: 0.0, y: 0.0, z: ${VTH} } }"

echo "Forward ${DUR}s @ ${VX} m/s, VTH=${VTH} rad/s on ${TOPIC} (rate ${HZ} Hz)..."
# 지정 시간 동안 주행
timeout "${DUR}s" rostopic pub -r "$HZ" "$TOPIC" geometry_msgs/Twist "$CMD_MSG"

# 안전 정지(트랩에서도 한 번 더 보냄)
rostopic pub -1 "$TOPIC" geometry_msgs/Twist "$STOP_MSG" >/dev/null || true
echo "Done."
