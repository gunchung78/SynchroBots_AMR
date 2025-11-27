#!/usr/bin/env bash
set -euo pipefail
WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

source /opt/ros/noetic/setup.bash
source "${WS_DIR}/devel/setup.bash"

# 필요시 환경 변수 로드 (.env 파일) — 포트/모델 등
if [[ -f "${WS_DIR}/scripts/.env" ]]; then
  set -a; source "${WS_DIR}/scripts/.env"; set +a
fi

# 예: LiDAR + teleop 키보드
# 실제 패키지/런치명으로 바꿔 넣으세요.
gnome-terminal -- bash -lc "roscore"
sleep 2

# LiDAR 드라이버 (ydlidar_ros_driver 라면 실제 런치명 확인)
gnome-terminal -- bash -lc "roslaunch ydlidar_ros_driver X2.launch"

# 텔레옵
gnome-terminal -- bash -lc "rosrun teleop_twist_keyboard teleop_twist_keyboard.py"
