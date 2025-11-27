#!/usr/bin/env bash
set -euo pipefail

# ===== 0) 기본 =====
WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SRC_DIR="${WS_DIR}/src"
TP_DIR="${WS_DIR}/third_party"
SDK_DIR="${TP_DIR}/ydlidar_sdk"

echo "[bootstrap] workspace: ${WS_DIR}"

# ===== 1) 필수 패키지 =====
echo "[bootstrap] apt install base deps..."
sudo apt-get update
sudo apt-get install -y \
  build-essential cmake git pkg-config dos2unix \
  libusb-1.0-0-dev \
  ros-noetic-desktop-full

# ROS 편의 도구
sudo apt-get install -y \
  ros-noetic-tf2-ros ros-noetic-tf2-tools ros-noetic-tf2-geometry-msgs ros-noetic-tf2-sensor-msgs \
  ros-noetic-navigation \
  ros-noetic-robot-state-publisher ros-noetic-joint-state-publisher ros-noetic-joint-state-publisher-gui \
  ros-noetic-teleop-twist-keyboard

# EKF는 바이너리로 사용 권장
sudo apt-get install -y ros-noetic-robot-pose-ekf || true

# ===== 2) rosdep =====
echo "[bootstrap] rosdep..."
sudo rosdep init 2>/dev/null || true
rosdep update
rosdep install --from-paths "${SRC_DIR}" --ignore-src -r -y || true

# ===== 3) 파이썬 요구사항 =====
if [[ -f "${WS_DIR}/requirements.txt" ]]; then
  echo "[bootstrap] python requirements..."
  python3 -m pip install --user -U pip
  python3 -m pip install --user -r "${WS_DIR}/requirements.txt"
fi

# ===== 4) udev & dialout =====
echo "[bootstrap] udev rules & dialout..."
sudo usermod -a -G dialout "$USER" || true
sudo bash -c 'cat >/etc/udev/rules.d/99-serial-usb.rules' <<'EOF'
# General USB serial devices (e.g., Lidar on /dev/ttyUSB*)
KERNEL=="ttyUSB[0-9]*", MODE:="0666"
EOF
sudo udevadm control --reload-rules && sudo udevadm trigger

# ===== 5) YDLidar SDK (벤더링) =====
echo "[bootstrap] YDLidar SDK..."
mkdir -p "${TP_DIR}"

# 비어있거나 CMakeLists.txt가 없으면 다시 클론
if [[ ! -d "${SDK_DIR}" || ! -f "${SDK_DIR}/CMakeLists.txt" ]]; then
  rm -rf "${SDK_DIR}"
  git clone https://github.com/YDLIDAR/YDLidar-SDK.git "${SDK_DIR}"
fi

pushd "${SDK_DIR}"
mkdir -p build && cd build
cmake ..
make -j"$(nproc)"
sudo make install
popd

# CMake가 /usr/local 을 찾도록 보조
export CMAKE_PREFIX_PATH="/usr/local:${CMAKE_PREFIX_PATH:-}"

# SDK가 /usr/local에 설치되었음을 보장하기 위해 CMAKE_PREFIX_PATH 추가(빌드 시 사용)
export CMAKE_PREFIX_PATH="/usr/local:${CMAKE_PREFIX_PATH:-}"

# ===== 6) CRLF 정리 & 권한 =====
echo "[bootstrap] fix CRLF & exec perms..."
find "${SRC_DIR}" -type f \( -name "*.py" -o -name "*.sh" \) -print0 | xargs -0 dos2unix || true
find "${SRC_DIR}" -type f -path "*/scripts/*" -name "*.py" -exec chmod +x {} +

# ===== 7) bashrc 편의 =====
if ! grep -q "/opt/ros/noetic/setup.bash" ~/.bashrc; then
  echo 'source /opt/ros/noetic/setup.bash' >> ~/.bashrc
fi
if ! grep -q "${WS_DIR}/devel/setup.bash" ~/.bashrc; then
  echo "source ${WS_DIR}/devel/setup.bash" >> ~/.bashrc
fi

echo "[bootstrap] DONE. *터미널을 한 번 껐다 켜거나* 'newgrp dialout' 하세요."
