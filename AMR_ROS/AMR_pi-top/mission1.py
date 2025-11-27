#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
원격(myAGV)에서 아래 두 launch를 실행/정지:
  1) roslaunch myagv_navigation navigation_active.launch
  2) (5초 뒤) roslaunch multi_goals_navigation multi_goals_navigation.launch

로컬 PC는 단지 ssh로 명령만 보냄. GUI 터미널 없이 headless로 동작.
원격 /tmp에 PID 파일을 남깁니다:
  /tmp/myagv_navigation.pid
  /tmp/myagv_multi_goals.pid
"""

import argparse
import subprocess
import sys

# === 환경 설정 ===
AGV_USER = "er"              # 원격 myAGV 사용자
AGV_HOST = "172.30.1.94"     # 원격 myAGV IP/호스트
ROS_SETUP = "/opt/ros/noetic/setup.bash"
WS_SETUP  = "/home/er/MyAGV_Project/myagv_ros/devel/setup.bash"  # 워크스페이스

PID_NAV   = "/tmp/myagv_navigation.pid"
PID_MULTI = "/tmp/myagv_multi_goals.pid"

LAUNCH_NAV   = "roslaunch myagv_navigation navigation_active.launch"
LAUNCH_MULTI = "roslaunch multi_goals_navigation multi_goals_navigation.launch"

def ssh(cmd: str) -> int:
    """원격에서 bash -lc 로 실행"""
    full = ["ssh", f"{AGV_USER}@{AGV_HOST}", "bash", "-lc", cmd]
    return subprocess.call(full)

def start_remote():
    # 1) navigation: 원격에서 환경 로드 후 nohup 백그라운드 + PID 기록
    cmd_nav = f"""
      set -e
      source {ROS_SETUP} >/dev/null 2>&1 || true
      if [ -f {WS_SETUP} ]; then source {WS_SETUP}; fi
      nohup bash -lc '{LAUNCH_NAV}' >/tmp/nav.log 2>&1 & echo $! > {PID_NAV}
    """
    print("[remote] start navigation...")
    if ssh(cmd_nav) != 0:
        print("[ERR] navigation start failed")
        sys.exit(1)

    # 2) 5초 대기 후 multi_goals
    cmd_multi = f"""
      set -e
      sleep 5
      source {ROS_SETUP} >/dev/null 2>&1 || true
      if [ -f {WS_SETUP} ]; then source {WS_SETUP}; fi
      nohup bash -lc '{LAUNCH_MULTI}' >/tmp/multi.log 2>&1 & echo $! > {PID_MULTI}
    """
    print("[remote] start multi_goals after 5s...")
    if ssh(cmd_multi) != 0:
        print("[ERR] multi_goals start failed")
        sys.exit(1)

    print("[OK] started both on remote. logs: /tmp/nav.log, /tmp/multi.log (on myAGV)")

def stop_remote():
    # multi_goals 먼저 정지, 그 다음 navigation
    cmd_stop = f"""
      for f in {PID_MULTI} {PID_NAV}; do
        if [ -f "$f" ]; then
          pid=$(cat "$f" 2>/dev/null || true)
          if [ -n "$pid" ]; then
            # 순차 종료: INT -> TERM -> KILL
            kill -INT "$pid" 2>/dev/null || true
            sleep 1
            kill -TERM "$pid" 2>/dev/null || true
            sleep 1
            kill -KILL "$pid" 2>/dev/null || true
          fi
          rm -f "$f"
        fi
      done
      # roslaunch 트리 누수 대비(프로세스명 기준 추가 정리, 실패해도 무시)
      pkill -f 'roslaunch myagv_navigation navigation_active.launch' 2>/dev/null || true
      pkill -f 'roslaunch multi_goals_navigation multi_goals_navigation.launch' 2>/dev/null || true
    """
    print("[remote] stopping both...")
    ssh(cmd_stop)
    print("[OK] stopped both on remote")

def main():
    ap = argparse.ArgumentParser(description="remote myAGV navigation + multi_goals starter")
    g = ap.add_mutually_exclusive_group()
    g.add_argument("--start", action="store_true", help="start on remote (default)")
    g.add_argument("--stop",  action="store_true", help="stop on remote")
    args = ap.parse_args()

    if args.stop:
        stop_remote()
    else:
        start_remote()

if __name__ == "__main__":
    main()

