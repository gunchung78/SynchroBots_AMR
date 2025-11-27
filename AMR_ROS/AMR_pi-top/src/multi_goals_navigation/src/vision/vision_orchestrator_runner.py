#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
vision_orchestrator_runner.py

역할:
- 같은 디렉토리에 있는 vision_orchestrator.py 를 별도 프로세스로 실행한다.
- 그 안에서 다시 /start_camera → yolo(sub) → vision_result.py 가 실행된다.
- vision_result.py 가 '0' 또는 '1' (또는 -1) 을 **마지막 줄에 숫자만** 출력하도록 되어 있으므로,
  runner는 stdout의 마지막 숫자 한 줄만 읽어 최종 결과로 리턴한다.
"""

import subprocess
import rospy
import os
import sys
from pathlib import Path

def run_orchestrator(timeout_sec: int = 60) -> int:
    """
    vision_orchestrator.py 를 실행해서 YOLO 결과(0/1/-1)를 int 로 돌려준다.
    - stdout 마지막 줄은 반드시 숫자라고 가정 (vision_result.py 에서 보장)
    - 실패/파싱불가 시 -1 리턴
    """
    # 현재 파일 기준으로 orchestrator 경로 찾기
    current_dir = Path(__file__).resolve().parent
    orchestrator_path = current_dir / "vision_orchestrator.py"

    if not orchestrator_path.exists():
        rospy.logerr(f"[runner] orchestrator not found: {orchestrator_path}")
        return -1

    # 환경 가져오기 (ROS_MASTER_URI, ROS_IP 등 현재 쉘 값 그대로 사용)
    env = os.environ.copy()

    cmd = ["python3", str(orchestrator_path)]
    rospy.loginfo(f"[runner] launching orchestrator: {' '.join(cmd)}")

    try:
        # run 으로 한 번에 실행해서 stdout 전부 가져옴
        completed = subprocess.run(
            cmd,
            text=True,
            capture_output=True,
            env=env,
            timeout=timeout_sec,
        )
    except subprocess.TimeoutExpired:
        rospy.logwarn("[runner] orchestrator timed out")
        return -1
    except Exception as e:
        rospy.logerr(f"[runner] orchestrator failed: {e}")
        return -1

    # 표준출력 / 표준에러 로깅 (필요하면 주석)
    if completed.stderr:
        rospy.loginfo(f"[runner] orchestrator stderr:\n{completed.stderr}")

    out = completed.stdout.strip()
    rospy.loginfo(f"[runner] orchestrator stdout:\n{out}")

    if not out:
        rospy.logwarn("[runner] orchestrator produced no output")
        return -1

    # 마지막 줄만 본다 → vision_result.py 가 숫자만 찍도록 이미 만들어둔 상태
    last_line = out.splitlines()[-1].strip()
    if last_line.lstrip("-").isdigit():
        val = int(last_line)
        rospy.loginfo(f"[runner] final parsed value = {val}")
        return val
    else:
        rospy.logwarn(f"[runner] last line is not a pure number: '{last_line}'")
        return -1


if __name__ == "__main__":
    rospy.init_node("vision_orchestrator_runner", anonymous=True)
    val = run_orchestrator()
    print(f"[runner] YOLO result value = {val}")
    # 필요하면 exit code 로도 넘겨줄 수 있음
    sys.exit(0)
