#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import subprocess
import time
import os
import sys
from pathlib import Path

# AGV / PC 환경 (필요하면 하드코딩 유지)
AGV_MASTER = "http://10.229.15.61:11311"
PC_IP      = "10.229.15.47"

YOLO_CMD   = "rosrun multi_goals_navigation vision_sub_node.py"
RESULT_CMD = "rosrun multi_goals_navigation vision_result.py"

def call_rosservice(service_name, env=None):
    cmd = ["rosservice", "call", service_name]
    out = subprocess.check_output(cmd, text=True, env=env)
    return out

def main():
    # 실행 위치 기준으로 필요하면 경로 보정
    env = os.environ.copy()
    env["ROS_MASTER_URI"] = AGV_MASTER
    env["ROS_IP"] = PC_IP

    # 최종 결과 (디폴트는 -1)
    final_result = -1

    # 1) 카메라 ON
    print("vison step 1: start camera on AGV (call /start_camera)")
    try:
        out = call_rosservice("/start_camera", env=env)
        print(out, end="")
    except Exception as e:
        print(f"[A_pub] /start_camera failed: {e}")
        # 그래도 계속 진행할지 말지 선택 가능
        # 여기서는 일단 계속
        pass

    time.sleep(1.0)

    # 2) YOLO 노드 실행
    print(f"vison step 2: run YOLO subscriber: {YOLO_CMD}")
    yolo_proc = subprocess.Popen(
        YOLO_CMD,
        shell=True,
        env=env,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )

    # 3) 결과 수신 노드 실행
    print(f"vison step 3: run result listener: {RESULT_CMD}")
    result_proc = subprocess.Popen(
        RESULT_CMD,
        shell=True,
        env=env,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )

    # 4) 두 프로세스 로그 읽기 + result 값 뽑기
    procs = {
        "yolo": yolo_proc,
        "result": result_proc,
    }
    alive = set(procs.keys())

    while alive:
        for name, p in list(procs.items()):
            line = p.stdout.readline()
            if line:
                line_s = line.strip()
                print(f"[{name}] {line_s}")
                # ★ result 노드가 숫자만 찍는 줄이 있을 때 그걸 최종값으로 저장
                if name == "result":
                    # 예: "1" 혹은 "[result] 1" 둘 다 처리
                    parts = line_s.split()
                    for token in parts[::-1]:   # 뒤에서부터 보면 "1"을 먼저 발견하기 쉬움
                        if token.lstrip("-").isdigit():
                            final_result = int(token)
                            break
            if p.poll() is not None:
                alive.discard(name)
        time.sleep(0.05)

    # 5) (선택) 카메라 OFF
    # 여기서 중복 종료 문제가 있었다면 주석 처리해도 됨
    # try:
    #     call_rosservice("/stop_camera", env=env)
    # except Exception:
    #     pass

    print("[A_pub] done.")

    # ★★★ 여기서 “숫자만” 딱 찍어준다 → runner가 이 줄만 보면 됨
    print(final_result)

    # 필요하면 exit code는 0으로 두자 (ROS 노드들이 이걸 프로세스 실패로 안 보게)
    sys.exit(0)


if __name__ == "__main__":
    main()
