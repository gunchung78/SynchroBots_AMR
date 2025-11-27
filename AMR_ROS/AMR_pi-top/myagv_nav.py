#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
myAGV navigation runner (fire-and-forget)

- 실행 즉시: roslaunch myagv_navigation navigation_active.launch (기본 동작)
- 옵션:
    --start : 명시적으로 시작 (기본 동작과 동일)
    --stop  : 이전에 띄운 navigation 종료(저장된 PID 사용)
- PID 파일: /tmp/myagv_navigation.pid
- 새 터미널 스폰 지원: gnome-terminal/konsole/xfce4-terminal/lxterminal/xterm

※ map.yaml 경로 전달 옵션은 제거했습니다. launch 파일 내부 기본 설정 또는 외부에서 map_server를 사용하세요.
"""

import argparse
import os
import signal
import subprocess
import sys
import time
from pathlib import Path
from shutil import which

LAUNCH_CMD = "roslaunch myagv_navigation navigation_active.launch"
PID_FILE = Path("/tmp/myagv_navigation.pid")

# ----------------------------
# Terminal detection
# ----------------------------

def detect_terminal():
    if which("gnome-terminal"):
        def build(term, title, inner):
            return ["gnome-terminal", "--title", title, "--", "bash", "-lc", inner]
        return "gnome-terminal", build
    if which("konsole"):
        def build(term, title, inner):
            return ["konsole", "-p", f"tabtitle={title}", "-e", "bash", "-lc", inner]
        return "konsole", build
    if which("xfce4-terminal"):
        def build(term, title, inner):
            return ["xfce4-terminal", "--title", title, "--command", f"bash -lc \"{inner}\""]
        return "xfce4-terminal", build
    if which("lxterminal"):
        def build(term, title, inner):
            return ["lxterminal", "-t", title, "-e", "bash", "-lc", inner]
        return "lxterminal", build
    if which("xterm"):
        def build(term, title, inner):
            return ["xterm", "-T", title, "-e", "bash", "-lc", inner]
        return "xterm", build
    if which("x-terminal-emulator"):
        def build(term, title, inner):
            return ["x-terminal-emulator", "-T", title, "-e", "bash", "-lc", inner]
        return "x-terminal-emulator", build
    return None, None

# ----------------------------
# Helpers
# ----------------------------

def spawn_in_new_terminal(cmd: str, pid_path: Path):
    title = "myAGV:navigation"
    inner = f"{cmd} & echo $! > {pid_path}; wait $!; exec bash"
    term, builder = detect_terminal()
    if term is None:
        print("[WARN] No GUI terminal found. Running in background without new window.")
        proc = subprocess.Popen(["bash", "-lc", cmd], start_new_session=True)
        pid_path.write_text(str(proc.pid))
        print(f"[OK] started: '{cmd}' (pid={proc.pid}) pidfile={pid_path}")
        return proc
    argv = builder(term, title, inner)
    try:
        proc = subprocess.Popen(argv, start_new_session=True)
        print(f"[OK] terminal '{term}' spawned for navigation (pid={proc.pid})")
    except Exception as e:
        print(f"[ERR] terminal spawn failed: {e}")
        print("[WARN] Falling back to background run.")
        proc = subprocess.Popen(["bash", "-lc", cmd], start_new_session=True)
        pid_path.write_text(str(proc.pid))
        print(f"[OK] started: '{cmd}' (pid={proc.pid}) pidfile={pid_path}")
        return proc

    # child PID 기록 대기 (최대 5s)
    t0 = time.time()
    while time.time() - t0 < 5.0:
        if pid_path.exists():
            try:
                int(pid_path.read_text().strip())
                print(f"[OK] recorded roslaunch PID in {pid_path}")
                break
            except Exception:
                pass
        time.sleep(0.2)
    return proc

def read_pid(pid_path: Path):
    try:
        return int(pid_path.read_text().strip())
    except Exception:
        return None

def kill_with_grace(pid: int, timeout: float = 5.0):
    if pid is None:
        print("[INFO] navigation: no pid.")
        return
    try:
        os.killpg(pid, signal.SIGINT)
        t0 = time.time()
        while time.time() - t0 < timeout:
            try:
                os.kill(pid, 0)
            except ProcessLookupError:
                print("[OK] navigation: exited on SIGINT.")
                return
            time.sleep(0.2)
        os.killpg(pid, signal.SIGTERM)
        t1 = time.time()
        while time.time() - t1 < timeout:
            try:
                os.kill(pid, 0)
            except ProcessLookupError:
                print("[OK] navigation: exited on SIGTERM.")
                return
            time.sleep(0.2)
        os.killpg(pid, signal.SIGKILL)
        print("[OK] navigation: killed with SIGKILL.")
    except ProcessLookupError:
        print("[OK] navigation: already exited.")
    except PermissionError:
        print("[ERR] navigation: permission error while killing.")

# ----------------------------
# Actions
# ----------------------------

def start_navigation():
    spawn_in_new_terminal(LAUNCH_CMD, PID_FILE)

def stop_navigation():
    pid = read_pid(PID_FILE)
    kill_with_grace(pid)
    try:
        PID_FILE.unlink(missing_ok=True)
    except Exception:
        pass

# ----------------------------
# Main
# ----------------------------

def main():
    parser = argparse.ArgumentParser(
        description="myAGV navigation runner. Default: start navigation."
    )
    grp = parser.add_mutually_exclusive_group()
    grp.add_argument("--start", action="store_true", help="start navigation (default)")
    grp.add_argument("--stop",  action="store_true", help="stop navigation")

    args = parser.parse_args()

    # 기본 동작: 아무 옵션도 없으면 start
    if not args.stop:
        start_navigation()
        return

    # --stop 지정 시
    stop_navigation()

if __name__ == "__main__":
    main()
