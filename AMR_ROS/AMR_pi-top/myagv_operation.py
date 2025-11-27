#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
myAGV terminal controller with:
- CLI subcommands (radar/teleop/gmapping/cartographer/navigation/joystick_alpha/joystick_number/save-map/status/pump)
- Interactive menu (if run without args): type "1 start", "lidar stop", etc.
- Per-service GPIO hook: set HIGH on start, LOW on stop
- New terminal window per service (tries gnome-terminal, konsole, xfce4-terminal, lxterminal, xterm)
- Records real roslaunch PID into /tmp/myagv_<name>.pid
- Navigation can take a map yaml (default: ~/map/map.yaml)
ASCII-only source to avoid unicode issues.
"""

import argparse
import os
import signal
import subprocess
import sys
import time
from pathlib import Path
from shutil import which

# ----------------------------
# Config
# ----------------------------

LAUNCH_CMDS = {
    "radar":           "roslaunch myagv_odometry myagv_active.launch",
    "teleop":          "roslaunch myagv_teleop myagv_teleop.launch",
    "gmapping":        "roslaunch myagv_navigation myagv_slam_laser.launch",
    "cartographer":    "roslaunch cartographer_ros demo_myagv.launch",
    "navigation":      "roslaunch myagv_navigation navigation_active.launch",
    "joystick_alpha":  "roslaunch myagv_ps2 myagv_ps2.launch",
    "joystick_number": "roslaunch myagv_ps2 myagv_ps2_number.launch",
}

PID_DIR = Path("/tmp")
PID_FILES = {name: PID_DIR / f"myagv_{name}.pid" for name in LAUNCH_CMDS.keys()}

GPIO_HOOKS = {
    "radar":           20,
    "teleop":          None,
    "gmapping":        None,
    "cartographer":    None,
    "navigation":      None,
    "joystick_alpha":  None,
    "joystick_number": None,
}

PUMP_PIN_A = 2
PUMP_PIN_B = 3

# Default map for navigation
DEFAULT_MAP = os.path.expanduser("~/map/map.yaml")

ON_POSIX = (os.name == "posix")
GPIO = None
if ON_POSIX:
    try:
        import RPi.GPIO as _GPIO
        GPIO = _GPIO
        GPIO.setmode(GPIO.BCM)
    except Exception:
        GPIO = None

# For interactive menu mapping numbers -> service keys
MENU_ITEMS = [
    ("1", "lidar",            "radar"),
    ("2", "keyboard",         "teleop"),
    ("3", "gmapping",         "gmapping"),
    ("4", "navigation",       "navigation"),
    ("5", "cartographer",     "cartographer"),
    ("6", "joystick_alpha",   "joystick_alpha"),
    ("7", "joystick_number",  "joystick_number"),
    ("8", "save_map",         "save-map"),   # special
    ("9", "status",           "status"),     # special
    ("0", "pump",             "pump"),       # special
]

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

def gpio_set(pin: int, high: bool):
    if not GPIO or pin is None:
        return
    try:
        GPIO.setup(pin, GPIO.OUT)
        GPIO.output(pin, GPIO.HIGH if high else GPIO.LOW)
    except Exception:
        pass

def build_command(name: str, map_path: str = None) -> str:
    """
    Build the final roslaunch command.
    For navigation, append map:=<yaml> (with default if not provided).
    """
    base = LAUNCH_CMDS[name]
    if name == "navigation":
        mp = os.path.expanduser(map_path) if map_path else DEFAULT_MAP
        # Do not fail hard if missing; map_server will error out visibly.
        return f"{base} map:={mp}"
    return base

def spawn_in_new_terminal(service_name: str, cmd: str, pid_path: Path):
    title = f"myAGV:{service_name}"
    inner = f"{cmd} & echo $! > {pid_path}; wait $!; exec bash"
    term, builder = detect_terminal()
    if term is None:
        print("[WARN] No GUI terminal found. Running without new window.")
        proc = subprocess.Popen(["bash", "-lc", cmd], start_new_session=True)
        pid_path.write_text(str(proc.pid))
        print(f"[OK] started: '{cmd}' (pid={proc.pid}) pidfile={pid_path}")
        return proc
    argv = builder(term, title, inner)
    try:
        proc = subprocess.Popen(argv, start_new_session=True)
        print(f"[OK] terminal '{term}' spawned for '{service_name}' (pid={proc.pid})")
    except Exception as e:
        print(f"[ERR] failed to spawn terminal '{term}': {e}")
        print("[WARN] Falling back to direct run without terminal.")
        proc = subprocess.Popen(["bash", "-lc", cmd], start_new_session=True)
        pid_path.write_text(str(proc.pid))
        print(f"[OK] started: '{cmd}' (pid={proc.pid}) pidfile={pid_path}")
        return proc
    # Wait briefly for child pid file
    t0 = time.time()
    while time.time() - t0 < 5.0:
        if pid_path.exists():
            try:
                child = int(pid_path.read_text().strip())
                print(f"[OK] recorded roslaunch PID={child} in {pid_path}")
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

def kill_with_grace(pid: int, name: str, timeout: float = 5.0):
    if pid is None:
        print(f"[INFO] {name}: no pid.")
        return
    try:
        os.killpg(pid, signal.SIGINT)
        t0 = time.time()
        while time.time() - t0 < timeout:
            try:
                os.kill(pid, 0)
            except ProcessLookupError:
                print(f"[OK] {name}: exited on SIGINT.")
                return
            time.sleep(0.2)
        os.killpg(pid, signal.SIGTERM)
        t1 = time.time()
        while time.time() - t1 < timeout:
            try:
                os.kill(pid, 0)
            except ProcessLookupError:
                print(f"[OK] {name}: exited on SIGTERM.")
                return
            time.sleep(0.2)
        os.killpg(pid, signal.SIGKILL)
        print(f"[OK] {name}: killed with SIGKILL.")
    except ProcessLookupError:
        print(f"[OK] {name}: already exited.")
    except PermissionError:
        print(f"[ERR] {name}: permission error while killing.")

# ----------------------------
# Commands
# ----------------------------

def start_service(name: str, gpio_pin_override: int = None, map_path: str = None):
    if name not in LAUNCH_CMDS:
        print(f"[ERR] unknown service: {name}")
        sys.exit(1)
    pin = gpio_pin_override if gpio_pin_override is not None else GPIO_HOOKS.get(name)
    if pin is not None:
        print(f"[GPIO] {name}: PIN {pin} -> HIGH")
        gpio_set(pin, True)
        time.sleep(0.05)
    cmd = build_command(name, map_path=map_path)
    spawn_in_new_terminal(name, cmd, PID_FILES[name])

def stop_service(name: str, kill_rviz: bool = False, gpio_pin_override: int = None):
    pid = read_pid(PID_FILES[name])
    kill_with_grace(pid, name)
    try:
        PID_FILES[name].unlink(missing_ok=True)
    except Exception:
        pass
    if kill_rviz:
        try:
            subprocess.run("pkill -2 rviz", shell=True, check=False)
            time.sleep(0.3)
        except Exception:
            pass
    pin = gpio_pin_override if gpio_pin_override is not None else GPIO_HOOKS.get(name)
    if pin is not None:
        print(f"[GPIO] {name}: PIN {pin} -> LOW")
        gpio_set(pin, False)
        time.sleep(0.05)

def cmd_save_map(prefix: str):
    cmd = f"rosrun map_server map_saver -f {prefix}"
    print(f"[RUN] {cmd}")
    subprocess.run(["bash", "-lc", cmd], check=False)
    print(f"[OK] map saved (prefix={prefix})")

def cmd_status():
    print("=== myAGV services ===")
    for name, pidfile in PID_FILES.items():
        pid = read_pid(pidfile) if pidfile.exists() else None
        alive = False
        if pid:
            try:
                os.kill(pid, 0)
                alive = True
            except ProcessLookupError:
                alive = False
        state = "RUNNING" if alive else "STOPPED"
        has_pidfile = "yes" if pidfile.exists() else "no"
        print(f"- {name:16s}: {state:8s} pidfile={has_pidfile}")

def cmd_pump(action: str):
    if not GPIO:
        print("[ERR] GPIO not available (RPi.GPIO missing or non-RPi env).")
        sys.exit(1)
    try:
        import time as _t
        GPIO.setup(PUMP_PIN_A, GPIO.OUT)
        GPIO.setup(PUMP_PIN_B, GPIO.OUT)
        def a(level): GPIO.output(PUMP_PIN_A, GPIO.HIGH if level else GPIO.LOW)
        def b(level): GPIO.output(PUMP_PIN_B, GPIO.HIGH if level else GPIO.LOW)
        if action == "on":
            print(f"[GPIO] pump ON (pin {PUMP_PIN_B} HIGH)")
            b(True)
        elif action == "off":
            print(f"[GPIO] pump OFF (pin {PUMP_PIN_B} LOW; toggle pin {PUMP_PIN_A})")
            b(False); _t.sleep(0.05)
            a(True);  _t.sleep(0.05)
            a(False); _t.sleep(0.05)
            a(True);  _t.sleep(0.05)
        elif action == "pulse":
            print("[GPIO] pump PULSE (4s on then off)")
            b(True);  _t.sleep(4)
            b(False); _t.sleep(0.05)
            a(True);  _t.sleep(0.05)
            a(False); _t.sleep(0.05)
            a(True);  _t.sleep(0.05)
        else:
            print("[ERR] pump action must be on/off/pulse")
            sys.exit(1)
    except Exception as e:
        print(f"[ERR] pump: {e}")
        sys.exit(1)

# ----------------------------
# CLI parser
# ----------------------------

def build_parser():
    parser = argparse.ArgumentParser(description="myAGV CLI (ROS launch + GPIO + new terminals). If no args: interactive menu.")
    sub = parser.add_subparsers(dest="cmd")

    if sub is not None:
        for name in LAUNCH_CMDS.keys():
            sp = sub.add_parser(name, help=f"{name} service control")
            sp_sub = sp.add_subparsers(dest="action", required=True)
            sp_start = sp_sub.add_parser("start", help=f"start {name}")
            sp_start.add_argument("--gpio-pin", type=int, default=None, help="override GPIO pin (HIGH on start)")
            # navigation only: --map
            if name == "navigation":
                sp_start.add_argument("--map", type=str, default=None,
                                      help=f"path to map.yaml (default: {DEFAULT_MAP})")
            sp_stop = sp_sub.add_parser("stop", help=f"stop {name}")
            sp_stop.add_argument("--kill-rviz", action="store_true", help="also kill rviz")
            sp_stop.add_argument("--gpio-pin", type=int, default=None, help="override GPIO pin (LOW on stop)")

        p_save = sub.add_parser("save-map", help="run map_saver")
        p_save.add_argument("-f", "--prefix", required=True, help="output prefix path (no extension)")

        sub.add_parser("status", help="show service states")

        p_pump = sub.add_parser("pump", help="pump control on GPIO 2/3")
        p_pump.add_argument("action", choices=["on", "off", "pulse"], help="pump action")

        sub.add_parser("interactive", help="start interactive menu")
    return parser

# ----------------------------
# Interactive menu
# ----------------------------

def print_menu():
    print("\n==== myAGV interactive menu ====")
    for key, alias, svc in MENU_ITEMS:
        label = f"{key}. {alias}"
        print(label)
    print("q. quit")
    print("--------------------------------")
    print("Type examples:")
    print("  1 start                      (same as: lidar start)")
    print("  4 start /abs/path/map.yaml   (navigation with map)")
    print("  2 stop                       (keyboard stop)")
    print("  8 save /path/prefix          (save-map)")
    print("  9                            (status)")
    print("  0 on|off|pulse               (pump)")

def resolve_service(token: str):
    t = token.strip().lower()
    for num, alias, svc in MENU_ITEMS:
        if t == num:
            return svc
    for num, alias, svc in MENU_ITEMS:
        if t == alias or t == svc:
            return svc
    if t in ("lidar", "laser", "radar"):
        return "radar"
    if t in ("keyboard", "teleop"):
        return "teleop"
    if t in ("joy", "joystick", "ps2", "joy_alpha"):
        return "joystick_alpha"
    if t in ("joy_num", "joystick_number"):
        return "joystick_number"
    return None

def interactive_loop():
    while True:
        print_menu()
        line = input("cmd> ").strip()
        if not line:
            continue
        if line.lower() in ("q", "quit", "exit"):
            print("bye.")
            return

        parts = line.split()
        if len(parts) == 1:
            svc = resolve_service(parts[0])
            if svc is None:
                print("[ERR] unknown selection.")
                continue
            if svc == "status":
                cmd_status()
            elif svc == "save-map":
                print("[ERR] need: 8 save /path/prefix")
            elif svc == "pump":
                print("[ERR] need: 0 on|off|pulse")
            else:
                print("[ERR] need action: start/stop (e.g., '1 start')")
            continue

        sel = parts[0]
        action = parts[1].lower()
        arg3 = parts[2] if len(parts) >= 3 else None

        svc = resolve_service(sel)
        if svc is None:
            print("[ERR] unknown selection.")
            continue

        if svc in LAUNCH_CMDS:
            if action == "start":
                if svc == "navigation":
                    # allow "4 start /path/to/map.yaml"; default if missing
                    chosen_map = arg3 if arg3 else None
                    start_service(svc, map_path=chosen_map)
                else:
                    start_service(svc)
            elif action == "stop":
                kill_rviz = (arg3 == "--kill-rviz")
                stop_service(svc, kill_rviz=kill_rviz)
            else:
                print("[ERR] action must be start|stop")
        elif svc == "save-map":
            if action not in ("save", "savemap"):
                print("[ERR] usage: 8 save /path/prefix")
                continue
            if not arg3:
                print("[ERR] missing prefix path")
                continue
            cmd_save_map(arg3)
        elif svc == "status":
            cmd_status()
        elif svc == "pump":
            if action not in ("on", "off", "pulse"):
                print("[ERR] usage: 0 on|off|pulse")
                continue
            cmd_pump(action)
        else:
            print("[ERR] unsupported selection.")

# ----------------------------
# Main
# ----------------------------

def main():
    parser = build_parser()
    if len(sys.argv) == 1:
        interactive_loop()
        return

    args = parser.parse_args()

    if args.cmd == "interactive":
        interactive_loop()
        return

    try:
        if args.cmd in LAUNCH_CMDS:
            if args.action == "start":
                if args.cmd == "navigation":
                    start_service(args.cmd,
                                  gpio_pin_override=getattr(args, "gpio_pin", None),
                                  map_path=getattr(args, "map", None))
                else:
                    start_service(args.cmd,
                                  gpio_pin_override=getattr(args, "gpio_pin", None))
            elif args.action == "stop":
                stop_service(args.cmd,
                             kill_rviz=getattr(args, "kill_rviz", False),
                             gpio_pin_override=getattr(args, "gpio_pin", None))
        elif args.cmd == "save-map":
            cmd_save_map(args.prefix)
        elif args.cmd == "status":
            cmd_status()
        elif args.cmd == "pump":
            cmd_pump(args.action)
        else:
            parser.print_help()
    finally:
        pass

if __name__ == "__main__":
    main()
