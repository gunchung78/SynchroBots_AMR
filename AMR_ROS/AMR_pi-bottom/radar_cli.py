#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import time
import subprocess

RADAR_PIN = 20
LAUNCH_CMD = "roslaunch myagv_odometry myagv_active.launch"

# Use GPIO only on Raspberry Pi
GPIO = None
if os.name == "posix":
    try:
        import RPi.GPIO as RPiGPIO
        GPIO = RPiGPIO
    except ImportError:
        GPIO = None
        print("[WARN] RPi.GPIO module not found. Running without GPIO control.")


def setup_gpio():
    if GPIO is None:
        return
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(RADAR_PIN, GPIO.OUT)


def radar_on():
    if GPIO is None:
        print("[RADAR] GPIO not used (test mode)")
        return
    setup_gpio()
    time.sleep(0.1)
    GPIO.output(RADAR_PIN, GPIO.HIGH)
    print(f"[RADAR] GPIO {RADAR_PIN} HIGH (radar ON)")


def radar_off():
    if GPIO is None:
        print("[RADAR] GPIO not used (test mode) - only stopping roslaunch.")
        return
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(RADAR_PIN, GPIO.OUT)
    GPIO.output(RADAR_PIN, GPIO.LOW)
    print(f"[RADAR] GPIO {RADAR_PIN} LOW (radar OFF)")
    GPIO.cleanup()


def start_roslaunch_in_terminal():
    """
    Start roslaunch in a new gnome-terminal.
    The terminal will close when roslaunch exits.
    """
    cmd = [
        "gnome-terminal",
        "--",
        "bash",
        "-lc",
        LAUNCH_CMD,  # roslaunch myagv_odometry myagv_active.launch
    ]

    print("[ROS] Starting in new terminal:")
    print("      " + " ".join(cmd))

    try:
        proc = subprocess.Popen(cmd)
    except FileNotFoundError:
        print("[ERROR] gnome-terminal not found.")
        print("       Check if GUI environment and gnome-terminal are available.")
        sys.exit(1)

    # Do not wait here, just return
    return proc


def stop_roslaunch():
    """
    Stop roslaunch process that uses myagv_active.launch.
    This follows the pattern used in the GUI code:
    ps -ef | grep -E myagv_active.launch | grep -v 'grep' | awk '{print $2}' | xargs kill -2
    """
    pattern = "myagv_active.launch"
    cmd = (
        "ps -ef | grep -E " + pattern +
        " | grep -v 'grep' | awk '{print $2}' | xargs -r kill -2"
    )
    print("[ROS] Stopping roslaunch with pattern:", pattern)
    os.system(cmd)


def usage():
    print("Usage:")
    print("  python3 radar.py        # default: radar ON + start roslaunch")
    print("  python3 radar.py on     # radar ON + start roslaunch")
    print("  python3 radar.py off    # stop roslaunch + radar OFF")


def main():
    if len(sys.argv) == 1:
        cmd = "on"
    else:
        cmd = sys.argv[1].lower()

    if cmd == "on":
        print("==========================================")
        print("  myAGV Radar + Odometry CLI Controller  ")
        print("                 [ON]                    ")
        print("==========================================")

        radar_on()
        start_roslaunch_in_terminal()
        print("[MAIN] Command 'on' executed.")
        print("       roslaunch is running in a new terminal.")
        sys.exit(0)

    elif cmd == "off":
        print("==========================================")
        print("  myAGV Radar + Odometry CLI Controller  ")
        print("                 [OFF]                   ")
        print("==========================================")

        # First stop roslaunch, then turn radar off
        stop_roslaunch()
        radar_off()
        print("[MAIN] Command 'off' executed. roslaunch stopped and radar OFF.")
        sys.exit(0)

    else:
        print(f"[ERROR] Unknown command: {cmd}")
        usage()
        sys.exit(1)


if __name__ == "__main__":
    main()
