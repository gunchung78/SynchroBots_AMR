#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import subprocess

LAUNCH_CMD = "roslaunch myagv_navigation navigation_active.launch"


def start_roslaunch_in_terminal():
    """
    Start navigation in a new gnome-terminal.
    The terminal will close automatically when roslaunch exits.
    """
    cmd = [
        "gnome-terminal",
        "--",
        "bash",
        "-lc",
        LAUNCH_CMD,
    ]

    print("[NAV] Starting navigation in new terminal...")
    print("[NAV] Command:")
    print("      " + " ".join(cmd))

    try:
        proc = subprocess.Popen(cmd)
    except FileNotFoundError:
        print("[ERROR] gnome-terminal not found.")
        print("       Check if GUI environment and gnome-terminal are available.")
        sys.exit(1)

    print(f"[NAV] gnome-terminal started with PID {proc.pid}")
    print("[NAV] Navigation should now be running.")
    # We do not wait here so the caller can continue.
    return proc


def stop_roslaunch():
    """
    Stop roslaunch process that uses navigation_active.launch.
    This follows the pattern used in the original GUI code.
    """
    pattern = "navigation_active.launch"
    cmd = (
        "ps -ef | grep -E " + pattern +
        " | grep -v 'grep' | awk '{print $2}' | xargs -r kill -2"
    )

    print("[NAV] Stopping navigation...")
    print("[NAV] Matching pattern:", pattern)
    print("[NAV] Shell command:")
    print("      " + cmd)

    os.system(cmd)
    print("[NAV] Kill signal sent. If roslaunch was running,")
    print("[NAV] the gnome-terminal window should close automatically.")


def usage():
    print("Usage:")
    print("  python3 navigation_cli.py        # default: start navigation (ON)")
    print("  python3 navigation_cli.py on     # start navigation in a new terminal")
    print("  python3 navigation_cli.py off    # stop navigation and close terminal")


def main():
    # Default to "on" when no argument is given
    if len(sys.argv) == 1:
        cmd = "on"
    else:
        cmd = sys.argv[1].lower()

    if cmd == "on":
        print("==============================================")
        print("  myAGV Navigation CLI Controller  [ON]       ")
        print("==============================================")
        start_roslaunch_in_terminal()
        print("[MAIN] Command 'on' completed.")
        print("[MAIN] Navigation is running in a new terminal.")
        sys.exit(0)

    elif cmd == "off":
        print("==============================================")
        print("  myAGV Navigation CLI Controller  [OFF]      ")
        print("==============================================")
        stop_roslaunch()
        print("[MAIN] Command 'off' completed.")
        print("[MAIN] Navigation should now be stopped.")
        sys.exit(0)

    else:
        print(f"[ERROR] Unknown command: {cmd}")
        usage()
        sys.exit(1)


if __name__ == "__main__":
    main()
