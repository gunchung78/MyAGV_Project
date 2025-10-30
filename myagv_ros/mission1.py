#!/usr/bin/env python3

"""
myAGV navigation runner (fire-and-forget) + multi-goals (after 5s)

- Default / --start :
    1) roslaunch myagv_navigation navigation_active.launch
    2) 5 seconds later -> roslaunch multi_goals_navigation multi_goals_navigation.launch
- --stop :
    stop both processes (navigation + multi_goals)
- PID files:
    /tmp/myagv_navigation.pid
    /tmp/myagv_multi_goals.pid
- New terminal spawn supported: gnome-terminal / konsole / xfce4-terminal / lxterminal / xterm
"""

import argparse
import os
import signal
import subprocess
import sys
import time
from pathlib import Path
from shutil import which

LAUNCH_NAV   = "roslaunch myagv_navigation navigation_active.launch"
LAUNCH_MULTI = "roslaunch multi_goals_navigation multi_goals_navigation.launch"

PID_NAV   = Path("/tmp/myagv_navigation.pid")
PID_MULTI = Path("/tmp/myagv_multi_goals.pid")

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

def spawn_in_new_terminal(cmd: str, pid_path: Path, title: str):
    inner = f"{cmd} & echo $! > {pid_path}; wait $!; exec bash"
    term, builder = detect_terminal()
    if term is None:
        print(f"[WARN] No GUI terminal found for '{title}'. Running in background.")
        proc = subprocess.Popen(["bash", "-lc", cmd], start_new_session=True)
        try:
            pid_path.write_text(str(proc.pid))
        except Exception:
            pass
        print(f"[OK] started: '{cmd}' (pid={proc.pid}) pidfile={pid_path}")
        return proc
    argv = builder(term, title, inner)
    try:
        proc = subprocess.Popen(argv, start_new_session=True)
        print(f"[OK] terminal '{term}' spawned for '{title}' (pid={proc.pid})")
    except Exception as e:
        print(f"[ERR] terminal spawn failed for '{title}': {e}")
        print("[WARN] Falling back to background run.")
        proc = subprocess.Popen(["bash", "-lc", cmd], start_new_session=True)
        try:
            pid_path.write_text(str(proc.pid))
        except Exception:
            pass
        print(f"[OK] started: '{cmd}' (pid={proc.pid}) pidfile={pid_path}")
        return proc

    # wait for roslaunch PID to be recorded
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

def kill_with_grace(pid: int, name: str, timeout: float = 5.0):
    if pid is None:
        print(f"[INFO] {name}: no pid.")
        return
    try:
        # try SIGINT, then SIGTERM, then SIGKILL
        for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
            try:
                os.kill(pid, sig)
            except ProcessLookupError:
                print(f"[OK] {name}: already exited.")
                return
            if sig != signal.SIGKILL:
                t0 = time.time()
                while time.time() - t0 < timeout:
                    try:
                        os.kill(pid, 0)
                    except ProcessLookupError:
                        print(f"[OK] {name}: exited on {sig.name}.")
                        return
                    time.sleep(0.2)
        print(f"[OK] {name}: killed with SIGKILL.")
    except PermissionError:
        print(f"[ERR] {name}: permission error while killing.")
    except ProcessLookupError:
        print(f"[OK] {name}: already exited.")

# ----------------------------
# Actions
# ----------------------------

def start_both():
    # 1) Navigation first
    spawn_in_new_terminal(LAUNCH_NAV, PID_NAV, "myAGV:navigation")

    # 2) Wait 5s then multi-goals
    print("[INFO] waiting 5s before starting multi_goals_navigation...")
    time.sleep(5.0)
    spawn_in_new_terminal(LAUNCH_MULTI, PID_MULTI, "myAGV:multi_goals")

def stop_both():
    # stop multi-goals first
    kill_with_grace(read_pid(PID_MULTI), "multi_goals_navigation")
    try:
        PID_MULTI.unlink(missing_ok=True)
    except Exception:
        pass

    # then navigation
    kill_with_grace(read_pid(PID_NAV), "navigation")
    try:
        PID_NAV.unlink(missing_ok=True)
    except Exception:
        pass

# ----------------------------
# Main
# ----------------------------

def main():
    parser = argparse.ArgumentParser(
        description="myAGV navigation runner (+multi_goals after 5s). Default: start both."
    )
    grp = parser.add_mutually_exclusive_group()
    grp.add_argument("--start", action="store_true", help="start navigation, then multi_goals after 5s (default)")
    grp.add_argument("--stop",  action="store_true", help="stop both (navigation + multi_goals)")

    args = parser.parse_args()

    if args.stop:
        stop_both()
    else:
        start_both()

if __name__ == "__main__":
    main()

