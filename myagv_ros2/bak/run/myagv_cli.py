#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Terminal CLI for myAGV:
- Start/stop ROS launch processes
- Per-service GPIO hook: set HIGH on start, LOW on stop
- PID files for reliable stop
- Optional pump control (GPIO 2/3)
- Includes joystick (alphabet and number) launchers

ASCII-only source to avoid unicode decode errors.
"""

import argparse
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

# ----------------------------
# Config
# ----------------------------

# Commands for each service
LAUNCH_CMDS = {
    "radar":           "roslaunch myagv_odometry myagv_active.launch",
    "teleop":          "roslaunch myagv_teleop myagv_teleop.launch",
    "gmapping":        "roslaunch myagv_navigation myagv_slam_laser.launch",
    "cartographer":    "roslaunch cartographer_ros demo_myagv.launch",
    "navigation":      "roslaunch myagv_navigation navigation_active.launch",
    "joystick_alpha":  "roslaunch myagv_ps2 myagv_ps2.launch",
    "joystick_number": "roslaunch myagv_ps2 myagv_ps2_number.launch",
}

# PID files per service
PID_DIR = Path("/tmp")
PID_FILES = {name: PID_DIR / f"myagv_{name}.pid" for name in LAUNCH_CMDS.keys()}

# GPIO hooks per service.
# Default: radar uses GPIO 20; others None by default.
# You can override per start/stop call with --gpio-pin option.
GPIO_HOOKS = {
    "radar":           20,
    "teleop":          None,
    "gmapping":        None,
    "cartographer":    None,
    "navigation":      None,
    "joystick_alpha":  None,
    "joystick_number": None,
}

# Pump pins
PUMP_PIN_A = 2
PUMP_PIN_B = 3

# Optional RPi.GPIO
ON_POSIX = (os.name == "posix")
GPIO = None
if ON_POSIX:
    try:
        import RPi.GPIO as _GPIO
        GPIO = _GPIO
        GPIO.setmode(GPIO.BCM)
    except Exception:
        GPIO = None

# ----------------------------
# Helpers
# ----------------------------

def gpio_set(pin: int, high: bool):
    if not GPIO:
        return
    if pin is None:
        return
    try:
        GPIO.setup(pin, GPIO.OUT)
        GPIO.output(pin, GPIO.HIGH if high else GPIO.LOW)
    except Exception:
        pass

def spawn_launch(cmd: str, pid_path: Path):
    """
    Start roslaunch in a new process group, record PID.
    """
    if pid_path.exists():
        print(f"[WARN] pid file exists: {pid_path}")
    proc = subprocess.Popen(["bash", "-lc", cmd], start_new_session=True)
    pid_path.write_text(str(proc.pid))
    print(f"[OK] started: '{cmd}' (pid={proc.pid}) pidfile={pid_path}")
    return proc

def read_pid(pid_path: Path):
    try:
        return int(pid_path.read_text().strip())
    except Exception:
        return None

def kill_with_grace(pid: int, name: str, timeout: float = 5.0):
    """
    Send SIGINT -> wait -> SIGTERM -> wait -> SIGKILL to the process group.
    """
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

def start_service(name: str, gpio_pin_override: int = None):
    if name not in LAUNCH_CMDS:
        print(f"[ERR] unknown service: {name}")
        sys.exit(1)

    pin = gpio_pin_override if gpio_pin_override is not None else GPIO_HOOKS.get(name)
    if pin is not None:
        print(f"[GPIO] {name}: PIN {pin} -> HIGH")
        gpio_set(pin, True)
        time.sleep(0.05)

    spawn_launch(LAUNCH_CMDS[name], PID_FILES[name])

def stop_service(name: str, kill_rviz: bool = False, gpio_pin_override: int = None):
    pid = read_pid(PID_FILES[name])
    kill_with_grace(pid, name)
    try:
        PID_FILES[name].unlink(missing_ok=True)
    except Exception:
        pass

    if kill_rviz:
        # Try to close rviz as well
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
    """
    Save map with map_server.
    Ex: /home/ubuntu/myagv_ros/src/myagv_navigation/map/my_map
    """
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
        GPIO.setup(PUMP_PIN_A, GPIO.OUT)
        GPIO.setup(PUMP_PIN_B, GPIO.OUT)
        def a(level): GPIO.output(PUMP_PIN_A, GPIO.HIGH if level else GPIO.LOW)
        def b(level): GPIO.output(PUMP_PIN_B, GPIO.HIGH if level else GPIO.LOW)

        if action == "on":
            print(f"[GPIO] pump ON (pin {PUMP_PIN_B} HIGH)")
            b(True)
        elif action == "off":
            print(f"[GPIO] pump OFF (pin {PUMP_PIN_B} LOW; toggle pin {PUMP_PIN_A})")
            b(False); time.sleep(0.05)
            a(True);  time.sleep(0.05)
            a(False); time.sleep(0.05)
            a(True);  time.sleep(0.05)
        elif action == "pulse":
            print("[GPIO] pump PULSE (4s on then off)")
            b(True)
            time.sleep(4)
            b(False); time.sleep(0.05)
            a(True);  time.sleep(0.05)
            a(False); time.sleep(0.05)
            a(True);  time.sleep(0.05)
        else:
            print("[ERR] pump action must be one of: on, off, pulse")
            sys.exit(1)
    except Exception as e:
        print(f"[ERR] pump: {e}")
        sys.exit(1)

# ----------------------------
# CLI
# ----------------------------

def build_parser():
    parser = argparse.ArgumentParser(description="myAGV terminal CLI (ROS launch + per-service GPIO hooks)")

    sub = parser.add_subparsers(dest="cmd", required=True)

    # services
    for name in LAUNCH_CMDS.keys():
        sp = sub.add_parser(name, help=f"{name} service control")
        sp_sub = sp.add_subparsers(dest="action", required=True)

        sp_start = sp_sub.add_parser("start", help=f"start {name}")
        sp_start.add_argument("--gpio-pin", type=int, default=None,
                              help="override GPIO pin for this start (set HIGH)")

        sp_stop = sp_sub.add_parser("stop", help=f"stop {name}")
        sp_stop.add_argument("--kill-rviz", action="store_true", help="also kill rviz on stop")
        sp_stop.add_argument("--gpio-pin", type=int, default=None,
                             help="override GPIO pin for this stop (set LOW)")

    # save map
    p_save = sub.add_parser("save-map", help="run map_saver")
    p_save.add_argument("-f", "--prefix", required=True, help="output prefix path without extension")

    # status
    sub.add_parser("status", help="show service states")

    # pump
    p_pump = sub.add_parser("pump", help="pump control on GPIO 2/3")
    p_pump.add_argument("action", choices=["on", "off", "pulse"], help="pump action")

    return parser

def main():
    parser = build_parser()
    args = parser.parse_args()

    try:
        if args.cmd in LAUNCH_CMDS:
            if args.action == "start":
                start_service(args.cmd, gpio_pin_override=args.gpio_pin)
            elif args.action == "stop":
                stop_service(args.cmd, kill_rviz=args.kill_rviz, gpio_pin_override=args.gpio_pin)
        elif args.cmd == "save-map":
            cmd_save_map(args.prefix)
        elif args.cmd == "status":
            cmd_status()
        elif args.cmd == "pump":
            cmd_pump(args.action)
        else:
            parser.print_help()
    finally:
        # No global GPIO cleanup here to keep other services stable.
        pass

if __name__ == "__main__":
    main()

