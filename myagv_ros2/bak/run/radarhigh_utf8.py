# encoding:utf-8
import os
import time
import subprocess
import signal

if os.name == "posix":
    import RPi.GPIO as GPIO

PIN = 20

def radar_high():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(PIN, GPIO.OUT)
    time.sleep(0.1)
    GPIO.output(PIN, GPIO.HIGH)
    print("[INFO] Radar ON")

def radar_low():
    GPIO.output(PIN, GPIO.LOW)
    GPIO.cleanup()
    print("[INFO] Radar OFF")

def launch_ros(file_path):
    """주어진 ROS launch 파일을 새로운 gnome-terminal에서 실행"""
    return subprocess.Popen([
        'gnome-terminal', '--', 'bash', '-c', f'roslaunch {file_path}; exec bash'
    ])

if __name__ == "__main__":
    try:
        radar_high()
        # 실행할 launch 파일 목록
        launch_files = [
            os.path.expanduser("~/myagv_ros/src/myagv_odometry/launch/myagv_active.launch"),
#            os.path.expanduser("~/myagv_ros/src/myagv_teleop/launch/myagv_teleop.launch"),
#            os.path.expanduser("~/myagv_ros/src/myagv_navigation/launch/myagv_slam_laser.launch")
#            os.path.expanduser("/home/er/myagv_ros/src/myagv_navigation/launch/navigation_active.launch")
        ]

        processes = []
        for f in launch_files:
            p = launch_ros(f)
            processes.append(p)
            print(f"[INFO] Launched {f}")
            time.sleep(7)  # 각 launch 실행 사이에 2초 대기

        print("[INFO] All launch files started. Press Ctrl+C to stop.")
        while True:
            time.sleep(1)

    except KeyboardInterrupt:
        print("\n[INFO] Stopping all processes...")
        radar_low()
        for p in processes:
            os.kill(p.pid, signal.SIGTERM)
        print("[INFO] Finished.")
