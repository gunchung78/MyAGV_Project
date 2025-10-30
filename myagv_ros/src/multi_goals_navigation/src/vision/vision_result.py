#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import rospy
from std_msgs.msg import Int32, Bool

class ResultListener:
    def __init__(self):
        self.topic_result = rospy.get_param("~topic", "/yolo_result")
        self.shutdown_topic = rospy.get_param("~shutdown_topic", "/shutdown_signal")
        self.print_on_repeat = bool(rospy.get_param("~print_on_repeat", False))
        self.last_val = None

        rospy.loginfo(f"[A_sub] waiting on topic: {self.topic_result}")
        self.sub = rospy.Subscriber(self.topic_result, Int32, self._cb, queue_size=10)

        self.pub_shutdown = rospy.Publisher(self.shutdown_topic, Bool, queue_size=1)

    def _cb(self, msg: Int32):
        val = int(msg.data)
        if self.print_on_repeat or (self.last_val != val):
            print(val, flush=True)
            self.last_val = val

        # ★ 여기서 “최종 완료” 조건을 네 로직에 맞게 판단하고,
        #   완료 시 shutdown 신호를 보낸다.
        #   (예: 값이 한 번이라도 들어오면 종료시키고 싶다면 아래처럼 바로)
        self._send_shutdown_once()

    def _send_shutdown_once(self):
        # 구독자가 연결될 때까지 잠깐 대기(놓침 방지)
        t0 = time.time()
        while (self.pub_shutdown.get_num_connections() == 0) and (time.time() - t0 < 2.0):
            rospy.sleep(0.05)
        self.pub_shutdown.publish(Bool(data=True))
        rospy.loginfo("[A_sub] published shutdown signal")

def main():
    rospy.init_node("A_sub", anonymous=True)
    ResultListener()
    rospy.loginfo("[A_sub] started. Ctrl+C to exit.")
    rospy.spin()
    rospy.loginfo("[A_sub] stopped.")

if __name__ == "__main__":
    main()
