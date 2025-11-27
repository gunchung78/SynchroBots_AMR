#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import rospy
from std_msgs.msg import Int32, Bool

class ResultListener:
    def __init__(self):
        self.topic_result   = rospy.get_param("~topic", "/yolo_result")
        self.shutdown_topic = rospy.get_param("~shutdown_topic", "/shutdown_signal")
        self.print_on_repeat = bool(rospy.get_param("~print_on_repeat", False))

        self.last_val = None
        self.sent_shutdown = False   # 한 번만 처리

        rospy.loginfo(f"[A_sub] waiting on topic: {self.topic_result}")
        self.sub = rospy.Subscriber(self.topic_result, Int32, self._cb, queue_size=10)

        self.pub_shutdown = rospy.Publisher(self.shutdown_topic, Bool, queue_size=1)

    def _cb(self, msg: Int32):
        # 이미 처리했으면 무시
        if self.sent_shutdown:
            return

        val = int(msg.data)

        # (옵션) 바뀔 때만 출력
        if self.print_on_repeat or (self.last_val != val):
            # 🔴 여기서 숫자만 출력 → runner가 이 줄만 보면 됨
            print(val, flush=True)
            self.last_val = val

        # shutdown 신호 보내기
        self._send_shutdown_once()

        # 한 번만 처리하도록 플래그
        self.sent_shutdown = True

        # 🔴 종료 직전에 한 번 더 “숫자만” 출력해서
        #    runner가 "맨 마지막 줄"만 봐도 되게 한다.
        print(val, flush=True)

        rospy.loginfo("[A_sub] got result once, shutting down this node...")
        rospy.signal_shutdown("result received")

    def _send_shutdown_once(self):
        # 구독자가 붙을 때까지 잠깐 대기
        t0 = time.time()
        while (self.pub_shutdown.get_num_connections() == 0) and (time.time() - t0 < 2.0):
            rospy.sleep(0.05)
        self.pub_shutdown.publish(Bool(data=True))
        rospy.loginfo("[A_sub] published shutdown signal")

def main():
    rospy.init_node("A_sub", anonymous=True)
    ResultListener()
    rospy.loginfo("[A_sub] started. waiting for first /yolo_result ...")
    rospy.spin()
    rospy.loginfo("[A_sub] stopped.")

if __name__ == "__main__":
    main()
