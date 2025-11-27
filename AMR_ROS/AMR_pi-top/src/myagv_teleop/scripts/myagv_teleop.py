#!/usr/bin/env python3
from __future__ import print_function

import threading
import sys
from select import select

# catkin 기반이면 roslib.load_manifest는 보통 불필요합니다.
# import roslib; roslib.load_manifest('myagv_teleop')

import rospy
from geometry_msgs.msg import Twist, TwistStamped

if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty

TwistMsg = Twist

msg = """
Reading from the keyboard and Publishing to Twist!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

anything else : stop

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

CTRL-C to quit
"""

moveBindings = {
    'i': (1, 0, 0, 0),
    'o': (0, 0, 0, -1),
    'j': (0, 1, 0, 0),
    'l': (0, -1, 0, 0),
    'u': (0, 0, 0, 1),
    ',': (-1, 0, 0, 0),
    '.': (-1, 0, 0, -1),
    'm': (-1, 0, 0, 1),
    'O': (0, 0, 0, -1),
    'I': (1, 0, 0, 0),
    'J': (0, 1, 0, 0),
    'L': (0, -1, 0, 0),
    'U': (0, 0, 0, 1),
    '<': (-1, 0, 0, 0),
    '>': (-1, 0, 0, -1),
    'M': (-1, 0, 0, 1),
}

speedBindings = {
    'q': (1.1, 1.1),
    'z': (0.9, 0.9),
    'w': (1.1, 1.0),
    'x': (0.9, 1.0),
    'e': (1.0, 1.1),
    'c': (1.0, 0.9),
}

class PublishThread(threading.Thread):
    def __init__(self, rate):
        super(PublishThread, self).__init__()
        self.publisher = rospy.Publisher('cmd_vel', TwistMsg, queue_size=1)
        self.x = self.y = self.z = self.th = 0.0
        self.speed = self.turn = 0.0
        self.condition = threading.Condition()
        self.done = False
        self.timeout = (1.0 / rate) if rate != 0.0 else None
        self.start()

    def wait_for_subscribers(self):
        i = 0
        while not rospy.is_shutdown() and self.publisher.get_num_connections() == 0:
            if i == 4:
                print("Waiting for subscriber to connect to {}".format(self.publisher.name))
            rospy.sleep(0.5)
            i = (i + 1) % 5
        if rospy.is_shutdown():
            raise Exception("Got shutdown request before subscribers connected")

    def update(self, x, y, z, th, speed, turn):
        with self.condition:
            self.x, self.y, self.z, self.th = x, y, z, th
            self.speed, self.turn = speed, turn
            self.condition.notify()

    def stop(self):
        self.done = True
        self.update(0, 0, 0, 0, 0, 0)
        self.join()

    def run(self):
        twist_msg = TwistMsg()
        twist = twist_msg if not stamped else twist_msg.twist

        if stamped:
            twist_msg.header.stamp = rospy.Time.now()
            twist_msg.header.frame_id = twist_frame

        while not self.done:
            if stamped:
                twist_msg.header.stamp = rospy.Time.now()

            with self.condition:
                self.condition.wait(self.timeout)
                twist.linear.x = self.x * self.speed
                twist.linear.y = self.y * self.speed
                twist.linear.z = self.z * self.speed
                twist.angular.x = 0.0
                twist.angular.y = 0.0
                twist.angular.z = self.th * self.turn

            self.publisher.publish(twist_msg)

        # stop on exit
        twist.linear.x = twist.linear.y = twist.linear.z = 0.0
        twist.angular.x = twist.angular.y = twist.angular.z = 0.0
        self.publisher.publish(twist_msg)

def getKey(settings, timeout):
    if sys.platform == 'win32':
        return msvcrt.getwch()
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select([sys.stdin], [], [], timeout)
    key = sys.stdin.read(1) if rlist else ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def saveTerminalSettings():
    if sys.platform == 'win32':
        return None
    return termios.tcgetattr(sys.stdin)

def restoreTerminalSettings(old_settings):
    if sys.platform == 'win32':
        return
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

def vels(speed, turn):
    return "currently:\tspeed %s\tturn %s " % (speed, turn)

if __name__ == "__main__":
    settings = saveTerminalSettings()
    rospy.init_node('teleop_twist_keyboard')

    speed = rospy.get_param("~speed", 0.5)
    turn = rospy.get_param("~turn", 0.5)
    speed_limit = rospy.get_param("~speed_limit", 1.0)
    turn_limit  = rospy.get_param("~turn_limit", 1.0)
    repeat      = rospy.get_param("~repeat_rate", 0.0)
    key_timeout = rospy.get_param("~key_timeout", 0.52)
    stamped     = rospy.get_param("~stamped", False)
    twist_frame = rospy.get_param("~frame_id", '')

    if stamped:
        TwistMsg = TwistStamped

    pub_thread = PublishThread(repeat)
    x = y = z = th = 0
    status = 0

    try:
        pub_thread.wait_for_subscribers()
        pub_thread.update(x, y, z, th, speed, turn)

        print(msg)
        print(vels(speed, turn))
        while True:
            key = getKey(settings, key_timeout)
            if key in moveBindings:
                x, y, z, th = moveBindings[key]
            elif key in speedBindings:
                speed = min(speed_limit, speed * speedBindings[key][0])
                turn  = min(turn_limit,  turn  * speedBindings[key][1])
                if speed == speed_limit:
                    print("Linear speed limit reached!")
                if turn == turn_limit:
                    print("Angular speed limit reached!")
                print(vels(speed, turn))
                if status == 14:
                    print(msg)
                status = (status + 1) % 15
            else:
                if key == '' and x == 0 and y == 0 and z == 0 and th == 0:
                    continue
                x = y = z = th = 0
                if key == '\x03':  # CTRL-C
                    break
            pub_thread.update(x, y, z, th, speed, turn)
    except Exception as e:
        print(e)
    finally:
        pub_thread.stop()
        restoreTerminalSettings(settings)

