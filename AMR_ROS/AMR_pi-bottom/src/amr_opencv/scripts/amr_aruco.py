# coding=utf8
import math
import time
import threading
import numpy as np
import amr_opencv.scripts.aruco_detector
from amr_opencv.scripts.aruco_detector import aruco_detector

import rospy
from std_msgs.msg import Int8
from geometry_msgs.msg import Twist

from std_srvs.srv import SetBool, SetBoolResponse

DETECT = False

aruco_detector_res = None
ids = None
_id_get = 0

rospy.init_node("qcode_detect", anonymous=True)
rate = rospy.Rate(30)

pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

time.sleep(1)

States = ["first point ", "first point back", "second point back", "second point"]


def movelittle():
    print("once")


def pub_vel(x, y, theta):
    twist = Twist()
    twist.linear.x = x
    twist.linear.y = y
    twist.linear.z = 0.0
    twist.angular.x = 0.0
    twist.angular.y = 0.0
    twist.angular.z = theta
    pub.publish(twist)


def stop():
    pub_vel(0.0, 0.0, 0.0)


def processMovements(_state, _detect_res, z_target):
    kp = 0.1
    default_x_vel = 0.1

    z_dis = _detect_res[0][2]
    z_differ = z_target - abs(z_dis)
    print(z_dis, z_differ)

    if abs(z_differ) > 5:
        pub_vel(-np.sign(z_differ) * default_x_vel, 0.0, 0.0)
        return _state
    else:
        stop()
        print("state 1 complete")
        time.sleep(1.0)
        return _state + 1


def processY_Movements(_state, _detect_res, dir_val, id_find, _id_get_val):
    kp = 0.05
    default_y_vel = 0.2

    print(id_find, _id_get_val)
    if id_find == _id_get_val:
        cam_y = _detect_res[0][0]

        print("current y dis " + str(cam_y))

        if abs(cam_y) < 2:
            print("stop")
            stop()
            time.sleep(1.0)
            return _state + 1
        else:
            pub_vel(0.0, -np.sign(cam_y) * default_y_vel, 0.0)
            return _state
    else:
        print("searching")
        pub_vel(0.0, dir_val * default_y_vel, 0.0)
        return _state


def processY_small(_state, _detect_res):
    print("process y small")
    default_y_vel = 0.15

    if _detect_res is not None:
        cam_y = _detect_res[0][0]

        print("current y dis " + str(cam_y))

        if abs(cam_y) < 5:
            print("stop")
            stop()
            time.sleep(1.0)
            return _state + 1
        else:
            pub_vel(0.0, -np.sign(cam_y) * default_y_vel, 0.0)
            return _state
    else:
        print("no qr in short dis")
        return _state


def findMovement(_detect_res):
    if _detect_res is None:
        angle_vel = -0.15
        print("reverse")
        pub_vel(0.0, 0.0, angle_vel)
    else:
        print("stop")
        stop()


def faceToQr(_st, _detect_res):
    l_val = _detect_res[0][2]
    theta = _detect_res[0][4]
    theta_rad = theta * math.pi / 180.0
    y_distance = math.sin(theta_rad) * l_val

    kp_y = 1.5
    kp_theta = 0.7

    y_vel = (y_distance / 100.0) * kp_y
    theta_vel = (theta / 100.0) * kp_theta

    if abs(theta) > 8 or abs(y_distance) > 4:
        print((y_distance, theta), (y_vel, theta_vel))
        pub_vel(0.0, -y_vel, theta_vel)
        return _st
    else:
        print("stop face to qr")
        stop()
        time.sleep(1.0)
        return _st + 1


def goPosition(cmd=0):
    global aruco_detector_res, ids, _id_get

    state = 0
    protect_sec = 6.0
    time_count_start = time.time()

    while not rospy.is_shutdown():
        data_rec = aruco_detector.getArucoCode(display_mode=True)
        if data_rec is not None:
            aruco_detector_res, ids = data_rec[0], data_rec[1]
            _id_get = ids[0][0]
            time_count_start = time.time()
            target = 1
        else:
            aruco_detector_res = None
            ids = None
            _id_get = 0

            time_gap = time.time() - time_count_start
            if time_gap > protect_sec:
                print("stop running")
                stop()
                cmd = 10
                break

        if aruco_detector_res is not None:
            if len(aruco_detector_res) == 1:
                pass

        if cmd == 0:
            if aruco_detector_res is not None:
                if len(aruco_detector_res) == 1:
                    state = faceToQr(state, aruco_detector_res)

        elif cmd == 1:
            y_dir = -1
            id_find = 4
            back_dis = 20

            if state == 0:
                print("state 0")
                if aruco_detector_res is not None:
                    state = processMovements(state, aruco_detector_res, back_dis)
                else:
                    pub_vel(-0.1, 0.0, 0.0)
            elif state == 1:
                print("state 1")
                state = processY_Movements(state, aruco_detector_res, y_dir, id_find, _id_get)
            elif state == 2:
                print("state 2")
                if aruco_detector_res is not None:
                    state = faceToQr(state, aruco_detector_res)
            elif state == 3:
                print("state 3")
                if aruco_detector_res is not None:
                    state = processMovements(state, aruco_detector_res, back_dis / 2.0)
            elif state == 4:
                print("state 4")
                state = processY_small(state, aruco_detector_res)
            elif state == 5:
                pub_vel(0.2, 0.0, 0.0)
                time.sleep(3.0)
                state = state + 1
            else:
                stop()
                break

        elif cmd == 2:
            y_dir = 1
            id_find = 1
            back_dis = 20

            if state == 0:
                print("state 0")
                if aruco_detector_res is not None:
                    state = processMovements(state, aruco_detector_res, back_dis)
                else:
                    pub_vel(-0.1, 0.0, 0.0)
            elif state == 1:
                print("state 1")
                state = processY_Movements(state, aruco_detector_res, y_dir, id_find, _id_get)
            elif state == 2:
                print("state 2")
                if aruco_detector_res is not None:
                    state = faceToQr(state, aruco_detector_res)
            elif state == 3:
                print("state 3")
                if aruco_detector_res is not None:
                    state = processMovements(state, aruco_detector_res, back_dis / 2.0)
            elif state == 4:
                print("state 4")
                state = processY_small(state, aruco_detector_res)
            elif state == 5:
                pub_vel(0.2, 0.0, 0.0)
                time.sleep(3.0)
                state = state + 1
            else:
                stop()
                break
        else:
            pub_vel(0.0, 0.0, 0.0)

        rate.sleep()


def rot_once(_dir=1, time_gap_input=0.5, sp=0.9, notIgnoreQR=True):
    pub_vel(0.0, 0.0, sp * _dir)
    time_ini = time.time()
    while not rospy.is_shutdown():
        _time_gap = time.time() - time_ini
        res = aruco_detector.process_qr_data()
        if res != -1 and notIgnoreQR:
            stop()
            return 1
        if _time_gap > time_gap_input:
            stop()
            return 0


def stage_quick_rot(fir_dir=1, first_rot_times=3, second_rot_times=6):
    time_wait = 1.0
    print("Start stage quick rot")

    def rot_dir_times(_dir, _times):
        for i in range(_times):
            res = aruco_detector.process_qr_data()
            if res != -1:
                return 1

            if rot_once(_dir):
                return 1

            rot_once(1, time_wait, 0.0, 0)
        print("Nothing find in this round")
        return 0

    if rot_dir_times(fir_dir, first_rot_times) == 1:
        print("counter clock found")
        return 1
    if rot_dir_times(-fir_dir, second_rot_times) == 1:
        print("clock found")
        return 1
    print("nothing found")
    return 0


def stage_slow_rot(slow_rot_times=6):
    _dir = 1
    sp = 0.5
    time_gap = 0.40

    rot_once(1, 1.0, 0.0, 1)

    for i in range(slow_rot_times):
        res = aruco_detector.process_qr_data()

        if res != -1:
            _perc = res[2]

            if _perc < 0.4:
                _dir = 1
            elif _perc > 0.6:
                _dir = -1
            else:
                print("slow move success")
                stop()
                return 1

            rot_once(_dir, time_gap, sp, notIgnoreQR=False)
        else:
            if rot_once(1, 1.0, 0.0) != 1:
                print("miss the target")
                return -1

        rot_once(1, 1.0, 0.0, 0)

    print("slow focus fail")
    return 0


def front_once(time_gap=0.5, sp=0.32):
    pub_vel(sp, 0.0, 0.0)

    time_ini = time.time()
    while not rospy.is_shutdown():
        _time_gap = time.time() - time_ini
        res = aruco_detector.process_qr_data()
        if _time_gap > time_gap:
            stop()
            return 0


def stages_rot(_dir=1, _first_dir_times=3, _second_dir_times=6):
    if stage_quick_rot(_dir, _first_dir_times, _second_dir_times):
        if stage_slow_rot(9):
            return 1
    return 0


def move_to_center():
    _dir = 1
    center_range = 25
    l_time_ratio = 1.4

    rot_once(1, 1.0, 0.0, 1)

    res = aruco_detector.process_qr_data()

    if res != -1:
        l_val, angle = res[0], res[1]
        print("Step 2 : angle " + str(angle))
        if angle > center_range:
            _dir = -1
        elif angle < -center_range:
            _dir = 1
        else:
            return 1

        rot_once(_dir, time_gap_input=1.0, sp=1.0, notIgnoreQR=False)
        front_once(time_gap=l_val / 100.0 * l_time_ratio, sp=0.5)
        rot_once(-_dir, time_gap_input=0.8, sp=1.0, notIgnoreQR=False)
        if stages_rot(-_dir, 4, 6) == 0:
            return 0

        return 1
    else:
        print("miss target")
        return 0


def main_process(first_dir=1):
    rot_once(1, 3.0, 0.0, 0)

    print("Step 1")
    if stages_rot(first_dir, 2, 5) == 0:
        print("initial found failed")
        return 0

    print("Step 2")
    while not rospy.is_shutdown():
        res = aruco_detector.process_qr_data()
        if res != -1:
            l_val = res[0]
            ag = res[1]
            print("l is " + str(l_val) + " angle is " + str(ag))

            if l_val > 30:
                if stage_slow_rot(6):
                    front_once(2.5, 0.01)
                    continue
                else:
                    stages_rot(1, 2, 4)

            elif 10 < l_val < 30:
                if stage_slow_rot(6):
                    front_once(1.9, 0.01)
                    continue
                else:
                    stages_rot(1, 2, 4)

            elif l_val < 10:
                front_once(0.21, sp=0.01)
                print("Finish doing")
                continue
        else:
            print("Can't detect aruco.")
            break

    rot_once(1, 1.0, 0.0, 0)
    return 1

def handle_go_aruco(req):
    if not req.data:
        rospy.loginfo("[amr_aruco] /go_aruco called with False -> nothing to do")
        return SetBoolResponse(success=False, message="go_aruco False: ignored")

    rospy.loginfo("[amr_aruco] /go_aruco True received. Starting aruco navigation...")

    try:
        result = main_process(first_dir=-1)
        if result:
            msg = "aruco navigation finished successfully"
            rospy.loginfo("[amr_aruco] " + msg)
            return SetBoolResponse(success=True, message=msg)
        else:
            msg = "aruco navigation failed or aborted"
            rospy.logwarn("[amr_aruco] " + msg)
            return SetBoolResponse(success=False, message=msg)
    except Exception as e:
        msg = "exception during aruco navigation: {}".format(e)
        rospy.logerr("[amr_aruco] " + msg)
        return SetBoolResponse(success=False, message=msg)


if __name__ == "__main__":
    try:
        service = rospy.Service("/go_aruco", SetBool, handle_go_aruco)
        rospy.loginfo("[amr_aruco] Service /go_aruco ready.")
        rospy.spin()

    except rospy.exceptions.ROSException:
        print("node already initialized")
