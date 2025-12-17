#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import threading
import time

import rospy
import serial
from amr_msg.srv import SetMcuValue, SetMcuValueResponse


class McuSerialBridge:
    def __init__(self):
        self.port = rospy.get_param("~port", "/dev/ttyUSB0")
        self.baud = int(rospy.get_param("~baud", 115200))
        self.timeout = float(rospy.get_param("~timeout", 1.0))
        self.write_newline = bool(rospy.get_param("~write_newline", True))
        self.reconnect_on_fail = bool(rospy.get_param("~reconnect_on_fail", True))
        self.reconnect_delay = float(rospy.get_param("~reconnect_delay", 0.5))

        self._lock = threading.Lock()
        self._ser = None
        self._connect()

        self._srv = rospy.Service("/amr_mcu", SetMcuValue, self._handle_service)
        rospy.loginfo("[amr_mcu] Service /amr_mcu ready (serial=%s @ %d)", self.port, self.baud)

    def _connect(self):
        with self._lock:
            if self._ser and self._ser.is_open:
                return True
            try:
                self._ser = serial.Serial(
                    port=self.port,
                    baudrate=self.baud,
                    timeout=self.timeout,
                    write_timeout=self.timeout,
                )
                time.sleep(0.2)  # allow Arduino reset on serial open
                try:
                    self._ser.reset_input_buffer()
                    self._ser.reset_output_buffer()
                except Exception:
                    pass
                rospy.loginfo("[amr_mcu] Connected to %s", self.port)
                return True
            except Exception as e:
                rospy.logerr("[amr_mcu] Serial connect failed: %s", str(e))
                self._ser = None
                return False

    def _ensure_connected(self):
        if self._ser and self._ser.is_open:
            return True
        if not self.reconnect_on_fail:
            return False
        return self._connect()

    def _send_value(self, value: int):
        payload = f"{value}\n" if self.write_newline else f"{value}"
        data = payload.encode("ascii", errors="ignore")

        with self._lock:
            if not self._ensure_connected():
                return False, "serial not connected"

            try:
                self._ser.write(data)
                self._ser.flush()
                return True, f"sent '{payload.strip()}'"
            except Exception as e:
                rospy.logerr("[amr_mcu] Serial write failed: %s", str(e))
                try:
                    if self._ser:
                        self._ser.close()
                except Exception:
                    pass
                self._ser = None

                if self.reconnect_on_fail:
                    time.sleep(self.reconnect_delay)
                    if self._connect():
                        try:
                            self._ser.write(data)
                            self._ser.flush()
                            return True, f"sent '{payload.strip()}' (after reconnect)"
                        except Exception as e2:
                            return False, f"write failed after reconnect: {e2}"

                return False, f"write failed: {e}"

    def _handle_service(self, req):
        value = int(req.value)

        if value not in (0, 1, 2):
            return SetMcuValueResponse(False, "value must be 0, 1, or 2")

        ok, msg = self._send_value(value)
        return SetMcuValueResponse(ok, msg)

    def shutdown(self):
        with self._lock:
            try:
                if self._ser and self._ser.is_open:
                    self._ser.close()
            except Exception:
                pass
        rospy.loginfo("[amr_mcu] Shutdown complete")


def main():
    rospy.init_node("amr_mcu_serial_node", anonymous=False)
    node = McuSerialBridge()
    rospy.on_shutdown(node.shutdown)
    rospy.spin()


if __name__ == "__main__":
    main()
