#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

import serial
import time


def parse_imu_line(line):
    """
    Expected format:
    H:123.45,R:1.23,P:-4.56
    """
    try:
        line = line.decode(errors="ignore").strip()

        parts = line.split(",")

        heading = float(parts[0].split(":")[1])
        roll    = float(parts[1].split(":")[1])
        pitch   = float(parts[2].split(":")[1])

        return heading, roll, pitch

    except Exception as e:
        print("Parse error:", e, "Raw:", line)
        return None


class IMUPublisher(Node):
    def __init__(self):
        super().__init__('imu_publisher')

        # Publisher ¼±¾ð
        self.pub = self.create_publisher(Float32MultiArray, 'imu_data', 10)

        # UART Open
        self.ser = serial.Serial(
                port='/dev/serial0',
                baudrate=115200,
                timeout=1
        )
        self.get_logger().info(f"UART opened: {self.ser.is_open}")

        # Timer: 100Hz ¡æ 0.01s
        self.timer = self.create_timer(0.01, self.loop)

    def loop(self):
        line = self.ser.readline()

        if line:
            result = parse_imu_line(line)

            if result:
                heading, roll, pitch = result

                msg = Float32MultiArray()
                msg.data = [heading, roll, pitch]

                self.pub.publish(msg)

                self.get_logger().info(
                    f"Published IMU: H={heading:.2f}, R={roll:.2f}, P={pitch:.2f}"
                )


def main(args=None):
    rclpy.init(args=args)
    node = IMUPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
