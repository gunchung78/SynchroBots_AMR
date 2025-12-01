import serial
import time

def parse_imu_line(line):
    """
    STM32에서 전송하는 형식:
    H:123.45,R:1.23,P:-4.56
    """
    try:
        line = line.decode().strip()

        # "H:123.45,R:1.23,P:-4.56" → ["H:123.45", "R:1.23", "P:-4.56"]
        parts = line.split(",")

        heading = float(parts[0].split(":")[1])
        roll    = float(parts[1].split(":")[1])
        pitch   = float(parts[2].split(":")[1])

        return heading, roll, pitch

    except Exception as e:
        print("Parse error:", e, "Raw:", line)
        return None


def main():
    # UART 포트 오픈
    # Raspberry Pi 기본 UART = /dev/serial1
    ser = serial.Serial(
            port="/dev/serial0",
        baudrate=115200,
        timeout=1
    )

    print("UART Opened:", ser.is_open)

    while True:
        line = ser.readline()

        if line:
            result = parse_imu_line(line)
            if result:
                heading, roll, pitch = result
                print(f"Heading={heading:.2f}, Roll={roll:.2f}, Pitch={pitch:.2f}")

        # CPU 점유율 방지
        time.sleep(0.01)


if __name__ == "__main__":
    main()