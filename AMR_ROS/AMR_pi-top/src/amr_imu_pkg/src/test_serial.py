import serial

ser = serial.Serial('/dev/serial0', 115200, timeout=1)
print("OPEN:", ser.is_open)

while True:
    line = ser.readline()
    print("RAW:", line)
