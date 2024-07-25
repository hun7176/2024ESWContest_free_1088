import serial
import time
try:
    ser = serial.Serial("/dev/ttyAMA2", baudrate=115200, timeout=1)
    while True:
        ser.write(b'A')
        print("toggle LD1")
        time.sleep(1)
        ser.write(b'B')
        print("toggle LD2")
        time.sleep(1)
        ser.write(b'C')
        print("toggle LD3")
        time.sleep(5)
except KeyboardInterrupt:
    print('')
    ser.close()
