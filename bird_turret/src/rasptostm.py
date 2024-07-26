#!/usr/bin/env python3
import os
import rospy
from geometry_msgs.msg import Point
import serial
import time

class UART_START:
    def __init__(self):
        rospy.init_node('rasptostm', anonymous=True)
        self.coordinate_sub = rospy.Subscriber('/burd_turret/coordinates', Point, self.callback)

    def callback(self, data):
        try:
            ser = serial.Serial("/dev/ttyAMA2", baudrate=115200, timeout=1)
            while True:
                ser.write(f'X:{data.x},Y:{data.y},Z:{data.z}\n'.encode())
                print(f'Sent coordinates: X={data.x}, Y={data.y}, Z={data.z}')
                time.sleep(1)
        except KeyboardInterrupt:
            print('Interrupted')
            ser.close()

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    my_uart = UART_START()
    my_uart.run()
