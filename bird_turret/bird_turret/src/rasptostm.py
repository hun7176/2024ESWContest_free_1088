#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Point
import serial

class UART_START:
    def __init__(self):
        rospy.init_node('rasptostm', anonymous=True)
        self.coordinate_sub = rospy.Subscriber('/burd_turret/coordinates', Point, self.callback)
        self.ser = serial.Serial("/dev/ttyACM0", baudrate=115200, timeout=1)
        rospy.on_shutdown(self.cleanup)

    def callback(self, data):
        try:
            self.ser.write(f'X:{data.x},Y:{data.y},Z:{data.z}\n'.encode())
            rospy.loginfo(f'Sent coordinates: X={data.x}, Y={data.y}, Z={data.z}')
        except Exception as e:
            rospy.logerr(f'Error writing to serial port: {e}')

    def cleanup(self):
        self.ser.close()
        rospy.loginfo("Serial port closed.")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    my_uart = UART_START()
    my_uart.run()

