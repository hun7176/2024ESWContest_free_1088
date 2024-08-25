#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Point
import serial
from std_msgs.msg import Int32  # 1바이트 데이터를 위한 메시지 타입

class UART_START:
    def __init__(self):
        rospy.init_node('rasptostm', anonymous=True)
        
        # 구독자 및 발행자 설정
        self.coordinate_sub = rospy.Subscriber('/bird_detection_2/angles', Point, self.callback)
        self.shooting_done_pub = rospy.Publisher('/shooting_done', Int32, queue_size=10)
        
        # 시리얼 포트 설정
        self.ser = serial.Serial("/dev/ttyUSB1", baudrate=115200, timeout=1)
        rospy.on_shutdown(self.cleanup)
        
        # 주기적으로 UART를 읽어오는 타이머 설정
        self.rx_thread = rospy.Timer(rospy.Duration(0.1), self.read_uart)

    def callback(self, data):
        try:
            data.x=data.x/320*127
            data.y=data.y/240*127 
            # X, Y, Z 값을 1바이트로 변환
            x = int(data.x).to_bytes(1, 'big', signed=True)
            y = int(data.y).to_bytes(1, 'big', signed=True)
            z = int(data.z).to_bytes(1, 'big', signed=True)
            
            # x, y, z가 모두 0이 아닐 경우에만 전송
            if x != b'\x00' or y != b'\x00' or z != b'\x00':
                # z, x, y, 1 순서로 전송
                self.ser.write(z)
                self.ser.write(x)
                self.ser.write(y)
                self.ser.write(b'\x02')  # 1을 추가로 전송
                
                rospy.loginfo(f'좌표를 int8로 전송함: Z={z[0]}, X={x[0]}, Y={y[0]}, 상수=1')
            else:
                rospy.loginfo("좌표가 (0, 0, 0)입니다. 데이터가 전송되지 않았습니다.")
        except Exception as e:
            rospy.logerr(f'시리얼 포트로 전송 중 오류 발생: {e}')

    def read_uart(self, event):
        try:
            if self.ser.in_waiting > 0:
                # 1바이트의 데이터 읽기
                byte_data = self.ser.read(1)
                byte_value = ord(byte_data)  # 바이트를 정수로 변환
                
                # 수신된 데이터를 /shooting_done 토픽으로 발행
                self.shooting_done_pub.publish(byte_value)
                
                rospy.loginfo(f'수신된 바이트를 /shooting_done으로 발행함: {byte_value}')
        except Exception as e:
            rospy.logerr(f'시리얼 포트에서 읽는 중 오류 발생: {e}')

    def cleanup(self):
        self.ser.close()
        rospy.loginfo("시리얼 포트가 닫혔습니다.")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    my_uart = UART_START()
    my_uart.run()
