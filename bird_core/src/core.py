#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist

class AutonomousVehicleNode:
    def __init__(self):
        rospy.init_node('core')

        # State
        self.current_mode = 'driving'
        self.shooting_done = False

        # Publishers and Subscribers
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.shooting_done_sub = rospy.Subscriber('/shooting_done', Int32, self.shooting_done_callback)
        self.lidar_trigger_sub = rospy.Subscriber('/lidar_trigger', Int32, self.lidar_trigger_callback)
        self.detect_sub = rospy.Subscriber('/detection_1/is_triggered', Int32, self.detect_callback)
        self.shooting_mode_pub = rospy.Publisher('/shooting_mode_trigger', Int32, queue_size=10)

        # Main loop rate
        self.rate = rospy.Rate(10)

        # Twist message for controlling the vehicle
        self.twist = Twist()

    def shooting_done_callback(self, msg):
        self.shooting_done = bool(msg.data)
        if self.shooting_done:
            self.current_mode = 'driving'
        rospy.loginfo("Shooting done: %s, mode: %s", self.shooting_done, self.current_mode)

    def lidar_trigger_callback(self, msg):
        if self.current_mode == 'driving' and msg.data == 1:
            self.current_mode = 'obstacle'
        rospy.loginfo("Lidar trigger received, mode: %s", self.current_mode)

    def detect_callback(self, msg):
        if msg.data == 1:
            self.current_mode = 'shooting'
            self.shooting_mode_pub.publish(Int32(data=1))  # 트리거 발행
        rospy.loginfo("Detection triggered, mode: %s", self.current_mode)

    def main_loop(self):
        while not rospy.is_shutdown():
            if self.current_mode == 'driving':
                self.twist.linear.x = 0.1  # 직진
                self.twist.angular.z = 0.0
                rospy.loginfo("Current mode: driving")
            elif self.current_mode == 'obstacle':
                self.twist.linear.x = 0.0
                self.twist.angular.z = 1.0  # 회전하여 장애물 회피
                self.cmd_pub.publish(self.twist)
                rospy.loginfo("Current mode: obstacle, turning")
                self.current_mode = 'driving'
                rospy.sleep(1)
                continue  # 다음 루프를 바로 실행하여 정지 상태로 가지 않도록 함
            elif self.current_mode == 'shooting':
                self.twist.linear.x = 0.0  # 정지
                self.twist.angular.z = 0.0
                rospy.loginfo("Current mode: shooting, waiting for shooting to complete")
                while not self.shooting_done:
                    self.cmd_pub.publish(self.twist)  # 정지 명령을 계속 유지
                    self.rate.sleep()

            self.cmd_pub.publish(self.twist)
            self.rate.sleep()

    def run(self):
        self.main_loop()

if __name__ == '__main__':
    node = AutonomousVehicleNode()
    try:
        node.run()
    except rospy.ROSInterruptException:
        pass
