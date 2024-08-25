#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import rospy
import roslaunch
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

        # Base path
        base_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../'))

        # Relative Launch Management
        self.detection_1_launch_file = os.path.join(base_path, 'bird_camera/bird_detection_1/launch/bird_detection_1.launch')
        self.detection_2_launch_file = os.path.join(base_path, 'bird_turret/bird_detection_2/launch/bird_detection_2.launch')

        self.detection_1_launch = None
        self.detection_2_launch = None

        # Start detection 1
        self.start_detection_1()
        self.start_detection_2() #이거

    def start_detection_1(self):
        rospy.loginfo("Starting detection 1")
        self.detection_1_launch = roslaunch.parent.ROSLaunchParent(rospy.get_param("/run_id"), [self.detection_1_launch_file])
        self.detection_1_launch.start()

    def start_detection_2(self):
        rospy.loginfo("Starting detection 2")
        self.detection_2_launch = roslaunch.parent.ROSLaunchParent(rospy.get_param("/run_id"), [self.detection_2_launch_file])
        self.detection_2_launch.start()

    def stop_detection_1(self):
        if self.detection_1_launch is not None:
            rospy.loginfo("Stopping detection 1")
            self.detection_1_launch.shutdown()
            self.detection_1_launch = None

    def stop_detection_2(self):
        if self.detection_2_launch is not None:
            rospy.loginfo("Stopping detection 2")
            self.detection_2_launch.shutdown()
            self.detection_2_launch = None

    def shooting_done_callback(self, msg):
        self.shooting_done = True
        self.current_mode = 'driving'
        #self.stop_detection_2()  # Stop detection 2 when shooting is done
        #self.start_detection_1()  # Restart detection 1
        rospy.loginfo("Shooting done: %s, mode: %s", self.shooting_done, self.current_mode)

    def lidar_trigger_callback(self, msg):
        if self.current_mode == 'shooting':
            #rospy.loginfo("현재 shooting 모드이므로 라이다 트리거를 무시합니다")
            return  # shooting 모드에서는 아무 것도 하지 않음

        if msg.data == 1:  # 장애물 감지
            self.current_mode = 'obstacle'
        elif msg.data == 0:  # 장애물 제거됨
            self.current_mode = 'driving'

    def detect_callback(self, msg):
        if msg.data == 1:
            self.current_mode = 'shooting'
            self.shooting_mode_pub.publish(Int32(data=1))  # 트리거 발행
            #self.stop_detection_1()  # Stop detection 1 before starting detection 2
            #self.start_detection_2()

    def main_loop(self):
        while not rospy.is_shutdown():
            if self.current_mode == 'driving':
                self.twist.linear.x = 0.3  # 직진 #0.3
                self.twist.angular.z = 0.0
                rospy.loginfo("Current mode: driving")
            elif self.current_mode == 'obstacle':
                self.twist.linear.x = 0.0
                self.twist.angular.z = -1.0  # 회전하여 장애물 회피
                self.cmd_pub.publish(self.twist)
                rospy.loginfo("Current mode: obstacle, turning")
                # 장애물이 없어질 때까지 회전
                while self.current_mode == 'obstacle':
                    self.cmd_pub.publish(self.twist)
                    self.rate.sleep()
                rospy.loginfo("Obstacle cleared, switching to driving mode")
                continue  # 장애물이 없어지면 다음 루프로 넘어감
            elif self.current_mode == 'shooting':
                self.twist.linear.x = 0.15
                self.twist.angular.z = 0.0
                rospy.loginfo("Current mode: shooting, waiting for shooting to complete")
                while not self.shooting_done:
                    self.cmd_pub.publish(self.twist)  # 정지 명령을 계속 유지
                    self.rate.sleep()
                self.shooting_done = False

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
