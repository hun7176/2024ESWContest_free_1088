#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import LaserScan

class LidarTestNode:
    def __init__(self):
        rospy.init_node('lidar_test_node')

        # Subscriber
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)

    def scan_callback(self, data):
        angle_increment_degrees = data.angle_increment * 180.0 / 3.141592653589793  # 각도 증분을 도 단위로 변환
        total_angles = int((data.angle_max - data.angle_min) / data.angle_increment)
        angle = data.angle_min
        for i in range(total_angles):
            if i % int(5.0 / angle_increment_degrees) == 0:  # 5도마다 출력
                distance = data.ranges[i]
                rospy.loginfo("Angle: %f degrees, Distance: %f meters", angle * 180.0 / 3.141592653589793, distance)
            angle += data.angle_increment

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    node = LidarTestNode()
    node.run()
