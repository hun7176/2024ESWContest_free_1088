#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import math
from std_msgs.msg import Int32
from sensor_msgs.msg import LaserScan

class LidarProcessingNode:
    def __init__(self):
        rospy.init_node('lidar_processing_node')

        # Publisher
        self.obstacle_pub = rospy.Publisher('/lidar_trigger', Int32, queue_size=10)

        # Subscriber
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)

        # Parameters
        self.distance_threshold = rospy.get_param('~distance_threshold', 0.3)  # 거리 임계값 (디폴트: 0.5미터)
        self.min_valid_distance = rospy.get_param('~min_valid_distance', 0.15)  # 최소 유효 거리 (디폴트: 0.3미터)

        # Fixed angles for detection (in radians)
        self.front_angle_min = 0.0  # 0도 (라디안)
        self.front_angle_max = math.radians(15)  # 15도 (라디안)
        self.rear_angle_min = math.radians(345)  # 345도 (라디안)
        self.rear_angle_max = math.radians(360)  # 360도 (라디안)

    def scan_callback(self, data):
        valid_ranges = []

        for i, distance in enumerate(data.ranges):
            angle = data.angle_min + i * data.angle_increment
            
            # 0~15도 또는 345~360도 범위 내의 각도에서만 데이터 처리
            if ((self.front_angle_min <= angle <= self.front_angle_max) or
                (self.rear_angle_min <= angle <= self.rear_angle_max)):

                if self.min_valid_distance < distance < float('inf') and not math.isnan(distance):
                    valid_ranges.append((distance, angle))

        if valid_ranges:
            # 최소 거리와 그에 해당하는 각도 찾기
            min_distance, min_angle = min(valid_ranges, key=lambda x: x[0])

            rospy.loginfo("Min distance within specified angles: %f at angle: %f radians",
                          min_distance, min_angle)
            
            if min_distance < self.distance_threshold:
                self.obstacle_pub.publish(Int32(data=1))
                rospy.loginfo("Obstacle detected within %f meters at angle: %f radians", 
                              self.distance_threshold, min_angle)
            else:
                self.obstacle_pub.publish(Int32(data=0))
                rospy.loginfo("No obstacle within %f meters", self.distance_threshold)
        else:
            self.obstacle_pub.publish(Int32(data=0))
            rospy.loginfo("No valid distance measurements within specified angles")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    node = LidarProcessingNode()
    node.run()
