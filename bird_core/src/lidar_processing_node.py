#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
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
        self.distance_threshold = rospy.get_param('~distance_threshold', 0.6)  # 거리 임계값 (디폴트: 0.5미터)
        self.min_valid_distance = rospy.get_param('~min_valid_distance', 0.3)  # 최소 유효 거리 (디폴트: 0.1미터)

        # Fixed angles for detection
        self.start_angle = 300  # 시작 각도
        self.end_angle = 350    # 끝 각도

    # 수정 후
    def scan_callback(self, data):
        total_angles = int((data.angle_max - data.angle_min) / data.angle_increment)
        center_index = total_angles // 2

        start_index = int(center_index - (self.front_angle_range / 2) / data.angle_increment)
        end_index = int(center_index + (self.front_angle_range / 2) / data.angle_increment)

        # 범위 초과 방지
        start_index = max(0, start_index)
        end_index = min(len(data.ranges) - 1, end_index)

        # 유효 거리 값 필터링 (inf, NaN 값 필터링)
        valid_ranges = [dist for dist in data.ranges[start_index:end_index] if self.min_valid_distance < dist < float('inf')]

        if valid_ranges:
            min_distance = min(valid_ranges)
            rospy.loginfo("Min distance in front 120 degrees: %f", min_distance)
            if min_distance < self.distance_threshold:
                self.obstacle_pub.publish(Int32(data=1))
                rospy.loginfo("Obstacle detected within %f meters", self.distance_threshold)
            else:
                self.obstacle_pub.publish(Int32(data=0))
                rospy.loginfo("No obstacle within %f meters", self.distance_threshold)
        else:
            self.obstacle_pub.publish(Int32(data=0))
            rospy.loginfo("No valid distance measurements in front 120 degrees")



""" 수정 전
    def scan_callback(self, data):
        # 전체 각도의 인덱스를 계산
        total_angles = int((data.angle_max - data.angle_min) / data.angle_increment)
        angle_increment_degrees = data.angle_increment * (180 / 3.141592653589793)

        # 300도와 350도의 인덱스를 계산
        start_index = int((self.start_angle * (3.141592653589793 / 180) - data.angle_min) / data.angle_increment)
        end_index = int((self.end_angle * (3.141592653589793 / 180) - data.angle_min) / data.angle_increment)

        # 유효 거리 값 필터링
        valid_ranges = [dist for dist in data.ranges[start_index:end_index] if dist > self.min_valid_distance]

        # 지정 각도 내에서 장애물 감지
        if valid_ranges:
            min_distance = min(valid_ranges)
            rospy.loginfo("Min distance in 300-350 degrees: %f", min_distance)
            if min_distance < self.distance_threshold:  # 임계값 설정
                self.obstacle_pub.publish(Int32(data=1))
                rospy.loginfo("Obstacle detected within %f meters", self.distance_threshold)
            else:
                self.obstacle_pub.publish(Int32(data=0))
                rospy.loginfo("No obstacle within %f meters", self.distance_threshold)
        else:
            self.obstacle_pub.publish(Int32(data=0))
<<<<<<< HEAD
            rospy.loginfo("No valid distance measurements in 300-350 degrees")

    def run(self):
=======
            rospy.loginfo("No valid distance measurements in front 120 degrees")
"""
def run(self):


if __name__ == '__main__':
    node = LidarProcessingNode()
    node.run()
