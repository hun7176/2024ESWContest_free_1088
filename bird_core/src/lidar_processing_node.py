#!/usr/bin/env python

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
        self.front_angle_range = rospy.get_param('~front_angle_range', 30)  # 앞쪽 각도 범위 (디폴트: 30도)
        self.distance_threshold = rospy.get_param('~distance_threshold', 0.5)  # 거리 임계값 (디폴트: 0.5미터)
        self.min_valid_distance = rospy.get_param('~min_valid_distance', 0.1)  # 최소 유효 거리 (디폴트: 0.1미터)

    def scan_callback(self, data):
        # 앞 120도 각도의 인덱스를 계산
        total_angles = int((data.angle_max - data.angle_min) / data.angle_increment)
        start_index = int((total_angles / 2) - (60 / data.angle_increment))
        end_index = int((total_angles / 2) + (60 / data.angle_increment))

        # 유효 거리 값 필터링
        valid_ranges = [dist for dist in data.ranges[start_index:end_index] if dist > self.min_valid_distance]

        # 앞 120도 내에서 장애물 감지
        if valid_ranges:
            min_distance = min(valid_ranges)
            rospy.loginfo("Min distance in front 120 degrees: %f", min_distance)
            if min_distance < self.distance_threshold:  # 임계값 설정
                self.obstacle_pub.publish(Int32(data=1))
                rospy.loginfo("Obstacle detected within %f meters", self.distance_threshold)
            else:
                self.obstacle_pub.publish(Int32(data=0))
                rospy.loginfo("No obstacle within %f meters", self.distance_threshold)
        else:
            self.obstacle_pub.publish(Int32(data=0))
            rospy.loginfo("No valid distance measurements in front 120 degrees")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    node = LidarProcessingNode()
    node.run()