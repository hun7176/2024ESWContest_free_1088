#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, roslaunch
import subprocess
import os
import sys
from std_msgs.msg import UInt8, Float64
from sensor_msgs.msg import Image

class Core():
    def __init__(self):
        self.ros_package_path = os.path.dirname(os.path.realpath(__file__))
        self.ros_package_path = self.ros_package_path.replace('bird_autodriving_core/nodes', '')

        # subscribes : status returned
        self.sub_bird_detected = rospy.Subscriber('/bird_detector/image_with_boxes', Image, self.callback, queue_size=10)

        self.launch_bird_autodriving_decide = False
        self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)

        self.bird_detected = False
        self.is_triggered = False

        loop_rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            print("i'm working")
            if self.is_triggered == True:
                self.fnLaunch()
            
            loop_rate.sleep()
        
    def callback(self, data):
        rospy.loginfo("bird is detected")
        self.fnLaunch(self.launch_bird_autodriving_decide, True)

        self.is_triggerd = True

    def fnLaunch(self, launch_num, is_start):
        if launch_num == self.launch_bird_autodriving_decide:
            if is_start == True:    
                if self.bird_detected == False:
                    self.launch_bird_autodriving = roslaunch.scriptapi.ROSLaunch()
                    self.launch_bird_autodriving = roslaunch.parent.ROSLaunchParent(self.uuid, [self.ros_package_path + "bird_autodriving/launch/bird_autodriving.launch"])
                    self.bird_detected = True
                    self.launch_bird_autodriving.start()
            else:
                pass
        else:
            pass

    def main(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('core_node')
    node = Core()
    node.main()
