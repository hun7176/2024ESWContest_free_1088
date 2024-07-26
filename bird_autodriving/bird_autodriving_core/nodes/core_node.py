#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, roslaunch
import subprocess
import os
import sys
from enum import Enum
from std_msgs.msg import UInt8, Float64
from sensor_msgs.msg import Image

class Core():
    def __init__(self):
        # 패키지 경로 현재 경로로 설정
        self.ros_package_path = os.path.dirname(os.path.realpath(__file__))
        self.ros_package_path = self.ros_package_path.replace('bird_autodriving_core/nodes', '')

        # subscribes : status returned
        self.sub_mode_control = rospy.Subscriber('/core/decided_mode', UInt8, self.cbReceiveMode, queue_size=1)
        self.sub_obstacle_stamped = rospy.Subscriber('/detect/obstacle_stamped', UInt8, self.cbObstacleStamped, queue_size=1)
        self.sub_road_stamped = rospy.Subscriber('/detect/road_stamped', UInt8, self.cbRoadStamped, queue_size=1)
        
        # publishes orders
        self.pub_obstacle_order = rospy.Publisher('/detect/obstacle_order', UInt8, queue_size=1)
        self.pub_road_order = rospy.Publisher('/detect/road_order', UInt8, queue_size=1)
        self.pub_mode_return = rospy.Publisher('/core/returned_mode', UInt8, queue_size=1)

        # 열거형 정의 (현재 모드, 활주로 단계, 장애물 단계, 런처)
        self.CurrentMode = Enum('CurrentMode', 'idle autodrive_mode shooting_mode obstale_mode')
        self.current_mode = self.CurrentMode.idle.value

        self.StepOfRoad = Enum('StepOfRoad', 'avoid_road exit')
        self.current_step_road = self.StepOfRoad.exit.value

        self.StepOfObstacle = Enum('StepOfObstacle', 'avoid_obstacle exit')
        self.current_step_road = self.StepOfObstacle.exit.value        

        self.Launcher = Enum('Launcher', 'launch_camera_ex_calib launch_detect_road launch_detect_obstacle launch_control_straight launch_control_avoid launch_control_pause' )
        self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)

        # launch status 초기화
        self.launch_camera_launched = False
        self.launch_detect_road_launched = False
        self.launch_detect_obstacle_launched = False
        self.launch_control_straight_launched = False
        self.launch_control_avoid_launched = False
        self.launch_control_pause_launched = False

        # Ros 노드 동작 여부 초기화
        self.is_triggered = False

        # 노드가 종료될 때까지 초당 10회 루프하며 노드 컨트롤
        loop_rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            print("i'm working")
            if self.is_triggered == True:
                self.fnControlNode()
            
            loop_rate.sleep()

    # 받은 데이터로 현재 모드 변경(decide_mode 노드에서 기본적으로 autodriving_mode라고 보내주거나 Current_mode를 autodriving_mode로 설정해야 될 것 같음)
    # 아니면 triggered가 안됐을 때(장애물o, 활주로o) 안 움직일 수 있음
    def cbReceiveMode(self, mode_msg):
        rospy.loginfo("mode is decided")

        self.current_mode = mode_msg.data
        self.is_triggered = True

    # 장애물 모드 변경 / exit면 자율주행 모드로 변경, 현재 모드(자율주행) 상태 재전파
    def cbObstacleStamped(self, obstacle_msg):  
        rospy.loginfo("obstacle Step changed from %d", self.current_step_obstacle)
        self.current_step_obstacle = obstacle_msg.data
        
        if self.current_step_obstacle == self.StepOfObstacle.exit.value:
            self.current_mode = self.CurrentMode.autodrive_mode.value
            msg_mode_return = UInt8()
            msg_mode_return.data = self.current_mode
            self.pub_mode_return.publish(msg_mode_return)
        
        self.is_triggered = True

    # 활주로 모드 변경 / exit면 자율주행 모드로 변경, 현재 모드(자율주행) 상태 재전파
    def cbRoadStamped(self, road_msg):  
        rospy.loginfo("road Step changed from %d", self.current_step_road)
        self.current_step_road = road_msg.data

        if self.current_step_road == self.StepOfRoad.exit.value:
            self.current_mode = self.CurrentMode.autodrive_mode.value
            msg_mode_return = UInt8()
            msg_mode_return.data = self.current_mode
            self.pub_mode_return.publish(msg_mode_return)
        
        self.is_triggered = True

    # 현재 모드에 따른 런치파일 실행 및 종료
    def fnControlNode(self):
        # autodrive_mode 
        if self.current_mode == self.CurrentMode.autodrive_mode.value:
            rospy.loginfo("New trigger for autodrive_mode")
            self.fnLaunch(self.Launcher.launch_camera_ex_calib.value, True)

            self.fnLaunch(self.Launcher.launch_detect_road.value, True)
            self.fnLaunch(self.Launcher.launch_detect_obstacle.value, True)

            self.fnLaunch(self.Launcher.launch_control_straight.value, True)

        # shooting_mode
        elif self.current_mode == self.CurrentMode.shooting_mode.value:
            rospy.loginfo("New trigger for shooting_mode")
            msg_pub_intersection_order = UInt8()



            if self.current_step_intersection == self.StepOfIntersection.detect_direction.value:
                rospy.loginfo("Current step : searching_intersection_sign")
                rospy.loginfo("Go to next step : exit")

                msg_pub_intersection_order.data = self.StepOfIntersection.detect_direction.value

                self.fnLaunch(self.Launcher.launch_detect_lane.value, True)
                self.fnLaunch(self.Launcher.launch_detect_sign.value, True)
                self.fnLaunch(self.Launcher.launch_detect_intersection.value, True)

                self.fnLaunch(self.Launcher.launch_control_lane.value, True)
                self.fnLaunch(self.Launcher.launch_control_moving.value, True)
              
            elif self.current_step_intersection == self.StepOfIntersection.exit.value:
                rospy.loginfo("Current step : exit")

                msg_pub_intersection_order.data = self.StepOfIntersection.exit.value

                self.fnLaunch(self.Launcher.launch_detect_lane.value, True)
                self.fnLaunch(self.Launcher.launch_detect_sign.value, True)
                self.fnLaunch(self.Launcher.launch_detect_intersection.value, False)
               
                self.fnLaunch(self.Launcher.launch_control_lane.value, True)
                self.fnLaunch(self.Launcher.launch_control_moving.value, False)

            rospy.sleep(3)
            self.pub_intersection_order.publish(msg_pub_intersection_order)

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
