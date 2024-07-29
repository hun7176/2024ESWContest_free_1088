#!/usr/bin/env python
# BEGIN ALL
import rospy
import os
import cv2
import cv_bridge
import numpy
import math
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from traffic_light_classifier.msg import traffic_light




global perr, ptime, serr, dt, move, ray_angle, green_cnt, ang_zt1, ang_zt2
perr = 0 #1
ptime = 0
serr = 0
dt = 0 #1
move = False
angle_step_deg = 20
green_cnt = 0
ang_zt1 = 0.0
ang_zt2 = 0.0

class Follower:
        def __init__(self):
            self.bridge = cv_bridge.CvBridge()
            self.image_sub = rospy.Subscriber('/usb_cam/image_raw', Image, self.image_callback)
            self.lidar_sub = rospy.Subscriber('/scan_raw', LaserScan, self.lidar_callback)
            self.traffic_sub = rospy.Subscriber('/light_color', traffic_light, self.traffic_callback)
            self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
            self.image_pub = rospy.Publisher('/lane_image', Image, queue_size=1)
            self.twist = Twist()
            self.ray_angle = [x for x in range(angle_step_deg, 180, angle_step_deg)]
            self.dists = None
            self.traffic_color = 0
            self.cmd_vel_pub.publish(self.twist)

        def traffic_callback(self, msg):
                global move, green_cnt
                self.traffic_color = msg.recognition_result

                if self.traffic_color == 1:
                    green_cnt += 1
                    #print(green_cnt)

        def lidar_callback(self, msg):
                # static offset
                angles = [x for x in range(-5, -90, -5)] #-5, -90, -5
                self.dists = [msg.ranges[x*2] for x in angles]

        def get_obstacle_threshold(self):
		#print(self.dist)
		#print()
                if self.dists == None:
                        return 0

                lateral_count = 0
                for d in self.dists:
                        if d<0.55 : #0.55
                                lateral_count += 1
                        #return 40 #70
                if lateral_count >= 1 :
			print("lateral_cnt : {}".format(lateral_count))
                        return 80
		else:
			print("lateral_cnt : {}".format(lateral_count))
                        return 0

        def image_callback(self, msg):
                global perr, ptime, serr, dt, ang_zt1, ang_zt2
                image0 = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

                # transformation
                img = cv2.resize(image0, None, fx=0.6, fy=0.6,
                                                 interpolation=cv2.INTER_CUBIC)
                #print img.shape
                rows, cols, ch = img.shape
                pts1 = numpy.float32([[10,80],[0,130],[180,80],[190,130]]) #[[15,60],[5,130],[180,60],[190,130]]
                pts2 = numpy.float32([[0, 0], [0, 300], [300, 0], [300, 300]])

                M = cv2.getPerspectiveTransform(pts1, pts2)
                img_size = (img.shape[1], img.shape[0])
                image = cv2.warpPerspective(img, M, (300, 300))  # img_size

                hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)


                #lower_red = numpy.array([0,100,100])
                #upper_red = numpy.array([10,255,255])
                #lower_red2 = numpy.array([160,100,100])
                #upper_red2 = numpy.array([180,255,255])

                lower_red = numpy.array([-10,50,100])
                upper_red = numpy.array([10,255,255])
                lower_red2 = numpy.array([160,50,100])
                upper_red2 = numpy.array([179,255,255])

                #lower_red = numpy.array([0,40,175])
                #upper_red = numpy.array([20,200,255])
                #lower_red2 = numpy.array([110,0,200])
                #upper_red2 = numpy.array([180,160,255])

                maskr = cv2.inRange(hsv, lower_red, upper_red)
                maskr2 = cv2.inRange(hsv, lower_red2, upper_red2)
                maskr4 = cv2.bitwise_or(maskr2, maskr)

                # filter mask
                kernel = numpy.ones((7, 7), numpy.uint8)
		opening = cv2.morphologyEx(maskr4, cv2.MORPH_OPEN, kernel)
		rgb_yb2 = cv2.morphologyEx(opening, cv2.MORPH_CLOSE, kernel)
                #_, rgb_yb2 = cv2.threshold(rgb_yb2, 210, 255, cv2.THRESH_BINARY)

                # out_img = cv2.morphologyEx(opening, cv2.MORPH_CLOSE, kernel)

                # ROI
                out_img = rgb_yb2.copy()
                h, w = out_img.shape
                search_top = int(1*h/4)#+20)
                search_bot = int(4*h/4)
                search_mid = int(w/2)
                out_img[0:search_top, 0:w] = 0
                out_img[search_bot:h, 0:w] = 0
                M = cv2.moments(out_img)
                c_time = rospy.Time.now()
		
		
                #if green_cnt >= 2:
                if green_cnt >= 1:
                        if M['m00'] > 0:
                                cxm = int(M['m10']/M['m00'])
                                cym = int(M['m01']/M['m00'])

                                offset = self.get_obstacle_threshold()
                                #print(offset)

                                #original cx
                                cx = cxm -60 -offset
                                #if offset == 0:
                                #    cx = cxm - 45
                                #else:
                                #    cx = cxm - offset - 45

                                cv2.circle(out_img, (cxm, cym), 20, (255, 0, 0), -1)
                                cv2.circle(out_img, (cx, cym), 20, (255, 0, 0), 2)

                        # BEGIN CONTROL
                                err = cx - 4*w/8

                                dt = rospy.get_time() - ptime

                                #ang_z = (float(err) / 100)*(0.15) + \
                                #        ((err - perr)/(rospy.get_time() - ptime))*1/25/126
                                #ang_z = min(0.8, max(-0.8, ang_z))

                                ang_z = (float(err) / 100)*0.1 + ((err - perr)/(rospy.get_time() - ptime))*1/20/100
				lin_x = ang_z
				
				ang_z = (float(err) / 100)*0.2 + ((err - perr)/(rospy.get_time() - ptime))*1/20/100
                                #print(ang_z)
				if ang_z > 0:
					ang_z+=0.2
				ang_z = min(0.8, max(-0.8, ang_z))
			

                                #K_p = 0.5


                                if offset != 0 and abs(ang_z) > 0.1 :
                                       K_p = 0.8
                                else:
                                        #K_p = 2.3
                                        K_p = 1.0


			
                                #
                                if lin_x < 0:
                                        lin_x = -(lin_x)
                                lin_x = K_p * (1-lin_x)

				if(abs(ang_z) >= 0.001):
					ang_zt1 = ang_z
				else:
					ang_zt1 = 0.0



            			self.twist.linear.x = lin_x  
			
            			self.twist.angular.z = -ang_z

            			#print("cx:{}, cxm:{}, err:{:.4f}, ang_z:{:4f}, lin_x:{:4f}".format(
            #cx, cxm, err*0.0015, ang_z, lin_x))
            			serr = err + serr
            			perr = err
            			ptime = rospy.get_time()

         		else:
            			self.twist.linear.x = 0.1#1.57


				if ang_zt1 < 0:
					ang_zt1 += (-0.01)
				else:
					ang_zt1 += 0.01

				ang_zt1 = min(0.8, max(-0.8, ang_zt1))



            			self.twist.angular.z = -ang_zt1 #-0.3
				#print(str(ang_zt1)+"\n")
            			err = 0

                self.cmd_vel_pub.publish(self.twist)
                #output_img = self.bridge.cv2_to_imgmsg(out_img)
                #self.image_pub.publish(output_img)

      # END CONTROL



rospy.init_node('follower')
follower = Follower()
rospy.spin()
# END ALL
