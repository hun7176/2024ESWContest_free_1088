import os
import time  # Added import for time
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import tensorflow as tf

class BirdDetector:
    def __init__(self):
        # ROS 노드 초기화
        rospy.init_node('detection_2', anonymous=True)

        # CvBridge 객체 생성
        self.bridge = CvBridge()

        # 카메라 이미지 구독
        self.image_sub = rospy.Subscriber('/usb_cam2/image_raw', Image, self.callback)
        self.image_pub = rospy.Publisher('/bird_detection_2/image_with_boxes', Image, queue_size=10)

        # PID 제어 결과 퍼블리셔
        self.angle_pub = rospy.Publisher('/bird_detection_2/angles', Vector3, queue_size=10)

        # TensorFlow 모델 로드
        self.detection_model = self.load_model()

        # PID 제어 변수 초기화
        self.prev_error_x = 0.0
        self.prev_error_y = 0.0
        self.integral_x = 0.0
        self.integral_y = 0.0

        # PID 제어 상수
        self.kp = 0.1
        self.ki = 0.01
        self.kd = 0.05

        # 중심과의 거리가 얼마 이내일 때 색을 변경할지 설정
        self.proximity_threshold = 50  # 픽셀 단위 거리
        
        # 주기 설정
        self.frame_interval = 1.0 / 15  # 15fps
        self.last_frame_time = time.time()  # 초기화 추가

    def load_model(self):
        # 모델 파일 경로 설정
        script_dir = os.path.dirname(__file__)  # 현재 스크립트가 위치한 디렉토리
        model_dir = os.path.join(script_dir, '..', 'models', 'ssd_mobilenet_v2_fpnlite_320x320_coco17_tpu-8', 'saved_model')
        model_dir = os.path.abspath(model_dir)
        rospy.loginfo(f"Loading model from {model_dir}")
        model = tf.saved_model.load(model_dir)
        return model

    def callback(self, data):
        try:
            # ROS 이미지 메시지를 OpenCV 이미지로 변환
            cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')

            # 현재 시간과 마지막 프레임 시간 비교
            current_time = time.time()
            if (current_time - self.last_frame_time) >= self.frame_interval:
                self.last_frame_time = current_time

                # 이미지 크기 조정 (모델 입력 크기)
                image_resized = cv2.resize(cv_image, (320, 320))
                input_tensor = tf.convert_to_tensor(image_resized)
                input_tensor = input_tensor[tf.newaxis, ...]

                # 객체 감지 수행
                output_dict = self.detection_model(input_tensor)

                # 결과 해석
                num_detections = int(output_dict['num_detections'][0].numpy())
                boxes = output_dict['detection_boxes'][0].numpy()
                class_ids = output_dict['detection_classes'][0].numpy().astype(int)
                scores = output_dict['detection_scores'][0].numpy()

                # 새의 클래스 ID (예: COCO 데이터셋에서는 16)
                bird_class_id = 16

                # 중심점 계산
                image_center_x = cv_image.shape[1] // 2
                image_center_y = cv_image.shape[0] // 2

                detected = False
                cross_color = (255, 255, 255)  # 기본 색상은 흰색
                show_shoot_text = False  # shoot 텍스트 표시 여부
                angle_msg = Vector3()  # 초기화

                # 이미지에서 감지된 새를 그리기 및 정보 추가
                for i in range(num_detections):
                    if scores[i] > 0.2 and class_ids[i] == bird_class_id:  # 감지 신뢰도와 클래스 ID 기준
                        box = boxes[i]
                        (ymin, xmin, ymax, xmax) = box
                        (left, right, top, bottom) = (xmin * cv_image.shape[1], xmax * cv_image.shape[1],
                                                       ymin * cv_image.shape[0], ymax * cv_image.shape[0])
                        cv2.rectangle(cv_image, (int(left), int(top)), (int(right), int(bottom)), (255, 0, 0), 2)
                        
                        # 중심점 계산
                        center_x = int((left + right) / 2)
                        center_y = int((top + bottom) / 2)
                        
                        # 텍스트 추가
                        text = f'ID: {class_ids[i]}, Score: {scores[i]:.2f}'
                        cv_image = cv2.putText(cv_image, text, (int(left), int(top) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                        
                        # 중심점 에러 계산 및 콘솔 출력
                        error_x = center_x - image_center_x
                        error_y = center_y - image_center_y
                        rospy.loginfo(f"Error X: {error_x}, Error Y: {error_y}")

                        # 에러 값을 이미지에 표시
                        error_text = f'Error X: {error_x}, Error Y: {error_y}'
                        cv_image = cv2.putText(cv_image, error_text, (10, cv_image.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

                        # 에러 값을 퍼블리시하기 전에 정수형으로 변환
                        angle_msg.x = int(error_x)
                        angle_msg.y = int(error_y)

                        # 물체가 중심에 가까운지 확인
                        if abs(error_x) < self.proximity_threshold and abs(error_y) < self.proximity_threshold:
                            cross_color = (0, 0, 255)  # 빨간색으로 변경
                            show_shoot_text = True  # shoot 텍스트 표시
                            angle_msg.z = 1  # z값을 1로 설정 (정수형)
                            rospy.loginfo("shoot!!!")
                        else:
                            angle_msg.z = 0  # z값을 0으로 설정 (정수형)

                        detected = True
                        break

                if not detected:
                    # 객체가 감지되지 않은 경우 제어 신호를 0으로 설정
                    angle_msg.x = 0
                    angle_msg.y = 0
                    angle_msg.z = 0

                self.angle_pub.publish(angle_msg)

                # 중심에 흰색 또는 빨간색 십자 그리기
                cv_image = cv2.line(cv_image, (image_center_x - 50, image_center_y), (image_center_x + 50, image_center_y), cross_color, 2)
                cv_image = cv2.line(cv_image, (image_center_x, image_center_y - 50), (image_center_x, image_center_y + 50), cross_color, 2)

                # shoot 텍스트 표시
                if show_shoot_text:
                    cv_image = cv2.putText(cv_image, 'shoot!!', (image_center_x - 100, image_center_y - 60), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 255), 4)

                # 이미지를 ROS 이미지 메시지로 변환하여 발행
                image_message = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
                self.image_pub.publish(image_message)

        except CvBridgeError as e:
            rospy.logerr(f"CvBridge Error: {e}")
        except Exception as e:
            rospy.logerr(f"Exception in callback: {e}")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    detector = BirdDetector()
    detector.run()
