#!/usr/bin/env python3
import os
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import tensorflow as tf

class BirdDetector:
    def __init__(self):
        # ROS 노드 초기화
        rospy.init_node('bird_detector', anonymous=True)

        # CvBridge 객체 생성
        self.bridge = CvBridge()

        # 카메라 이미지 구독(수정 필요)
        self.image_sub = rospy.Subscriber('/usb_cam/image_raw', Image, self.callback)
        self.image_pub = rospy.Publisher('/bird_detector/image_with_boxes', Image, queue_size=10)

        # TensorFlow 모델 로드
        self.detection_model = self.load_model()

    def load_model(self):
        # 모델 파일 경로 설정(수정 필요)
        script_dir = os.path.dirname(__file__)  # 현재 스크립트가 위치한 디렉토리
        model_dir = os.path.join(script_dir, '..', 'models', 'ssd_mobilenet_v2_fpnlite_320x320_coco17_tpu-8', 'saved_model')
        model_dir = os.path.abspath(model_dir)
        model = tf.saved_model.load(model_dir)
        return model

    def callback(self, data):
        try:
            # ROS 이미지 메시지를 OpenCV 이미지로 변환
            cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')

            # TensorFlow 모델을 사용한 객체 감지
            image_np = np.array(cv_image)
            input_tensor = tf.convert_to_tensor(image_np)
            input_tensor = input_tensor[tf.newaxis, ...]

            # 객체 감지 수행
            output_dict = self.detection_model(input_tensor)

            # 결과 해석
            num_detections = int(output_dict['num_detections'][0])
            boxes = output_dict['detection_boxes'][0].numpy()
            class_ids = output_dict['detection_classes'][0].numpy().astype(int)
            scores = output_dict['detection_scores'][0].numpy()

            # 새의 클래스 ID (예: COCO 데이터셋에서는 16)
            bird_class_id = 16

            # 이미지에서 감지된 새를 그리기 및 정보 추가
            for i in range(num_detections):
                if scores[i] > 0.5 and class_ids[i] == bird_class_id:  # 감지 신뢰도와 클래스 ID 기준
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
                    cv2.putText(cv_image, text, (int(left), int(top) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                    
                    # 깊이 이미지에서 거리 추출 (깊이 이미지가 있을 경우)
                    # distance = depth_image[center_y, center_x]
                    # cv2.putText(cv_image, f'{distance:.2f}m', (center_x, center_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

            image_message = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
            self.image_pub.publish(image_message)

        except Exception as e:
            rospy.logerr(f"Exception in callback: {e}")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    detector = BirdDetector()
    detector.run()
