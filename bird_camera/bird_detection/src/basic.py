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

            # 이미지에서 감지된 객체를 그리기
            for i in range(num_detections):
                if scores[i] > 0.5:  # 감지 신뢰도 기준
                    box = boxes[i]
                    (ymin, xmin, ymax, xmax) = box
                    (left, right, top, bottom) = (xmin * cv_image.shape[1], xmax * cv_image.shape[1],
                                                   ymin * cv_image.shape[0], ymax * cv_image.shape[0])
                    cv2.rectangle(cv_image, (int(left), int(top)), (int(right), int(bottom)), (255, 0, 0), 2)

            # 결과 이미지 표시
            cv2.imshow("Bird Detection", cv_image)
            cv2.waitKey(3)
        except Exception as e:
            rospy.logerr(f"Exception in callback: {e}")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    detector = BirdDetector()
    detector.run()
