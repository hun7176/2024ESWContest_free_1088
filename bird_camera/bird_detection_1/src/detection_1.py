import rospy
import os
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from cv_bridge import CvBridge, CvBridgeError
import cv2
import tensorflow as tf
import time

class BirdDetection:
    def __init__(self):
        # ROS 노드 초기화
        rospy.init_node('detection_1', anonymous=True)
        
        # CvBridge 객체 생성
        self.bridge = CvBridge()
        
        # 이미지와 트리거 신호를 수신하고 발행할 ROS Publisher와 Subscriber 설정
        self.image_sub = rospy.Subscriber('/usb_cam1/image_raw', Image, self.callback)  
        self.image_pub = rospy.Publisher('/detection_1/image', Image, queue_size=10)
        self.trigger_pub = rospy.Publisher('/detection_1/is_triggered', Int32, queue_size=10)
        
        # 딥러닝 모델 로드
        self.detection_model = self.load_model()
        
        # 주기 설정
        self.frame_interval = 1.0 / 4  # 원하는 FPS (예: 15 FPS)
        self.last_frame_time = time.time()

    def load_model(self):
        """
        모델을 로드합니다.
        """
        script_dir = os.path.dirname(__file__)
        model_dir = os.path.join(script_dir, '..', 'models', 'ssd_mobilenet_v2_fpnlite_320x320_coco17_tpu-8', 'saved_model')
        model_dir = os.path.abspath(model_dir)
        model = tf.saved_model.load(model_dir)
        return model

    def callback(self, data):
        """
        이미지 콜백 함수입니다. 
        객체 감지 후, 결과를 이미지로 출력하고 트리거 신호를 발행합니다.
        """
        try:
            # ROS Image 메시지를 OpenCV 이미지로 변환
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            
            # 현재 시간과 마지막 프레임 시간 비교
            current_time = time.time()
            if (current_time - self.last_frame_time) >= self.frame_interval:
                self.last_frame_time = current_time

                # 이미지 리사이즈
                image_resized = cv2.resize(cv_image, (320, 320), interpolation=cv2.INTER_AREA)
                
                # 텐서 변환
                input_tensor = tf.convert_to_tensor(image_resized)
                input_tensor = input_tensor[tf.newaxis, ...]

                # 객체 감지 수행
                output_dict = self.detection_model(input_tensor)

                # 모델 출력 처리
                num_detections = int(output_dict['num_detections'][0].numpy())
                boxes = output_dict['detection_boxes'][0].numpy()
                class_ids = output_dict['detection_classes'][0].numpy().astype(int)
                scores = output_dict['detection_scores'][0].numpy()

                bird_detected = False
                for i in range(num_detections):
                    if scores[i] > 0.38 and class_ids[i] == 16:  # 'bird' 클래스 확인
                        box = boxes[i]
                        ymin, xmin, ymax, xmax = box
                        left, right, top, bottom = (xmin * cv_image.shape[1], xmax * cv_image.shape[1],
                                                    ymin * cv_image.shape[0], ymax * cv_image.shape[0])
                        
                        # 바운딩 박스와 점수 그리기
                        cv_image = cv2.rectangle(cv_image, (int(left), int(top)), (int(right), int(bottom)), (255, 0, 0), 2)
                        score_text = f"Score: {scores[i]:.2f}"
                        cv_image = cv2.putText(cv_image, score_text, (int(left), int(top) - 10), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                        cv_image = cv2.putText(cv_image, "Detect!", (int(left), int(top) - 30), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                        
                        bird_detected = True

                # 트리거 신호 발행
                self.trigger_pub.publish(1 if bird_detected else 0)

                # 이미지 ROS 메시지로 변환 후 발행
                ros_image = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
                self.image_pub.publish(ros_image)

        except CvBridgeError as e:
            rospy.logerr(f"CvBridge Error: {e}")
        except Exception as e:
            rospy.logerr(f"Callback 예외: {e}")

    def run(self):
        """
        ROS 노드 실행
        """
        rospy.spin()

if __name__ == '__main__':
    detector = BirdDetection()
    detector.run()

