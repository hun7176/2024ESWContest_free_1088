import os
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from cv_bridge import CvBridge
import cv2
import numpy as np
import tensorflow as tf
import time

class BirdDetection:
    def __init__(self):
        rospy.init_node('detection_1', anonymous=True)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/usb_cam/image_raw', Image, self.callback)
        self.image_pub = rospy.Publisher('/detection_1/image_with_boxes', Image, queue_size=10)
        self.trigger_pub = rospy.Publisher('/detection_1/is_triggered', Int32, queue_size=10)
        self.detection_model = self.load_model()

    def load_model(self):
        script_dir = os.path.dirname(__file__)
        model_dir = os.path.join(script_dir, '..', 'models', 'ssd_mobilenet_v2_fpnlite_320x320_coco17_tpu-8', 'saved_model')
        model_dir = os.path.abspath(model_dir)
        model = tf.saved_model.load(model_dir)
        return model

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
            image_np = np.array(cv_image)
            input_tensor = tf.convert_to_tensor(image_np)
            input_tensor = input_tensor[tf.newaxis, ...]

            start_time = time.time()
            output_dict = self.detection_model(input_tensor)
            num_detections = int(output_dict['num_detections'][0])
            boxes = output_dict['detection_boxes'][0].numpy()
            class_ids = output_dict['detection_classes'][0].numpy().astype(int)
            scores = output_dict['detection_scores'][0].numpy()

            bird_detected = False
            for i in range(num_detections):
                if scores[i] > 0.5:
                    box = boxes[i]
                    ymin, xmin, ymax, xmax = box
                    left, right, top, bottom = (xmin * cv_image.shape[1], xmax * cv_image.shape[1],
                                                ymin * cv_image.shape[0], ymax * cv_image.shape[0])
                    cv2.rectangle(cv_image, (int(left), int(top)), (int(right), int(bottom)), (255, 0, 0), 2)
                    bird_detected = True
                    break  # 새가 감지되면 루프를 종료

            if bird_detected:
                self.trigger_pub.publish(1)

            end_time = time.time()
            elapsed_time = end_time - start_time
            rospy.loginfo(f"Detection and trigger signal processing time: {elapsed_time:.6f} seconds")

            image_message = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
            self.image_pub.publish(image_message)

        except Exception as e:
            rospy.logerr(f"Exception in callback: {e}")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    detector = BirdDetection()
    detector.run()
