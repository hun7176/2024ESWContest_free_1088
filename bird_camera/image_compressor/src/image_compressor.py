import rospy
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2

class ImageCompressor:
    def __init__(self):
        rospy.init_node('image_compressor', anonymous=True)  
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/usb_cam/image_raw', Image, self.image_callback)
        self.image_pub = rospy.Publisher('/usb_cam/image_raw/compressed', CompressedImage, queue_size=10)

    def image_callback(self, data):
        try:
            # ROS 이미지 메시지를 OpenCV 이미지로 변환
            cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
            
            # OpenCV 이미지를 압축
            _, compressed_image = cv2.imencode('.jpg', cv_image, [cv2.IMWRITE_JPEG_QUALITY, 90])
            
            # 압축된 이미지를 ROS CompressedImage 메시지로 변환
            compressed_msg = CompressedImage()
            compressed_msg.header = data.header
            compressed_msg.format = "jpeg"
            compressed_msg.data = compressed_image.tobytes()

            # 압축된 이미지 메시지를 발행
            self.image_pub.publish(compressed_msg)
        except Exception as e:
            rospy.logerr(f"Error processing image: {e}")

def main():
    ImageCompressor()
    rospy.spin()

if __name__ == '__main__':
    main()
