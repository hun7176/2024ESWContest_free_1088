import rospy
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2

class ImageCompressor:
    def __init__(self):
        rospy.init_node('image_compressor', anonymous=True)
        self.bridge = CvBridge()
        
        self.image_sub1 = rospy.Subscriber('/usb_cam1/image_raw', Image, self.image_callback1)
        self.image_sub2 = rospy.Subscriber('/usb_cam2/image_raw', Image, self.image_callback2)
        
        self.image_pub1 = rospy.Publisher('/usb_cam1/image_compressed', CompressedImage, queue_size=10)
        self.image_pub2 = rospy.Publisher('/usb_cam2/image_compressed', CompressedImage, queue_size=10)

    def image_callback1(self, data):
        self.compress_and_publish(data, self.image_pub1)

    def image_callback2(self, data):
        self.compress_and_publish(data, self.image_pub2)

    def compress_and_publish(self, data, publisher):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
            _, compressed_image = cv2.imencode('.jpg', cv_image, [cv2.IMWRITE_JPEG_QUALITY, 90])
            
            compressed_msg = CompressedImage()
            compressed_msg.header = data.header
            compressed_msg.format = "jpeg"
            compressed_msg.data = compressed_image.tobytes()

            publisher.publish(compressed_msg)
        except Exception as e:
            rospy.logerr(f"Error processing image: {e}")

def main():
    ImageCompressor()
    rospy.spin()

if __name__ == '__main__':
    main()

