import rospy
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2

class ImageCompressor:
    def __init__(self):
        rospy.init_node('image_compressor', anonymous=True)
        rospy.loginfo("ImageCompressor node initialized.")
        self.bridge = CvBridge()
        
        self.image_sub1 = rospy.Subscriber('/usb_cam1/image_raw', Image, self.image_callback1)
        self.image_sub2 = rospy.Subscriber('/usb_cam2/image_raw', Image, self.image_callback2)
        
        self.image_pub1 = rospy.Publisher('/usb_cam1/image_compressed', CompressedImage, queue_size=10)
        self.image_pub2 = rospy.Publisher('/usb_cam2/image_compressed', CompressedImage, queue_size=10)
        
        rospy.loginfo("Subscribers and publishers set up.")

    def image_callback1(self, data):
        rospy.loginfo("Received image from /usb_cam1/image_raw")
        self.compress_and_publish(data, self.image_pub1)

    def image_callback2(self, data):
        rospy.loginfo("Received image from /usb_cam2/image_raw")
        self.compress_and_publish(data, self.image_pub2)

    def compress_and_publish(self, data, publisher):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')

            # Resize the image (e.g., reduce to half of the original size)
            height, width = cv_image.shape[:2]
            new_size = (width // 2, height // 2)
            resized_image = cv2.resize(cv_image, new_size, interpolation=cv2.INTER_LINEAR)

            # Compress the resized image
            _, compressed_image = cv2.imencode('.jpg', resized_image, [cv2.IMWRITE_JPEG_QUALITY, 90])
            
            # Create and publish the CompressedImage message
            compressed_msg = CompressedImage()
            compressed_msg.header = data.header
            compressed_msg.format = "jpeg"
            compressed_msg.data = compressed_image.tobytes()

            publisher.publish(compressed_msg)
            rospy.loginfo("Published compressed image.")
        except Exception as e:
            rospy.logerr(f"Error processing image: {e}")

def main():
    ImageCompressor()
    rospy.spin()

if __name__ == '__main__':
    main()

