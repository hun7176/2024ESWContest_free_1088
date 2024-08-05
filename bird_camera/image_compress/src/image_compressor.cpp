#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>

class ImageCompressor
{
public:
    ImageCompressor()
        : it_(nh_)
    {
        // 첫 번째 이미지 구독자 및 퍼블리셔
        image_sub1_ = it_.subscribe("/usb_cam1/image_raw", 1, &ImageCompressor::imageCallback1, this);
        image_pub1_ = it_.advertise("/usb_cam1/image_compressed", 1);

        // 두 번째 이미지 구독자 및 퍼블리셔
        image_sub2_ = it_.subscribe("/usb_cam2/image_raw", 1, &ImageCompressor::imageCallback2, this);
        image_pub2_ = it_.advertise("/usb_cam2/image_compressed", 1);
    }

    void imageCallback1(const sensor_msgs::ImageConstPtr& msg)
    {
        processImage(msg, image_pub1_);
    }

    void imageCallback2(const sensor_msgs::ImageConstPtr& msg)
    {
        processImage(msg, image_pub2_);
    }

private:
    void processImage(const sensor_msgs::ImageConstPtr& msg, const image_transport::Publisher& pub)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        // 해상도 낮추기
        cv::Mat resized_image;
        cv::resize(cv_ptr->image, resized_image, cv::Size(), 0.5, 0.5);

        // 압축
        std::vector<uchar> buf;
        cv::imencode(".jpg", resized_image, buf);

        // 이미지 메시지로 변환
        cv_bridge::CvImage out_msg;
        out_msg.header = msg->header;
        out_msg.encoding = sensor_msgs::image_encodings::BGR8;
        out_msg.image = cv::imdecode(buf, cv::IMREAD_COLOR);

        // 퍼블리시
        pub.publish(out_msg.toImageMsg());
    }

    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub1_;
    image_transport::Subscriber image_sub2_;
    image_transport::Publisher image_pub1_;
    image_transport::Publisher image_pub2_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_compressor");
    ImageCompressor ic;
    ros::spin();
    return 0;
}

