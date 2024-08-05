#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>

class ImageCompressor
{
public:
    ImageCompressor(const std::string& input_topic, const std::string& output_topic)
        : it_(nh_)
    {
        image_sub_ = it_.subscribe(input_topic, 1, &ImageCompressor::imageCallback, this);
        image_pub_ = it_.advertise(output_topic, 1);
    }

    void imageCallback(const sensor_msgs::ImageConstPtr& msg)
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
        image_pub_.publish(out_msg.toImageMsg());
    }

private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_compressor");

    if (argc != 3) {
        ROS_ERROR("Usage: image_compressor <input_topic> <output_topic>");
        return -1;
    }

    std::string input_topic = argv[1];
    std::string output_topic = argv[2];

    ImageCompressor ic(input_topic, output_topic);
    ros::spin();
    return 0;
}

