#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <iostream>

using namespace std;
using namespace cv;

class VideoPublisherNode : public rclcpp::Node {
public:
    VideoPublisherNode() : Node("video_publisher_node") {
        
        this->declare_parameter<std::string>("video_file_path", "/home/hadryan/fourth.mp4");
        std::string video_path = this->get_parameter("video_file_path").as_string();

        RCLCPP_INFO(this->get_logger(), "Membuka file video dari: %s", video_path.c_str());

        cap_.open(video_path);
        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Error: Tidak bisa membuka file video.");
            rclcpp::shutdown();
        }

        raw_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("raw_image", 10);
        
        mask_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("mask_image", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(30),
            std::bind(&VideoPublisherNode::timer_callback, this)
        );

        RCLCPP_INFO(this->get_logger(), "Node Video Publisher telah dimulai. Mempublikasikan ke /raw_image dan /mask_image");
    }

private:
    void timer_callback() {
        Mat frame;
        cap_.read(frame);

        if (frame.empty()) {
            RCLCPP_INFO(this->get_logger(), "Video selesai. Mengulang dari awal.");
            std::string video_path = this->get_parameter("video_file_path").as_string();
            cap_.open(video_path);
            cap_.read(frame);
            if(frame.empty()) {
                RCLCPP_ERROR(this->get_logger(), "Gagal membaca ulang video. Node berhenti.");
                rclcpp::shutdown();
                return;
            }
        }

        publish_image(frame, raw_image_pub_, "bgr8");

        Mat hsvFrame, mask1, mask2, finalMask;
        
        cvtColor(frame, hsvFrame, COLOR_BGR2HSV);

        int hMin1 = 0, hMax1 = 32;
        int hMin2 = 150, hMax2 = 180;
        int sMin = 100, sMax = 255;
        int vMin = 100, vMax = 255;

        Scalar lower_range_1(hMin1, sMin, vMin);
        Scalar upper_range_1(hMax1, sMax, vMax);
        Scalar lower_range_2(hMin2, sMin, vMin);
        Scalar upper_range_2(hMax2, sMax, vMax);

        inRange(hsvFrame, lower_range_1, upper_range_1, mask1);
        inRange(hsvFrame, lower_range_2, upper_range_2, mask2);
        bitwise_or(mask1, mask2, finalMask);

        publish_image(finalMask, mask_image_pub_, "mono8");
    }

    void publish_image(const Mat& image, 
                       const rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr& publisher, 
                       const std::string& encoding) 
    {
        std_msgs::msg::Header header;
        header.stamp = this->get_clock()->now();
        header.frame_id = "camera_frame";

        sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(
            header,
            encoding,
            image
        ).toImageMsg();

        publisher->publish(*msg);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr raw_image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr mask_image_pub_;
    VideoCapture cap_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VideoPublisherNode>());
    rclcpp::shutdown();
    return 0;
}