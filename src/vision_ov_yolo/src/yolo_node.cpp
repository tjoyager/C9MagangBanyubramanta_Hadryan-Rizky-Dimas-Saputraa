#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include "vision_ov_yolo/yolo_engine.hpp"
#include "yolo_msgs/msg/detection.hpp"

class OpenVinoNode : public rclcpp::Node {
public:
    OpenVinoNode() : Node("openvino_yolo_node") {
        // 1. Declare Parameters
        this->declare_parameter("model_path", "");
        this->declare_parameter("video_path", ""); // Jika diisi -> Mode Video File
        this->declare_parameter("image_topic", "/camera/image_raw"); // Jika video kosong -> Mode Gazebo

        std::string model_path = this->get_parameter("model_path").as_string();
        std::string video_path = this->get_parameter("video_path").as_string();
        std::string image_topic = this->get_parameter("image_topic").as_string();

        // 2. Cek Model Path
        if (model_path.empty()) {
            RCLCPP_ERROR(this->get_logger(), "CRITICAL: Parameter 'model_path' wajib diisi!");
            exit(1);
        }

        // 3. Init Engine
        try {
            RCLCPP_INFO(this->get_logger(), "Loading Yolo Engine: %s", model_path.c_str());
            // Threshold: Conf 0.4, NMS 0.4, Score 0.4
            engine_ = std::make_unique<YoloEngine>(model_path, 0.4f, 0.4f, 0.4f);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Engine Init Failed: %s", e.what());
            exit(1);
        }

        // 4. Setup Publisher
        detect_pub_ = this->create_publisher<yolo_msgs::msg::Detection>("yolo/detections", 10);
        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("yolo/debug_image", 10);

        // 5. Logika Hybrid (Pilih Mode)
        if (!video_path.empty()) {
            // === MODE 1: VIDEO FILE (TUGAS 4) ===
            RCLCPP_INFO(this->get_logger(), "Mode: VIDEO FILE PLAYBACK (%s)", video_path.c_str());
            cap_.open(video_path);
            
            if (!cap_.isOpened()) {
                RCLCPP_ERROR(this->get_logger(), "Gagal membuka video!");
                exit(1);
            }

            // Timer loop (30 FPS -> ~33ms)
            timer_ = this->create_wall_timer(
                std::chrono::milliseconds(33), 
                std::bind(&OpenVinoNode::video_timer_callback, this));

        } else {
            // === MODE 2: CAMERA SUBSCRIBER (TUGAS 6 GAZEBO) ===
            RCLCPP_INFO(this->get_logger(), "Mode: REAL-TIME SUBSCRIBER (Topic: %s)", image_topic.c_str());
            
            sub_ = this->create_subscription<sensor_msgs::msg::Image>(
                image_topic, 
                rclcpp::SensorDataQoS(), 
                std::bind(&OpenVinoNode::camera_topic_callback, this, std::placeholders::_1));
        }
    }

private:
    // --- FUNGSI PROSES UTAMA (Dipakai kedua mode) ---
    void process_and_publish(cv::Mat& frame, std_msgs::msg::Header header) {
        if (frame.empty()) return;

        // 1. Inference
        std::vector<BBox> results = engine_->run_inference(frame);

        // 2. Visualisasi
        cv::Mat debug_frame = frame.clone();

        for (const auto& det : results) {
            // A. Publish Custom Message
            yolo_msgs::msg::Detection msg;
            msg.label = det.label;
            msg.confidence = det.confidence;
            msg.x = det.box.x;
            msg.y = det.box.y;
            msg.w = det.box.width;
            msg.h = det.box.height;
            detect_pub_->publish(msg);

            // B. Gambar Kotak
            cv::Scalar color = (det.label == "Flare") ? cv::Scalar(0, 165, 255) : cv::Scalar(0, 255, 0);
            cv::rectangle(debug_frame, det.box, color, 2);
            
            std::string label_text = det.label + " " + std::to_string((int)(det.confidence * 100)) + "%";
            cv::putText(debug_frame, label_text, cv::Point(det.box.x, det.box.y - 5), 
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 2);
        }

        // 3. Publish Debug Image ke RQT
        try {
            sensor_msgs::msg::Image::SharedPtr img_msg = 
                cv_bridge::CvImage(header, "bgr8", debug_frame).toImageMsg();
            image_pub_->publish(*img_msg);
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "CV Bridge Error: %s", e.what());
        }
    }

    // --- CALLBACK MODE VIDEO ---
    void video_timer_callback() {
        cv::Mat frame;
        cap_ >> frame;

        if (frame.empty()) {
            cap_.set(cv::CAP_PROP_POS_FRAMES, 0); // Loop video
            return;
        }

        std_msgs::msg::Header header;
        header.stamp = this->now();
        header.frame_id = "video_file";
        
        process_and_publish(frame, header);
    }

    // --- CALLBACK MODE KAMERA/GAZEBO ---
    void camera_topic_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            // Convert ROS Image -> OpenCV Mat
            cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
            process_and_publish(frame, msg->header);
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Could not convert image: %s", e.what());
        }
    }

    // Variables
    std::unique_ptr<YoloEngine> engine_;
    
    // Pub/Sub
    rclcpp::Publisher<yolo_msgs::msg::Detection>::SharedPtr detect_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    
    // Video Mode
    cv::VideoCapture cap_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    // Camera Mode
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OpenVinoNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}