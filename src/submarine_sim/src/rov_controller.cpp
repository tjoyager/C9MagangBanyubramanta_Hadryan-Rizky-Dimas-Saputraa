#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/rendering/Camera.hh>
#include <gazebo_ros/node.hpp>
#include <gazebo_ros/conversions/geometry_msgs.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <rclcpp/rclcpp.hpp>

#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>

namespace gazebo
{
    class ROVController : public ModelPlugin
    {
    public:
        ROVController() = default;

        void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override
        {
            model_ = model;
            world_ = model_->GetWorld();

            // Initialize ROS node
            node_ = gazebo_ros::Node::Get(sdf);

            // Subscriber & Publisher
            sub_cmd_vel_ = node_->create_subscription<geometry_msgs::msg::Twist>(
                "/cmd_vel", 10, std::bind(&ROVController::OnCmdVel, this, std::placeholders::_1));
            
            odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
            pub_camera_ = node_->create_publisher<sensor_msgs::msg::Image>("/camera/image_raw", 10);
            tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);

            // Hook ke Update Loop Gazebo
            update_connection_ = event::Events::ConnectWorldUpdateBegin(
                std::bind(&ROVController::OnUpdate, this));

            // Koneksi ke Sensor Kamera Gazebo
            // Pastikan nama sensor di xacro sesuai ("camera_sensor")
            auto sensor = gazebo::sensors::SensorManager::Instance()->GetSensor("camera_sensor");
            if (sensor)
            {
                camera_sensor_ = std::dynamic_pointer_cast<gazebo::sensors::CameraSensor>(sensor);
                camera_ = camera_sensor_->Camera();
                camera_connection_ = camera_->ConnectNewImageFrame(
                    std::bind(&ROVController::OnNewFrame, this,
                              std::placeholders::_1, std::placeholders::_2,
                              std::placeholders::_3, std::placeholders::_4,
                              std::placeholders::_5));
                RCLCPP_INFO(node_->get_logger(), "Kamera Terdeteksi!");
            }
            else
            {
                RCLCPP_WARN(node_->get_logger(), "Sensor kamera tidak ditemukan! Cek nama di Xacro.");
            }

            RCLCPP_INFO(node_->get_logger(), "ROV Controller Siap! (Submarine Mode)");
        }

        // Fungsi Loop Utama (Fisika)
        void OnUpdate()
        {
            ignition::math::Pose3d pose = model_->WorldPose();
            
            // Publish Odometry (Posisi Robot)
            nav_msgs::msg::Odometry odom;
            odom.header.stamp = node_->now();
            odom.header.frame_id = "odom";
            odom.child_frame_id = "base_link";

            // Set Posisi
            odom.pose.pose = gazebo_ros::Convert<geometry_msgs::msg::Pose>(pose);
            
            // Set Kecepatan Linear & Angular
            auto linear_vel = model_->WorldLinearVel();
            auto angular_vel = model_->WorldAngularVel();
            
            odom.twist.twist.linear.x = linear_vel.X();
            odom.twist.twist.linear.y = linear_vel.Y();
            odom.twist.twist.linear.z = linear_vel.Z();
            odom.twist.twist.angular.x = angular_vel.X();
            odom.twist.twist.angular.y = angular_vel.Y();
            odom.twist.twist.angular.z = angular_vel.Z();

            odom_pub_->publish(odom);

            // Publish TF (Transformasi Koordinat)
            geometry_msgs::msg::TransformStamped tf_msg;
            tf_msg.header.stamp = node_->now();
            tf_msg.header.frame_id = "odom";
            tf_msg.child_frame_id = "base_link";
            tf_msg.transform = gazebo_ros::Convert<geometry_msgs::msg::Transform>(pose);
            tf_broadcaster_->sendTransform(tf_msg);
        }

        // Menerima Perintah Gerak
        void OnCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg)
        {
            // Logika Gerak: Mengubah cmd_vel (body frame) menjadi world frame
            // Agar robot maju sesuai arah hadapnya
            ignition::math::Pose3d pose = model_->WorldPose();
            double yaw = pose.Rot().Yaw();

            // Kalkulasi rotasi sederhana untuk gerak linear
            double x_val = msg->linear.x * cosf(yaw) - msg->linear.y * sinf(yaw);
            double y_val = msg->linear.x * sinf(yaw) + msg->linear.y * cosf(yaw);
            double z_val = msg->linear.z; // Naik turun (Heave)

            model_->SetLinearVel(ignition::math::Vector3d(x_val, y_val, z_val));
            model_->SetAngularVel(ignition::math::Vector3d(msg->angular.x, msg->angular.y, msg->angular.z));
        }

        // Mengirim Data Gambar ke ROS
        void OnNewFrame(const unsigned char *image,
                        unsigned int width, unsigned int height,
                        unsigned int depth, const std::string &format)
        {
            sensor_msgs::msg::Image msg;
            msg.header.stamp = node_->now();
            msg.header.frame_id = "camera_frame";
            msg.height = height;
            msg.width = width;
            msg.encoding = "bgr8"; // Sesuai format Gazebo B8G8R8 biasanya mapping ke bgr8 di ROS
            msg.is_bigendian = 0;
            msg.step = width * 3;
            msg.data.resize(height * msg.step);

            memcpy(msg.data.data(), image, height * msg.step);
            pub_camera_->publish(msg);
        }

    private:
        physics::ModelPtr model_;
        physics::WorldPtr world_;
        event::ConnectionPtr update_connection_;
        
        std::shared_ptr<rclcpp::Node> node_;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_camera_;
        std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

        gazebo::sensors::CameraSensorPtr camera_sensor_;
        gazebo::rendering::CameraPtr camera_;
        event::ConnectionPtr camera_connection_;
    };

    GZ_REGISTER_MODEL_PLUGIN(ROVController)
}