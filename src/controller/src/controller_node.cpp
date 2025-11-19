#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "interfaces/msg/controller_command.hpp"

using std::placeholders::_1;

class ControllerNode : public rclcpp::Node
{
public:
    ControllerNode()
    : Node("controller_node")
    {
        publisher_ = this->create_publisher<interfaces::msg::ControllerCommand>("/cmd_vel", 10);
    
        subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10, std::bind(&ControllerNode::joy_callback, this, _1));
        
        RCLCPP_INFO(this->get_logger(), "Controller Node (Stateful Depth & Yaw) has been started.");
        RCLCPP_INFO(this->get_logger(), "Listening to /joy, publishing to /cmd_vel");
    }

private:
    void joy_callback(const sensor_msgs::msg::Joy & msg)
    {
        auto cmd_msg = interfaces::msg::ControllerCommand();

        cmd_msg.x_cmd = msg.axes[1] * 250.0;
        cmd_msg.y_cmd = msg.axes[0] * -250.0;

        const float deadzone = 0.2f;

        float depth_stick_input = msg.axes[4]; 
        const float depth_change_speed = 0.02f;

        if (depth_stick_input > deadzone)
        {
            current_depth_ -= depth_change_speed;
        }
        else if (depth_stick_input < -deadzone)
        {
            current_depth_ += depth_change_speed;
        }

        if (current_depth_ < 0.0f) { current_depth_ = 0.0f; }
        if (current_depth_ > 10.0f) { current_depth_ = 10.0f; }

        float yaw_stick_input = msg.axes[3];
        const float yaw_change_speed = 0.5f;

        if (yaw_stick_input > deadzone)
        {
            current_yaw_ -= yaw_change_speed;
        }
        else if (yaw_stick_input < -deadzone)
        {
            current_yaw_ += yaw_change_speed;
        }

        if (current_yaw_ < -180.0f) { current_yaw_ = -180.0f; }
        if (current_yaw_ > 180.0f) { current_yaw_ = 180.0f; }

        cmd_msg.depth = current_depth_;
        cmd_msg.yaw = current_yaw_;
        
        publisher_->publish(cmd_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
    rclcpp::Publisher<interfaces::msg::ControllerCommand>::SharedPtr publisher_;
    
    float current_depth_ = 0.0f;
    float current_yaw_ = 0.0f;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControllerNode>());
    rclcpp::shutdown();
    return 0;
}