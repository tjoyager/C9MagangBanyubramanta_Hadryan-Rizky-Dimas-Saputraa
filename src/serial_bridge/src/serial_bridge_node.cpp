#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <asio.hpp> // <- Pustaka ASIO untuk serial
#include <string>
#include <sstream> // <- Untuk format string
#include <iomanip> // <- Untuk std::setprecision

// Gunakan namespace agar lebih mudah dibaca
using namespace std::placeholders;

class SerialBridgeNode : public rclcpp::Node
{
public:
    SerialBridgeNode() : Node("serial_bridge_node"), io_(), port_(io_)
    {
        // 1. Deklarasi Parameter (Port dan Baud Rate)
        // Ini adalah cara terbaik agar bisa diubah saat runtime
        this->declare_parameter<std::string>("serial_port", "/dev/ttyACM0");
        this->declare_parameter<int>("baud_rate", 115200);

        std::string port_name = this->get_parameter("serial_port").as_string();
        int baud = this->get_parameter("baud_rate").as_int();

        // 2. Setup Komunikasi Serial
        try
        {
            port_.open(port_name); // Buka port
            // Set opsi (sesuai dari contoh Anda)
            port_.set_option(asio::serial_port_base::baud_rate(baud));
            port_.set_option(asio::serial_port_base::flow_control(asio::serial_port_base::flow_control::none));
            port_.set_option(asio::serial_port_base::parity(asio::serial_port_base::parity::none));
            port_.set_option(asio::serial_port_base::stop_bits(asio::serial_port_base::stop_bits::one));
            port_.set_option(asio::serial_port_base::character_size(8));
            
            RCLCPP_INFO(this->get_logger(), "Berhasil membuka port serial: %s @ %d baud", port_name.c_str(), baud);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Gagal membuka port %s: %s", port_name.c_str(), e.what());
            // Jika gagal, matikan node
            rclcpp::shutdown();
            return;
        }

        // 3. Buat Subscriber ke /cmd_vel
        subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 
            10, // QoS depth
            std::bind(&SerialBridgeNode::cmd_vel_callback, this, _1)
        );
        
        RCLCPP_INFO(this->get_logger(), "Serial bridge node dimulai, mendengarkan di /cmd_vel...");
    }

private:
    // Fungsi ini dipanggil setiap ada pesan baru di /cmd_vel
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        // 4. Menerjemahkan Pesan Twist ke Format String
        // Format: "<lin_x, ang_z>\n"
        // Contoh: "<0.50, -0.20>\n"
        // Format ini mudah diparsing di Arduino/ESP32
        
        std::stringstream ss;
        ss << std::fixed << std::setprecision(2); // 2 angka di belakang koma
        ss << "<" << msg->linear.x << "," << msg->angular.z << ">\n";
        
        std::string data_to_send = ss.str();

        // 5. Mengirim Data ke Port Serial
        try
        {
            asio::write(port_, asio::buffer(data_to_send));
            RCLCPP_INFO(this->get_logger(), "Mengirim: %s", data_to_send.c_str());
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Gagal menulis ke serial: %s", e.what());
        }
    }

    // Anggota kelas
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
    asio::io_service io_;
    asio::serial_port port_;
};

// Fungsi main standar ROS 2
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SerialBridgeNode>());
    rclcpp::shutdown();
    return 0;
}