#include "canable/canable_node.hpp"

namespace canable {
    canable_node::canable_node(const rclcpp::NodeOptions & options)
        : Node("canable_node", options),
          canable_fd_(-1),
          canable_device_port_("/dev/ttyACM0")
    {
        this->declare_parameter<std::string>("canable_device_port", canable_device_port_);
        this->get_parameter("canable_device_port", canable_device_port_);

        canable_fd_ = open(canable_device_port_.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
        if (canable_fd_ == -1) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open CANABLE device: %s", canable_device_port_.c_str());
            return;
        }

        canable_pub_ = this->create_publisher<canable_msgs::msg::Can>("/can/receive", 10);
        canable_sub_ = this->create_subscription<canable_msgs::msg::Can>(
            "/can/transmit", 10, std::bind(&canable_node::can_topic_callback, this, std::placeholders::_1));

        canable_recive_thread_ = std::thread(&canable_node::canable_recive_loop, this);
    }

    canable_node::~canable_node() {
        if (canable_fd_ != -1) {
            close(canable_fd_);
        }
        if (canable_recive_thread_.joinable()) {
            canable_recive_thread_.join();
        }
    }

    void canable_node::can_topic_callback(const canable_msgs::msg::Can::SharedPtr msg) {
        if (canable_fd_ == -1) {
            RCLCPP_ERROR(this->get_logger(), "CANABLE device not open");
            return;
        }

        // Send the CAN message to the CANABLE device (SLCAN)
        std::ostringstream ss;
        ss << 't' << std::hex << std::uppercase << std::setw(3) << std::setfill('0') << msg->id;
        ss << msg->dlc;
        for(size_t i = 0; i < msg->dlc; ++i) {
            ss << std::hex << std::uppercase << std::setw(2) << std::setfill('0') << static_cast<int>(msg->data[i] & 0xFF);
        }
        ss << '\r';
        std::string command = ss.str();
        write(canable_fd_, command.c_str(), command.size());
        RCLCPP_INFO(this->get_logger(), "Sent CAN message: %s", command.c_str());
    }

    void canable_node::canable_recive_loop() {
        if (canable_fd_ == -1) {
            RCLCPP_ERROR(this->get_logger(), "CANABLE device not open");
            return;
        }

        char buffer[256];
        while (rclcpp::ok()) {
            int bytes_read = read(canable_fd_, buffer, sizeof(buffer) - 1);
            if (bytes_read > 0) {
                buffer[bytes_read] = '\0';
                std::string response(buffer);
                if (response[0] == 'r') {
                    canable_msgs::msg::Can msg;
                    msg.id = std::stoul(response.substr(1, 3), nullptr, 16);
                    msg.dlc = response[4] - '0';
                    for (size_t i = 0; i < msg.dlc; ++i) {
                        msg.data[i] = std::stoul(response.substr(5 + i * 2, 2), nullptr, 16);
                    }
                    canable_pub_->publish(msg);
                }
            }
        }
    }