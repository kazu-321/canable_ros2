#include "canable/canable_node.hpp"

namespace canable {

canable_node::canable_node(const rclcpp::NodeOptions &node_options)
    : Node("canable_node", node_options) {
    this->declare_parameter("retry_open_can", true);
    this->declare_parameter("retry_write_can", true);
    this->declare_parameter("max_retry_write_count", 5);
    this->get_parameter("retry_open_can", retry_open_can);
    this->get_parameter("retry_write_can", retry_write_can);
    this->get_parameter("max_retry_write_count", max_retry_write_count);

    if(init_can_socket() != 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize CAN socket");
        return;
    }

    canable_pub_ = this->create_publisher<canable_msgs::msg::Can>("/can/receive", 10);
    canable_sub_ = this->create_subscription<canable_msgs::msg::Can>(
        "/can/transmit", 10,
        [this](const canable_msgs::msg::Can::SharedPtr msg) {
            this->write_can_socket(*msg);
        });

    // Start CAN read loop
    std::thread([this]() { this->read_can_socket(); }).detach();
}

canable_node::~canable_node() {
    if (can_socket_ >= 0) {
        close(can_socket_);
    }
}

int canable_node::init_can_socket() {
    can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (can_socket_ < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to create CAN socket");
        return -1;
    }

    std::strcpy(ifr_.ifr_name, "can0"); // Use CAN interface "can0"
    if (ioctl(can_socket_, SIOCGIFINDEX, &ifr_) < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get interface index");
        close(can_socket_);
        if(retry_open_can) {
            RCLCPP_INFO(this->get_logger(), "Retrying to open CAN socket...");
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            return init_can_socket();
        }
    }

    addr_.can_family = AF_CAN;
    addr_.can_ifindex = ifr_.ifr_ifindex;
    if (bind(can_socket_, (struct sockaddr *)&addr_, sizeof(addr_)) < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to bind CAN socket");
        return -1;
    }
    
    RCLCPP_INFO(this->get_logger(), "CAN socket initialized and bound to %s", ifr_.ifr_name);
    return 0;
}

void canable_node::read_can_socket() {
    struct can_frame frame;
    while (rclcpp::ok()) {
        int nbytes = read(can_socket_, &frame, sizeof(struct can_frame));
        if (nbytes > 0) {
            auto msg = canable_msgs::msg::Can();
            msg.id = frame.can_id;
            msg.dlc = frame.can_dlc;
            std::copy(std::begin(frame.data), std::begin(frame.data) + frame.can_dlc, msg.data.begin());
            canable_pub_->publish(msg);
        }
    }
}

void canable_node::write_can_socket(const canable_msgs::msg::Can &msg) {
    struct can_frame frame;
    std::memset(&frame, 0, sizeof(struct can_frame));
    frame.can_id = msg.id;
    frame.can_dlc = msg.dlc;
    std::copy(msg.data.begin(), msg.data.begin() + msg.dlc, frame.data);

    if (write(can_socket_, &frame, sizeof(struct can_frame)) < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to send CAN message");
        if (retry_write_can) {
            retry_write_count++;
            if (retry_write_count < max_retry_write_count) {
                RCLCPP_INFO(this->get_logger(), "Retrying to send CAN message...");
                write_can_socket(msg);
            } else {
                RCLCPP_ERROR(this->get_logger(), "Max retry count reached. Giving up.");
                retry_write_count = 0;
                init_can_socket();
            }
        }
    } else {
        retry_write_count = 0;
    }
}

} // namespace canable

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(canable::canable_node)