#ifndef __CANABLE_NODE_HPP__
#define __CANABLE_NODE_HPP__

#include "rclcpp/rclcpp.hpp"
#include "canable_msgs/msg/can.hpp"
#include <linux/can/raw.h>
#include <linux/can.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <cstring>
#include <unistd.h>

namespace canable {
class canable_node : public rclcpp::Node {
public:
    canable_node(const rclcpp::NodeOptions &node_options);
    ~canable_node();

private:
    void init_can_socket(); // Initialize CAN socket
    void read_can_socket(); // Read messages from CAN socket
    void write_can_socket(const canable_msgs::msg::Can &frame); // Write messages to CAN socket

    int can_socket_;
    struct sockaddr_can addr_;
    struct ifreq ifr_;
    rclcpp::Publisher<canable_msgs::msg::Can>::SharedPtr canable_pub_;
    rclcpp::Subscription<canable_msgs::msg::Can>::SharedPtr canable_sub_;
};
} // namespace canable

#endif // __CANABLE_NODE_HPP__