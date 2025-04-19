#ifndef __CANABLE_NODE_HPP__
#define __CANABLE_NODE_HPP__

#include "rclcpp/rclcpp.hpp"
#include "canable_msgs/msg/Can.hpp"

namespace canable {
class canable_node : public rclcpp::Node {
public:
    canable_node(const rclcpp::NodeOptions & options);
    ~canable_node();
    void can_topic_callback(const canable_msgs::msg::Can::SharedPtr msg);

private:
    int canable_fd_;
    std::string canable_device_port_;
    std::thread canable_recive_thread_;
    void canable_recive_loop();
    rclcpp::Publisher<canable_msgs::msg::Can>::SharedPtr canable_pub_;
    rclcpp::Subscription<canable_msgs::msg::Can>::SharedPtr canable_sub_;
};
} // namespace canable

#endif// __CANABLE_NODE_HPP__