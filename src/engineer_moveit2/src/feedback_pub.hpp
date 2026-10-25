#pragma once
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace feedback_pub{
    void init(rclcpp::Node::SharedPtr node);
    void publish_feedback(const float arm[7]);
    void save_feedback(int step_idx, const float arm[7]);
}
