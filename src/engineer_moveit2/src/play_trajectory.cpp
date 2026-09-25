#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("play_trajectory");
  
  std::string csv_file = node->get_parameter_or<std::string>("csv_file", std::string("trajectory.csv"));
  double delay_ms = node->get_parameter_or<double>("delay_ms", 200.0);
  bool loop_play = node->get_parameter_or<bool>("loop", false);
  int pause_at_end = node->get_parameter_or<int>("pause_ms", 1000);
  
  auto pub = node->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
  
  std::ifstream file(csv_file);
  if (!file.is_open()) {
    RCLCPP_ERROR(node->get_logger(), "Failed to open file: %s", csv_file.c_str());
    return 1;
  }
  
  std::vector<std::vector<double>> trajectory;
  std::vector<std::string> joint_names;
  std::string line;
  
  std::getline(file, line);
  std::stringstream header_ss(line);
  std::string token;
  while (std::getline(header_ss, token, ',')) {
    if (token != "time") {
      joint_names.push_back(token);
    }
  }
  
  while (std::getline(file, line)) {
    std::stringstream ss(line);
    std::vector<double> point;
    while (std::getline(ss, token, ',')) {
      point.push_back(std::stod(token));
    }
    trajectory.push_back(point);
  }
  
  RCLCPP_INFO(node->get_logger(), "Loaded trajectory with %zu points", trajectory.size());
  RCLCPP_INFO(node->get_logger(), "Delay per point: %.0fms, Loop: %s, Pause at end: %dms", 
              delay_ms, loop_play ? "true" : "false", pause_at_end);
  
  rclcpp::Rate rate(100);
  
  while (rclcpp::ok()) {
    for (size_t i = 0; i < trajectory.size(); ++i) {
      sensor_msgs::msg::JointState msg;
      msg.header.stamp = node->get_clock()->now();
      msg.name = joint_names;
      
      auto& point = trajectory[i];
      for (size_t j = 1; j < point.size(); ++j) {
        msg.position.push_back(point[j]);
        msg.velocity.push_back(0.0);
        msg.effort.push_back(0.0);
      }
      
      pub->publish(msg);
      
      if (i == trajectory.size() - 1) {
        for (int p = 0; p < pause_at_end / 10; ++p) {
          pub->publish(msg);
          rate.sleep();
        }
      } else {
        for (int d = 0; d < delay_ms / 10; ++d) {
          pub->publish(msg);
          rate.sleep();
        }
      }
      
      if (!rclcpp::ok()) break;
    }
    
    if (!loop_play) break;
    RCLCPP_INFO(node->get_logger(), "Restarting loop...");
  }
  
  if (trajectory.size() > 0) {
    RCLCPP_INFO(node->get_logger(), "Holding final position... (Ctrl+C to exit)");
    sensor_msgs::msg::JointState final_msg;
    auto& final_point = trajectory.back();
    while (rclcpp::ok()) {
      final_msg.header.stamp = node->get_clock()->now();
      final_msg.name = joint_names;
      final_msg.position.clear();
      for (size_t j = 1; j < final_point.size(); ++j) {
        final_msg.position.push_back(final_point[j]);
      }
      pub->publish(final_msg);
      rate.sleep();
    }
  }
  
  rclcpp::shutdown();
  return 0;
}
