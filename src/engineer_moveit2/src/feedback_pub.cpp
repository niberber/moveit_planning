#include <fstream>
#include "feedback_pub.hpp"


namespace feedback_pub{

    static rclcpp::Node::SharedPtr g_node;

    static rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr g_pub;
    
    static const char* jointNames[7] = {
        "joint5",
        "joint6",
        "joint7",
        "joint8",
        "joint9",
        "joint10",
        "joint11",
    };

    static const char* CsvPath = "/home/yh/2026_Engineer_ws/src/feedback.csv";

    void init(rclcpp::Node::SharedPtr node) {
        g_node = node;
        g_pub = node->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
    }
    void publish_feedback(const float arm[7]) {
        if(!g_pub)return;

        sensor_msgs::msg::JointState msg;
        msg.header.stamp = g_node->get_clock()->now();
        for(int i = 0; i < 7; i++){
            msg.name.push_back(jointNames[i]);
            msg.position.push_back(arm[i]);
        }
        g_pub->publish(msg);
    }
    void save_feedback(int step_idx, const float arm[7]){
        bool flag = !std::ifstream(CsvPath).good();
        std::ofstream f(CsvPath, std::ios::app);

        if(!f.is_open()){
            RCLCPP_ERROR(rclcpp::get_logger("feedback_pub"), "CSV open failed: %s", CsvPath);
            return;
        }

        if(flag)f << "step,joint5,joint6,joint7,joint8,joint9,joint10,joint11\n";
        f << step_idx;
        for(int i = 0; i < 7; i++)f << "," << arm[i];
        f << "\n";
    }
}