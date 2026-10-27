#include <fstream>
#include <sstream>
#include <ctime>
#include "feedback_pub.hpp"


namespace feedback_pub{

    static rclcpp::Node::SharedPtr g_node;
    static rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr g_pub;
    static std::string g_traj_path;    // 本次运行的轨迹记录文件
    static std::string g_event_path;   // 本次运行的事件记录文件
    
    static const char* jointNames[7] = {
        "joint5",
        "joint6",
        "joint7",
        "joint8",
        "joint9",
        "joint10",
        "joint11",
    };

    static const char* CsvPath = "feedback.csv";   // 相对当前工作目录

    void init(rclcpp::Node::SharedPtr node) {
        g_node = node;
        g_pub = node->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

        // 每次启动生成独立文件名，避免多局数据混在一起
        const std::time_t tt = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
        char buf[32] = {0};
        std::strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", std::localtime(&tt));
        const std::string prefix = std::string("rec_") + buf;
        g_traj_path  = prefix + "_traj.csv";
        g_event_path = prefix + "_event.jsonl";
        RCLCPP_INFO(node->get_logger(), "记录文件: %s | %s", g_traj_path.c_str(), g_event_path.c_str());
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
    void save_cmd_traj(uint32_t request_id, int step,
                       const std::vector<std::vector<float> >& hardware_angles,
                       const std::vector<double>& time_steps){
        if(g_traj_path.empty())return;

        const bool new_file = !std::ifstream(g_traj_path).good();
        std::ofstream f(g_traj_path, std::ios::app);
        if(!f.is_open()){
            RCLCPP_ERROR(rclcpp::get_logger("feedback_pub"), "CSV open failed: %s", g_traj_path.c_str());
            return;
        }

        if(new_file){
            f << "req,kind,idx,dt,joint5,joint6,joint7,joint8,joint9,joint10,joint11\n";
        }
        for(size_t i = 0; i < hardware_angles.size(); i++){
            f << request_id << "," << step << "," << i << ",";
            f << (i < time_steps.size() ? time_steps[i] : 0.0);
            for(size_t j = 0; j < hardware_angles[i].size(); j++)f << "," << hardware_angles[i][j];
            f << "\n";
        }
    }

    double milliseconds_since(std::chrono::steady_clock::time_point start){
        return std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - start).count();
    }

    // 往事件文件追加一行 JSON
    static void write_event(uint32_t request_id, int step, const char* event_name,
                            const std::string& extra_fields){
        if(g_event_path.empty())return;

        std::ostringstream line;
        line << "{\"t\":" << g_node->now().seconds()
             << ",\"req\":" << request_id
             << ",\"kind\":" << step
             << ",\"evt\":\"" << event_name << "\"";
        if(!extra_fields.empty())line << "," << extra_fields;
        line << "}";

        std::ofstream f(g_event_path, std::ios::app);
        if(f.is_open())f << line.str() << "\n";
    }

    void record_plan_result(uint32_t request_id, int step, bool success,
                            double elapsed_millis, int point_count,
                            const std::string& fail_reason){
        std::ostringstream fields;
        fields << "\"plan_ms\":" << elapsed_millis;
        if(success)fields << ",\"pts\":" << point_count;
        else       fields << ",\"reason\":\"" << fail_reason << "\"";
        write_event(request_id, step, success ? "PLAN_OK" : "PLAN_FAIL", fields.str());
    }

    void record_send_result(uint32_t request_id, int step, bool aborted,
                            int sent_points, int planned_points,
                            double estimated_millis, double actual_millis){
        std::ostringstream fields;
        fields << "\"sent\":" << sent_points
               << ",\"pts\":" << planned_points
               << ",\"est_ms\":" << estimated_millis
               << ",\"real_ms\":" << actual_millis;
        write_event(request_id, step, aborted ? "SEND_ABORT" : "SEND_DONE", fields.str());
    }

    void record_arrival(uint32_t request_id, int step,
                        double position_error_mm, double rotation_error_deg){
        std::ostringstream fields;
        fields << "\"res_p_mm\":" << position_error_mm
               << ",\"res_r_deg\":" << rotation_error_deg
               << ",\"step\":" << step;
        write_event(request_id, step, "ARRIVED", fields.str());
    }

    void record_failure(uint32_t request_id, int step, const char* reason){
        std::ostringstream fields;
        fields << "\"reason\":\"" << reason << "\"";
        write_event(request_id, step, "FAIL", fields.str());
    }
}