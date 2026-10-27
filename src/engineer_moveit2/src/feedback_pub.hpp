#pragma once
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <chrono>
#include <string>
#include <vector>

namespace feedback_pub{
    // 初始化：建立 /joint_states 并生成本次运行的记录文件名
    void init(rclcpp::Node::SharedPtr node);

    // 关节反馈 -> /joint_states（下位机原始角）
    void publish_feedback(const float arm[7]);

    // 反馈落 CSV（调试辅助）
    void save_feedback(int step_idx, const float arm[7]);

    // 下发轨迹落 CSV：hardware_angles 为已换算的下位机 raw 角，time_steps 为每点的真实下发间隔(s)
    void save_cmd_traj(uint32_t request_id, int step,
                       const std::vector<std::vector<float> >& hardware_angles,
                       const std::vector<double>& time_steps);

    // 距 start 的毫秒数，用于统计耗时
    double milliseconds_since(std::chrono::steady_clock::time_point start);

    // 规划结果：success 为 false 时 fail_reason 说明原因
    void record_plan_result(uint32_t request_id, int step, bool success,
                            double elapsed_millis, int point_count,
                            const std::string& fail_reason);

    // 下发实况：aborted 为 true 表示中途被打断
    void record_send_result(uint32_t request_id, int step, bool aborted,
                            int sent_points, int planned_points,
                            double estimated_millis, double actual_millis);

    // 执行到位残差
    void record_arrival(uint32_t request_id, int step,
                        double position_error_mm, double rotation_error_deg);

    // 执行阶段失败（卡死等）
    void record_failure(uint32_t request_id, int step, const char* reason);
}
