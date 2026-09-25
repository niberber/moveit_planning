// 完全复用 moveit_planning.cpp 的初始化方式 + constrained_plan_ 测试 + RViz 可视化
#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <fstream>
#include <Eigen/Dense>
#include <Eigen/Geometry>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("test_constrained");
static moveit::core::RobotModelPtr g_robot_model;
static const moveit::core::JointModelGroup* g_joint_group;
static planning_scene::PlanningScenePtr g_planning_scene;
static std::shared_ptr<planning_pipeline::PlanningPipeline> g_pipeline;

static bool init_moveit(rclcpp::Node::SharedPtr node) {
    std::ifstream urdf_f("/home/yh/2026_Engineer_ws/src/engineer/urdf/engineer.urdf");
    std::string urdf_str((std::istreambuf_iterator<char>(urdf_f)), std::istreambuf_iterator<char>());
    std::ifstream srdf_f("/home/yh/2026_Engineer_ws/src/engineer_moveit2/config/engineer.srdf");
    std::string srdf_str((std::istreambuf_iterator<char>(srdf_f)), std::istreambuf_iterator<char>());

    node->declare_parameter("robot_description", urdf_str);
    node->declare_parameter("arm_group.kinematics_solver", "kdl_kinematics_plugin/KDLKinematicsPlugin");
    node->declare_parameter("arm_group.kinematics_solver_search_resolution", 0.005);
    node->declare_parameter("arm_group.kinematics_solver_timeout", 1.0);

    robot_model_loader::RobotModelLoader::Options opts(urdf_str, srdf_str);
    robot_model_loader::RobotModelLoader loader(node, opts);
    g_robot_model = loader.getModel();
    g_joint_group = g_robot_model->getJointModelGroup("arm_group");

    g_planning_scene = std::make_shared<planning_scene::PlanningScene>(g_robot_model);
    moveit::core::RobotState& cs = g_planning_scene->getCurrentStateNonConst();
    cs.setJointGroupPositions(g_joint_group, std::vector<double>(g_joint_group->getVariableCount(), 0.0));
    g_planning_scene->setCurrentState(cs);

    g_pipeline = std::make_shared<planning_pipeline::PlanningPipeline>(g_robot_model, node, "planning_pipeline");
    RCLCPP_INFO(LOGGER, "MoveIt initialized: %d joints", g_joint_group->getVariableCount());
    return true;
}

// ==================== III型约束规划（跟 moveit_planning.cpp 完全一样）====================
static std::vector<std::vector<double>> constrained_plan_(
    const Eigen::Isometry3d& Tb_E,
    double xi_start, double xi_end,
    int N,
    double roll_min, double roll_max,
    int M)
{
    const auto& kin_solver = g_joint_group->getSolverInstance();
    if (!kin_solver) { RCLCPP_ERROR(LOGGER, "No IK solver"); return {}; }

    const double INF = 1e9;
    const int W = 3;
    int nj = g_joint_group->getVariableCount();

    // 1. 构建分层图
    std::vector<std::vector<std::vector<double>>> nodes(N, std::vector<std::vector<double>>(M));
    std::vector<std::vector<bool>> vis(N, std::vector<bool>(M, false));

    for (int i = 0; i < N; i++) {
        double xi = xi_start + (xi_end - xi_start) * i / (N - 1);
        for (int j = 0; j < M; j++) {
            double r = roll_min + (roll_max - roll_min) * j / (M - 1);

            Eigen::Isometry3d T_target = Tb_E *
                Eigen::Translation3d(0, 0, xi) *
                Eigen::AngleAxisd(r, Eigen::Vector3d::UnitZ());

            geometry_msgs::msg::Pose pose;
            pose.position.x = T_target.translation().x();
            pose.position.y = T_target.translation().y();
            pose.position.z = T_target.translation().z();
            Eigen::Quaterniond q(T_target.rotation());
            pose.orientation.w = q.w(); pose.orientation.x = q.x();
            pose.orientation.y = q.y(); pose.orientation.z = q.z();

            std::vector<double> seed(nj, 0.0);
            if (i > 0 && vis[i-1][j]) seed = nodes[i-1][j];
            std::vector<double> sol;
            moveit_msgs::msg::MoveItErrorCodes ec;
            if (kin_solver->searchPositionIK(pose, seed, 0.2, sol, ec)) {
                nodes[i][j] = sol;
                vis[i][j] = true;
            }
        }
    }

    // 统计 IK 成功率
    int ok = 0, total = N * M;
    for (int i = 0; i < N; i++)
        for (int j = 0; j < M; j++)
            if (vis[i][j]) ok++;
    RCLCPP_INFO(LOGGER, "IK stats: %d/%d (%.0f%%)", ok, total, 100.0 * ok / total);

    // 2. Viterbi 递推
    std::vector<std::vector<double>> dp(N, std::vector<double>(M, INF));
    std::vector<std::vector<int>> prev(N, std::vector<int>(M, -1));
    for (int j = 0; j < M; j++)
        if (vis[0][j]) dp[0][j] = 0.0;

    for (int i = 0; i < N - 1; i++) {
        for (int j2 = 0; j2 < M; j2++) {
            if (!vis[i+1][j2]) continue;
            for (int j1 = 0; j1 < M; j1++) {
                if (!vis[i][j1]) continue;
                int dist = std::abs(j2 - j1);
                if (std::min(dist, M - dist) > W) continue;
                double cost = 0.0;
                for (int k = 0; k < nj; k++) {
                    double d = nodes[i][j1][k] - nodes[i+1][j2][k];
                    cost += d * d;
                }
                if (dp[i][j1] + cost < dp[i+1][j2]) {
                    dp[i+1][j2] = dp[i][j1] + cost;
                    prev[i+1][j2] = j1;
                }
            }
        }
    }

    int best_j = 0;
    for (int j = 0; j < M; j++)
        if (dp[N-1][j] < dp[N-1][best_j]) best_j = j;
    if (dp[N-1][best_j] >= INF) {
        RCLCPP_ERROR(LOGGER, "No path found");
        return {};
    }

    std::vector<std::vector<double>> result(N);
    std::vector<int> path(N);
    int cur = best_j;
    for (int i = N - 1; i >= 0; i--) { path[i] = cur; cur = prev[i][cur]; }
    for (int i = 0; i < N; i++)
        result[i] = nodes[i][path[i]];

    RCLCPP_INFO(LOGGER, "Constrained plan: %d points", N);
    return result;
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("test_constrained");

    if (!init_moveit(node)) {
        RCLCPP_FATAL(LOGGER, "Init failed");
        return 1;
    }

    auto kin_solver = g_joint_group->getSolverInstance();
    RCLCPP_INFO(LOGGER, "IK solver: %s", kin_solver ? "OK" : "NULL");
    if (!kin_solver) return 1;

    // ---- 先算 FK：从合理关节角算 TCP 位姿 ----
    Eigen::Isometry3d home_pose;
    {
        // 臂稍微伸出去一点的姿态（不全是 0）
        moveit::core::RobotState rs(g_robot_model);
        std::vector<double> init_joints = {0.0, 0.0, -0.3, -0.3, 0.0, -0.1, 0.0};
        rs.setJointGroupPositions(g_joint_group, init_joints);
        g_planning_scene->setCurrentState(rs);

        // FK: 算 TCP (link11) 在 base_link 下的位姿
        const Eigen::Isometry3d& tcp = rs.getGlobalLinkTransform("link11");
        home_pose = tcp;
        Eigen::Vector3d p = tcp.translation();
        Eigen::Quaterniond q(tcp.rotation());
        RCLCPP_INFO(LOGGER, "FK home: TCP at (%.3f, %.3f, %.3f) quat(%.3f %.3f %.3f %.3f)",
                    p.x(), p.y(), p.z(), q.w(), q.x(), q.y(), q.z());
    }

    // ---- 在 FK 算出来的位置附近测 IK ----
    {
        geometry_msgs::msg::Pose target;
        target.position.x = home_pose.translation().x();
        target.position.y = home_pose.translation().y();
        target.position.z = home_pose.translation().z();
        Eigen::Quaterniond q(home_pose.rotation());
        target.orientation.w = q.w(); target.orientation.x = q.x();
        target.orientation.y = q.y(); target.orientation.z = q.z();

        std::vector<double> seed, sol;
        g_planning_scene->getCurrentStateNonConst().copyJointGroupPositions(g_joint_group, seed);
        moveit_msgs::msg::MoveItErrorCodes ec;
        kinematics::KinematicsQueryOptions opt;
        bool ok = kin_solver->searchPositionIK(target, seed, 1.0, sol, ec, opt);
        RCLCPP_INFO(LOGGER, "IK at FK home: %s (err=%d)  joints=[%.2f %.2f %.2f %.2f %.2f %.2f %.2f]",
                    ok ? "OK" : "FAILED", ec.val,
                    ok ? sol[0] : 0, ok ? sol[1] : 0, ok ? sol[2] : 0,
                    ok ? sol[3] : 0, ok ? sol[4] : 0, ok ? sol[5] : 0, ok ? sol[6] : 0);

        // 如果 FK 位置 IK 成功，再测一个偏移位置
        if (ok) {
            target.position.x += 0.05;
            bool ok2 = kin_solver->searchPositionIK(target, sol, 1.0, sol, ec, opt);
            RCLCPP_INFO(LOGGER, "IK at offset (+0.05,0,0): %s", ok2 ? "OK" : "FAILED");
        }
    }

    // ---- 调 constrained_plan_ ----
    // Tb_E 位置 = FK home 位置，旋转 = 单位阵（E系 z 轴 = 世界 z 轴，即向上）
    Eigen::Isometry3d Tb_E = Eigen::Isometry3d::Identity();
    Tb_E.translation() = home_pose.translation();
    // 不改旋转 = 单位阵 → E系 z 轴朝上 → Translate(0,0,xi) 就是向上平移
    RCLCPP_INFO(LOGGER, "Running constrained_plan_ around (%.2f, %.2f, %.2f), translate +0.1m along world Z...",
                Tb_E.translation().x(), Tb_E.translation().y(), Tb_E.translation().z());
    auto traj = constrained_plan_(Tb_E, 0.0, 0.1, 10, -0.3, 0.3, 8);

    if (traj.empty()) {
        RCLCPP_FATAL(LOGGER, "Constrained plan failed");
        return 1;
    }

    // ---- 发布到 RViz ----
    // 方式A: /display_planned_path（ghost 路径）
    auto display_pub = node->create_publisher<moveit_msgs::msg::DisplayTrajectory>("/display_planned_path", 10);
    moveit_msgs::msg::DisplayTrajectory display_msg;
    display_msg.trajectory.resize(1);
    auto& robot_traj = display_msg.trajectory[0];
    robot_traj.joint_trajectory.header.frame_id = "base_link";
    robot_traj.joint_trajectory.header.stamp = node->get_clock()->now();
    auto jnames = g_joint_group->getActiveJointModelNames();
    for (auto& n : jnames) robot_traj.joint_trajectory.joint_names.push_back(n);
    for (auto& pt : traj) {
        trajectory_msgs::msg::JointTrajectoryPoint jtp;
        for (auto& v : pt) jtp.positions.push_back(v);
        robot_traj.joint_trajectory.points.push_back(jtp);
    }
    for (int i = 0; i < 5; i++) { display_pub->publish(display_msg); rclcpp::sleep_for(std::chrono::milliseconds(500)); }

    // 方式B: /joint_states（实际动）
    auto js_pub = node->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
    RCLCPP_INFO(LOGGER, "Playing trajectory...");
    rclcpp::Rate rate(50);
    while (rclcpp::ok()) {
        for (auto& pt : traj) {
            sensor_msgs::msg::JointState js;
            js.header.stamp = node->get_clock()->now();
            for (auto& n : jnames) js.name.push_back(n);
            for (auto& v : pt) { js.position.push_back(v); js.velocity.push_back(0); js.effort.push_back(0); }
            js_pub->publish(js);
            for (int k = 0; k < 10; k++) { rate.sleep(); if (!rclcpp::ok()) return 0; }
        }
        RCLCPP_INFO(LOGGER, "Cycle done, holding. Ctrl+C to exit.");
        while (rclcpp::ok()) {
            sensor_msgs::msg::JointState js;
            js.header.stamp = node->get_clock()->now();
            for (auto& n : jnames) js.name.push_back(n);
            for (auto& v : traj.back()) { js.position.push_back(v); js.velocity.push_back(0); js.effort.push_back(0); }
            js_pub->publish(js);
            rate.sleep();
        }
    }
    rclcpp::shutdown();
    return 0;
}
