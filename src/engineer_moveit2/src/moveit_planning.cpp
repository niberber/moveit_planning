#include <rclcpp/rclcpp.hpp>                                // 用于创建节点
#include <moveit/planning_interface/planning_interface.h>   // 用于创建规划接口
#include <moveit/robot_model_loader/robot_model_loader.h>   // 用于加载机器人模型
#include <moveit/planning_pipeline/planning_pipeline.h>     // 用于创建规划管道
#include <moveit/kinematic_constraints/utils.h>             // 用于定义约束
#include <moveit/robot_model/robot_model.h>                 // 用于加载机器人模型
#include <moveit/planning_scene/planning_scene.h>           // 用于创建规划场景
#include <moveit/collision_detection/collision_common.h>    // 用于碰撞检测
#include <fstream>                                          // 用于读取URDF文件
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>          // 用于转换几何消息
#include <termios.h>                                        // 用于串口配置
#include <fcntl.h>                                          // 用于open
#include <unistd.h>                                         // 用于close/read/write
#include <thread>                                           // 用于多线程
#include <mutex>                                            // 用于线程同步
#include <atomic>                                           // 用于线程安全的布尔值
#include <future>                                           // 用于异步操作(async)
#include <Eigen/Dense>                                      // 矩阵运算
#include <Eigen/Geometry>                                   // 旋转矩阵
#include <algorithm>
#include <vector>
#include <chrono>
#include <cstring>
#include <cstdio>
#include "feedback_pub.hpp"
#define NUM_JOINTS 7
uint8_t g_num_joints = NUM_JOINTS;

namespace protocal {
enum Cmd : uint16_t {
    PLAN_REQ      = 0x01,
    PLAN_RESP     = 0x02,
    PLAN_FAIL     = 0x03,
    PLAN_FEEDBACK = 0x04,
    PLAN_RECORD   = 0x05,
};
// ==================== 帧格式定义 ====================
// STM32 -> PC: CMD_PLAN_REQ (0x01) + joints[7](28)
// PC -> STM32: CMD_PLAN_RESP (0x02) + seq(1) + joints[7](28)
#pragma pack(push, 1)
struct PlanReq {
    uint16_t cmd;        // 0x01
    float joints[NUM_JOINTS]; // 目标关节角度
};

struct PlanResp {
    uint16_t cmd;        // 0x02
    uint8_t point_count; // 轨迹点数
    // 后面紧跟 point_count × 7 × 4 字节的关节角数据
};

struct PlanFeedback {
    uint16_t cmd;           // 0x04
    uint16_t step_idx;     // 当前执行到的轨迹点索引
    float joints[NUM_JOINTS]; // 当前关节角度
};
#pragma pack(pop)
}
static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_planning");


// ==================== CRC16-CCITT ====================
static const uint16_t CRC16_TABLE[256] = {
    0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
    0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
    0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
    0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
    0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
    0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
    0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
    0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
    0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
    0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
    0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
    0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
    0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
    0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
    0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
    0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
    0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
    0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
    0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
    0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
    0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
    0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
    0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
    0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
    0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
    0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
    0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
    0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
    0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
    0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
    0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
    0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
};

static uint16_t crc16(const uint8_t* data, size_t len){
    uint16_t crc = 0xFFFF;
    for(size_t i = 0; i < len; i++){
        uint8_t index = (crc ^ data[i]) & 0xFF;
        crc = (crc >> 8) ^ CRC16_TABLE[index];
    }
    return crc;
}

static rclcpp::Node::SharedPtr g_node;
static std::string g_serial_port = "/dev/ttyUSB0";
static int g_serial_fd = -1;

// 状态机状态
enum class State{
    IDLE,
    PLANNING,
    EXECUTING,
    REPLANNING
};
static std::atomic<State> g_state = State::IDLE; // 状态机状态
static std::atomic<bool> g_running = true;       // 是否在运行中

// 请求/反馈
static protocal::PlanReq g_recv_req;  // 接收的规划请求
static std::atomic<bool> g_new_plan = false; // 是否有新的规划请求
static std::atomic<bool> g_replan_requested = false;  // 监视线程请求
static int g_replan_attempts = 0;                     // 连续重规划失败次数

// PlanFeedback 缓冲区（无锁 atomic 供监视线程读）
static std::vector<double> g_feedback_joints(g_num_joints, 0.0);
static std::mutex g_feedback_mutex; // 保护反馈的互斥锁
static const double g_joint_offset[NUM_JOINTS] = {0.0, -0.006, -0.008, -0.030, 0.004, 0.808, 0.001};
// j9(roll)电机方向与URDF相反，反馈需取反
static const double g_joint_sign[NUM_JOINTS]  = {1.0, 1.0, 1.0, 1.0, -1.0, 1.0, 1.0};
static uint8_t g_send_seq = 0;  // 发送包序号

// 执行期监视
static std::atomic<int> g_exec_step = 0;            // 当前执行到第几步
static std::vector<std::vector<float>> g_exec_traj; // 正在执行轨迹（只读）
static std::mutex g_exec_mutex;                     // 保护执行轨迹的互斥锁

//moveit
static moveit::core::RobotModelPtr g_robot_model;
static const moveit::core::JointModelGroup* g_joint_group;
static planning_scene::PlanningScenePtr g_planning_scene;
static std::shared_ptr<planning_pipeline::PlanningPipeline> g_pipeline;


static bool open_serial() {
    g_serial_fd = open(g_serial_port.c_str(), O_RDWR | O_NOCTTY);
    if (g_serial_fd < 0) {
        RCLCPP_ERROR(LOGGER, "Failed to open %s", g_serial_port.c_str());
        return false;
    }
    
    struct termios tty;
    tcgetattr(g_serial_fd, &tty);
    cfsetispeed(&tty, B921600);
    cfsetospeed(&tty, B921600);
    tty.c_cflag &= ~PARENB;   // 无校验
    tty.c_cflag &= ~CSTOPB;   // 1位停止位
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;       // 8位数据
    tty.c_cflag &= ~CRTSCTS;  // 无流控
    tty.c_cflag |= (CREAD | CLOCAL);// 使能读取和本地模式
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);// 非规范模式
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);// 关闭流控制
    tty.c_oflag &= ~OPOST;    // 关闭输出处理
    tty.c_cc[VMIN] = 0;       // 非阻塞读
    tty.c_cc[VTIME] = 1;      // 0.1秒超时
    
    tcsetattr(g_serial_fd, TCSANOW, &tty);
    RCLCPP_INFO(LOGGER, "Serial opened: %s @ 921600", g_serial_port.c_str());
    return true;
}

static void close_serial() {
    if (g_serial_fd >= 0) {
        close(g_serial_fd);
        g_serial_fd = -1;
        RCLCPP_INFO(LOGGER, "Serial closed");
    }
}

static void send_frame(const uint8_t* data, size_t len) {
    uint8_t frame[len + 7];
    frame[0] = 0xAA;
    frame[1] = 0x55;
    frame[2] = g_send_seq++;
    frame[3] = (len - 4) & 0xFF;
    frame[4] = ((len - 4) >> 8) & 0xFF;
    memcpy(frame + 5, data, len);

    uint16_t crc = crc16(frame, len + 5);
    frame[len + 5] = crc & 0xFF;  // CRC高字节
    frame[len + 6] = (crc >> 8) & 0xFF;         // CRC低字节
    
    size_t total = len + 7, sent = 0;
    while(sent < total){
        size_t n = write(g_serial_fd, frame + sent, total - sent);
        if(n <= 0){
            if(errno == EINTR) continue;
            RCLCPP_ERROR(LOGGER, "Serial write failed: %s", strerror(errno));
            return;
        }
        sent += n;
    }
}

static bool parse_frame(const uint8_t* data, size_t len, protocal::PlanReq* out_req){
    //总长度 = 帧头(2) + seq(1) + 长度字段(2) + cmd(2) + 数据区 + CRC(2)
    if(len < 8)return false;

    if(data[0] != 0xAA || data[1] != 0x55)return false;

    uint16_t data_len = data[3] | (data[4] << 8);
    // RCLCPP_INFO(LOGGER, "data_len=%d", data_len);
    size_t total = 5 + 2 + data_len + 2;  //帧头(2)+seq(1)+长度(2)+cmd(2)+数据区+CRC(2)
    // RCLCPP_INFO(LOGGER, "total=%zu", total);
    if(len < total)return false;
    memcpy(out_req, data + 5, sizeof(protocal::PlanReq));
    uint16_t recv_crc = (uint16_t)data[total - 2] | (data[total - 1] << 8);
    uint16_t calc_crc = crc16(data, total - 2);
    
    if(recv_crc != calc_crc){
        RCLCPP_ERROR(LOGGER, "CRC check failed: %d != %d", recv_crc, calc_crc);
        return false;
    }
    return true;
}

static bool parse_feedback_frame(const uint8_t* data, size_t len, protocal::PlanFeedback* out_fb){
    if(data[0] != 0xAA || data[1] != 0x55)return false;
    uint16_t data_len = data[3] | (data[4] << 8);
    size_t total = 5 + 2 + data_len + 2;
    if(len < total)return false;

    memcpy(out_fb, data + 5, sizeof(protocal::PlanFeedback));

    uint16_t recv_crc = (uint16_t)data[total - 2] | (data[total - 1] << 8);
    uint16_t calc_crc = crc16(data, total - 2);
    if(recv_crc != calc_crc)return false;
    return true;
}
static bool init_moveit() {
    // =====================================
    // 1. 读取URDF和SRDF文件
    // =====================================
    std::ifstream urdf_f("/home/yh/2026_Engineer_ws/src/engineer/urdf/engineer.urdf");
    std::string urdf_str((std::istreambuf_iterator<char>(urdf_f)), std::istreambuf_iterator<char>());
    std::ifstream srdf_f("/home/yh/2026_Engineer_ws/src/engineer_moveit2/config/engineer.srdf");
    std::string srdf_str((std::istreambuf_iterator<char>(srdf_f)), std::istreambuf_iterator<char>());

    // =====================================
    // 2. 声明运动学参数
    // =====================================
    g_node->declare_parameter("robot_description", urdf_str);
    g_node->declare_parameter("arm_group.kinematics_solver", "kdl_kinematics_plugin/KDLKinematicsPlugin");
    g_node->declare_parameter("arm_group.kinematics_solver_search_resolution", 0.005);
    g_node->declare_parameter("arm_group.kinematics_solver_timeout", 1.0);

    // =====================================
    // 3. 创建机器人模型
    // =====================================
    robot_model_loader::RobotModelLoader::Options opts(urdf_str, srdf_str);
    robot_model_loader::RobotModelLoader loader(g_node, opts);
    g_robot_model = loader.getModel();

    // 获取规划组
    g_joint_group = g_robot_model->getJointModelGroup("arm_group");

    // 创建规划场景
    g_planning_scene = std::make_shared<planning_scene::PlanningScene>(g_robot_model);
    moveit::core::RobotState& cs = g_planning_scene->getCurrentStateNonConst();
    cs.setJointGroupPositions(g_joint_group, std::vector<double>(g_joint_group->getVariableCount(), 0.0));
    g_planning_scene->setCurrentState(cs);

    // 创建规划管道
    g_node->declare_parameter("planning_pipeline.planning_plugin", rclcpp::ParameterValue("ompl_interface/OMPLPlanner"));
    g_pipeline = std::make_shared<planning_pipeline::PlanningPipeline>(g_robot_model, g_node, "planning_pipeline");
    RCLCPP_INFO(LOGGER, "MoveIt initialized: %d joints", g_joint_group->getVariableCount());
    g_num_joints = g_joint_group->getVariableCount();
    return true;

}

// 关节角 → TCP位姿 的FK转换函数，直接返回 Isometry3d 避免欧拉角多解
static Eigen::Isometry3d joints_to_pose(const std::vector<double>& joints){
    moveit::core::RobotState rs(g_robot_model);
    rs.setJointGroupPositions(g_joint_group, joints);
    return rs.getGlobalLinkTransform("link11");
}

static std::vector<std::vector<float>> best_plan_(const Eigen::Isometry3d& target){
    const char* planners[] = {
        "RRTConnectkConfigDefault",
        "RRTkConfigDefault",
        "PRMConfigDefault",
        "LBKPIECEkConfigDefault"
    };
    static const double timeouts[] = {0.5, 0.5, 1.0, 1.0};
    std::vector<std::vector<float>> best_trajectory;
    
    // 设置起始状态
    moveit::core::RobotState start_state(g_planning_scene->getCurrentStateNonConst());

    std::vector<double> start_joints;
    std::lock_guard<std::mutex> lock(g_feedback_mutex);
    start_joints.assign(g_feedback_joints.begin(), g_feedback_joints.end());
    if(start_joints.empty() || start_joints.size() < g_joint_group->getVariableCount()){
        start_joints.assign(g_joint_group->getVariableCount(), 0.0);//如果没有接收到反馈，默认0.0
    }
    start_state.setJointGroupPositions(g_joint_group, start_joints);
    
    // 检查起始状态是否违反关节限制，若则调整为符合限制的值
    start_state.enforceBounds();
    g_planning_scene->setCurrentState(start_state);

    // =====================================
    // 步骤1: IK求解 - 将笛卡尔目标(xyz + rpy)转换为关节角度
    // =====================================
    // CHOMP只支持关节空间目标，不支持笛卡尔空间目标
    // 因此需要先用IK把目标位姿转为关节角度，再用关节约束做规划

    // 1.1 创建目标位姿（笛卡尔空间）
    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = target.translation().x();
    target_pose.position.y = target.translation().y();
    target_pose.position.z = target.translation().z();
    {
        Eigen::Quaterniond q(target.rotation());
        target_pose.orientation.w = q.w();
        target_pose.orientation.x = q.x();
        target_pose.orientation.y = q.y();
        target_pose.orientation.z = q.z();
    }

    // 1.2 调用 IK solver
    //     TRAC-IK（kinematics.yaml 中 solve_type=Distance）内部并发跑 KDL 与 SQP，
    //     并返回离种子关节角最近的解，7DoF 冗余臂不会跳到另一支，
    //     因此无需一致性限制、随机重启或碰撞过滤等手写兜底。
    auto kin_solver = g_joint_group->getSolverInstance();
    bool found = false;
    std::vector<double> solution(g_joint_group->getVariableCount());
    moveit_msgs::msg::MoveItErrorCodes ec;

    if (kin_solver) {
        found = kin_solver->searchPositionIK(target_pose, start_joints, 0.1, solution, ec);
    }

    if (!found) {
        RCLCPP_ERROR(LOGGER, "IK failed (no valid solution within bounds) for target (%.3f, %.3f, %.3f)",
            target.translation().x(), target.translation().y(), target.translation().z());
        return {};
    }
    RCLCPP_INFO(LOGGER, "IK solved: joints = [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
        solution[0], solution[1], solution[2], solution[3], solution[4], solution[5], solution[6]);

    // 1.5 用IK结果创建目标RobotState（关节空间）
    moveit::core::RobotState goal_state(g_robot_model);
    goal_state.setJointGroupPositions(g_joint_group, solution);

    // 碰撞诊断：检查起始状态和目标状态的自碰撞
    {
        collision_detection::CollisionRequest creq;
        creq.group_name = "arm_group";
        creq.distance = true;
        creq.contacts = true;
        creq.max_contacts = 100;
        creq.max_contacts_per_pair = 100;
        auto cscene = g_planning_scene->diff();
        {
            moveit::core::RobotState st(g_robot_model);
            st.setJointGroupPositions(g_joint_group, start_joints);
            collision_detection::CollisionResult cres;
            cscene->checkCollision(creq, cres, st);
            RCLCPP_INFO(LOGGER, "Start collision: %d contacts, collision=%d", (int)cres.contacts.size(), (int)cres.collision);
            for(auto& c : cres.contacts)
                RCLCPP_INFO(LOGGER, "  %s <-> %s", c.first.first.c_str(), c.first.second.c_str());
        }
        {
            collision_detection::CollisionResult cres;
            cscene->checkCollision(creq, cres, goal_state);
            RCLCPP_INFO(LOGGER, "Goal collision: %d contacts, collision=%d", (int)cres.contacts.size(), (int)cres.collision);
            for(auto& c : cres.contacts)
                RCLCPP_INFO(LOGGER, "  %s <-> %s", c.first.first.c_str(), c.first.second.c_str());
        }
    }

    // 1.6 创建关节空间约束（而不是pose约束）
    //    注意：constructGoalConstraints第3、4参数是位置/方向容差
    //    这里0.01表示关节位置容差约0.01弧度
    moveit_msgs::msg::Constraints goal = kinematic_constraints::constructGoalConstraints(goal_state, g_joint_group, 0.01, 0.01);
    
    // 1.7 调用规划管道，生成轨迹
    planning_interface::MotionPlanRequest req;
    req.group_name = "arm_group";
    req.goal_constraints.push_back(goal);
    // 单规划器调用（RRTConnect）
    auto scene = g_planning_scene->diff();
    planning_interface::MotionPlanRequest r = req;
    r.planner_id = planners[0];
    r.allowed_planning_time = timeouts[0];
    planning_interface::MotionPlanResponse best_res;
    g_pipeline->generatePlan(scene, r, best_res);
    if(best_res.error_code_.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS){
        RCLCPP_ERROR(LOGGER, "RRTConnect failed");
        return {};
    }
    
    // 解析轨迹
    moveit_msgs::msg::RobotTrajectory traj_msg;
    best_res.trajectory_->getRobotTrajectoryMsg(traj_msg);
    auto& pts = traj_msg.joint_trajectory.points;
    
    if(pts.empty()){
        RCLCPP_ERROR(LOGGER, "Planning returned empty trajectory");
        return {};
    }
    for(size_t i = 0; i < pts.size(); i++){
        float max_diff = 0.0;
        if(!best_trajectory.empty()){
            auto& last_pt = best_trajectory.back();
            for(int j = 0; j < g_num_joints; j++){
                max_diff = std::max(max_diff, (float)std::fabs(pts[i].positions[j] - last_pt[j]));
            }
        }
        if(best_trajectory.empty() || max_diff > 0.1 || i == pts.size() - 1){
            best_trajectory.push_back(std::vector<float>(pts[i].positions.begin(), pts[i].positions.end()));
        }
    }
    RCLCPP_INFO(LOGGER, "Planning succeeded: %zu trajectory points", best_trajectory.size());
    return best_trajectory;
}

// ==================== III型约束规划：Viterbi离散图搜索 ====================
// 五自由度约束：末端位置(x,z)固定 + 轴方向固定 + roll自由
// 沿任务进度 ξ 和 roll 角离散采样，分层最短路求解
static std::vector<std::vector<float> > constrained_plan_(
    const Eigen::Isometry3d& Tb_E,     // 装配站在机器人坐标下的位姿
    double xi_start, double xi_end,    // 任务进度(m), 如沿装配站z轴滑移
    int N,                             // 进度采样数
    double roll_min, double roll_max,  // roll的范围
    int M)                              // roll的采样数
 {
    const auto& kin_solver = g_joint_group->getSolverInstance();
    if(!kin_solver){
        RCLCPP_ERROR(LOGGER, "No IK solver");
        return {};
    }
    const double INF = 1e9;
    const int W = 3; //roll 带宽：相邻层允许跳变的索引数

    // 1.构建分层图节点
    // nodes[i][j] = 第i层进度，第j个roll采样点的关节状态
    std::vector<std::vector<std::vector<double>>> nodes(N, std::vector<std::vector<double>>(M));
    std::vector<std::vector<bool>> vis(N, std::vector<bool>(M, false));

    for(int i = 0; i < N; i++){
        double xi = xi_start + (xi_end - xi_start) * i / (N - 1); // 进度采样点x(i)
        for(int j = 0; j < M; j++){
            double r = roll_min + (roll_max - roll_min) * j / (M - 1); // roll采样点r
            // T_target = Tb_E * Translate(x(i)) * RotX(r)  （任务轴 = E系z轴）
            Eigen::Isometry3d T_target = Tb_E * Eigen::Translation3d(0, 0, xi) * Eigen::AngleAxisd(r, Eigen::Vector3d::UnitZ());
            geometry_msgs::msg::Pose pose;
            pose.position.x = T_target.translation().x();
            pose.position.y = T_target.translation().y();
            pose.position.z = T_target.translation().z();
            Eigen::Quaterniond q(T_target.rotation());
            pose.orientation.w = q.w();
            pose.orientation.x = q.x();
            pose.orientation.y = q.y();
            pose.orientation.z = q.z();

            // IK种子：优先用上一层同roll的解
            std::vector<double> seed(g_num_joints, 0.0);
            if(i > 0 && vis[i - 1][j]) seed = nodes[i - 1][j];
            std::vector<double> sol;
            moveit_msgs::msg::MoveItErrorCodes error_code;
            if(kin_solver->searchPositionIK(pose, seed, 0.1, sol, error_code)){
                nodes[i][j] = sol;
                vis[i][j] = true;
            }
        }
    }

    // 2.Viterbi 递推
    std::vector<std::vector<double> > dp(N, std::vector<double>(M, INF));
    std::vector<std::vector<int> > prev(N, std::vector<int>(M, -1)); // 记录上一层的索引
    for(int j = 0; j < M; j++){
        if(vis[0][j])dp[0][j] = 0.0;
    }
    for(int i = 0; i < N - 1; i++) {
        for(int j2 = 0; j2 < M; j2++){
            if(!vis[i + 1][j2])continue;
            for(int j1 = 0; j1 < M; j1++){
                if(!vis[i][j1])continue;
                // 带宽限制：只允许跳变W个索引
                int dist = std::abs(j2 - j1);
                int wrap_dist = std::min(dist, M - dist);
                if(wrap_dist > W)continue;

                double cost = 0.0;
                for(int k = 0; k < g_num_joints; k++){
                    double d = nodes[i][j1][k] - nodes[i + 1][j2][k]; // 计算关节角度差的欧氏距离
                    cost += d * d;
                }
                if(dp[i][j1] + cost < dp[i + 1][j2]){
                    dp[i + 1][j2] = dp[i][j1] + cost;
                    prev[i + 1][j2] = j1;
                }
            }
        }
    }
    // 3.回溯最优路径
    int best_j = 0;
    for(int j = 0; j < M; j++){
        if(dp[N - 1][j] < dp[N - 1][best_j])best_j = j;
    }
    if(dp[N - 1][best_j] >= INF){
        RCLCPP_ERROR(LOGGER, "Constrained Planning: no path found");
        return {};
    }

    std::vector<std::vector<float> > result(N);
    std::vector<int> path(N);
    int cur = best_j;
    for(int i = N - 1; i >= 0; i--)path[i] = cur, cur = prev[i][cur];
    for(int i = 0; i < N; i++){
        result[i].assign(nodes[i][path[i]].begin(), nodes[i][path[i]].end()); // 复制最优路径的关节角度
    }
    RCLCPP_INFO(LOGGER, "Constrained plan: %d points", N);
    return result;
}
// 监视线程：50HZ检查反馈偏差
static void monitor_thread_func(){
    const double diviation_threshold = 5.0 * M_PI / 180.0; // 角度偏差阈值5°
    const int consecutive_fails = 3;          // 连续3次失败后认为偏离计划
    int fail_count = 0;

    while(g_running){
        std::this_thread::sleep_for(std::chrono::milliseconds(20));

        if(g_state.load() != State::EXECUTING){
            fail_count = 0;
            continue;
        }
        
        std::vector<double> actual;
        {
            std::lock_guard<std::mutex> lock(g_feedback_mutex);
            actual = g_feedback_joints;
        }
        std::vector<float> excepted;
        {
            std::lock_guard<std::mutex> lock(g_exec_mutex);
            int step = g_exec_step.load();
            if(step >= 0 && step < (int)g_exec_traj.size())excepted = g_exec_traj[step];
        }
        if(actual.empty() || excepted.empty())continue;
        double maxn = 0.0;
        for(int i = 0; i < g_num_joints; i++)maxn = std::max(maxn, std::fabs(actual[i] - excepted[i]));

        if(maxn > diviation_threshold){
            fail_count++;
            if(fail_count >= consecutive_fails){
                RCLCPP_WARN(LOGGER, "Deviation %.3f persists, request replan", maxn);
                g_replan_requested = true;
                g_state = State::REPLANNING;
            }
        } else {
            fail_count = 0;
        }
    }
}

static void serial_thread_func() {
    uint8_t tmp[256];
    static std::vector<uint8_t> rx_buffer;

    auto last_log = std::chrono::steady_clock::now();

    while(g_running) {
        int bytes = read(g_serial_fd, tmp, sizeof(tmp));
        if(bytes <= 0)continue;

        rx_buffer.insert(rx_buffer.end(), tmp, tmp + bytes);
        
        int i = 0;
        for(i = 0; i < rx_buffer.size(); ){
            if(rx_buffer[i] == 0xAA && rx_buffer[i + 1] == 0x55){
                if(i + 4 >= (int)rx_buffer.size())break;
                uint16_t frame_len = rx_buffer[i + 3] | (rx_buffer[i + 4] << 8);
                size_t total = 5 + 2 + frame_len + 2; // 完整帧长度

                if(i + total > (int)rx_buffer.size())break;

                uint16_t recv_cmd = rx_buffer[i + 5] | (rx_buffer[i + 6] << 8);
                if(recv_cmd == protocal::Cmd::PLAN_REQ){
                    protocal::PlanReq req;
                    if(parse_frame(rx_buffer.data() + i, total, &req)){
                        g_recv_req = req;       // 保存新计划
                        g_new_plan = true;      // 标记有新计划
                        g_state = State::PLANNING;
                        RCLCPP_INFO(LOGGER, "Received PlanReq: joints = [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]", req.joints[0], req.joints[1], req.joints[2], req.joints[3], req.joints[4], req.joints[5], req.joints[6]);
                    } else {
                        RCLCPP_ERROR(LOGGER, "Failed to parse PlanReq");
                    }
                }
                else if(recv_cmd == protocal::Cmd::PLAN_FEEDBACK){
                    protocal::PlanFeedback fb;
                    if(parse_feedback_frame(rx_buffer.data() + i, total, &fb)){
                        
                        std::lock_guard<std::mutex> lock(g_feedback_mutex);

                        for(int k = 0; k < g_num_joints; k++)g_feedback_joints[k] = g_joint_sign[k] * fb.joints[k] * M_PI / 180.0 - g_joint_offset[k];

                        g_exec_step = fb.step_idx;
                        RCLCPP_INFO(LOGGER, "Received PlanFeedback: joints = [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f] step=%d", g_feedback_joints[0], g_feedback_joints[1], g_feedback_joints[2], g_feedback_joints[3], g_feedback_joints[4], g_feedback_joints[5], g_feedback_joints[6], fb.step_idx);
                    } else {
                        RCLCPP_ERROR(LOGGER, "Failed to parse PlanFeedback");
                        
                    }
                }
                else if(recv_cmd == protocal::Cmd::PLAN_RECORD){
                    float raw[7], arm[7];
                    memcpy(raw, rx_buffer.data() + i + 7, sizeof(raw));
                    for(int i = 0; i < 7; i++){
                        arm[i] = g_joint_sign[i] * raw[i] - g_joint_offset[i];
                    }
                    feedback_pub::publish_feedback(arm);
                }
                i += total;
            } else i++;
        }
        if(i > 0){
            rx_buffer.erase(rx_buffer.begin(), rx_buffer.begin() + i);
        }
        usleep(10000);
    }
}

// 主循环：状态机调度
static void send_trajectory(const std::vector<std::vector<float> >& traj){
    for(size_t i = 0; i < traj.size(); i++){
        if(!g_running)break;
        std::vector<uint8_t> buf;
        buf.push_back(0x02);
        buf.push_back(0x00);
        buf.push_back(i & 0xFF);
        buf.push_back(i >> 8);
        for(size_t j = 0; j < traj[i].size(); j++){
            float stm32_angle = g_joint_sign[j] * traj[i][j] + g_joint_offset[j];
            uint8_t* p = (uint8_t*)&stm32_angle;
            for(int k = 0; k < 4; k++)buf.push_back(p[k]);

        }
        RCLCPP_INFO(LOGGER, "Sent joints = [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]", traj[i][0], traj[i][1], traj[i][2], traj[i][3], traj[i][4], traj[i][5], traj[i][6]);
        send_frame(buf.data(), buf.size());
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

static void start_execution(const std::vector<std::vector<float> >& traj){
    {
        std::lock_guard<std::mutex> lock(g_exec_mutex);
        g_exec_traj = traj;
        g_exec_step = 0;
    }
    g_state = State::EXECUTING;
}

static void planning_thread_func(){
    while(g_running){
        switch(g_state.load(std::memory_order_acquire)){
            case State::IDLE: {

                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                break;
            }
            case State::PLANNING: {
                g_new_plan = false;

                std::vector<double> target_joints(g_num_joints);
                for(int k = 0; k < g_num_joints; k++){
                    target_joints[k] = g_joint_sign[k] * g_recv_req.joints[k] - g_joint_offset[k];
                }
                Eigen::Isometry3d target = joints_to_pose(target_joints);

                // 用四元数打印，避免 eulerAngles 的多解问题
                Eigen::Quaterniond q_print(target.rotation());
                RCLCPP_INFO(LOGGER, "Planning: (%.3f,%.3f,%.3f) quat(%.3f,%.3f,%.3f,%.3f)",
                        target.translation().x(), target.translation().y(), target.translation().z(),
                        q_print.w(), q_print.x(), q_print.y(), q_print.z());

                auto traj = best_plan_(target);
                
                if(!traj.empty()){
                    send_trajectory(traj);
                    size_t total_frames = traj.size();
                    RCLCPP_INFO(LOGGER, "Sent %zu points, entering EXECUTING", total_frames);
                    start_execution(traj);
                } else {
                    uint16_t fail = 0x03;
                    send_frame((uint8_t*)&fail, 2);
                    g_state = State::IDLE;
                    RCLCPP_ERROR(LOGGER, "Plan failed");
                }
                break;
            }
            case State::EXECUTING:{
                if(g_replan_requested.load()){
                    g_replan_requested = false;
                    RCLCPP_INFO(LOGGER, "Replan requested during execution");
                    g_state = State::REPLANNING;
                } else {
                    // 完成判定：反馈关节角与轨迹终点各轴足够接近
                    std::vector<double> actual;
                    {
                        std::lock_guard<std::mutex> lock(g_feedback_mutex);
                        actual = g_feedback_joints;
                    }
                    bool arrived = !actual.empty();
                    if(arrived){
                        std::lock_guard<std::mutex> lock(g_exec_mutex);
                        auto& last_pt = g_exec_traj.back();
                        for(int j = 0; j < g_num_joints; j++){
                            if(std::fabs(actual[j] - last_pt[j]) > 0.5){ arrived = false; break; }
                        }
                    }
                    if(arrived){
                        g_state = State::IDLE;
                        RCLCPP_INFO(LOGGER, "Execution complete, back to IDLE");
                    }
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
                break;
            }
            case State::REPLANNING: {
                g_replan_requested = false;
                //从当前实际关节状态重新规划
                RCLCPP_INFO(LOGGER, "Replanning from current state...");
                std::vector<double> target_joints(g_num_joints);
                for(int k = 0; k < g_num_joints; k++){
                    target_joints[k] = g_joint_sign[k] * g_recv_req.joints[k] - g_joint_offset[k];
                }
                Eigen::Isometry3d target = joints_to_pose(target_joints);

                auto traj = best_plan_(target);
                if(!traj.empty()){
                    send_trajectory(traj);
                    start_execution(traj);
                    g_replan_attempts = 0;
                    RCLCPP_INFO(LOGGER, "Replan succeeded");
                } else if(g_replan_attempts < 3){
                    // 重规划失败不直接停死，回到 EXECUTING，由监视线程再次触发重试
                    g_replan_attempts++;
                    g_state = State::EXECUTING;
                    RCLCPP_WARN(LOGGER, "Replan failed (%d/3), back to EXECUTING and retry", g_replan_attempts);
                } else {
                    uint16_t fail = 0x03;
                    send_frame((uint8_t*)&fail, 2);
                    g_state = State::IDLE;
                    g_replan_attempts = 0;
                    RCLCPP_ERROR(LOGGER, "Replan failed after 3 attempts, stop");
                }
                break;
            }
        }
    }
}

int main(int argc, char** argv){
    if(argc > 1) g_serial_port = argv[1];

    rclcpp::init(argc, argv);
    g_node = std::make_shared<rclcpp::Node>("moveit_planning");
    feedback_pub::init(g_node);
    if(!open_serial())return 1; // 打开串口
    if(!init_moveit()){
        RCLCPP_FATAL(LOGGER, "MoveIt! init failed");
        return 1;
    }
    RCLCPP_INFO(LOGGER, "System ready, waiting for requests...");
    
    // 启动串口线程和规划线程
    std::thread serial_thr(serial_thread_func);
    std::thread plan_thr(planning_thread_func);
    std::thread monitor_thr(monitor_thread_func);
    while(rclcpp::ok()){
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    // 停止线程
    RCLCPP_INFO(LOGGER, "Shutting down...");
    g_running = false;
    g_state = State::IDLE;
    close_serial();
    serial_thr.join();
    plan_thr.join();
    monitor_thr.join();

    rclcpp::shutdown();
    return 0;
}