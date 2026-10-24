#include <rclcpp/rclcpp.hpp>                                // 用于创建节点
#include <moveit/planning_interface/planning_interface.h>   // 用于创建规划接口
#include <moveit/robot_model_loader/robot_model_loader.h>   // 用于加载机器人模型
#include <moveit/planning_pipeline/planning_pipeline.h>     // 用于创建规划管道
#include <moveit/kinematic_constraints/utils.h>             // 用于定义约束
#include <moveit/robot_model/robot_model.h>                 // 用于加载机器人模型
#include <moveit/planning_scene/planning_scene.h>           // 用于创建规划场景
#include <moveit/collision_detection/collision_common.h>    // 用于碰撞检测
#include <moveit/robot_state/cartesian_interpolator.h>      // 用于笛卡尔直线规划
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
#include <cmath>
#include <cstring>
#include <cstdio>
#include "feedback_pub.hpp"
#define NUM_JOINTS 7
uint8_t g_num_joints = NUM_JOINTS;

// 标定模式：1=标定(跳过MoveIt)，0=正常规划。标定用法见 calib_loop()
#define CALIB_MODE 0

// 反馈录制开关：1=每帧0x04写CSV，0=关闭
#define RECORD_FEEDBACK 0

// 笛卡尔直线规划参数
#define CART_EEF_STEP 0.05       // 末端步长(m)
#define CART_JOINT_JUMP 0.5      // 相邻点关节跳变阈值(rad)，超此判奇异跳支

// 圆弧离散参数：每段转角(deg)，决定弦高误差 e = r*(1-cos(step/2))
#define ARC_STEP_DEG 3.0
#define ARC_MIN_SEGMENTS 6      // 最小段数，保证小角度也有足够执行时长

// 固件物理/动态限位(rad)：URDF 未覆盖或更宽，越限会被下位机夹住 → 关节≠下发 → 末端姿态漂移
static const double deg2rad = 3.14159265358979323846 / 180.0;
static const double rad2deg = 180.0 / 3.14159265358979323846;
static const double firmware_min[NUM_JOINTS] = {-115.3*deg2rad, 0.0, 1.0*deg2rad, -76.0*deg2rad, -323.0*deg2rad, -90.0*deg2rad, -224.0*deg2rad};
static const double firmware_max[NUM_JOINTS] = { 222.7*deg2rad, 105.0*deg2rad, 148.0*deg2rad, 0.0, 3.0*deg2rad, 90.0*deg2rad, 90.0*deg2rad};
static const double firmware_p2p1_max = 24.6*deg2rad;   // 动态限位: pitch2 <= pitch1 + 24.6°

// 卡死监视：到轨迹终点距离在 STALL_TIMEOUT_MS 内累计减小不足 PROGRESS_EPS(rad) 即判定卡死并重规划
#define STALL_TIMEOUT_MS 1000
#define PROGRESS_EPS 0.01

namespace protocal {
enum Cmd : uint16_t {
    PLAN_REQ      = 0x01,
    PLAN_RESP     = 0x02,
    PLAN_FAIL     = 0x03,
    PLAN_FEEDBACK = 0x04,
    PLAN_RECORD   = 0x05,
    PLAN_REQ3     = 0x06,
    PLAN_REQ_ARC  = 0x07,   // 绕固定轴旋转固定角度
};
// ==================== 帧格式定义 ====================
// STM32 -> PC: CMD_PLAN_REQ (0x01) + pose[6](24)   xyz[3] + rpy[3](固定轴XYZ)
// PC -> STM32: CMD_PLAN_RESP (0x02) + seq(1) + joints[7](28)
#pragma pack(push, 1)
struct PlanReq {
    uint16_t cmd;        // 0x01
    float pose[6];       // 目标TCP位姿: xyz(3) + rpy(3), base_link系
};

struct PlanResp {
    uint16_t cmd;        // 0x02
    uint8_t point_count; // 轨迹点数
    // 后面紧跟 point_count × 7 × 4 字节的关节角数据
};

struct PlanFeedback {
    uint16_t cmd;        // 0x04
    uint16_t step_idx;   // 当前执行到的轨迹点索引
    float joints[NUM_JOINTS]; // 当前关节角度
};

struct PlanReq3{
    uint16_t cmd;        // 0x06
    float delta[6];      // 基座系相对当前末端位姿的变换: [dx,dy,dz, drx,dry,drz]
};

struct PlanReqArc{
    uint16_t cmd;          // 0x07
    float axis_point[3];   // 轴上任意一点, base_link系 (m)
    float axis_dir[3];     // 旋转轴方向, base_link系, 不要求已归一化
    float theta;           // 带符号转角(rad), 右手定则, |theta| <= PI
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
static std::string g_serial_port = "/dev/ttyUSB1";
static int g_serial_fd = -1;

// 状态机状态
enum class State{
    IDLE,
    PLANNING,
    PLANNING3,
    PLANNING_ARC,
    EXECUTING
};
static std::atomic<State> g_state = State::IDLE; // 状态机状态
static std::atomic<bool> g_running = true;       // 是否在运行中

// 请求/反馈
static protocal::PlanReq g_recv_req;  // 接收的规划请求
static Eigen::Isometry3d g_cart_goal; // 0x06 解析出的绝对目标位姿
static protocal::PlanReqArc g_arc_req; // 0x07 圆弧请求
static std::mutex g_arc_mutex;         // 保护 g_arc_req（串口线程写，规划线程读）
static std::atomic<int> g_exec_fail_reason{0};  // 执行期失败原因：0=无 1=无进展卡死（监视线程写，规划线程统一处理）

// PlanFeedback 缓冲区（无锁 atomic 供监视线程读）
static std::vector<double> g_feedback_joints(g_num_joints, 0.0);
static std::mutex g_feedback_mutex; // 保护反馈的互斥锁
static std::atomic<int64_t> g_last_feedback_ms{0}; // 最近一次收到0x04反馈的时刻(steady_clock, ms)
static const double g_joint_offset[NUM_JOINTS] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
// 标定结论：7轴方向与URDF一致
static const double g_joint_sign[NUM_JOINTS]  = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
static uint8_t g_send_seq = 0;  // 发送包序号

// 执行期监视
static std::atomic<int> g_exec_step = 0;            // 当前执行到第几步
static int g_last_feedback_step = -1;                     // 已打印过的反馈 idx，用于边沿检测
static std::vector<std::vector<float>> g_exec_traj; // 正在执行轨迹（只读）
static std::mutex g_exec_mutex;                     // 保护执行轨迹的互斥锁
static int g_arrived_cnt = 0;                       // 连续满足到位阈值的帧数

//moveit
static moveit::core::RobotModelPtr g_robot_model;
static const moveit::core::JointModelGroup* g_joint_group;
static planning_scene::PlanningScenePtr g_planning_scene;
static std::shared_ptr<planning_pipeline::PlanningPipeline> g_pipeline;


static bool open_serial() {
    g_serial_fd = open(g_serial_port.c_str(), O_RDWR | O_NOCTTY);
    if (g_serial_fd < 0) {
        RCLCPP_ERROR(LOGGER, "串口打开失败: %s", g_serial_port.c_str());
        return false;
    }

    struct termios tty;
    tcgetattr(g_serial_fd, &tty);
    cfmakeraw(&tty);
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
    RCLCPP_INFO(LOGGER, "串口已打开: %s @ 921600", g_serial_port.c_str());
    return true;
}

static void close_serial() {
    if (g_serial_fd >= 0) {
        close(g_serial_fd);
        g_serial_fd = -1;
        RCLCPP_INFO(LOGGER, "串口已关闭");
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
            RCLCPP_ERROR(LOGGER, "串口写入失败: %s", strerror(errno));
            return;
        }
        sent += n;
    }
}

static bool parse_frame(const uint8_t* data, size_t len, protocal::PlanReq* out_req){
    //总长度 = 帧头(2) + seq(1) + 长度字段(2) + cmd(2) + 数据区 + CRC(2)
    if(len < 8)return false;

    uint16_t data_len = data[3] | (data[4] << 8);
    size_t total = 5 + 2 + data_len + 2;  //帧头(2)+seq(1)+长度(2)+cmd(2)+数据区+CRC(2)
    if(len < total)return false;

    memcpy(out_req, data + 5, sizeof(protocal::PlanReq));

    uint16_t recv_crc = (uint16_t)data[total - 2] | (data[total - 1] << 8);
    uint16_t calc_crc = crc16(data, total - 2);
    if(recv_crc != calc_crc){
        RCLCPP_ERROR(LOGGER, "CRC 校验失败: %d != %d", recv_crc, calc_crc);
        return false;
    }
    return true;
}
static bool parse_plan3_frame(const uint8_t* data, size_t len, protocal::PlanReq3* out_req){
    //总长度 = 帧头(2) + seq(1) + 长度字段(2) + cmd(2) + 数据区 + CRC(2)
    if(len < 8)return false;

    uint16_t data_len = data[3] | (data[4] << 8);
    size_t total = 5 + 2 + data_len + 2;
    if(len < total)return false;

    memcpy(out_req, data + 5, sizeof(protocal::PlanReq3));

    uint16_t recv_crc = (uint16_t)data[total - 2] | (data[total - 1] << 8);
    uint16_t calc_crc = crc16(data, total - 2);
    if(recv_crc != calc_crc){
        RCLCPP_ERROR(LOGGER, "CRC 校验失败: %d != %d", recv_crc, calc_crc);
        return false;
    }
    return true;
}

static bool parse_arc_frame(const uint8_t* data, size_t len, protocal::PlanReqArc* out_req){
    //总长度 = 帧头(2) + seq(1) + 长度字段(2) + cmd(2) + 数据区 + CRC(2)
    if(len < 8)return false;

    uint16_t data_len = data[3] | (data[4] << 8);
    size_t total = 5 + 2 + data_len + 2;
    if(len < total)return false;

    memcpy(out_req, data + 5, sizeof(protocal::PlanReqArc));

    uint16_t recv_crc = (uint16_t)data[total - 2] | (data[total - 1] << 8);
    uint16_t calc_crc = crc16(data, total - 2);
    if(recv_crc != calc_crc){
        RCLCPP_ERROR(LOGGER, "CRC 校验失败: %d != %d", recv_crc, calc_crc);
        return false;
    }
    return true;
}

static bool parse_feedback_frame(const uint8_t* data, size_t len, protocal::PlanFeedback* out_fb){

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
    g_node->declare_parameter("arm_group.kinematics_solver", "trac_ik_kinematics_plugin/TRAC_IKKinematicsPlugin");
    g_node->declare_parameter("arm_group.kinematics_solver_search_resolution", 0.005);
    g_node->declare_parameter("arm_group.kinematics_solver_timeout", 0.05);
    g_node->declare_parameter("arm_group.solve_type", "Distance");

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
    RCLCPP_INFO(LOGGER, "MoveIt 初始化完成: %d 个关节", g_joint_group->getVariableCount());
    g_num_joints = g_joint_group->getVariableCount();
    return true;

}

// TCP位姿(xyz + 固定轴XYZ的rpy) → Isometry3d
static Eigen::Isometry3d pose_to_isometry(const float pose[6]){
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.translation() = Eigen::Vector3d(pose[0], pose[1], pose[2]);
    T.linear() = (Eigen::AngleAxisd(pose[5], Eigen::Vector3d::UnitZ())
                * Eigen::AngleAxisd(pose[4], Eigen::Vector3d::UnitY())
                * Eigen::AngleAxisd(pose[3], Eigen::Vector3d::UnitX())).toRotationMatrix();
    return T;
}

// 当前末端(TCP)在 base_link 系下的位姿
static Eigen::Isometry3d current_tcp_pose(){
    std::vector<double> cur;
    {
        std::lock_guard<std::mutex> lock(g_feedback_mutex);
        cur = g_feedback_joints;
    }
    moveit::core::RobotState st(g_robot_model);
    st.setJointGroupPositions(g_joint_group, cur);
    st.update();
    return st.getGlobalLinkTransform(g_joint_group->getLinkModel("link11"));
}
// 由当前反馈关节角 FK 出末端位姿，按 base_link 固定轴 XYZ 打印 xyz 与 rpy(度)
// 旋转矩阵约定与 pose_to_isometry 一致：R = Rz(yaw) * Ry(pitch) * Rx(roll)
static void print_tcp_pose(){
    if(!g_robot_model)return;
    const Eigen::Isometry3d T = current_tcp_pose();
    const Eigen::Matrix3d R = T.linear();
    const double pitch = std::asin(std::max(-1.0, std::min(1.0, -R(2, 0)))) * deg2rad;
    const double roll  = std::atan2(R(2, 1), R(2, 2)) * deg2rad;
    const double yaw   = std::atan2(R(1, 0), R(0, 0)) * deg2rad;
    RCLCPP_INFO(LOGGER, "TCP 位姿: xyz=(%.4f, %.4f, %.4f) m, rpy=(%.2f, %.2f, %.2f) rad",
        T.translation().x(), T.translation().y(), T.translation().z(),
        roll * rad2deg, pitch * rad2deg, yaw * rad2deg);
}
static bool firmware_limit_check(const double q[NUM_JOINTS]){
    static const char* joint_name[NUM_JOINTS] = {"yaw", "p1", "p2", "p3", "roll", "ep", "er"};
    for(int k = 0; k < NUM_JOINTS; k++){
        if(q[k] < firmware_min[k] || q[k] > firmware_max[k]){
            RCLCPP_ERROR(LOGGER, "关节 %s 超出限位: %.3f < %.3f <= %.3f", joint_name[k], firmware_min[k] * rad2deg, q[k] * rad2deg, firmware_max[k] * rad2deg);
            return false;
        }
    }
    // if(q[2] - q[1] > firmware_p2p1_max){
    //     RCLCPP_WARN(LOGGER, "固件限位拒绝: p2-p1=%.1f 度 > %.1f", (q[2] - q[1]) * rad2deg, firmware_p2p1_max * rad2deg);
    //     return false;
    // }
    // if(q[3] < -(q[1] + q[2])){
    //     RCLCPP_WARN(LOGGER, "固件限位拒绝: p3 = %.1f 度 < -(p1+p2)=%.1f", q[3] * rad2deg, -(q[1] + q[2]) * rad2deg);
    //     return false;
    // }
    return true;
}
// 按每段 ARC_STEP_DEG 把转角切成 N 段，保证弦高误差并给足执行时长
static int arc_segments(double theta){
    int n = (int)std::ceil(std::fabs(theta) * rad2deg / ARC_STEP_DEG);
    return std::max(n, ARC_MIN_SEGMENTS);
}
// 生成绕固定轴旋转的 waypoint：axis_point 为轴上任意一点，axis_dir 为单位轴，theta 为带符号转角
static std::vector<Eigen::Isometry3d> arc_waypoints_(const Eigen::Vector3d& axis_point, const Eigen::Vector3d& axis_dir, double theta, const Eigen::Isometry3d& start, int N){
    std::vector<Eigen::Isometry3d> waypoints;
    if(N < 1 || theta == 0.0)return waypoints;

    const Eigen::Vector3d n = axis_dir.normalized();                    // 旋转轴
    const Eigen::Vector3d st = start.translation();                      // 起始点
    const Eigen::Vector3d center = axis_point + (st - axis_point).dot(n) * n; // 旋转中心

    waypoints.push_back(start);
    for(int i = 1; i <= N; i++){
        Eigen::AngleAxisd dR(theta * i / N, n);
        Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
        T.translation() = center + dR * (st - center);
        T.linear() = dR.toRotationMatrix() * start.linear();
        waypoints.push_back(T);
    }
    return waypoints;
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
    {
        std::lock_guard<std::mutex> lock(g_feedback_mutex);
        start_joints.assign(g_feedback_joints.begin(), g_feedback_joints.end());
    }

    if(start_joints.empty() || start_joints.size() < g_joint_group->getVariableCount()){
        start_joints.assign(g_joint_group->getVariableCount(), 0.0);//如果没有接收到反馈，默认0.0
    }
    RCLCPP_INFO(LOGGER, "起点关节角 = [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]", start_joints[0], start_joints[1], start_joints[2], start_joints[3], start_joints[4], start_joints[5], start_joints[6]);
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

    // 1.3 检查IK解是否有效
    if (!found) {
        RCLCPP_ERROR(LOGGER, "IK 失败(限位内无有效解), 目标 (%.3f, %.3f, %.3f)",
            target.translation().x(), target.translation().y(), target.translation().z());
        return {};
    }

    // 1.4 检查IK解是否违反固件限制
    if(!firmware_limit_check(solution.data())){
        RCLCPP_ERROR(LOGGER, "IK 解超出固件限位, 目标 (%.3f, %.3f, %.3f)",
            target.translation().x(), target.translation().y(), target.translation().z());
        return {};
    }

    RCLCPP_INFO(LOGGER, "IK 求解完成: 关节角 = [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
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
            RCLCPP_INFO(LOGGER, "起点碰撞: %d 个接触点, 碰撞=%d", (int)cres.contacts.size(), (int)cres.collision);
            for(auto& c : cres.contacts)
                RCLCPP_INFO(LOGGER, "  碰撞对: %s <-> %s", c.first.first.c_str(), c.first.second.c_str());
        }
        {
            collision_detection::CollisionResult cres;
            cscene->checkCollision(creq, cres, goal_state);
            RCLCPP_INFO(LOGGER, "目标碰撞: %d 个接触点, 碰撞=%d", (int)cres.contacts.size(), (int)cres.collision);
            for(auto& c : cres.contacts)
                RCLCPP_INFO(LOGGER, "  碰撞对: %s <-> %s", c.first.first.c_str(), c.first.second.c_str());
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
        RCLCPP_ERROR(LOGGER, "RRTConnect 规划失败");
        return {};
    }

    // 解析轨迹
    moveit_msgs::msg::RobotTrajectory traj_msg;
    best_res.trajectory_->getRobotTrajectoryMsg(traj_msg);
    auto& pts = traj_msg.joint_trajectory.points;

    if(pts.empty()){
        RCLCPP_ERROR(LOGGER, "规划返回空轨迹");
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
    RCLCPP_INFO(LOGGER, "规划成功: %zu 个轨迹点", best_trajectory.size());
    return best_trajectory;
}

// ==================== 笛卡尔直线规划：当前末端位姿 → target ====================
// 起点由当前关节反馈 FK 得到，直线插值到绝对目标位姿 target，全程回避碰撞
static std::vector<std::vector<float> > cartesian_plan_(const std::vector<Eigen::Isometry3d>& waypoints){
    if(waypoints.size() < 2){
        RCLCPP_ERROR(LOGGER, "路径点数量不足 2");
        return {};
    }
    std::vector<double> cur_joints;
    {
        std::lock_guard<std::mutex> lock(g_feedback_mutex);
        cur_joints = g_feedback_joints;
    }
    if(cur_joints.size() != g_num_joints){
        RCLCPP_ERROR(LOGGER, "反馈不正确");
        return {};
    }
    const moveit::core::LinkModel* tip = g_joint_group->getLinkModel("link11");

    //FK得到当前tip位姿
    moveit::core::RobotState fk(g_robot_model);
    fk.setJointGroupPositions(g_joint_group, cur_joints);
    fk.update();

    //调试日志：发了几段，每段去哪
    for(size_t k = 0; k < waypoints.size(); k++){
        RCLCPP_INFO(LOGGER, "路径点[%zu] = (%.3f, %.3f, %.3f)",
            k, waypoints[k].translation().x(), waypoints[k].translation().y(), waypoints[k].translation().z());
    }
    int firmware_limits = 0, collision = 0;
    const char* joint_names[g_num_joints] = {"yaw", "p1", "p2", "p3", "roll", "ep", "er"};
    moveit::core::GroupStateValidityCallbackFn valid = [&firmware_limits, &collision, &joint_names](moveit::core::RobotState* st, const moveit::core::JointModelGroup*, const double* q){
        if(!firmware_limit_check(q)){
            firmware_limits++;
            return false;
        }
        st->update();
        collision_detection::CollisionRequest creq;// 碰撞检测结果
        creq.group_name = "arm_group";
        collision_detection::CollisionResult cres;
        g_planning_scene->checkCollision(creq, cres, *st);
        if(cres.collision)collision++;
        return !cres.collision;
    };

    //逐段插值：每段从"当前末端"直线到 waypoints[s+1]，段间用上一段末点衔接
    moveit::core::RobotState start(g_robot_model);
    start.setJointGroupPositions(g_joint_group, cur_joints);//第0段是当前反馈
    start.update();

    std::vector<moveit::core::RobotStatePtr> all_states; // 拼完的总轨迹

    for(size_t s = 0; s < waypoints.size() - 1; s++){
        if(s > 0)start = *all_states.back();//这一段开始等于上一段末点

        std::vector<moveit::core::RobotStatePtr> traj;// 这一段轨迹
        auto frac = moveit::core::CartesianInterpolator::computeCartesianPath(&start, g_joint_group, traj, tip, waypoints[s + 1],
        true,
        moveit::core::MaxEEFStep(CART_EEF_STEP),
        moveit::core::JumpThreshold(CART_JOINT_JUMP, 0.0),
        valid);// 生成轨迹
        if((double(frac) < 1.0)){
            RCLCPP_ERROR(LOGGER, "笛卡尔段 %zu 未完成: %.1f%% (固件限位=%d, 碰撞=%d)", s, double(frac) * 100.0, firmware_limits, collision);
            return {};
        }
        //拼接
        for(auto it = traj.begin() + (s == 0 ? 0 : 1); it != traj.end(); ++it)all_states.push_back(*it);
    }
    std::vector<std::vector<float> > result;
    std::vector<double> q(g_num_joints);
    for(auto& st : all_states){
        st->copyJointGroupPositions(g_joint_group, q);
        result.emplace_back(q.begin(), q.end());
    }
    RCLCPP_INFO(LOGGER, "笛卡尔规划: %zu 段 -> %zu 点", waypoints.size() - 1, result.size());
    return result;
}
// 用 FK 校验规划结果：首/末点是否贴合期望 waypoint，相邻点是否发生跳支。
static void check_traj(const std::vector<std::vector<float> >& traj,
                       const std::vector<Eigen::Isometry3d>& waypoints,
                       const char* tag)
{
    if(traj.empty() || waypoints.empty() || !g_robot_model)return;
    const moveit::core::LinkModel* tip = g_joint_group->getLinkModel("link11");

    auto fk = [&](const std::vector<float>& q){
        moveit::core::RobotState st(g_robot_model);
        st.setJointGroupPositions(g_joint_group, std::vector<double>(q.begin(), q.end()));
        st.update();
        return st.getGlobalLinkTransform(tip);
    };

    const Eigen::Isometry3d T0 = fk(traj.front());
    const Eigen::Isometry3d T1 = fk(traj.back());
    const Eigen::Isometry3d& E1 = waypoints.back();
    const double pos_err = (T1.translation() - E1.translation()).norm() * 1000.0;              // mm
    const double rot_err = Eigen::AngleAxisd(T1.linear().transpose() * E1.linear()).angle() * rad2deg;

    RCLCPP_INFO(LOGGER, "[%s] 规划自检: %zu 点, 首点FK=(%.4f,%.4f,%.4f)",
        tag, traj.size(), T0.translation().x(), T0.translation().y(), T0.translation().z());
    RCLCPP_INFO(LOGGER, "[%s] 末点FK=(%.4f,%.4f,%.4f) 期望=(%.4f,%.4f,%.4f) 误差=%.3fmm/%.4fdeg",
        tag, T1.translation().x(), T1.translation().y(), T1.translation().z(),
        E1.translation().x(), E1.translation().y(), E1.translation().z(), pos_err, rot_err);

    // 相邻点最大跳变：抓限位/奇异导出的跳支
    double max_dq = 0, max_dp = 0, max_dr = 0;
    int dom_joint = -1;
    Eigen::Isometry3d prev = T0;
    for(size_t i = 1; i < traj.size(); i++){
        for(int j = 0; j < g_num_joints; j++){
            double d = std::fabs((double)traj[i][j] - traj[i-1][j]);
            if(d > max_dq){ max_dq = d; dom_joint = j; }
        }
        const Eigen::Isometry3d T = fk(traj[i]);
        max_dp = std::max(max_dp, (T.translation() - prev.translation()).norm() * 1000.0);
        max_dr = std::max(max_dr, Eigen::AngleAxisd(prev.linear().transpose() * T.linear()).angle() * rad2deg);
        prev = T;
    }
    RCLCPP_INFO(LOGGER, "[%s] 相邻点最大跳变: 关节=%.4frad(轴%d) 末端位移=%.2fmm 转角=%.3fdeg",
        tag, max_dq, dom_joint, max_dp, max_dr);
}
// 监视线程：检测"卡死/无进展"（到轨迹终点的距离长期不缩小即触发重规划）
// 只看"是否在靠近终点"，不比较中间轨迹点，因此对下位机滞后/不同步免疫
static void monitor_thread_func(){
    const double progress_eps = PROGRESS_EPS;                              // 认为有进展的最小距离减小量(rad)
    const auto stall_timeout = std::chrono::milliseconds(STALL_TIMEOUT_MS); // 无进展超过此时长即判定卡死

    State prev_state = State::IDLE;
    double best_dist = 1e9;                                        // 历史最近"到终点距离"
    auto last_progress = std::chrono::steady_clock::now();

    while(g_running){
        std::this_thread::sleep_for(std::chrono::milliseconds(20));

        State st = g_state.load();
        if(st != State::EXECUTING){
            prev_state = st;
            continue;
        }
        if(prev_state != State::EXECUTING){                        // 刚进入执行：重置进度基线
            best_dist = 1e9;
            last_progress = std::chrono::steady_clock::now();
            prev_state = st;
        }

        std::vector<double> actual;
        {
            std::lock_guard<std::mutex> lock(g_feedback_mutex);
            actual = g_feedback_joints;
        }
        std::vector<float> last_pt;
        {
            std::lock_guard<std::mutex> lock(g_exec_mutex);
            if(!g_exec_traj.empty())last_pt = g_exec_traj.back();
        }
        if(actual.empty() || last_pt.empty())continue;

        double dist = 0.0;
        for(int i = 0; i < g_num_joints; i++)dist = std::max(dist, std::fabs(actual[i] - last_pt[i]));

        // 诊断：主导偏差轴 + 该轴 实际->目标 + 反馈新鲜度（区分"到不了"与"反馈停更"）
        {
            int dom = 0; double dm = 0;
            for(int i = 0; i < g_num_joints; i++){
                double d = std::fabs(actual[i] - last_pt[i]);
                if(d > dm){ dm = d; dom = i; }
            }
            int64_t age = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now().time_since_epoch()).count() - g_last_feedback_ms.load();
            RCLCPP_WARN_THROTTLE(LOGGER, *g_node->get_clock(), 200,
                "诊断 dist=%.3f best=%.3f 主偏差轴=%d(%.3f->%.3f) 反馈延迟=%ldms",
                dist, best_dist, dom, actual[dom], last_pt[dom], (long)age);
        }

        auto now = std::chrono::steady_clock::now();
        if(dist < best_dist - progress_eps){                       // 仍在靠近终点
            best_dist = dist;
            last_progress = now;
        } else if(now - last_progress > stall_timeout){            // 长时间不再靠近 → 卡死
            RCLCPP_WARN(LOGGER, "长时间无进展 (dist=%.3f, best=%.3f), 请求中止", dist, best_dist);
            g_exec_fail_reason = 1;                                // 由规划线程统一发0x03并回IDLE
        }
    }
}

static void serial_thread_func() {
    uint8_t tmp[256];
    static std::vector<uint8_t> rx_buffer;

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
                        //新增进程锁
                        static std::mutex g_request_mutex;
                        {
                            std::lock_guard<std::mutex> lock(g_request_mutex);
                            g_recv_req = req; // 保存新计划
                        }
                        g_state = State::PLANNING;
                        RCLCPP_INFO(LOGGER, "收到 PlanReq(0x01): 位姿 = [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]", req.pose[0], req.pose[1], req.pose[2], req.pose[3], req.pose[4], req.pose[5]);
                    } else {
                        RCLCPP_ERROR(LOGGER, "PlanReq(0x01) 解析失败");
                    }
                }
                else if(recv_cmd == protocal::Cmd::PLAN_REQ3){
                    protocal::PlanReq3 req3;
                    if(parse_plan3_frame(rx_buffer.data() + i, total, &req3)){
                        // 绝对目标 = 当前末端位姿 ⊕ delta(基座系)，解析时定下供重规划复用
                        Eigen::Isometry3d d = pose_to_isometry(req3.delta);
                        Eigen::Isometry3d cur = current_tcp_pose();
                        {
                            static std::mutex g_request_mutex;
                            std::lock_guard<std::mutex> lock(g_request_mutex);
                            g_cart_goal = cur;
                            g_cart_goal.translation() += d.translation();
                            g_cart_goal.linear() = d.linear() * cur.linear();
                        }
                        g_state = State::PLANNING3;
                        RCLCPP_INFO(LOGGER, "收到 PlanReq3(0x06): 增量 = [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]", req3.delta[0], req3.delta[1], req3.delta[2], req3.delta[3], req3.delta[4], req3.delta[5]);
                        RCLCPP_INFO(LOGGER, "笛卡尔目标: (%.3f, %.3f, %.3f)", g_cart_goal.translation().x(), g_cart_goal.translation().y(), g_cart_goal.translation().z());
                    } else {
                        RCLCPP_ERROR(LOGGER, "PlanReq3(0x06) 解析失败");
                    }
                }
                else if(recv_cmd == protocal::Cmd::PLAN_REQ_ARC){
                    protocal::PlanReqArc req_arc;
                    if(parse_arc_frame(rx_buffer.data() + i, total, &req_arc)){
                        {
                            std::lock_guard<std::mutex> lock(g_arc_mutex);
                            g_arc_req = req_arc;
                        }
                        g_state = State::PLANNING_ARC;
                        RCLCPP_INFO(LOGGER, "收到 PlanReqArc(0x07): 轴点=(%.4f, %.4f, %.4f) 轴方向=(%.3f, %.3f, %.3f) 转角=%.2f度",
                            req_arc.axis_point[0], req_arc.axis_point[1], req_arc.axis_point[2],
                            req_arc.axis_dir[0], req_arc.axis_dir[1], req_arc.axis_dir[2],
                            req_arc.theta * rad2deg);
                    } else {
                        RCLCPP_ERROR(LOGGER, "PlanReqArc(0x07) 解析失败");
                    }
                }
                else if(recv_cmd == protocal::Cmd::PLAN_FEEDBACK){
                    protocal::PlanFeedback fb;
                    if(parse_feedback_frame(rx_buffer.data() + i, total, &fb)){
                        {
                            std::lock_guard<std::mutex> lock(g_feedback_mutex);
                            for(int k = 0; k < g_num_joints; k++)g_feedback_joints[k] = fb.joints[k];
                            g_feedback_joints[4] = -fb.joints[4];   // 固件roll反馈与URDF反号，取反后与其余轴一致
                        }   // 作用域结束自动解锁
                        g_last_feedback_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                            std::chrono::steady_clock::now().time_since_epoch()).count();

                        if constexpr(RECORD_FEEDBACK){
                            float arm[7];
                            for(int k = 0; k < g_num_joints; k++)arm[k] = fb.joints[k];
                            arm[4] = -fb.joints[4];   // 固件roll反馈与URDF反号
                            feedback_pub::save_feedback(fb.step_idx, arm);
                        }  // 文件IO在锁外
                        g_exec_step = fb.step_idx;

                        if constexpr(!CALIB_MODE){
                            if(fb.step_idx != g_last_feedback_step){
                                g_last_feedback_step = fb.step_idx;
                                RCLCPP_INFO(LOGGER, "收到 PlanFeedback(0x04): 关节角 = [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f] 步=%d", g_feedback_joints[0], g_feedback_joints[1], g_feedback_joints[2], g_feedback_joints[3], g_feedback_joints[4], g_feedback_joints[5], g_feedback_joints[6], fb.step_idx);
                            }
                            // 每 500ms 打印一次由反馈 FK 得到的 TCP 位姿，便于与 RViz/实机比对
                            static auto last_tcp_print = std::chrono::steady_clock::now();
                            auto now_tcp = std::chrono::steady_clock::now();
                            if(now_tcp - last_tcp_print >= std::chrono::milliseconds(1000)){
                                last_tcp_print = now_tcp;
                                print_tcp_pose();
                            }
                        }
                    }
                    //  else {
                    //     if constexpr(!CALIB_MODE){
                    //         RCLCPP_WARN_THROTTLE(LOGGER, *g_node->get_clock(), 2000, "PlanFeedback(0x04) 解析失败, 已跳过该帧");
                    //     }
                    // }
                }
                else if(recv_cmd == protocal::Cmd::PLAN_RECORD){
                    float raw[7], arm[7];
                    memcpy(raw, rx_buffer.data() + i + 7, sizeof(raw));
                    for(int k = 0; k < 7; k++){
                        arm[k] = g_joint_sign[k] * raw[k] - g_joint_offset[k];
                    }
                    arm[4] = -raw[4];   // 固件roll反馈与URDF反号
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

// apply_mapping=false 时跳过 sign/offset 换算，直接把输入当作下位机 raw 角下发（标定用）
static void send_trajectory(const std::vector<std::vector<float> >& traj, bool apply_mapping = true){
    for(size_t i = 0; i < traj.size(); i++){
        if(!g_running)break;
        std::vector<uint8_t> buf;
        buf.push_back(0x02);
        buf.push_back(0x00);
        buf.push_back(i & 0xFF);
        buf.push_back(i >> 8);
        for(size_t j = 0; j < traj[i].size(); j++){
            float stm32_angle = apply_mapping ? (g_joint_sign[j] * traj[i][j] + g_joint_offset[j]) : traj[i][j];
            uint8_t* p = (uint8_t*)&stm32_angle;
            for(int k = 0; k < 4; k++)buf.push_back(p[k]);

        }
        RCLCPP_INFO(LOGGER, "下发关节角 = [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]", traj[i][0], traj[i][1], traj[i][2], traj[i][3], traj[i][4], traj[i][5], traj[i][6]);
        send_frame(buf.data(), buf.size());
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

// 统一失败出口：回错误码0x03并回到IDLE，以便接收下一个目标（仅规划线程调用，避免串口帧交错）
static void fail_and_idle(const char* reason){
    RCLCPP_ERROR(LOGGER, "%s", reason);
    uint16_t fail = protocal::Cmd::PLAN_FAIL;
    send_frame((uint8_t*)&fail, 2);
    {
        std::lock_guard<std::mutex> lock(g_exec_mutex);
        g_exec_traj.clear();
    }
    g_arrived_cnt = 0;
    g_exec_fail_reason = 0;
    g_state = State::IDLE;
}

static void start_execution(const std::vector<std::vector<float> >& traj){
    {
        std::lock_guard<std::mutex> lock(g_exec_mutex);
        g_exec_traj = traj;
        g_exec_step = 0;
    }
    g_arrived_cnt = 0;
    g_exec_fail_reason = 0;   // 新轨迹清除上一轮的失败标记
    g_state = State::EXECUTING;
}

// 主循环：状态机调度
static void planning_thread_func(){
    while(g_running){
        switch(g_state.load(std::memory_order_acquire)){
            case State::IDLE: {

                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                break;
            }
            case State::PLANNING: {
                Eigen::Isometry3d target;
                static std::mutex g_request_mutex;
                {
                    std::lock_guard<std::mutex> lock(g_request_mutex);
                    target = pose_to_isometry(g_recv_req.pose);
                }

                // 用四元数打印，避免 eulerAngles 的多解问题
                Eigen::Quaterniond q_print(target.rotation());
                RCLCPP_INFO(LOGGER, "规划目标: (%.3f,%.3f,%.3f) 四元数(%.3f,%.3f,%.3f,%.3f)",
                        target.translation().x(), target.translation().y(), target.translation().z(),
                        q_print.w(), q_print.x(), q_print.y(), q_print.z());

                auto traj = best_plan_(target);

                if(!traj.empty()){
                    send_trajectory(traj);
                    size_t total_frames = traj.size();
                    RCLCPP_INFO(LOGGER, "已下发 %zu 点, 进入执行状态", total_frames);
                    start_execution(traj);
                } else {
                    fail_and_idle("规划失败");
                }
                break;
            }
            case State::PLANNING3: {
                Eigen::Isometry3d cart_goal;
                {
                    static std::mutex g_request_mutex;
                    std::lock_guard<std::mutex> lock(g_request_mutex);
                    cart_goal = g_cart_goal;
                }
                std::vector<Eigen::Isometry3d> waypoints;
                waypoints.push_back(current_tcp_pose());
                waypoints.push_back(cart_goal);
                auto traj = cartesian_plan_(waypoints);

                if(!traj.empty()){
                    check_traj(traj, waypoints, "CART");
                    send_trajectory(traj);
                    size_t total_frames = traj.size();
                    RCLCPP_INFO(LOGGER, "已下发 %zu 点, 进入执行状态", total_frames);
                    start_execution(traj);
                } else {
                    fail_and_idle("规划失败");
                }
                break;
            }
            case State::PLANNING_ARC: {
                protocal::PlanReqArc req;
                {
                    std::lock_guard<std::mutex> lock(g_arc_mutex);
                    req = g_arc_req;
                }
                const Eigen::Vector3d axis_point(req.axis_point[0], req.axis_point[1], req.axis_point[2]);
                Eigen::Vector3d axis_dir(req.axis_dir[0], req.axis_dir[1], req.axis_dir[2]);
                const double theta = req.theta;

                if(!axis_point.allFinite() || !axis_dir.allFinite() || axis_dir.norm() < 1e-6){
                    fail_and_idle("Arc rejected: invalid axis");
                    break;
                }
                // if(!std::isfinite(theta) || std::fabs(theta) < 1e-6 || std::fabs(theta) > M_PI + 1e-6){
                //     fail_and_idle("圆弧请求被拒绝: 转角大小必须满足 0 < |theta| <= PI");
                //     break;
                // }
                axis_dir.normalize();

                const Eigen::Isometry3d start = current_tcp_pose();
                const int N = arc_segments(theta);
                auto waypoints = arc_waypoints_(axis_point, axis_dir, theta, start, N);
                auto traj = cartesian_plan_(waypoints);

                if(!traj.empty()){
                    check_traj(traj, waypoints, "ARC");
                    send_trajectory(traj);
                    size_t total_frames = traj.size();
                    RCLCPP_INFO(LOGGER, "圆弧已下发 %zu 点 (%d 段), 进入执行状态", total_frames, N);
                    start_execution(traj);
                } else {
                    fail_and_idle("圆弧规划失败");
                }
                break;
            }
            case State::EXECUTING:{
                // 监视线程判定卡死：不再重规划，直接回错误码并回IDLE
                if(g_exec_fail_reason.load() != 0){
                    fail_and_idle("执行卡死, 中止");
                    break;
                }
                // 完成判定：反馈关节角与轨迹终点各轴足够接近，且连续多帧稳定在阈值内
                const double arrive_eps = 0.05;   // 到位阈值(rad)
                const int arrive_frames = 3;     // 连续满足帧数(约150ms)
                std::vector<double> actual;
                {
                    std::lock_guard<std::mutex> lock(g_feedback_mutex);
                    actual = g_feedback_joints;
                }
                std::vector<float> last_pt;   // 轨迹终点拷贝，锁外安全使用
                {
                    std::lock_guard<std::mutex> lock(g_exec_mutex);
                    if(!g_exec_traj.empty())last_pt = g_exec_traj.back();
                }
                bool arrived = !actual.empty() && !last_pt.empty();
                if(arrived){
                    for(int j = 0; j < g_num_joints; j++){
                        if(std::fabs(actual[j] - last_pt[j]) > arrive_eps){ arrived = false; break; }
                    }
                }
                g_arrived_cnt = arrived ? g_arrived_cnt + 1 : 0;
                if(g_arrived_cnt >= arrive_frames){
                    // 执行残差（只打印）：反馈FK 与 规划末点FK 的位置/姿态差
                    moveit::core::RobotState sa(g_robot_model), sc(g_robot_model);
                    sa.setJointGroupPositions(g_joint_group, actual);
                    sc.setJointGroupPositions(g_joint_group, std::vector<double>(last_pt.begin(), last_pt.end()));
                    sa.update(); sc.update();
                    const moveit::core::LinkModel* tip = g_joint_group->getLinkModel("link11");
                    const Eigen::Isometry3d Ta = sa.getGlobalLinkTransform(tip);
                    const Eigen::Isometry3d Tc = sc.getGlobalLinkTransform(tip);
                    RCLCPP_INFO(LOGGER, "到位残差: 位置=%.2fmm 姿态=%.3fdeg",
                        (Ta.translation() - Tc.translation()).norm() * 1000.0,
                        Eigen::AngleAxisd(Ta.linear().transpose() * Tc.linear()).angle() * rad2deg);
                    g_arrived_cnt = 0;
                    g_state = State::IDLE;
                    RCLCPP_INFO(LOGGER, "执行完成, 回到 IDLE");
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
                break;
            }
        }
    }
}

#if CALIB_MODE
/**
 * @brief 标定模式主循环（直接控制）
 *
 * 用法：
 *   1. 固件进入控制器模式(左拨杆下 + 右拨杆上)，臂停在 init 位
 *   2. 每次输入两个数：<轴号 0-6> <raw目标角 rad>  例: `0 0.4`
 *      - 目标轴：输入值**直接作为下位机 raw 角下发**，不做 sign/offset 换算
 *      - 其余轴：自动填充为**当前反馈值**，即保持当前位不动
 *   3. 到位后打印当前 raw 反馈
 *   4. Ctrl-D 退出
 *
 * 说明：固件是位置控制器，step2 决定"物理去哪"，打印的 raw 只是回读。
 *       offset 看"发 ros=0 时臂是否停在约定零位"；sign 看"发 +raw 时臂转向"。
 */
static void calib_loop() {
    RCLCPP_INFO(LOGGER, "==== 标定模式 ====  输入: <轴号0-6> <raw目标rad>  例: 0 0.4   其余轴保持当前位；Ctrl-D退出");
    while (g_running && rclcpp::ok()) {
        printf("[CALIB] 输入 <轴号0-6> <raw目标rad> > ");
        fflush(stdout);

        int idx; double val;
        if (scanf("%d %lf", &idx, &val) != 2) { g_running = false; return; } // EOF/非法输入即退出
        if (idx < 0 || idx >= NUM_JOINTS) { printf("[CALIB] 轴号越界(应为0-6)\n"); continue; }

        std::vector<std::vector<float>> traj(1, std::vector<float>(NUM_JOINTS));
        {
            std::lock_guard<std::mutex> lock(g_feedback_mutex);
            for (int k = 0; k < NUM_JOINTS; k++) traj[0][k] = static_cast<float>(g_feedback_joints[k]); // 其余轴保持当前位
        }
        traj[0][idx] = static_cast<float>(val); // 目标轴直接作为 raw 下发

        send_trajectory(traj, /*apply_mapping=*/false);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000)); // 等待到位

        std::vector<double> raw;
        {
            std::lock_guard<std::mutex> lock(g_feedback_mutex);
            raw = g_feedback_joints;
        }
        printf("[CALIB] 轴%d 目标raw=%.4f  当前raw = [", idx, val);
        for (int k = 0; k < NUM_JOINTS; k++) printf("%.4f%s", raw[k], k < NUM_JOINTS - 1 ? ", " : "");
        printf("]\n");
    }
}
#endif

int main(int argc, char** argv){
    if(argc > 1) g_serial_port = argv[1];

    rclcpp::init(argc, argv);
    g_node = std::make_shared<rclcpp::Node>("moveit_planning");
    feedback_pub::init(g_node);
    if(!open_serial())return 1; // 打开串口

#if CALIB_MODE
    // 标定模式：跳过 MoveIt，仅保留串口线程 + 手动下发
    RCLCPP_INFO(LOGGER, "标定模式: 跳过 MoveIt");
    std::thread serial_thr(serial_thread_func);
    calib_loop();
    g_running = false;
    close_serial();
    serial_thr.join();
    rclcpp::shutdown();
    return 0;
#else
    if(!init_moveit()){
        RCLCPP_FATAL(LOGGER, "MoveIt 初始化失败");
        return 1;
    }
    RCLCPP_INFO(LOGGER, "系统就绪, 等待请求...");

    // 启动串口线程和规划线程
    std::thread serial_thr(serial_thread_func);
    std::thread plan_thr(planning_thread_func);
    std::thread monitor_thr(monitor_thread_func);
    while(rclcpp::ok()){
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    // 停止线程
    RCLCPP_INFO(LOGGER, "正在关闭...");
    g_running = false;
    g_state = State::IDLE;
    close_serial();
    serial_thr.join();
    plan_thr.join();
    monitor_thr.join();

    rclcpp::shutdown();
    return 0;
#endif
}