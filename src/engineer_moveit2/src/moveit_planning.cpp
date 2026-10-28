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
#include <ament_index_cpp/get_package_share_directory.hpp>  // 用于定位包内 URDF/SRDF
#include <algorithm>
#include <cassert>
#include <vector>
#include <chrono>
#include <cmath>
#include <cstring>
#include <cstdio>
#include <utility>
#include "feedback_pub.hpp"
#include "camera.h"        // 海康相机：CameraControl + CameraStream（内部取帧线程）
#include "PnPDate.hpp"     // PnP 标定数据
#include "PnPSolver.hpp"   // solveCarTargetPose()
#define NUM_JOINTS 7

// 标定模式：1=标定(跳过MoveIt)，0=正常规划。标定用法见 calib_loop()
#define CALIB_MODE 0

// 反馈录制开关：1=每帧0x06写CSV，0=关闭
#define RECORD_FEEDBACK 0

// 相机预览开关：1=弹窗显示当前相机画面(ESC/q 关闭)，0=关闭
#define VISION_DEBUG_SHOW 1

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

// 拍点节奏：下发间隔由关节限速反推（下位机是固定 40ms 步长的线性插值器，指令速度 = Δq / dt）
// 限速数值取自下位机 algo_traj_playback 的默认参数(deg/s)
static const double joint_vel_deg[NUM_JOINTS] = {90.0, 60.0, 80.0, 90.0, 120.0, 120.0, 60.0};
#define JOINT_VEL_SCALE 1.0                 // 限速折扣，想保守就调小(如0.8)
#define MIN_SEND_DT 0.02                    // 最小下发间隔(s)，防止刷屏/串口过载
#define MAX_SEND_DT 0.1                     // 最大下发间隔(s)，即改造前固定的 100ms

// 步骤4~5 的目标参数：视觉尚未给出轴向/转角，先用常量占位
static const Eigen::Vector3d STEP_ROT_P_AXIS(1.0, 0.0, 0.0);  // 步骤4：P轴方向（待视觉/标定给出）
static const double STEP_ROT_P_THETA = 90.0 * deg2rad;        // 向上90°
static const Eigen::Vector3d STEP_ROT_Q_AXIS(0.0, 1.0, 0.0);  // 步骤5：Q轴方向（待视觉/标定给出）
static const double STEP_ROT_Q_THETA = 90.0 * deg2rad;        // 指定角（待规则/标定给出）
#define FEEDBACK_TIMEOUT_MS 1000             // 反馈超过该时长未更新则拒绝规划

// 指令号分配：STM32->PC 用 0x01~0x06 与 0x0D；PC->STM32 用 0x07~0x0C
namespace protocal {
enum Cmd : uint16_t {
    // ---------- STM32 -> PC ----------
    STEP_APPROACH = 0x01,   // 步骤1 接近：目标由视觉给出
    STEP_INSERT   = 0x02,   // 步骤2 插入100mm（一级矿完成）
    STEP_SHIFT    = 0x03,   // 步骤3 平移100mm（二级矿完成）
    STEP_ROT_P    = 0x04,   // 步骤4 绕P轴旋转
    STEP_ROT_Q    = 0x05,   // 步骤5 绕Q轴旋转（三级矿完成）
    PLAN_FEEDBACK = 0x06,   // 关节反馈
    STEP_ABORT    = 0x0D,   // 强行打断：停止下发，维持当前位姿
    // ---------- PC -> STM32 ----------
    DBG_PLAN_REQ  = 0x07,   // 绝对位姿请求
    DBG_DELTA_REQ = 0x08,   // 基座系增量
    DBG_REQ_ARC   = 0x09,   // 绕固定轴旋转固定角度
    TRAJ_POINT    = 0x0A,   // 轨迹点帧
    PLAN_FAIL     = 0x0B,   // 规划失败
    STEP_DONE     = 0x0C,   // 单步完成
};
// ==================== 帧格式定义 ====================
// STM32 -> PC: STEP_APPROACH~STEP_ROT_Q(0x01~0x05)、STEP_ABORT(0x0D) 无载荷
// STM32 -> PC: PLAN_FEEDBACK(0x06) + step_idx(2) + joints[7](28)
// PC -> STM32: TRAJ_POINT(0x0A) + idx(2) + joints[7](28)
#pragma pack(push, 1)
struct PlanReq {
    uint16_t cmd;        // 0x07
    float pose[6];       // 目标TCP位姿: xyz(3) + rpy(3), base_link系
};

struct PlanFeedback {
    uint16_t cmd;        // 0x06
    uint16_t step_idx;   // 当前执行到的轨迹点索引
    float joints[NUM_JOINTS]; // 当前关节角度
};

struct StepDone {
    uint16_t cmd;        // 0x0C
    uint16_t step;       // 完成的步骤号(1~5)
};

struct PlanDeltaReq{
    uint16_t cmd;        // 0x08
    float delta[6];      // 基座系相对当前末端位姿的变换: [dx,dy,dz, drx,dry,drz]
};

struct PlanReqArc{
    uint16_t cmd;          // 0x09
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
    PLANNING,       // 规划中（单步指令与手动调试入口共用）
    EXECUTING
};
static std::atomic<State> g_state = State::IDLE; // 状态机状态
static std::atomic<bool> g_running = true;       // 是否在运行中

// 规划请求种类：0x01~0x05 的单步指令与 0x07~0x09 的手动调试入口共用同一个规划状态
// 数值与 protocal::Cmd 一致，便于直接用 (int)kind 取步骤号
enum class PlanKind : int{
    NONE          = 0,
    STEP_APPROACH = protocal::Cmd::STEP_APPROACH, // 0x01 接近目标
    STEP_INSERT   = protocal::Cmd::STEP_INSERT,   // 0x02 插入目标
    STEP_SHIFT    = protocal::Cmd::STEP_SHIFT,    // 0x03 平移目标
    STEP_ROT_P    = protocal::Cmd::STEP_ROT_P,    // 0x04 旋转目标
    STEP_ROT_Q    = protocal::Cmd::STEP_ROT_Q,    // 0x05 旋转目标
    DBG_POSE      = protocal::Cmd::DBG_PLAN_REQ,  // 0x07 位姿调试
    DBG_DELTA     = protocal::Cmd::DBG_DELTA_REQ, // 0x08 位调试
    DBG_ARC       = protocal::Cmd::DBG_REQ_ARC    // 0x09 圆弧调试
};
static std::atomic<PlanKind> g_requested_step{PlanKind::NONE}; // 待规划项（串口线程写，规划线程取）
static std::atomic<bool> g_abort_stream{false};           // 0x0D 打断：停发并回 IDLE
static PlanKind g_executing_step = PlanKind::NONE;             // 本轮已下发的规划项（到位后据此决定是否回 0x0C）

// 请求/反馈
static protocal::PlanReq g_recv_req;  // 接收的规划请求
static std::mutex g_req_mutex;         // 保护 g_recv_req / g_cart_goal（串口线程写，规划线程读）
static Eigen::Isometry3d g_cart_goal; // 0x08 解析出的绝对目标位姿
static protocal::PlanReqArc g_arc_req; // 0x09 圆弧请求
static std::mutex g_arc_mutex;         // 保护 g_arc_req（串口线程写，规划线程读）
static std::atomic<int> g_exec_fail_reason{0};  // 执行期失败原因：0=无 1=无进展卡死（监视线程写，规划线程统一处理）

// PlanFeedback 缓冲区（无锁 atomic 供监视线程读）
static std::vector<double> g_feedback_joints(NUM_JOINTS, 0.0);
static std::mutex g_feedback_mutex; // 保护反馈的互斥锁
static std::atomic<int64_t> g_last_feedback_ms{0}; // 最近一次收到0x06反馈的时刻(steady_clock, ms)
static const double g_joint_offset[NUM_JOINTS] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
// 标定结论：7轴方向与URDF一致
static const double g_joint_sign[NUM_JOINTS]  = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
static uint8_t g_send_seq = 0;  // 发送包序号

// 执行期监视
static std::atomic<int> g_exec_step = 0;            // 当前执行到第几步
static int g_last_feedback_step = -1;               // 已打印过的反馈 idx，用于边沿检测
static std::vector<std::vector<float>> g_exec_traj; // 正在执行轨迹（只读）
static std::mutex g_exec_mutex;                     // 保护执行轨迹的互斥锁
static int g_arrived_cnt = 0;                       // 连续满足到位阈值的帧数

//记录复盘用
static std::atomic<uint32_t> g_req_id{0};           // 请求序号，贯穿一次请求的全过程
static std::string g_fail_reason;
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

// 组帧并发到串口：AA55 + seq + len(2,=载荷字节数) + cmd(2) + 载荷 + CRC16
// 串口线程(回0x0B)与规划线程(发点/回0x0C)都会调用，加锁避免帧交错
static std::mutex g_send_mutex;
static void send_frame(const uint8_t* data, size_t len) {
    std::lock_guard<std::mutex> lock(g_send_mutex);
    uint8_t frame[len + 7];
    frame[0] = 0xAA;
    frame[1] = 0x55;
    frame[2] = g_send_seq++;
    frame[3] = (len - 2) & 0xFF;      // 载荷字节数(不含cmd)，原为 len-4 会对2字节帧下溢
    frame[4] = ((len - 2) >> 8) & 0xFF;
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

// 回规划失败帧(0x0B)：串口线程(拒绝新指令)与规划线程(规划失败)共用
static void send_fail() {
    uint16_t cmd = protocal::Cmd::PLAN_FAIL;
    send_frame((uint8_t*)&cmd, 2);
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
static bool parse_delta_frame(const uint8_t* data, size_t len, protocal::PlanDeltaReq* out_req){
    //总长度 = 帧头(2) + seq(1) + 长度字段(2) + cmd(2) + 数据区 + CRC(2)
    if(len < 8)return false;

    uint16_t data_len = data[3] | (data[4] << 8);
    size_t total = 5 + 2 + data_len + 2;
    if(len < total)return false;

    memcpy(out_req, data + 5, sizeof(protocal::PlanDeltaReq));

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
    // 从工作区安装后的包共享目录定位，不依赖硬编码绝对路径与当前工作目录
    const std::string urdf_path = ament_index_cpp::get_package_share_directory("engineer") + "/urdf/engineer.urdf";
    const std::string srdf_path = ament_index_cpp::get_package_share_directory("engineer_moveit2") + "/config/engineer.srdf";
    std::ifstream urdf_f(urdf_path);
    std::ifstream srdf_f(srdf_path);
    if(!urdf_f.is_open() || !srdf_f.is_open()){
        RCLCPP_ERROR(LOGGER, "URDF/SRDF 打开失败: %s | %s", urdf_path.c_str(), srdf_path.c_str());
        return false;
    }
    std::string urdf_str((std::istreambuf_iterator<char>(urdf_f)), std::istreambuf_iterator<char>());
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
// 两组关节角逐轴取最大绝对差(rad)，返回 {最大偏差, 偏差最大的轴下标}
static std::pair<double, int> max_joint_error(const std::vector<double>& actual,
                                              const std::vector<float>& target){
    const size_t count = std::min(actual.size(), target.size());
    double max_error = 0.0;
    int worst_joint = 0;
    for(size_t i = 0; i < count; i++){
        const double error = std::fabs(actual[i] - target[i]);
        if(error > max_error){
            max_error = error;
            worst_joint = (int)i;
        }
    }
    return std::make_pair(max_error, worst_joint);
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
        double max_diff = 0.0;
        if(!best_trajectory.empty()){
            max_diff = max_joint_error(pts[i].positions, best_trajectory.back()).first;
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
    if(cur_joints.size() != NUM_JOINTS){
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
    const char* joint_names[NUM_JOINTS] = {"yaw", "p1", "p2", "p3", "roll", "ep", "er"};
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
    std::vector<double> q(NUM_JOINTS);
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
        for(int j = 0; j < NUM_JOINTS; j++){
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
        std::vector<float> target_joint_positions;
        {
            std::lock_guard<std::mutex> lock(g_exec_mutex);
            if(!g_exec_traj.empty())target_joint_positions = g_exec_traj.back();
        }
        if(actual.empty() || target_joint_positions.empty())continue;

        const std::pair<double, int> worst_joint = max_joint_error(actual, target_joint_positions);
        const double dist = worst_joint.first;

        // 诊断：主导偏差轴 + 该轴 实际->目标 + 反馈新鲜度（区分"到不了"与"反馈停更"）
        {
            const int dom = worst_joint.second;
            int64_t age = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now().time_since_epoch()).count() - g_last_feedback_ms.load();
            RCLCPP_WARN_THROTTLE(LOGGER, *g_node->get_clock(), 200,
                "诊断 dist=%.3f best=%.3f 主偏差轴=%d(%.3f->%.3f) 反馈延迟=%ldms",
                dist, best_dist, dom, actual[dom], target_joint_positions[dom], (long)age);
        }

        auto now = std::chrono::steady_clock::now();
        if(dist < best_dist - progress_eps){                       // 仍在靠近终点
            best_dist = dist;
            last_progress = now;
        } else if(now - last_progress > stall_timeout){            // 长时间不再靠近 → 卡死
            RCLCPP_WARN(LOGGER, "长时间无进展 (dist=%.3f, best=%.3f), 请求中止", dist, best_dist);
            g_exec_fail_reason = 1;                                // 由规划线程统一发0x0B并回IDLE
        }
    }
}

static void abort_to_idle();   // 定义在 fail_and_idle 之后

// 登记一个规划需求：非 IDLE 时拒绝(回0x0B)，避免执行中被新指令打断
static bool take_request(PlanKind kind){
    if(g_state.load() != State::IDLE){
        RCLCPP_WARN(LOGGER, "执行中收到新指令 0x%02X, 已拒绝", (int)kind);
        send_fail();
        return false;
    }
    g_req_id++;                 // 新请求序号
    g_fail_reason.clear();      // 清除上一次失败原因
    g_abort_stream = false;     // 清除上一次打断标记
    g_requested_step = kind;         // 记录新计划类型
    g_state = State::PLANNING;  // 切换到规划状态
    return true;
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
                if(recv_cmd >= protocal::Cmd::STEP_APPROACH && recv_cmd <= protocal::Cmd::STEP_ROT_Q){
                    // 单步指令(0x01~0x05)：只登记需求，视觉/规划/下发都在规划线程里串行做
                    if(take_request((PlanKind)recv_cmd)){
                        RCLCPP_INFO(LOGGER, "收到步骤指令 0x%02X", recv_cmd);
                    }
                }
                else if(recv_cmd == protocal::Cmd::STEP_ABORT){
                    abort_to_idle();                       // 维持当前位姿
                }
                else if(recv_cmd == protocal::Cmd::DBG_PLAN_REQ){
                    protocal::PlanReq req;
                    if(parse_frame(rx_buffer.data() + i, total, &req)){
                        {
                            std::lock_guard<std::mutex> lock(g_req_mutex);
                            g_recv_req = req; // 保存新计划
                        }
                        if(take_request(PlanKind::DBG_POSE)){
                            RCLCPP_INFO(LOGGER, "收到 PlanReq(0x07): 位姿 = [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]", req.pose[0], req.pose[1], req.pose[2], req.pose[3], req.pose[4], req.pose[5]);
                        }
                    } else {
                        RCLCPP_ERROR(LOGGER, "PlanReq(0x07) 解析失败");
                    }
                }
                else if(recv_cmd == protocal::Cmd::DBG_DELTA_REQ){
                    protocal::PlanDeltaReq delta_request;
                    if(parse_delta_frame(rx_buffer.data() + i, total, &delta_request)){
                        // 绝对目标 = 当前末端位姿 ⊕ delta(基座系)，解析时定下供重规划复用
                        Eigen::Isometry3d d = pose_to_isometry(delta_request.delta);
                        Eigen::Isometry3d cur = current_tcp_pose();
                        {
                            std::lock_guard<std::mutex> lock(g_req_mutex);
                            g_cart_goal = cur;
                            g_cart_goal.translation() += d.translation();
                            g_cart_goal.linear() = d.linear() * cur.linear();
                        }
                        if(take_request(PlanKind::DBG_DELTA)){
                            RCLCPP_INFO(LOGGER, "收到 PlanDeltaReq(0x08): 增量 = [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]", delta_request.delta[0], delta_request.delta[1], delta_request.delta[2], delta_request.delta[3], delta_request.delta[4], delta_request.delta[5]);
                            RCLCPP_INFO(LOGGER, "笛卡尔目标: (%.3f, %.3f, %.3f)", g_cart_goal.translation().x(), g_cart_goal.translation().y(), g_cart_goal.translation().z());
                        }
                    } else {
                        RCLCPP_ERROR(LOGGER, "PlanDeltaReq(0x08) 解析失败");
                    }
                }
                else if(recv_cmd == protocal::Cmd::DBG_REQ_ARC){
                    protocal::PlanReqArc req_arc;
                    if(parse_arc_frame(rx_buffer.data() + i, total, &req_arc)){
                        {
                            std::lock_guard<std::mutex> lock(g_arc_mutex);
                            g_arc_req = req_arc;
                        }
                        if(take_request(PlanKind::DBG_ARC)){
                            RCLCPP_INFO(LOGGER, "收到 PlanReqArc(0x09): 轴点=(%.4f, %.4f, %.4f) 轴方向=(%.3f, %.3f, %.3f) 转角=%.2f度",
                                req_arc.axis_point[0], req_arc.axis_point[1], req_arc.axis_point[2],
                                req_arc.axis_dir[0], req_arc.axis_dir[1], req_arc.axis_dir[2],
                                req_arc.theta * rad2deg);
                        }
                    } else {
                        RCLCPP_ERROR(LOGGER, "PlanReqArc(0x09) 解析失败");
                    }
                }
                else if(recv_cmd == protocal::Cmd::PLAN_FEEDBACK){
                    protocal::PlanFeedback fb;
                    if(parse_feedback_frame(rx_buffer.data() + i, total, &fb)){
                        {
                            std::lock_guard<std::mutex> lock(g_feedback_mutex);
                            for(int k = 0; k < NUM_JOINTS; k++)g_feedback_joints[k] = fb.joints[k];
                            g_feedback_joints[4] = -fb.joints[4];   // 固件roll反馈与URDF反号，取反后与其余轴一致
                        }   // 作用域结束自动解锁
                        g_last_feedback_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                            std::chrono::steady_clock::now().time_since_epoch()).count();

                        if constexpr(RECORD_FEEDBACK){
                            float arm[7];
                            for(int k = 0; k < NUM_JOINTS; k++)arm[k] = fb.joints[k];
                            arm[4] = -fb.joints[4];   // 固件roll反馈与URDF反号
                            feedback_pub::save_feedback(fb.step_idx, arm);
                        }  // 文件IO在锁外
                        g_exec_step = fb.step_idx;

                        if constexpr(!CALIB_MODE){
                            if(fb.step_idx != g_last_feedback_step){
                                g_last_feedback_step = fb.step_idx;
                                RCLCPP_INFO(LOGGER, "收到 PlanFeedback(0x06): 关节角 = [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f] 步=%d", g_feedback_joints[0], g_feedback_joints[1], g_feedback_joints[2], g_feedback_joints[3], g_feedback_joints[4], g_feedback_joints[5], g_feedback_joints[6], fb.step_idx);
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
                    //         RCLCPP_WARN_THROTTLE(LOGGER, *g_node->get_clock(), 2000, "PlanFeedback(0x06) 解析失败, 已跳过该帧");
                    //     }
                    // }
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

// 按关节限速反推每个点的下发间隔 dt(s)：dt = max_j(|Δq_j| / v_j)
// 点密的段发得快、点疏的段发得慢，取代原先固定 100ms；算出的 dt 夹在 [MIN_SEND_DT, MAX_SEND_DT]
static std::vector<double> time_parameterize_(const std::vector<std::vector<float> >& traj){
    std::vector<double> dts(traj.size(), MAX_SEND_DT);
    if(traj.size() < 2)return dts;              // 单点(标定)沿用原 100ms 间隔
    dts[0] = 0.0;                               // 首点立即下发

    for(size_t i = 1; i < traj.size(); i++){
        double dt = MIN_SEND_DT;
        for(int j = 0; j < NUM_JOINTS; j++){
            const double dq = std::fabs((double)traj[i][j] - traj[i - 1][j]);
            dt = std::max(dt, dq / (joint_vel_deg[j] * deg2rad * JOINT_VEL_SCALE));
        }
        dts[i] = std::min(dt, MAX_SEND_DT);
    }
    return dts;
}

// apply_mapping=false 时跳过 sign/offset 换算，直接把输入当作下位机 raw 角下发（标定用）
// 每点间隔按关节限速算出的真实 dt 下发，替代原先固定的 100ms
static void send_trajectory(const std::vector<std::vector<float> >& traj, bool apply_mapping = true){
    const std::vector<double> dts = time_parameterize_(traj);
    double estimated_duration = 0.0;
    for(size_t i = 0; i < dts.size(); i++)estimated_duration += dts[i];

    const uint32_t request_id = g_req_id.load();
    const int step = (int)g_requested_step.load();

    // 先算好下位机 raw 角：发送与落盘共用同一份数据，保证记录与实际下发字节一致
    std::vector<std::vector<float> > hardware_angles(traj.size(), std::vector<float>(NUM_JOINTS, 0.0f));
    for(size_t i = 0; i < traj.size(); i++){
        for(int j = 0; j < NUM_JOINTS; j++){
            hardware_angles[i][j] = apply_mapping ? (g_joint_sign[j] * traj[i][j] + g_joint_offset[j]) : traj[i][j];
        }
    }

    const auto send_start = std::chrono::steady_clock::now();
    int sent_points = 0;
    auto next = std::chrono::steady_clock::now();
    for(size_t i = 0; i < hardware_angles.size(); i++){
        if(!g_running || g_abort_stream.load())break;
        std::vector<uint8_t> buf;
        buf.push_back(protocal::Cmd::TRAJ_POINT & 0xFF);      // 0x0A 轨迹点帧
        buf.push_back((protocal::Cmd::TRAJ_POINT >> 8) & 0xFF);
        buf.push_back(i & 0xFF);
        buf.push_back(i >> 8);
        for(size_t j = 0; j < hardware_angles[i].size(); j++){
            uint8_t* p = (uint8_t*)&hardware_angles[i][j];
            for(int k = 0; k < 4; k++)buf.push_back(p[k]);

        }
        RCLCPP_INFO(LOGGER, "下发关节角 = [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]", traj[i][0], traj[i][1], traj[i][2], traj[i][3], traj[i][4], traj[i][5], traj[i][6]);
        send_frame(buf.data(), buf.size());
        sent_points++;
        // 按绝对时刻排点，避免 sleep 误差累积；分片睡眠保证能及时响应退出
        next += std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(dts[i]));
        while(g_running && !g_abort_stream.load()){
            auto now = std::chrono::steady_clock::now();
            if(now >= next)break;
            auto rem = next - now;
            std::this_thread::sleep_for(rem < std::chrono::milliseconds(5) ? rem : std::chrono::milliseconds(5));
        }
    }
    // 运动结束后统一落盘：发送循环内零 IO、零发布
    feedback_pub::save_cmd_traj(request_id, step, hardware_angles, dts);
    feedback_pub::record_send_result(request_id, step, g_abort_stream.load(),
                                     sent_points, (int)hardware_angles.size(),
                                     estimated_duration * 1000.0,
                                     feedback_pub::milliseconds_since(send_start));
    RCLCPP_INFO(LOGGER, "下发完成: %d/%zu 点, 预计耗时 %.0f ms (原固定节奏 %.0f ms)",
                sent_points, hardware_angles.size(), estimated_duration * 1000.0, hardware_angles.size() * 100.0);
}

// 统一失败出口：回错误码0x0B并回到IDLE，以便接收下一个目标
static void fail_and_idle(const char* reason){
    RCLCPP_ERROR(LOGGER, "%s", reason);
    send_fail();
    {
        std::lock_guard<std::mutex> lock(g_exec_mutex);
        g_exec_traj.clear();
    }
    g_arrived_cnt = 0;
    g_exec_fail_reason = 0;
    g_requested_step = PlanKind::NONE;
    g_state = State::IDLE;
}

// 打断出口(0x0D)：停发并回IDLE。臂会停在最后收到的点上，即维持当前位姿（串口线程/规划线程都可调用）
static void abort_to_idle(){
    if(g_state.load() == State::IDLE)return;
    g_abort_stream = true;
    {
        std::lock_guard<std::mutex> lock(g_exec_mutex);
        g_exec_traj.clear();
    }
    g_arrived_cnt = 0;
    g_exec_fail_reason = 0;
    g_requested_step = PlanKind::NONE;
    g_state = State::IDLE;
    RCLCPP_WARN(LOGGER, "已打断, 停止下发并维持当前位姿");
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

// ==================== 视觉接口：海康相机 + PnP ====================
// 相机参数（按现场调整）
static const int VISION_CAM_INDEX   = 0;      // 相机序列索引（枚举到的第几台）
static const int VISION_EXPOSURE_US = 8000;   // 曝光时间(us)
static const int VISION_GAIN_DB     = 15;     // 信号放大增益(dB)
static const int VISION_BUFFER_NUM  = 2;      // 图像缓存节点数

static PnPDate       g_pnp_data;              // 标定数据（内参/畸变/3D点，均已有静态默认值）
static CameraControl g_cam;                   // 相机句柄持有者
static CameraStream *g_stream = nullptr;      // ★ 相机内部线程：死循环取图，缓存最新一帧
static PnPTuner      g_pnp_tuner;             // 累计重投影误差，给出 points3D 调整建议

// vt[0] / vt[1] 取出来的项：6 维位姿 或 3 维向量
struct VisionItem{
    Eigen::Matrix<double,6,1> pose = Eigen::Matrix<double,6,1>::Zero();
    Eigen::Vector3d vec = Eigen::Vector3d::Zero();
    int n = 6;                       // 6 = 位姿；3 = 向量

    int size() const { return n; }

    // 内层索引：0 ~ n-1
    double operator[](int k) const {
        assert(k >= 0 && k < n);
        return n == 6 ? pose[k] : vec[k];
    }

    // 直接赋给定长向量：Eigen::Matrix<double,6,1> m = vt[0];
    operator Eigen::Matrix<double,6,1>() const { return pose; }
    operator Eigen::Vector3d() const { return vec; }
};

struct VisionTarget{
    bool ok = false;                                 // 是否解算成功
    Eigen::Vector3d p = Eigen::Vector3d::Zero();     // 目标点 base_link系(m)
    Eigen::Vector3d v = Eigen::Vector3d::UnitX();    // 目标向量 base_link系
    Eigen::Vector3d rpy = Eigen::Vector3d::Zero();   // 目标姿态 roll/pitch/yaw(rad)
    double err = -1.0;                               // 反投影误差(px)

    // ★ 两级索引：vt[0] = 六自由度 [x y z roll pitch yaw]，vt[1] = 三维向量 [x y z]
    //   再往下一层：vt[0][3] -> roll，vt[1][2] -> v.z
    //   也可直接赋给定长向量：Eigen::Matrix<double,6,1> m = vt[0];
    VisionItem operator[](int i) const {
        assert(i == 0 || i == 1);
        VisionItem it;
        if(i == 0){ it.pose << p, rpy; it.n = 6; }
        else      { it.vec = v;        it.n = 3; }
        return it;
    }
};

// 调试预览：弹窗显示当前相机画面。ESC/q 关闭预览；无显示环境时自动停用，不影响主流程
static void vision_show_debug(const cv::Mat &frame)
{
    static int state = 0;   // 0=未开窗 1=已开窗 2=已停用
    if(state == 2 || frame.empty())return;

    try{
        if(state == 0){
            cv::namedWindow("camera", cv::WINDOW_NORMAL | cv::WINDOW_KEEPRATIO);
            state = 1;
        }
        cv::Mat view;                                   // 原图可能远超屏幕，等比缩到 1280 宽
        const int max_w = 1280;
        if(frame.cols > max_w){
            const double s = (double)max_w / frame.cols;
            cv::resize(frame, view, cv::Size(), s, s);
        }else{
            view = frame;
        }
        cv::imshow("camera", view);
        const int key = cv::waitKey(1) & 0xFF;
        if(key == 27 || key == 'q'){                    // ESC / q：关闭预览
            cv::destroyWindow("camera");
            state = 2;
        }
    }catch(const cv::Exception &e){
        RCLCPP_WARN(LOGGER, "相机预览不可用, 已停用: %s", e.what());
        state = 2;
    }
}

// ★ 相机线程里的每帧回调：解算 + 打印重投影误差 + 定期给三维坐标修正建议
static void vision_on_frame(const cv::Mat &frame)
{
#if VISION_DEBUG_SHOW
    vision_show_debug(frame);      // 放在解算前，解算失败时也能看到画面
#endif
    // PnPTuner::add() 内部完成检测+解算，并累计每个 3D 点的偏差
    const CarTarget t = g_pnp_tuner.add(frame, g_pnp_data);
    if(!t.ok){
        RCLCPP_WARN_THROTTLE(LOGGER, *g_node->get_clock(), 2000,
                             "视觉解算失败: %s", t.reason.c_str());
        return;
    }

    // 重投影误差（每秒最多一条，避免刷屏）
    RCLCPP_INFO_THROTTLE(LOGGER, *g_node->get_clock(), 1000,
        "视觉: 重投影误差=%.2f px | 目标点=(%.1f, %.1f, %.1f) mm",
        t.errMeanPx, t.point[0], t.point[1], t.point[2]);

    // 每 100 帧输出一次 points3D 调整建议
    if(g_pnp_tuner.frames() % 100 == 0){
        RCLCPP_INFO(LOGGER, "%s", g_pnp_tuner.advice(g_pnp_data).c_str());
    }
}

// 开相机 + 起取帧线程；只在 main 启动时调一次
static bool vision_init()
{
    CameraControl::sdk_initialize();
    CameraControl::enumerate_devices();
    g_cam.create_camerahandle(VISION_CAM_INDEX);
    g_cam.open_camera();
    g_cam.set_exposuregain(VISION_EXPOSURE_US, VISION_GAIN_DB);
    g_cam.set_buffer(VISION_BUFFER_NUM);
    g_cam.start_streaming();              // 必须先起流，否则 grab 取不到图

    g_stream = new CameraStream(g_cam);
    g_stream->setCallback(vision_on_frame);   // ★ 相机线程内每帧解算 + 打印误差/建议
    g_stream->start();                    // ★ 起相机内部线程（死循环取图）

    // 自检：500ms 内能拿到一帧就算通了（目标可能不在视野内，不以此判失败）
    cv::Mat frame;
    if(!g_stream->wait(frame, 500)){
        RCLCPP_ERROR(LOGGER, "相机自检失败: 500ms 内没有收到图像");
        return false;
    }
    RCLCPP_INFO(LOGGER, "相机就绪: 首帧 %dx%d", frame.cols, frame.rows);
    return true;
}

// 停相机：顺序不能改
static void vision_shutdown()
{
    g_cam.stop_stream();                  // 让阻塞中的 grab 立刻返回
    if(g_stream){
        g_stream->stop();                 // 唤醒 wait() 并 join 相机线程
        delete g_stream;
        g_stream = nullptr;
    }
    g_cam.close_camera();
    g_cam.destroy_camerahandle();         // 必须在 sdk_finalize 之前
    CameraControl::sdk_finalize();
}

// ★ 取缓存里的最新一帧 -> PnP 解算
static VisionTarget vision_locate()
{
    VisionTarget t;
    if(!g_stream){
        RCLCPP_ERROR(LOGGER, "视觉未初始化");
        return t;
    }

    cv::Mat frame;
    if(!g_stream->wait(frame, 200)){      // ★ 相机线程的最新帧；没新帧最多等 200ms
        RCLCPP_ERROR(LOGGER, "视觉: 200ms 内没有新图");
        return t;
    }

    const CarTargetPose r = solveCarTargetPose(frame, g_pnp_data);
    if(!r.ok){
        RCLCPP_ERROR(LOGGER, "视觉解算失败: %s", r.reason.c_str());
        return t;
    }

    const cv::Vec6d pose6 = r[0];   // 0~5: x,y,z,roll,pitch,yaw
    const cv::Vec3d vec3  = r[1];   // 6~8: 目标向量

    t.p   = Eigen::Vector3d(pose6[0], pose6[1], pose6[2]) / 1000.0;  // mm -> m
    t.rpy = Eigen::Vector3d(pose6[3], pose6[4], pose6[5]);           // rad
    t.v   = Eigen::Vector3d(vec3[0], vec3[1], vec3[2]);
    t.err = r.errMeanPx;
    t.ok  = true;
    return t;
}

// 按需求种类生成待下发轨迹：单步指令走"视觉->规划"，手动调试入口走原逻辑
static bool build_plan_(PlanKind kind, std::vector<std::vector<float> >& traj){
    const Eigen::Isometry3d cur = current_tcp_pose();
    switch(kind){
        case PlanKind::STEP_APPROACH: {
            const VisionTarget vision_target = vision_locate();
            if(!vision_target.ok){
                 g_fail_reason = "NO_VISION_TARGET";
                return false;
            }
            Eigen::Isometry3d goal = cur;
            goal.translation() = vision_target.p;
            // 姿态用视觉给出的 rpy：R = Rz(yaw) * Ry(pitch) * Rx(roll)
            goal.linear() = (Eigen::AngleAxisd(vision_target.rpy[2], Eigen::Vector3d::UnitZ())
                           * Eigen::AngleAxisd(vision_target.rpy[1], Eigen::Vector3d::UnitY())
                           * Eigen::AngleAxisd(vision_target.rpy[0], Eigen::Vector3d::UnitX())).toRotationMatrix();
            RCLCPP_INFO(LOGGER, "步骤1 接近: 目标=(%.3f, %.3f, %.3f) rpy=(%.4f, %.4f, %.4f) rad err=%.2fpx",
                        vision_target.p.x(), vision_target.p.y(), vision_target.p.z(),
                        vision_target.rpy[0], vision_target.rpy[1], vision_target.rpy[2], vision_target.err);
            traj = best_plan_(goal);
            return !traj.empty();
        }
        case PlanKind::STEP_INSERT:
        case PlanKind::STEP_SHIFT: {
            const bool insert = (kind == PlanKind::STEP_INSERT);
            const VisionTarget vision_target = vision_locate();   // 0x02/0x03 的增量也由视觉给出
            if(!vision_target.ok){
                 g_fail_reason = "NO_VISION_TARGET";
                return false;
            }
            const Eigen::Vector3d delta = vision_target.v / 1000.0;   // v 单位 mm，转 m    
            Eigen::Isometry3d goal = cur;
            goal.translation() += delta;
            std::vector<Eigen::Isometry3d> waypoints;
            waypoints.push_back(cur);
            waypoints.push_back(goal);
            RCLCPP_INFO(LOGGER, "步骤%d %s: 增量=(%.3f, %.3f, %.3f) 目标=(%.3f, %.3f, %.3f) err=%.2fpx", insert ? 2 : 3, insert ? "插入" : "平移",
                        delta.x(), delta.y(), delta.z(),
                        goal.translation().x(), goal.translation().y(), goal.translation().z(), vision_target.err);
            traj = cartesian_plan_(waypoints);
            if(traj.empty()){
                g_fail_reason = "IK_FAIL";
                return false;
            }
            check_traj(traj, waypoints, insert ? "STEP2" : "STEP3");
            return true;
        }
        case PlanKind::STEP_ROT_P:
        case PlanKind::STEP_ROT_Q: {
            const bool rot_p = (kind == PlanKind::STEP_ROT_P);
            const Eigen::Vector3d axis_dir = (rot_p ? STEP_ROT_P_AXIS : STEP_ROT_Q_AXIS).normalized();
            const double theta = rot_p ? STEP_ROT_P_THETA : STEP_ROT_Q_THETA;
            const int N = arc_segments(theta);
            // 旋转中心暂取当前末端位置（真实P/Q轴需视觉或标定给出）
            auto waypoints = arc_waypoints_(cur.translation(), axis_dir, theta, cur, N);
            RCLCPP_INFO(LOGGER, "步骤%d 旋转: 轴点=(%.3f, %.3f, %.3f) 轴=(%.2f, %.2f, %.2f) 角度=%.1f度（轴点暂取当前末端位置）", rot_p ? 4 : 5,
                        cur.translation().x(), cur.translation().y(), cur.translation().z(),
                        axis_dir.x(), axis_dir.y(), axis_dir.z(), theta * rad2deg);
            traj = cartesian_plan_(waypoints);
            if(traj.empty()){
                g_fail_reason = "IK_FAIL";
                return false;
            }
            check_traj(traj, waypoints, rot_p ? "STEP4" : "STEP5");
            return true;
        }
        case PlanKind::DBG_POSE: {
            Eigen::Isometry3d target;
            {
                std::lock_guard<std::mutex> lock(g_req_mutex);
                target = pose_to_isometry(g_recv_req.pose);
            }
            // 用四元数打印，避免 eulerAngles 的多解问题
            Eigen::Quaterniond q_print(target.rotation());
            RCLCPP_INFO(LOGGER, "规划目标: (%.3f,%.3f,%.3f) 四元数(%.3f,%.3f,%.3f,%.3f)",
                    target.translation().x(), target.translation().y(), target.translation().z(),
                    q_print.w(), q_print.x(), q_print.y(), q_print.z());
            traj = best_plan_(target);
            return !traj.empty();
        }
        case PlanKind::DBG_DELTA: {
            Eigen::Isometry3d cart_goal;
            {
                std::lock_guard<std::mutex> lock(g_req_mutex);
                cart_goal = g_cart_goal;
            }
            std::vector<Eigen::Isometry3d> waypoints;
            waypoints.push_back(cur);
            waypoints.push_back(cart_goal);
            traj = cartesian_plan_(waypoints);
            if(traj.empty()){
                g_fail_reason = "IK_FAIL";
                return false;
            }
            check_traj(traj, waypoints, "CART");
            return true;
        }
        case PlanKind::DBG_ARC: {
            protocal::PlanReqArc req;
            {
                std::lock_guard<std::mutex> lock(g_arc_mutex);
                req = g_arc_req;
            }
            const Eigen::Vector3d axis_point(req.axis_point[0], req.axis_point[1], req.axis_point[2]);
            Eigen::Vector3d axis_dir(req.axis_dir[0], req.axis_dir[1], req.axis_dir[2]);
            const double theta = req.theta;
            if(!axis_point.allFinite() || !axis_dir.allFinite() || axis_dir.norm() < 1e-6){
                RCLCPP_ERROR(LOGGER, "Arc rejected: invalid axis");
                return false;
            }
            axis_dir.normalize();
            const int N = arc_segments(theta);
            auto waypoints = arc_waypoints_(axis_point, axis_dir, theta, cur, N);
            traj = cartesian_plan_(waypoints);
            if(traj.empty()){
                g_fail_reason = "IK_FAIL";
                return false;
            }
            check_traj(traj, waypoints, "ARC");
            RCLCPP_INFO(LOGGER, "圆弧 %d 段", N);
            return true;
        }
        case PlanKind::NONE:
        default:
            RCLCPP_ERROR(LOGGER, "未登记的规划请求 0x%02X", (int)kind);
            return false;
    }
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
                const PlanKind kind = g_requested_step.load();
                // 起点来自反馈，反馈太久没更新就不敢规划
                const int64_t now = std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::steady_clock::now().time_since_epoch()).count();
                const int64_t last = g_last_feedback_ms.load();
                const uint32_t request_id = g_req_id.load();
                if(last == 0 || now - last > FEEDBACK_TIMEOUT_MS){
                    feedback_pub::record_plan_result(request_id, (int)kind, false, 0.0, 0, "FEEDBACK_STALE");
                    fail_and_idle("反馈超时, 拒绝规划");
                    break;
                }

                std::vector<std::vector<float> > traj;
                const auto plan_start = std::chrono::steady_clock::now();
                if(!build_plan_(kind, traj)){
                    feedback_pub::record_plan_result(request_id, (int)kind, false,
                                                     feedback_pub::milliseconds_since(plan_start),
                                                     0, g_fail_reason);
                    fail_and_idle("规划失败");
                    break;
                }
                feedback_pub::record_plan_result(request_id, (int)kind, true,
                                                 feedback_pub::milliseconds_since(plan_start),
                                                 (int)traj.size(), "");
                if(g_abort_stream.load()){      // 规划期间被0x0D取消
                    abort_to_idle();
                    break;
                }
                send_trajectory(traj);
                if(g_abort_stream.load()){      // 下发期间被0x0D取消
                    abort_to_idle();
                    break;
                }
                g_executing_step = kind;
                RCLCPP_INFO(LOGGER, "已下发 %zu 点, 进入执行状态", traj.size());
                start_execution(traj);
                break;
            }
            case State::EXECUTING:{
                if(g_abort_stream.load()){      // 0x0D 打断：维持当前位姿
                    abort_to_idle();
                    break;
                }
                // 监视线程判定卡死：不再重规划，直接回错误码并回IDLE
                if(g_exec_fail_reason.load() != 0){
                    feedback_pub::record_failure(g_req_id.load(), (int)g_executing_step, "STALL");
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
                std::vector<float> target_joint_positions;   // 轨迹终点拷贝，锁外安全使用
                {
                    std::lock_guard<std::mutex> lock(g_exec_mutex);
                    if(!g_exec_traj.empty())target_joint_positions = g_exec_traj.back();
                }
                const bool arrived = !actual.empty() && !target_joint_positions.empty()
                                  && max_joint_error(actual, target_joint_positions).first <= arrive_eps;
                g_arrived_cnt = arrived ? g_arrived_cnt + 1 : 0;
                if(g_arrived_cnt >= arrive_frames){
                    // 执行残差（只打印）：反馈FK 与 规划末点FK 的位置/姿态差
                    moveit::core::RobotState actual_state(g_robot_model), target_state(g_robot_model);
                    actual_state.setJointGroupPositions(g_joint_group, actual);
                    target_state.setJointGroupPositions(g_joint_group, std::vector<double>(target_joint_positions.begin(), target_joint_positions.end()));
                    actual_state.update(); target_state.update();
                    const moveit::core::LinkModel* tip = g_joint_group->getLinkModel("link11");
                    const Eigen::Isometry3d target_transform = target_state.getGlobalLinkTransform(tip);
                    const Eigen::Isometry3d current_transform = actual_state.getGlobalLinkTransform(tip);
                    const double position_error_mm = (target_transform.translation() - current_transform.translation()).norm() * 1000.0;
                    const double rotation_error_deg = Eigen::AngleAxisd(target_transform.linear().transpose() * current_transform.linear()).angle() * rad2deg;
                    RCLCPP_INFO(LOGGER, "到位残差: 位置=%.2fmm 姿态=%.3fdeg", position_error_mm, rotation_error_deg);
                    feedback_pub::record_arrival(g_req_id.load(), (int)g_executing_step,
                                                 position_error_mm, rotation_error_deg);
                    g_arrived_cnt = 0;
                    // 单步指令才回 0x0C，下位机据此推进下一步；手动调试入口不回
                    const int step = (int)g_executing_step;
                    if(step >= (int)PlanKind::STEP_APPROACH && step <= (int)PlanKind::STEP_ROT_Q){
                        protocal::StepDone done;
                        done.cmd = protocal::Cmd::STEP_DONE;
                        done.step = (uint16_t)step;
                        send_frame((uint8_t*)&done, sizeof(done));
                        RCLCPP_INFO(LOGGER, "步骤%d 完成, 已回 0x0C", step);
                    }
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
    // ★ 开相机 + 起相机内部取帧线程（失败直接退出，避免跑起来没视觉）
    if(!vision_init()){
        RCLCPP_FATAL(LOGGER, "相机初始化失败");
        close_serial();
        rclcpp::shutdown();
        return 1;
    }

    // ★ 解算接口：拿到 6 自由度 + 3 维向量（不打印）
    const VisionTarget vt0 = vision_locate();
    if(!vt0.ok){
        RCLCPP_WARN(LOGGER, "视觉自检: 未解出目标，收到 0x01 时会重试");
    }
    // vt[0] → 六自由度；vt[1] → 三自由度量

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
    vision_shutdown();      // ★ 先停相机：唤醒可能阻塞在 wait() 里的规划线程
    close_serial();
    serial_thr.join();
    plan_thr.join();
    monitor_thr.join();

    rclcpp::shutdown();
    return 0;
#endif
}