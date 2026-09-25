// ========================================
// 路径规划核心代码学习模板（填空版）
// ========================================
// 本文件是一个学习模板，你需要根据注释填写代码
// 核心流程：初始化 → 加载模型 → IK求解 → 规划 → 导出

#include <moveit/planning_interface/planning_interface.h>      // 规划器接口
#include <moveit/robot_model/robot_model.h>                    // 机器人模型
#include <moveit/planning_scene/planning_scene.h>              // 规划场景
#include <moveit/collision_detection/collision_common.h>       // 碰撞检测
#include <moveit/kinematic_constraints/utils.h>                // 约束工具
#include <moveit/robot_model_loader/robot_model_loader.h>      // 模型加载器
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>            // 坐标变换
#include <fstream>                                             // 文件操作
#include <sstream>                                             // 字符串流
#include <pluginlib/class_loader.hpp>                          // 插件加载
#include <cstdlib>                                             // 随机数

// 定义日志器
static const rclcpp::Logger LOGGER = rclcpp::get_logger("plan_to_csv");

static std::string read_file(const std::string& path) {
  std::ifstream f(path);
  std::stringstream buf;
  buf << f.rdbuf();
  return buf.str();
}

int main(int argc, char** argv)
{
  srand(time(0));

  // =====================================
  // 1. 初始化 ROS 节点
  // =====================================
  // 需要完成：
  //   - 调用 rclcpp::init() 初始化 ROS 上下文
  //   - 创建节点，名称为 "plan_to_csv"
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("plan_to_csv");



  // =====================================
  // 2. 读取命令行参数
  // =====================================
  // 需要完成：
  //   - 读取 x, y, z（目标位置，单位米）
  //   - 读取 roll, pitch, yaw（欧拉角，单位度，需要乘 M_PI/180 转换为弧度）
  //   - 读取 output_file（输出文件路径，字符串）
  // 注意：ROS2 中必须先 declare_parameter 再 get_parameter 才能读到命令行传入的值
  node->declare_parameter<double>("x", 0.03);
  node->declare_parameter<double>("y", 0.0);
  node->declare_parameter<double>("z", 0.0);
  node->declare_parameter<double>("roll", 0.0);
  node->declare_parameter<double>("pitch", 0.0);
  node->declare_parameter<double>("yaw", 0.0);
  node->declare_parameter<std::string>("output_file", "trajectory.csv");

  double x = node->get_parameter("x").as_double();
  double y = node->get_parameter("y").as_double();
  double z = node->get_parameter("z").as_double();

  double roll = node->get_parameter("roll").as_double() * M_PI / 180.0;
  double pitch = node->get_parameter("pitch").as_double() * M_PI / 180.0;
  double yaw = node->get_parameter("yaw").as_double() * M_PI / 180.0;

  std::string output_file = node->get_parameter("output_file").as_string();



  // =====================================
  // 3. 设置运动学参数
  // =====================================
  // 需要完成：
  //   - 声明 kinematics_solver = "kdl_kinematics_plugin/KDLKinematicsPlugin"
  //   - 声明 kinematics_solver_search_resolution = 0.005
  //   - 声明 kinematics_solver_timeout = 1.0
  node->declare_parameter("arm_group.kinematics_solver", "kdl_kinematics_plugin/KDLKinematicsPlugin");
  node->declare_parameter("arm_group.kinematics_solver_search_resolution", 0.005);
  node->declare_parameter("arm_group.kinematics_solver_timeout", 1.0);



  // =====================================
  // 4. 从文件加载 URDF 和 SRDF
  // =====================================
  // 需要完成：
  //   - 读取 URDF 文件：/home/yh/2026_Engineer_ws/src/engineer/urdf/engineer.urdf
  //   - 读取 SRDF 文件：/home/yh/2026_Engineer_ws/src/engineer_moveit2/config/engineer.srdf
  //   - 保存为字符串 urdf_string 和 srdf_string
  std::string urdf_string = read_file("/home/yh/2026_Engineer_ws/src/engineer/urdf/engineer.urdf");
  std::string srdf_string = read_file("/home/yh/2026_Engineer_ws/src/engineer_moveit2/config/engineer.srdf");

  // =====================================
  // 5. 创建机器人模型
  // =====================================
  // 需要完成：
  //   - 将 URDF 写入 robot_description 参数（KDL 求解器初始化需要此参数）
  //   - 创建 RobotModelLoader::Options，传入 urdf_string 和 srdf_string
  //   - 创建 RobotModelLoader 对象
  //   - 调用 getModel() 获取 RobotModelPtr
  node->declare_parameter("robot_description", urdf_string);
  robot_model_loader::RobotModelLoader::Options options(urdf_string, srdf_string);
  robot_model_loader::RobotModelLoader robot_model_loader(node, options);
  moveit::core::RobotModelPtr robot_model = robot_model_loader.getModel();


  // =====================================
  // 6. 获取规划组
  // =====================================
  // 需要完成：
  //   - 调用 robot_model->getJointModelGroup("arm_group")
  //   - 保存为 const moveit::core::JointModelGroup* joint_model_group
  const moveit::core::JointModelGroup* joint_model_group = robot_model->getJointModelGroup("arm_group");



  // =====================================
  // 7. 创建规划场景
  // =====================================
  // 需要完成：
  //   - 创建 PlanningScenePtr，传入 robot_model
  //   - 获取当前状态：planning_scene->getCurrentStateNonConst()
  //   - 创建初始关节数组：全零，大小为 joint_model_group->getVariableCount()
  //   - 设置关节角度：current_state.setJointGroupPositions()
  planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));
  moveit::core::RobotState& current_state = planning_scene->getCurrentStateNonConst();
  current_state.setJointGroupPositions(joint_model_group,
      std::vector<double>(joint_model_group->getVariableCount(), 0.0));
  // 将初始状态同步回 planning_scene
  planning_scene->setCurrentState(current_state);



  // =====================================
  // 8. 加载 OMPL 规划器
  // =====================================
  // 需要完成：
  //   - 创建 pluginlib::ClassLoader，加载 "moveit_core" 的 "planning_interface::PlannerManager"
  //   - 调用 createUnmanagedInstance("ompl_interface/OMPLPlanner") 创建规划器
  //   - 调用 planner->initialize(robot_model, node, namespace)
  auto planner_loader = std::make_unique<pluginlib::ClassLoader<planning_interface::PlannerManager>>(
      "moveit_core", "planning_interface::PlannerManager");
  planning_interface::PlannerManagerPtr planner(
      planner_loader->createUnmanagedInstance("ompl_interface/OMPLPlanner"));
  planner->initialize(robot_model, node, node->get_namespace());


  // =====================================
  // 9. 逆运动学（IK）求解
  // =====================================
  // 9.1 创建目标位姿
  // 需要完成：
  //   - 创建 geometry_msgs::msg::Pose target_pose
  //   - 设置 position.x, y, z
  //   - 创建 tf2::Quaternion q，调用 setRPY(roll, pitch, yaw)
  //   - 设置 target_pose.orientation = tf2::toMsg(q)
  // 提示：输出零位 FK 位置/朝向方便调试 IK 可达性
  Eigen::Isometry3d fk_pose = current_state.getGlobalLinkTransform("link11");
  Eigen::Quaterniond fk_q(fk_pose.linear());
  RCLCPP_INFO(LOGGER, "FK at zero: link11 pos=(%.3f %.3f %.3f) quat=(%.2f %.2f %.2f %.2f)",
      fk_pose.translation().x(), fk_pose.translation().y(), fk_pose.translation().z(),
      fk_q.w(), fk_q.x(), fk_q.y(), fk_q.z());

  geometry_msgs::msg::Pose target_pose;
  target_pose.position.x = x;
  target_pose.position.y = y;
  target_pose.position.z = z;
  tf2::Quaternion q;
  q.setRPY(roll, pitch, yaw);
  target_pose.orientation = tf2::toMsg(q);


  // 9.2 获取当前关节值作为 IK 种子
  // 需要完成：
  //   - 创建 vector<double> seed_joints
  //   - 调用 current_state.copyJointGroupPositions(joint_model_group, seed_joints)
  std::vector<double> seed_joints;
  current_state.copyJointGroupPositions(joint_model_group, seed_joints);



  // 9.3 IK 求解（带碰撞检查）
  // 需要完成：
  //   - 获取运动学求解器：joint_model_group->getSolverInstance()
  //   - 循环最多 10 次，每次：
  //     - 如果 i > 0，添加随机扰动到种子状态
  //     - 调用 kin_solver->searchPositionIK()
  //     - 检查 error_code 是否为 SUCCESS
  //     - 如果成功，创建临时 RobotState，设置关节角度
  //     - 检查碰撞：temp_scene->checkCollision()
  //     - 如果无碰撞，退出循环
  //   - 如果找不到无碰撞解，报错退出
  const kinematics::KinematicsBaseConstPtr kin_solver = joint_model_group->getSolverInstance();
  if (!kin_solver) { RCLCPP_ERROR(LOGGER, "No kinematics solver available for arm_group"); return 1; }

  std::vector<double> goal_joints;
  bool ik_success = false;
  kinematics::KinematicsQueryOptions ik_options;
  for (int i = 0; i < 10; i++) {
    std::vector<double> current_seed = seed_joints;
    if (i > 0) {
      for (auto& val : current_seed)
        val += (rand() % 200 - 100) * 0.02;  // ±2.0 弧度扰动
    }
    std::vector<double> solution;
    moveit_msgs::msg::MoveItErrorCodes error_code;
    kin_solver->searchPositionIK(target_pose, current_seed, 1.0, solution, error_code, ik_options);

    if (error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
      // 碰撞检测
      moveit::core::RobotState temp_state(robot_model);
      temp_state.setJointGroupPositions(joint_model_group, solution);
      collision_detection::CollisionRequest col_req;
      collision_detection::CollisionResult col_res;
      planning_scene->checkCollision(col_req, col_res, temp_state);
      if (!col_res.collision) {
        goal_joints = solution;
        ik_success = true;
        break;
      }
    }
  }
  if (!ik_success) {
    RCLCPP_ERROR(LOGGER, "Failed to find a IK solution");
    return 1;
  }
  RCLCPP_INFO(LOGGER, "IK solved successfully");




  // =====================================
  // 10. 设置目标状态
  // =====================================
  // 需要完成：
  //   - 创建 moveit::core::RobotState goal_state(robot_model)
  //   - 设置关节角度：goal_state.setJointGroupPositions(joint_model_group, goal_joints)
  moveit::core::RobotState goal_state(robot_model);
  goal_state.setJointGroupPositions(joint_model_group, goal_joints);



  // =====================================
  // 11. 创建规划约束
  // =====================================
  // 需要完成：
  //   - 调用 kinematic_constraints::constructGoalConstraints()
  //   - 传入 goal_state, joint_model_group, 0.01, 0.01
  //   - 保存为 moveit_msgs::msg::Constraints goal_constraints
  moveit_msgs::msg::Constraints goal_constraints =
      kinematic_constraints::constructGoalConstraints(goal_state, joint_model_group, 0.01, 0.01);



  // =====================================
  // 12. 执行路径规划
  // =====================================
  // 需要完成：
  //   - 创建 MotionPlanRequest req
  //   - 设置 req.group_name = "arm_group"
  //   - 添加约束：req.goal_constraints.push_back(goal_constraints)
  //   - 设置 req.allowed_planning_time = 5.0
  //   - 创建 MotionPlanResponse res
  //   - 调用 planner->getPlanningContext() 获取 context
  //   - 调用 context->solve(res)
  planning_interface::MotionPlanRequest req;
  req.group_name = "arm_group";
  req.goal_constraints.push_back(goal_constraints);
  req.allowed_planning_time = 5.0;

  planning_interface::MotionPlanResponse res;
  planning_interface::PlanningContextPtr context = planner->getPlanningContext(planning_scene, req);
  context->solve(res);
  if (res.error_code_.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
    RCLCPP_ERROR(LOGGER, "Planning failed! Error code: %d", res.error_code_.val);
    return 1;
  }


  // =====================================
  // 13. 获取规划结果
  // =====================================
  // 需要完成：
  //   - 创建 MotionPlanResponse response
  //   - 调用 res.getMessage(response)
  //   - 获取轨迹：auto& traj = response.trajectory.joint_trajectory
  moveit_msgs::msg::MotionPlanResponse response;
  res.getMessage(response);
  auto& traj = response.trajectory.joint_trajectory;



  // =====================================
  // 14. 导出轨迹到 CSV
  // =====================================
  // 需要完成：
  //   - 创建 std::ofstream csv(output_file)
  //   - 写入表头：time,joint5,joint6,...
  //   - 遍历 traj.points，写入时间和关节角度
  //   - 关闭文件
  std::ofstream csv(output_file);
  if (!csv.is_open()) {
    RCLCPP_ERROR(LOGGER, "Failed to open %s", output_file.c_str());
    return 1;
  }
  csv << "time";
  for (auto& joint : traj.joint_names) csv << "," << joint;
  csv << std::endl;
  for (auto& point : traj.points) {
    csv << point.time_from_start.sec + point.time_from_start.nanosec * 1e-9;
    for (auto val : point.positions) {
      csv << "," << val;
    }
    csv << std::endl;
  }
  RCLCPP_INFO(LOGGER, "Trajectory saved to %s (%zu points)", output_file.c_str(), traj.points.size());



  // =====================================
  // 15. 清理并退出
  // =====================================
  // 需要完成：
  //   - 调用 rclcpp::shutdown()
  //   - return 0
  rclcpp::shutdown();
  return 0;



}

// ========================================
// 参考答案提示（遇到困难时查看）
// ========================================

// 1. 初始化节点：
//    rclcpp::init(argc, argv);
//    auto node = rclcpp::Node::make_shared("plan_to_csv");

// 2. 读取参数：
//    node->declare_parameter("x", 0.03);
//    double x = node->get_parameter("x").as_double();
//    double roll = node->get_parameter("roll").as_double() * M_PI / 180.0;
//    std::string output_file = node->get_parameter("output_file").as_string();

// 3. 声明参数：
//    node->declare_parameter<std::string>("arm_group.kinematics_solver", "kdl_kinematics_plugin/KDLKinematicsPlugin");

// 4. 读取文件（复用 read_file 函数）：
//    std::string urdf_string = read_file("/home/yh/.../engineer.urdf");

// 5. 创建模型：
//    node->declare_parameter("robot_description", urdf_string);
//    robot_model_loader::RobotModelLoader::Options options(urdf_string, srdf_string);
//    robot_model_loader::RobotModelLoader robot_model_loader(node, options);
//    moveit::core::RobotModelPtr robot_model = robot_model_loader.getModel();

// 6. 获取规划组：
//    const moveit::core::JointModelGroup* joint_model_group = robot_model->getJointModelGroup("arm_group");

// 7. 创建规划场景：
//    planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));
//    moveit::core::RobotState& current_state = planning_scene->getCurrentStateNonConst();
//    current_state.setJointGroupPositions(joint_model_group, vector<double>(N, 0.0));
//    planning_scene->setCurrentState(current_state);

// 8. 加载规划器：
//    auto planner_loader = make_unique<ClassLoader<PlannerManager>>("moveit_core", "planning_interface::PlannerManager");
//    PlannerManagerPtr planner(planner_loader->createUnmanagedInstance("ompl_interface/OMPLPlanner"));
//    planner->initialize(robot_model, node, node->get_namespace());

// 9. IK求解：
//    geometry_msgs::msg::Pose target_pose;
//    target_pose.position.x = x;
//    tf2::Quaternion q;
//    q.setRPY(roll, pitch, yaw);
//    target_pose.orientation = tf2::toMsg(q);
//    const kinematics::KinematicsBaseConstPtr kin_solver = joint_model_group->getSolverInstance();
//    kin_solver->searchPositionIK(target_pose, seed, timeout, solution, error_code, options);

// 10. 碰撞检测：
//     collision_detection::CollisionRequest req;
//     collision_detection::CollisionResult res;
//     planning_scene->checkCollision(req, res, temp_state);

// 11. 规划约束：
//     moveit_msgs::msg::Constraints goal_constraints =
//         kinematic_constraints::constructGoalConstraints(goal_state, joint_model_group, 0.01, 0.01);

// 12. 执行规划：
//     planning_interface::MotionPlanRequest req;
//     req.group_name = "arm_group";
//     req.goal_constraints.push_back(goal_constraints);
//     req.allowed_planning_time = 5.0;
//     PlanningContextPtr context = planner->getPlanningContext(planning_scene, req);
//     context->solve(res);

// 13. 导出CSV：
//     std::ofstream csv(output_file);
//     csv << "time";
//     for (const auto& name : traj.joint_names) csv << "," << name;
//     csv << "\n";
//     for (const auto& point : traj.points) {
//       csv << pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9;
//       for (const auto& pos : point.positions) csv << "," << pos;
//       csv << "\n";
//     }

// 14. 清理退出：
//     rclcpp::shutdown();
//     return 0;

// ========================================
// 使用方法
// ========================================
// 编译：
//   cd /home/yh/2026_Engineer_ws
//   colcon build --packages-select engineer_moveit2
//
// 运行（以 roll=-131° 为例，该朝向与零位 FK 一致，IK 易收敛）：
// source install/setup.bash
// ros2 run engineer_moveit2 plan_to_csv --ros-args \
//   -p x:=0.28 -p y:=0.0 -p z:=0.35 \
//   -p roll:=-131.0 -p pitch:=0.0 -p yaw:=0.0 \
//   -p output_file:=/home/yh/trajectory.csv
//
