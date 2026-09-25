#include <fstream>
#include <sstream>
#include <rclcpp/rclcpp.hpp>
#include <engineer_moveit2/srv/compute_ik.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/kinematic_constraints/utils.h>

using namespace std::placeholders;

static std::string read_file(const std::string& path){
    std::ifstream f(path);
    std::stringstream buf;
    buf << f.rdbuf();
    return buf.str();
}
class PathPlanningServer {
public:
    rclcpp::Node::SharedPtr node_;
    
    PathPlanningServer() : node_(std::make_shared<rclcpp::Node>("path_planning_server")) {
        //0. 指定逆运动学求解器
        node_->declare_parameter("arm_group.kinematics_solver", rclcpp::ParameterValue("kdl_kinematics_plugin/KDLKinematicsPlugin"));
        node_->declare_parameter("arm_group.kinematics_solver_search_resolution", rclcpp::ParameterValue(0.005));
        node_->declare_parameter("arm_group.kinematics_solver_timeout", rclcpp::ParameterValue(1.0));
        
        //1.加载机器人模型
        std::string urdf_string = read_file("/home/yh/2026_Engineer_ws/src/engineer/urdf/engineer.urdf");
        std::string srdf_string = read_file("/home/yh/2026_Engineer_ws/src/engineer_moveit2/config/engineer.srdf");
        node_->declare_parameter("robot_description", urdf_string);
        robot_model_loader::RobotModelLoader::Options options(urdf_string, srdf_string);
        robot_model_loader::RobotModelLoader robot_model_loader(node_, options);
        robot_model_ = robot_model_loader.getModel();

        //2.获取规划组
        joint_model_group_ = robot_model_->getJointModelGroup("arm_group");

        //3.创建规划场景
        planning_scene_ = std::make_shared<planning_scene::PlanningScene>(robot_model_);

        //4. 指定规划器
        node_->declare_parameter("planning_pipeline.planning_plugin", rclcpp::ParameterValue("ompl_interface/OMPLPlanner"));

        //5.创建规划管道
        planning_pipeline_ = std::make_shared<planning_pipeline::PlanningPipeline>(robot_model_, node_, "planning_pipeline");

        //5.创建服务
        service_ = node_->create_service<engineer_moveit2::srv::ComputeIK>(
            "compute_ik",
            std::bind(&PathPlanningServer::handle_request, this, _1, _2)
        );
        node_->declare_parameter("arm_joint_count", rclcpp::ParameterValue(0));
        int joint_count = joint_model_group_->getVariableCount();
        node_->set_parameter(rclcpp::Parameter("arm_joint_count", joint_count));
        RCLCPP_INFO(node_->get_logger(), "Published arm_joint_count: %d", joint_count);

        RCLCPP_INFO(node_->get_logger(), "Path planning server ready!");
    }
    rclcpp::Node::SharedPtr get_node() {return node_;}

private:
    rclcpp::Service<engineer_moveit2::srv::ComputeIK>::SharedPtr service_;
    
    moveit::core::RobotModelPtr robot_model_;
    const moveit::core::JointModelGroup* joint_model_group_;
    planning_scene::PlanningScenePtr planning_scene_;
    planning_pipeline::PlanningPipelinePtr planning_pipeline_;
    double evaluate_smoothness(const trajectory_msgs::msg::JointTrajectory& traj) {
        double smoothness = 0.0;
        int n = traj.points.size();
        if(n < 3) return 0.0;
        int joint_count = traj.points[0].positions.size();
        for(int j = 0; j < joint_count; j++){
            for(int i = 1; i < n - 1; i++){
                double acc_prev = traj.points[i].positions[j] - traj.points[i - 1].positions[j];
                double acc_next = traj.points[i + 1].positions[j] - traj.points[i].positions[j];
                smoothness += (acc_next - acc_prev) * (acc_next - acc_prev);
            }
            return smoothness;
        }
    }
    void handle_request(
        const std::shared_ptr<engineer_moveit2::srv::ComputeIK::Request> req,
        std::shared_ptr<engineer_moveit2::srv::ComputeIK::Response> res
    )   {
        // 0. 指定规划器
        const std::vector<std::string> planners = {
            "RRTConnectkConfigDefault",
            "RRTkConfigDefault",
            "PRMConfigDefault",
            "LBKPIECEkConfigDefault"
        };

        // 1. 创建规划请求
        planning_interface::MotionPlanRequest plan_req;
        plan_req.group_name = "arm_group";
        plan_req.allowed_planning_time = 5.0;

        // 2. 设置起始状态
        moveit::core::RobotState start_state(robot_model_);
        start_state.setJointGroupPositions(joint_model_group_, req->start_joint_angles);
        planning_scene_->setCurrentState(start_state);

        // 3.设置目标位置约束
        geometry_msgs::msg::PoseStamped target_pose;
        target_pose.header.frame_id = "base_link";

        target_pose.pose.position.x = req->x;
        target_pose.pose.position.y = req->y;
        target_pose.pose.position.z = req->z;

        tf2::Quaternion q;
        q.setRPY(req->roll * M_PI / 180.0, req->pitch * M_PI / 180.0, req->yaw * M_PI / 180.0);
        target_pose.pose.orientation.w = q.w();
        target_pose.pose.orientation.x = q.x();
        target_pose.pose.orientation.y = q.y();
        target_pose.pose.orientation.z = q.z();
        // 创建位姿约束
        moveit_msgs::msg::Constraints goal_constraints = 
            kinematic_constraints::constructGoalConstraints(
                "link11",
                target_pose,
                0.01,
                0.01
            );
        plan_req.goal_constraints.push_back(goal_constraints);

        // 4.调用规划器,选择最平滑的轨迹
        double best_smoothness = std::numeric_limits<double>::max();
        bool found = false;
        moveit_msgs::msg::RobotTrajectory best_trajectory;

        for (const auto& planner_id : planners) {
            plan_req.planner_id = planner_id;
            
            planning_interface::MotionPlanResponse plan_res;
            planning_pipeline_->generatePlan(planning_scene_, plan_req, plan_res);

            if (plan_res.error_code_.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
                moveit_msgs::msg::RobotTrajectory traj_msg;
                plan_res.trajectory_->getRobotTrajectoryMsg(traj_msg);
                
                double smoothness = evaluate_smoothness(traj_msg.joint_trajectory);
                RCLCPP_INFO(node_->get_logger(), "Planner %s: smoothness=%.4f, points=%zu",
                            planner_id.c_str(), smoothness, traj_msg.joint_trajectory.points.size());

                if (smoothness < best_smoothness) {
                    best_smoothness = smoothness;
                    best_trajectory = traj_msg;
                }
                found = true;
            }
        }

        if (found) {
            res->trajectory = best_trajectory;
            res->success = true;
            res->msg = "Path planned successfully (smoothness: " + std::to_string(best_smoothness) + ")";
        } else {
            res->success = false;
            res->msg = "All planners failed";
        }
     }
};

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    auto server = std::make_shared<PathPlanningServer>();
    rclcpp::spin(server->get_node());
    rclcpp::shutdown();
    return 0;
}
