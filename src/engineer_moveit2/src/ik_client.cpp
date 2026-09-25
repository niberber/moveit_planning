#include <rclcpp/rclcpp.hpp>
#include <engineer_moveit2/srv/compute_ik.hpp>
#include <chrono>
#include <thread>
class PathPlanningClient{
public:
    PathPlanningClient() : node_(std::make_shared<rclcpp::Node>("path_planning_client")) {
        client_ = node_->create_client<engineer_moveit2::srv::ComputeIK>("compute_ik");
        joint_count_ = get_joint_count_();
    }
    int get_joint_count()const{return joint_count_;}
    bool send_request(
        const std::vector<double>& start_joint_angles,
        double x, double y, double z,
        double roll, double pitch, double yaw
    ) {
        // 0. 等待服务端启动
        if(!client_->wait_for_service(std::chrono::seconds(5))) {
            RCLCPP_ERROR(node_->get_logger(), "Service not available!");
            return false;
        }
        // 1. 创建请求
        auto request = std::make_shared<engineer_moveit2::srv::ComputeIK::Request>();
        request->start_joint_angles = start_joint_angles;
        request->x = x;
        request->y = y;
        request->z = z;
        request->roll = roll;
        request->pitch = pitch;
        request->yaw = yaw;
        
        // 2. 发送请求
        auto future = client_->async_send_request(request);
        if(rclcpp::spin_until_future_complete(node_, future) != rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(node_->get_logger(), "Service call failed!");
            return false;
        }

        // 3. 处理响应
        auto response = future.get();
        if(response->success){
            RCLCPP_INFO(node_->get_logger(), "Path planned successfully!");
            RCLCPP_INFO(node_->get_logger(), "WayPoints: %zu", response->trajectory.joint_trajectory.points.size());
            return true;
        } else {
            RCLCPP_ERROR(node_->get_logger(), "Failed: %s", response->msg.c_str());
            return false;
        }
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Client<engineer_moveit2::srv::ComputeIK>::SharedPtr client_;
    int joint_count_;
    
    int get_joint_count_(){
        int wait_count = 0;
        while(!node_->has_parameter("arm_joint_count") && wait_count < 20) {
            rclcpp::spin_some(node_);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            wait_count++;
        }
        int count = 7;
        node_->get_parameter("arm_joint_count", count);
        return count;
    }
};

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    
    // 0. 创建客户端
    PathPlanningClient client;

    // 1.设置起点关节角度
    std::vector<double> start_joint_angles(client.get_joint_count(), 0.0);

    //2.发送请求
    bool success = client.send_request(
        start_joint_angles,
        0.28, 0.0, 0.35,
        -131.0, 0.0, 0.0
    );
    rclcpp::shutdown();
    return success ? 0 : 1;
}
