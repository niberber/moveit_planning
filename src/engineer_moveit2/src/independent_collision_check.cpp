// ========================================
// 独立碰撞检测 — 填空学习模板
// 完全不依赖 MoveIt，只用 FCL + Eigen
// ========================================

// ===== 第0步：包含头文件 =====
// 需要完成：
//   - rclcpp（ROS 日志）
//   - fcl/fcl.h（FCL 碰撞库总入口）
//   - Eigen/Dense（矩阵运算）
//   - vector, string, map, memory（容器）
#include <rclcpp/rclcpp.hpp>
#include <fcl/fcl.h>          // TODO: FCL 的头文件
#include <Eigen/Dense>        // TODO: Eigen 的头文件
#include <vector>
#include <string>
#include <map>
#include <memory>
#include <limits>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("indep_col_check");


// ============================================================
// 第一步：定义碰撞检测报告结构
// ============================================================
// 你需要在碰撞检测后拿到这些结果。想一下，一个完整的报告
// 应该包含哪些信息？
//
// 提示：
//   - 有没有自碰撞？  → bool self_collision
//   - 有没有撞障碍物？→ bool external_collision
//   - 哪些连杆撞了？ → vector<string> colliding_pairs  （格式："link1 <-> link2"）
//   - 总共几个接触点？→ size_t contact_count
struct CollisionReport {
    // TODO: 在这里定义上面的 4 个成员变量
    bool self_collision = false;
    bool external_collision = false;
    std::vector<std::string> colliding_pairs;
    size_t contact_count = 0;
};


// ============================================================
// 第二步：定义碰撞检测器类骨架
// ============================================================
// 这个类是核心，负责所有碰撞检测逻辑。

class IndependentCollisionChecker {
public:
    // ===== 数据成员：存储机器人连杆和世界障碍物的 FCL 对象 =====

    // 每个 robot link 就是一个 CollisionObject<double>
    // key = link名称("link1")，value = FCL 对象指针
    std::map<std::string, fcl::CollisionObject<double>*> robot_;
    std::map<std::string, fcl::CollisionObject<double>*> world_;

    // BroadPhase 管理器 —— AABB 树，用于加速 N^2 对碰撞检测
    std::unique_ptr<fcl::DynamicAABBTreeCollisionManager<double>> manager_;

    // ===== 构造函数 =====
    IndependentCollisionChecker() {
        manager_ = std::make_unique<fcl::DynamicAABBTreeCollisionManager<double>>();
    }

    // ===== 析构函数 =====
    ~IndependentCollisionChecker() = default;

    // ============================================================
    // 第三步：添加机器人连杆
    // ============================================================
    // 每个 robot link 用一个简单的几何体近似（BOX / CYLINDER / SPHERE）。
    // 流程：创建几何体 → 创建 CollisionObject → setUserData → 存入 map
    void addRobotLink(const std::string& name, double x_size, double y_size, double z_size) {
        auto box = std::make_unique<fcl::Box<double> >(x_size, y_size, z_size);
        auto obj = std::make_unique<fcl::CollisionObject<double> >(std::shared_ptr<fcl::CollisionGeometry<double> >(box.release()));
        obj->setUserData(const_cast<char*>(name.c_str()));
        robot_[name] = obj.release();
    }

    void addRobotLink(const std::string& name, double radius, double length) {
        auto cyl = std::make_unique<fcl::Cylinder<double> >(radius, length);
        auto obj = std::make_unique<fcl::CollisionObject<double> > (std::shared_ptr<fcl::CollisionGeometry<double> >(cyl.release()));
        obj->setUserData(const_cast<char*>(name.c_str()));
        robot_[name] = obj.release();
    }

    void addRobotLink(const std::string& name, double radius) {
        auto sphere = std::make_unique<fcl::Sphere<double> >(radius);
        auto obj = std::make_unique<fcl::CollisionObject<double> >(std::shared_ptr<fcl::CollisionGeometry<double> >(sphere.release()));
        obj->setUserData(const_cast<char*>(name.c_str()));
        robot_[name] = obj.release();
    }


    // ============================================================
    // 第四步：添加世界障碍物
    // ============================================================
    // 和 robot link 类似，但额外指定在世界坐标系中的位置（tx, ty, tz）

    void addWorldObject(const std::string& name, double tx, double ty, double tz,
                        double sx, double sy, double sz) {
        auto box = std::make_unique<fcl::Box<double> >(sx, sy, sz);
        auto obj = std::make_unique<fcl::CollisionObject<double> >(std::shared_ptr<fcl::CollisionGeometry<double> >(box.release()),
            Eigen::Matrix3d::Identity(), Eigen::Vector3d(tx, ty, tz));
        obj->setUserData(const_cast<char*>(name.c_str()));
        world_[name] = obj.release();
    }


    // ============================================================
    // 第五步：正运动学(FK) — 关节角度 → 连杆位姿
    // ============================================================
    // 这是碰撞检测前最关键的一步！
    // 我们需要根据当前关节角度计算每个连杆在 base frame 下的变换矩阵。

    // 输入：连杆索引(i)，关节角度数组(joints)
    // 输出：该连杆相对于 base 的 Eigen::Isometry3d 变换
    Eigen::Isometry3d computeLinkTransform(int i, const std::vector<double>& joints) {
        Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();

        // 每个关节的旋转+平移（相对于前一个关节）
        auto seg = [&](int idx) -> Eigen::Isometry3d {
            switch (idx) {
                case 0: return Eigen::Isometry3d::Identity() * Eigen::AngleAxisd(joints[0], Eigen::Vector3d::UnitZ());
                case 1: return Eigen::AngleAxisd(joints[1], Eigen::Vector3d::UnitY())
                            * Eigen::Translation<double, 3>(0, 0, 0.35);
                case 2: return Eigen::AngleAxisd(joints[2], Eigen::Vector3d::UnitY())
                            * Eigen::Translation<double, 3>(0, 0, 0.30);
                case 3: return Eigen::AngleAxisd(joints[3], Eigen::Vector3d::UnitY())
                            * Eigen::Translation<double, 3>(0, 0, 0.28);
                case 4: return Eigen::Isometry3d::Identity() * Eigen::AngleAxisd(joints[4], Eigen::Vector3d::UnitZ());
                case 5: return Eigen::AngleAxisd(joints[5], Eigen::Vector3d::UnitY())
                            * Eigen::Translation<double, 3>(0, 0, 0.09);
                case 6: return Eigen::Isometry3d::Identity() * Eigen::AngleAxisd(joints[6], Eigen::Vector3d::UnitZ())
                            * Eigen::Translation<double, 3>(0, 0, 0.08);
            }
            return Eigen::Isometry3d::Identity();
        };

        // 链式乘：T_base→link_i = seg(0) × seg(1) × ... × seg(i)
        for (int k = 0; k <= i; ++k) {
            pose = pose * seg(k);
        }

        return pose;
        
    }


    // ============================================================
    // 第六步：更新所有机器人的 FCL transform
    // ============================================================
    // 把 FK 的结果同步到 FCL 对象的内部变换中

    void updateRobotTransforms(const std::vector<double>& joints) {
        // TODO: 遍历 robot_，用 computeLinkTransform 计算每个 link 的位姿
        //       然后把 Eigen::Isometry3d 转成 fcl::Quaternion + fcl::Vector3d
        //       调用 obj->setTransform(quat, trans) 更新 FCL 对象的变换
        int i = 0;
        for(auto& [name, obj] : robot_) {
            Eigen::Isometry3d pose = computeLinkTransform(i, joints);
            Eigen::Quaterniond q(pose.linear());
            double tx = pose.translation().x();
            double ty = pose.translation().y();
            double tz = pose.translation().z();
            obj->setTransform(fcl::Quaternion<double>(q.w(), q.x(), q.y(), q.z()),
                              fcl::Vector3d(tx, ty, tz));
            ++i;
        }
        // 提示：
        //   Eigen::Quaterniond q(pose.linear());
        //   obj->setTransform(
        //       fcl::Quaternion<double>(q.w(), q.x(), q.y(), q.z()),
        //       fcl::Vector3d(pose.translation().x(),
        //                     pose.translation().y(),
        //                     pose.translation().z()));
        
    }


    // ============================================================
    // 第七步：自碰撞检测（self collision）
    // ============================================================
    // 检查机器人各连杆之间是否互相碰撞。

    CollisionReport checkSelfCollision(const std::vector<double>& joints) {
        CollisionReport report;

        // --- 第1小步：更新所有 link 的 transform ---
        // TODO: 调用 updateRobotTransforms(joints)
        updateRobotTransforms(joints);
        // --- 第2小步：定义碰撞请求和结果 ---
        // TODO:
        //   fcl::CollisionRequest<double> col_req(100, true);
        //   fcl::CollisionResult<double> col_res;
        //   // 100 = 最多返回100个接触点，true = 启用接触点信息
        fcl::CollisionRequest<double> col_req(100, true);
        fcl::CollisionResult<double> col_res;
        // --- 第3小步：两两遍历 robot link，检测碰撞 ---
        // TODO: 双重循环遍历 robot_，k1 < k2 时用 fcl::collide(v1, v2, req, res)
        //       如果 col_res.isCollision()，填充 report 并累计 contact_count
        for(auto& [k1, v1] : robot_){
            for(auto& [k2, v2] : robot_){
                if(k1 >= k2)continue;
                col_res.clear();
                fcl::collide(v1, v2, col_req, col_res);
                if(col_res.isCollision()){
                    report.self_collision = true;
                    report.contact_count += col_res.numContacts();
                    report.colliding_pairs.push_back(k1 + "<->" + k2);
                }
            }
        }
        // 提示：
        //   for (auto& [k1, v1] : robot_)
        //     for (auto& [k2, v2] : robot_) {
        //       if (k1 >= k2) continue;  // 跳过自己和重复对
        //       col_res.clear();
        //       fcl::collide(v1, v2, col_req, col_res);
        //       if (col_res.isCollision()) { ... }
        //     }

        return report;
    }


    // ============================================================
    // 第八步：外部碰撞检测（撞障碍物）
    // ============================================================

    CollisionReport checkExternalCollision() {
        CollisionReport report;

        // TODO: 双重循环 robot_ × world_，用 fcl::collide 两两检测
        //       逻辑和 checkSelfCollision 类似，只是遍历的是不同容器
        fcl::CollisionRequest<double> col_req(100, true);
        fcl::CollisionResult<double> col_res;
        for(auto& [k1, v1] : robot_){
            for(auto& [k2, v2] : world_){
                col_res.clear();
                fcl::collide(v1, v2, col_req, col_res);
                if(col_res.isCollision()){
                    report.external_collision = true;
                    report.contact_count += col_res.numContacts();
                    report.colliding_pairs.push_back(k1 + "<->" + k2);
                }
            }
        }
        return report;
    }


    // ============================================================
    // 第九步：完整碰撞检测
    // ============================================================

    CollisionReport checkAll(const std::vector<double>& joints) {
        // TODO: 分别调用 checkSelfCollision 和 checkExternalCollision
        //       合并两个 CollisionReport 的结果并返回
        CollisionReport report, self_report, external_report;
        self_report = checkSelfCollision(joints);
        external_report = checkExternalCollision();
        report = self_report;
        report.colliding_pairs.insert(report.colliding_pairs.end(), external_report.colliding_pairs.begin(), external_report.colliding_pairs.end());
        
        report.contact_count += external_report.contact_count;
        return report;
    }


    // ============================================================
    // 第十步（可选）：最小距离查询
    // ============================================================

    double minimumDistance(const std::vector<double>& joints) {
        // TODO: 思路和 checkSelfCollision 类似，但用 fcl::distance() 替代 fcl::collide()
        //       遍历所有 pair，取最小距离返回
        //
        // 提示：
        //   fcl::DistanceRequest<double> dist_req(true);
        //   fcl::DistanceResult<double> dist_res;
        //   fcl::distance(v1, v2, dist_req, dist_res);
        //   // dist_res.min_distance 即为该对物体的最小距离
        updateRobotTransforms(joints);
        fcl::DistanceRequest<double> dist_req(true);
        fcl::DistanceResult<double> dist_res;
        double min_dist = std::numeric_limits<double>::max();
        for(auto& [k1, v1] : robot_){
            for(auto& [k2, v2] : robot_){
                if(k1 >= k2)continue;
                dist_res.clear();
                fcl::distance(v1, v2, dist_req, dist_res);
                min_dist = std::min(min_dist, dist_res.min_distance);
            }
        }
        for(auto& [k1, v1] : robot_){
            for(auto& [k2, v2] : world_){
                dist_res.clear();
                fcl::distance(v1, v2, dist_req, dist_res);
                min_dist = std::min(min_dist, dist_res.min_distance);
            }
        }
        return min_dist;
    }
};


// ============================================================
// main 函数 — 测试入口
// ============================================================

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("indep_col_checker");

    RCLCPP_INFO(LOGGER, "=============================================");
    RCLCPP_INFO(LOGGER, "  Independent Collision Checker (Learning Template)");
    RCLCPP_INFO(LOGGER, "  (No MoveIt dependencies)");
    RCLCPP_INFO(LOGGER, "=============================================\n");

    IndependentCollisionChecker checker;

    // TODO: 添加 7 个 robot link（根据你的机器人实际尺寸）
    checker.addRobotLink("link1", 0.06, 0.06, 0.12);
    checker.addRobotLink("link2", 0.08, 0.08, 0.24);
    checker.addRobotLink("link3", 0.06, 0.06, 0.22);
    checker.addRobotLink("link4", 0.05, 0.05, 0.08);
    checker.addRobotLink("link5", 0.04, 0.04, 0.04);
    checker.addRobotLink("link6", 0.04, 0.04, 0.06);
    checker.addRobotLink("link7", 0.03, 0.03, 0.04);
    // TODO: 添加世界障碍物
    //   checker.addWorldObject("table", 0.5, 0, -0.1, 0.8, 0.6, 0.02);
    checker.addWorldObject("table", 0.5, 0, -0.1, 0.8, 0.6, 0.02);
    // TODO: 定义关节角度并测试
    //   std::vector<double> zero(7, 0.0);
    //   CollisionReport rpt = checker.checkAll(zero);
    //   RCLCPP_INFO(LOGGER, "self=%s external=%s",
    //               rpt.self_collision ? "YES" : "NO",
    //               rpt.external_collision ? "YES" : "NO");
    std::vector<double> zero(7, 0.0);
    CollisionReport rpt = checker.checkAll(zero);
    RCLCPP_INFO(LOGGER, "self=%s external=%s",
                rpt.self_collision ? "YES" : "NO",
                rpt.external_collision ? "YES" : "NO");
    rclcpp::shutdown();
    return 0;
}
