// ============================================================================
//  PnP.cpp —— 最小调用示例
//
//  每帧调一次 tuner.add()，拿到车体系下的目标点和目标向量（交给客户），
//  终端只打印本帧反投影误差；按 ESC 退出时自动打印三维坐标调整建议。
//  所有算法都在 PnPSolver 库里。
// ============================================================================

#include <opencv2/opencv.hpp>

#include <cstdio>

#include "PnPDate.hpp"
#include "PnPSolver.hpp"

static const char *kVideoPath = "../video/techcore.mp4";

int main()
{
    PnPDate data;

    cv::VideoCapture cap(kVideoPath);
    if (!cap.isOpened())
        return 1;

    cv::Vec3d targetPoint; // ★ 交给客户：目标点（车体系, mm）
    cv::Vec3d targetVec;   // ★ 交给客户：目标向量（车体系）

    PnPTuner tuner;

    cv::Mat frame;
    int idx = 0;
    while (cap.read(frame))
    {
        ++idx;

        const CarTarget t = tuner.add(frame, data);
        const CarTargetPose r = toCarTargetPose(t); // 6 自由度位姿 + 目标向量

        targetPoint = t.point;
        targetVec = t.vec;

        if (r.ok)
        {
            // r[0] = 六自由度 [x y z roll pitch yaw]
            // r[1] = 三自由度向量 [x y z]
            const cv::Vec6d pose = r[0];
            const cv::Vec3d vec = r[1];

            std::printf("[%4d] err %.2f px | 点(%.1f, %.1f, %.1f) mm"
                        " | rpy(%.1f, %.1f, %.1f) deg"
                        " | vec(%.3f, %.3f, %.3f)\n",
                        idx, r.errMeanPx,
                        pose[0], pose[1], pose[2],
                        r.rpyDeg()[0], r.rpyDeg()[1], r.rpyDeg()[2],
                        vec[0], vec[1], vec[2]);
        }
        else
        {
            std::printf("[%4d] 未解出：%s\n", idx, r.reason.c_str());
        }

        cv::imshow("PnP", frame);
        if (cv::waitKey(1) == 27)
            break;
    }

    std::printf("%s", tuner.advice(data).c_str());
    return 0;
}
// 现状
//   src/vision 是独立的 cmake 工程(可执行 PnP)，只读 ../video/techcore.mp4 并打印误差，
//   不能直接当库调用；本文件的 vision_locate() 目前是桩，步骤1 会直接回 0x08。
//
// 1) 视觉侧要补"姿态"（必须做，否则步骤1只能用当前末端姿态凑）
//    - src/vision/PnPSolver/lib/include/PnPSolver.hpp 的 struct CarTarget 增加：
//          cv::Matx33d R;   // 目标坐标系在车体系(base_link)下的旋转
//    - src/vision/PnPSolver/src/PnPSolver.cpp 的 poseImpl() 内部已经算出 R_armor_to_car，
//      在返回前直接赋值： out.R = R_armor_to_car;
//    - 角度/位置单位保持 mm，由上位的 vision_locate() 统一换算成 m。
//
// 2) 上位机侧新增薄封装（建议新文件 src/vision_target.cpp，约50行）
//    - 取帧源二选一，用文件顶部常量切换：cv::VideoCapture(VISION_CAM_INDEX) 或读视频；
//    - 连续取帧调 solveCarTarget(frame, data, opt)，只累计 errMeanPx < 2.0 的好帧；
//    - 取够 N=5 帧后平均 point 与 R（R 用四元数平均再转回矩阵），point/1000 得到 m，
//      填 VisionTarget{p, R, ok, err}；
//    - 超时(如500ms)无好帧则返回 ok=false，状态机会自动回 0x08。
//
// 3) 本文件的 VisionTarget 需要补姿态字段
//          Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
//    build_plan_() 的 STEP_APPROACH 分支把 goal.linear() 从"沿用当前姿态"改成 vt.R。
//
// 4) CMakeLists.txt 增加（vision 不是 ament 包，只能用 add_subdirectory）
//          find_package(OpenCV REQUIRED)
//          add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../vision/PnPDate   ${CMAKE_CURRENT_BINARY_DIR}/PnPDate)
//          add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../vision/PnPSolver ${CMAKE_CURRENT_BINARY_DIR}/PnPSolver)
//          target_link_libraries(moveit_planning PnPSolver)
//    注意 vision/CMakeLists.txt 里自己的 add_executable(PnP) 与本包互不影响；
//    但 `colcon build --packages-select PnP` 仍会因"没有 install 目标"告警，属既有问题。
//
// 5) 坐标系与单位约定（接入前必须对齐）
//    - 视觉输出的 car 系即 base_link；t_cam_in_car=(492.4,726.45,72.83)mm 为相机装配值；
//    - 视觉内部全程 mm，MoveIt/Eigen 全程 m，只在 vision_locate() 出口换算一次；
//    - 步骤2 插入方向优先用视觉给的 vec（换到 base_link 后取反向），当前用 STEP_INSERT_DIR 代替；
//      步骤3/4/5 同理，P/Q 轴待标定或视觉给出，对应的常量在文件顶部。
//
// 6) 线程与时序
//    vision_locate() 在规划线程里同步调用（取帧+解算约几十ms），期间状态机不推进；
//    若要更快，可在串口线程收到 0x01 时先预取一帧缓存，规划时直接复用。
// ============================================================================