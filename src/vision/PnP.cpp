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
// ============================================================================
//  现状（本文件只是最小调用示例，真正的算法都在 PnPSolver 库里）
//
//  1) 库侧接口（src/vision/PnPSolver）
//     - solveCarTargetPose(frame, data, opt) -> CarTargetPose
//           pose[0..2] = 目标点 (mm, 车系)      pose[3..5] = roll/pitch/yaw (rad, ZYX)
//           R          = R_car_from_target = R_car_from_armor * R_armor_from_target
//           vec        = 目标向量 (车系, mm)
//     - 单位全程 mm，由上位的 vision_locate() 在出口统一换成 m。
//     - 目标系定义：原点 = 目标点；X = -装甲板X，Y = -装甲板Z，Z = -装甲板Y
//       （R_armor_from_target = [[-1,0,0],[0,0,-1],[0,-1,0]]，常量在 CarTargetOptions 里）。
//     - 二值化阈值可运行时改：binaryDiffThreshold() / setBinaryDiffThreshold()，
//       mask = (R - B) > diff（默认 70）。
//
//  2) 上位机侧（engineer_moveit2/src/moveit_planning.cpp）
//     - 实时链路：相机线程 -> vision_on_frame() -> PnPTuner::add()（检测+解算+累计误差），
//       每 100 帧打印一次 points3D 调整建议；VISION_DEBUG_SHOW=1 时另有三个调试窗口
//       camera / camera_mask / tune（滑条调阈值）。
//     - 规划链路：vision_locate() 取最新帧同步解算一次，出口换算成
//       VisionTarget{p(m), rpy(rad), v(mm), err}。
//     - 步骤1 接近：goal.translation = p - INSERT_POINT_OFFSET_IN_BASE，
//       goal.linear = Rz(yaw) * Ry(pitch) * Rx(roll)（即视觉给的目标系姿态）。
//     - 步骤2/3 插入、平移：delta = v / 1000 加到当前 TCP 平移，走 cartesian_plan_()。
//
//  3) 构建
//     - src/vision 是独立 cmake 工程（可执行 PnP），不能被 ament 当包；
//       engineer_moveit2/CMakeLists.txt 用 add_subdirectory 引入 PnPDate / PnPSolver / task4/lib，
//       并 link PnPSolver + HKCamera。
//
//  4) 坐标系与单位约定（接入前必须对齐）
//     - 视觉输出的 car 系即 base_link（x 左 / y 前 / z 上）；
//       t_cam_in_car = (72.83, -492.4, 726.45) mm 为相机光心在车系下的坐标，
//       与 R_car_from_cam 一起写在 CarTargetOptions 的默认值里，标定变了要同步改这里。
//     - 视觉内部全程 mm，MoveIt/Eigen 全程 m；p 在 vision_locate() 出口除 1000，
//       v 在步骤2/3 使用处除 1000。
//
//  5) 线程与时序
//     vision_locate() 在规划线程里同步调用（取帧+解算约几十 ms），期间状态机不推进；
//     若要更快，可在串口线程收到 0x01 时先预取一帧缓存，规划时直接复用。
// ============================================================================