// ============================================================================
//  PnPSolver.hpp
//  从一帧图像解出【车体系下的目标位姿 + 目标向量】。
//
//  ★ 主用法（一个函数拿到全部结果，只用两个索引取值）：
//      PnPDate       data;                                  // 标定数据
//      CarTargetPose r = solveCarTargetPose(frame, data);   // 每帧调一次
//      if (!r.ok) { /* r.reason 里有原因 */ }
//
//      r[0]        // ★ 六自由度 [x, y, z, roll, pitch, yaw]
//      r[1]        // ★ 三自由度向量 [x, y, z]
//
//      r[0][0..2]  // 目标点在车体系下的坐标 (mm)
//      r[0][3..5]  // 姿态 roll / pitch / yaw (rad)
//      r[1][0..2]  // 目标向量在车体系下的表示
//
//      cv::Vec6d p6 = r[0];     // 也可以直接转成定长向量
//      cv::Vec3d p3 = r[1];
//      r.rpyDeg();              // 姿态换成角度 (deg)
//
//  目标系（"目标坐标系"）定义：
//      原点 = 目标点（即 CarTarget::point，不是装甲板原点）
//      Z 轴 = 垂直装甲板平面（与装甲板系 Z 同向）
//      X 轴 = 与装甲板系 X 同向
//      Y 轴 = 与装甲板系 Y 反向
//    数学上：R_target_in_car = R_armor_in_car * diag(1, -1, 1)
//
//  旧接口（只关心目标点和向量时用）：
//      CarTarget t = solveCarTarget(frame, data);
//      t.point;   // 目标点  (mm, 车体系)
//      t.vec;     // 目标向量      (车体系)
// ============================================================================
#pragma once

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

#include "PnPDate.hpp"

// ----------------------------------------------------------------------------
//  解算结果
// ----------------------------------------------------------------------------
struct CarTarget
{
    cv::Vec3d point = cv::Vec3d(0, 0, 0); // ★ 目标点   (mm, 车体系)
    cv::Vec3d vec = cv::Vec3d(0, 0, 0);   // ★ 目标向量     (车体系)

    // 装甲板系 -> 车体系 的旋转矩阵（列 = 装甲板三轴在车体系下的方向）
    cv::Matx33d R_armor_to_car = cv::Matx33d::eye();

    double errMeanPx = -1.0; // 本帧反投影误差 (px)，未解出时为 -1
    bool ok = false;         // 是否解算成功
    std::string reason;      // ok == false 的原因
};

// ----------------------------------------------------------------------------
//  可选参数，用默认值即可
// ----------------------------------------------------------------------------
struct CarTargetOptions
{
    // 相机系 -> 车体系：车体系三轴与相机系全反向 => -I
    cv::Matx33d R_car_from_cam = cv::Matx33d(-1, 0, 0,
                                              0, -1, 0,
                                              0, 0, -1);
    // 车体原点 -> 相机光心 (mm)
    cv::Vec3d t_cam_in_car = cv::Vec3d(492.4, 726.45, 72.83);

    bool refineWithLM = false; // IPPE 之后再做一步 LM 精修
};

// ----------------------------------------------------------------------------
//  ★ 一帧的完整结果：6 自由度位姿 + 目标向量
//
//  为什么不用 std::vector<double> 装？
//    1) 9 个数的含义只能靠下标记，容易错位；
//    2) 长度可变，编译器帮不上忙；
//    3) 位姿本身是"3 平移 + 3 姿态"，向量是"3 分量"，分组信息会丢。
//  为什么不用 std::pair<cv::Vec6d, cv::Vec3d>？
//    能存，但 .first / .second 读起来不知道是什么，也不能用 r[0] / r[1]。
//  => 用结构体 + 一个"项"视图：r[0] 是六自由度，r[1] 是三自由度向量，
//     这个项既能再下标（r[0][3]），也能隐式转成 cv::Vec6d / cv::Vec3d。
// ----------------------------------------------------------------------------

// r[0] / r[1] 取出来的东西（一个 6 维或 3 维的小数组）
struct CarTargetItem
{
    cv::Vec6d pose = cv::Vec6d(0, 0, 0, 0, 0, 0); // 仅当 n == 6 时有效
    cv::Vec3d vec = cv::Vec3d(0, 0, 0);           // 仅当 n == 3 时有效
    int n = 6;                                    // 6 = 位姿；3 = 向量

    int size() const { return n; }

    // 内层索引：0 .. n-1，越界触发 CV_Assert
    double operator[](int k) const;

    // 直接转成定长向量：cv::Vec6d p = r[0];  cv::Vec3d v = r[1];
    operator cv::Vec6d() const;
    operator cv::Vec3d() const;
};

struct CarTargetPose
{
    // pose[0..2] = 目标点 (x, y, z)，mm，车体系
    // pose[3..5] = 姿态 (roll, pitch, yaw)，rad，ZYX 顺序：
    //              R = Rz(yaw) * Ry(pitch) * Rx(roll)
    cv::Vec6d pose = cv::Vec6d(0, 0, 0, 0, 0, 0);

    cv::Vec3d vec = cv::Vec3d(0, 0, 0); // ★ 目标向量（车体系）

    // 目标系 -> 车体系 的旋转矩阵，与 pose[3..5] 一定是同一个姿态
    cv::Matx33d R = cv::Matx33d::eye();

    double errMeanPx = -1.0; // 本帧反投影误差 (px)，未解出时为 -1
    bool ok = false;         // 是否解算成功
    std::string reason;      // ok == false 的原因

    // ★ 两级索引（只有两个索引号）：
    //     r[0]  -> 六自由度  [x, y, z, roll, pitch, yaw]
    //     r[1]  -> 三自由度向量 [x, y, z]
    //   再往下一层：
    //     r[0][0]  -> x (mm)       r[1][0] -> vec.x
    //     r[0][3]  -> roll (rad)   r[1][2] -> vec.z
    //   也可以直接赋给 cv::Vec6d / cv::Vec3d：
    //     cv::Vec6d p = r[0];      cv::Vec3d v = r[1];
    //   索引不是 0/1，或内层越界，都会触发 CV_Assert
    CarTargetItem operator[](int i) const;

    cv::Vec3d rpyDeg() const; // 姿态换成角度，方便打印
};

// ----------------------------------------------------------------------------
//  ★ 主函数：一帧图像 -> 6 自由度位姿 + 目标向量
// ----------------------------------------------------------------------------
CarTargetPose solveCarTargetPose(const cv::Mat &bgrFrame, const PnPDate &data,
                                 const CarTargetOptions &opt = CarTargetOptions());

// 由旧的 CarTarget 结果转换（内部就是"补一个 R 再转欧拉角"，不重新解算）
CarTargetPose toCarTargetPose(const CarTarget &t);

// ----------------------------------------------------------------------------
//  ★ 主函数：一帧图像 -> 车体系下的目标点 + 目标向量
// ----------------------------------------------------------------------------
CarTarget solveCarTarget(const cv::Mat &bgrFrame, const PnPDate &data,
                         const CarTargetOptions &opt = CarTargetOptions());

// ----------------------------------------------------------------------------
//  分步接口（一般用不到）
// ----------------------------------------------------------------------------

// 图像 -> 8 个角点。顺序：左条 左上→左下→右下→右上，右条同样（屏幕逆时针）
bool detectCorners(const cv::Mat &bgrFrame, std::vector<cv::Point2f> &points2D);

// 8 个角点 -> 目标点 + 目标向量
CarTarget solveCarTargetFromCorners(const std::vector<cv::Point2f> &points2D,
                                    const PnPDate &data,
                                    const CarTargetOptions &opt = CarTargetOptions());

// ----------------------------------------------------------------------------
//  调试工具：累计多帧误差，退出时给出 points3D 调整建议
//
//      PnPTuner tuner;
//      while (cap.read(frame)) {
//          const CarTarget t = tuner.add(frame, data);   // 等价 solveCarTarget
//          printf("%.2f px\n", t.errMeanPx);
//          if (cv::waitKey(1) == 27) break;
//      }
//      printf("%s", tuner.advice(data).c_str());
// ----------------------------------------------------------------------------
class PnPTuner
{
public:
    CarTarget add(const cv::Mat &bgrFrame, const PnPDate &data); // 每帧一次
    std::string advice(const PnPDate &data) const;               // 可直接 printf 的多行文本

    void reset();
    int frames() const { return frames_; }
    double avgErrMeanPx() const { return frames_ ? errSum_ / frames_ : 0.0; }

private:
    int frames_ = 0;
    double errSum_ = 0.0;
    cv::Vec3d dSum_[8]{}; // 每个 3D 点的累计调整量（装甲板系, mm）
};
