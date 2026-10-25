// ============================================================================
//  PnPSolver.hpp
//  从一帧图像解出【车体系下的目标点 + 目标向量】。
//
//  最小用法：
//      PnPDate   data;                              // 标定数据
//      CarTarget t = solveCarTarget(frame, data);   // 每帧调一次
//      t.point;   // ★ 目标点  (mm, 车体系)
//      t.vec;     // ★ 目标向量      (车体系)
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
