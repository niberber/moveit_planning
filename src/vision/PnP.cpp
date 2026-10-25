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

        targetPoint = t.point;
        targetVec = t.vec;
        std::printf("[%4d] 反投影误差 %.2f px\n", idx, t.errMeanPx);

        cv::imshow("PnP", frame);
        if (cv::waitKey(1) == 27)
            break;
    }

    std::printf("%s", tuner.advice(data).c_str());
    return 0;
}
