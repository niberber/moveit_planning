#pragma once
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

// 存放 PnP 需要的输入数据：相机内参、畸变系数、三维世界坐标。
// 数据来源：MATLAB cameraCalibrator 得到的 cameraParams 对象。
class PnPDate
{
public:

    // 3x3 相机内参（OpenCV 布局：[fx 0 cx; 0 fy cy; 0 0 1]）
static    cv::Mat cameraMatrix;

    // 畸变系数 1x5：[k1 k2 p1 p2 k3]
static   std::vector<double> distCoeffs;

    // 8 个角点的三维坐标（装甲板系，mm），顺序：左条 TL/BL/BR/TR + 右条 TL/BL/BR/TR，
    // 与 detectCorners() 输出的 points2D 一一对应
static    std::vector<cv::Point3f> points3D;

    // 目标点（柱子/插入点）在【装甲板系】下的坐标，mm：
    //   point_car = R_car_from_armor * m_point + t_armor_in_car
static    cv::Point3f m_point;

static    cv::Vec3d v1;   // 目标向量 1（装甲板系，mm，纯方向只旋转）

    // 目标向量 2 在【装甲板系】下的表示，mm（纯方向，只旋转不平移）：
    //   vec2_car = R_car_from_armor * v2
static    cv::Vec3d v2;

};
