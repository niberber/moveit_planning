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

    // 三维世界坐标，8 个点，单位：毫米
static    std::vector<cv::Point3f> points3D;

   // 柱子的在装甲板系下的坐标
static    cv::Point3f m_point;

  //兑矿点需要推的向量表示
static    cv::Vec3d v;

};
