#include <opencv2/opencv.hpp>
#include "PnPDate.hpp"

cv::Mat PnPDate::cameraMatrix = (cv::Mat_<double>(3, 3) << 1202.3, 0.0, 705.8,
                                 0.0, 1202.8, 556.0,
                                 0.0, 0.0, 1.0);

std::vector<double> PnPDate::distCoeffs = {-0.0899, 0.0753, 0.0013, -0.000020506, 0.0};

std::vector<cv::Point3f> PnPDate::points3D = {{-106.0, 63.9, 0.0}, {-106.0, -63.9, 0.0}, 
                                              {-84.8, -70.3, 0.0}, {-84.8, 70.3, 0.0}, 
                                              {84.8, 70.3, 0.0}, {84.8, -70.3, 0.0}, 
                                              {106.0, -63.9, 0.0}, {106.0, 63.9, 0}};

cv::Point3f PnPDate::m_point = {0.0,-102.3,126.2};

cv::Vec3d PnPDate::v =cv::Vec3d(0.0,100.0,0.0);


