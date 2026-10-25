#include "PnPSolver.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>

namespace
{

// ============================================================================
//  固定的算法参数（要调就改这里）
// ============================================================================
const cv::Scalar kHsvLow(0, 174, 49); // 二值化下限
const cv::Scalar kHsvHigh(43, 255, 255);
const int kMorphKernel = 5; // 闭运算核（MORPH_ELLIPSE）
const int kBlurKernel = 5;  // 高斯核
const double kCannyLow = 50.0;
const double kCannyHigh = 150.0;
const int kCannyAperture = 3;

const int kMinContourPoints = 40;      // 轮廓点数太少直接跳过
const float kMinGradientMag = 30.f;    // 梯度幅值低于此值，方向不可信
const float kEdgeAngleTolDeg = 3.f;    // 与主方向的夹角容差
const int kHistHalfWindow = 2;         // 方向直方图 ±N 分箱平滑
const int kMinBestScore = 20;          // 主方向最少票数
const int kMinLongPoints = 40;         // 长边至少这么多点
const float kMinLineGapPx = 6.f;       // 两条长边最小间距
const int kMinPointsPerSide = 20;      // 每条长边最少点数
const float kMaxParallelCross = 0.20f; // 两条长边不平行度上限（≈12°）

const double kMaxCornerDistPx = 30.0;     // 交点离轮廓超过此值视为跑飞
const double kMinPointSeparationPx = 3.0; // 去重距离
const double kMinShortEdgeDevDeg = 40.0;  // 短边与长边方向至少要差这么多度

const double kMinBarSpacingPx = 20.0; // 两光条质心最小水平间距

// 8 个角点的名字，顺序与 points3D 一致
const char *const kCornerName[8] = {"L-TL", "L-BL", "L-BR", "L-TR",
                                    "R-TL", "R-BL", "R-BR", "R-TR"};

// ============================================================================
//  小工具
// ============================================================================

// 二维直线 a*x + b*y + c = 0，(a, b) 为单位法向量
struct Line2D
{
    double a, b, c;
};

// cv::fitLine 输出 (vx, vy, x0, y0) -> (a, b, c)
Line2D makeLine(const cv::Vec4f &fit)
{
    double vx = fit[0], vy = fit[1], x0 = fit[2], y0 = fit[3];
    const double nrm = std::sqrt(vx * vx + vy * vy);
    vx /= nrm;
    vy /= nrm;
    Line2D L{-vy, vx, 0.0}; // 法向量 = (vx, vy) 逆时针转 90°
    L.c = -(L.a * x0 + L.b * y0);
    return L;
}

// 两条直线求交，平行时返回 false
bool lineIntersect(const Line2D &l1, const Line2D &l2, cv::Point2f &p)
{
    const double det = l1.a * l2.b - l2.a * l1.b;
    if (std::abs(det) < 1e-9)
        return false;
    p.x = static_cast<float>((l1.b * l2.c - l2.b * l1.c) / det);
    p.y = static_cast<float>((l2.a * l1.c - l1.a * l2.c) / det);
    return true;
}

// 4 个角点规范成：0=左上，1/2/3 按屏幕逆时针。
// 只由点位决定，与轮廓起点、主方向等无关，所以顺序不会跳变。
void orderCornersCCW(std::vector<cv::Point2f> &pts)
{
    if (pts.size() < 3)
        return;

    cv::Point2f c(0.f, 0.f);
    for (const auto &p : pts)
        c += p;
    c *= 1.f / static_cast<float>(pts.size());

    // 图像 y 轴朝下 → atan2 降序 = 屏幕逆时针
    std::sort(pts.begin(), pts.end(),
              [&](const cv::Point2f &a, const cv::Point2f &b)
              {
                  return std::atan2(a.y - c.y, a.x - c.x) >
                         std::atan2(b.y - c.y, b.x - c.x);
              });

    // 把"左上角"（x+y 最小，并列时取更靠左的）转到 0 号位
    int k = 0;
    float best = 1e18f;
    for (int i = 0; i < static_cast<int>(pts.size()); ++i)
    {
        const float s = pts[i].x + pts[i].y;
        if (s < best - 1e-4f ||
            (std::abs(s - best) <= 1e-4f && pts[i].x < pts[k].x))
        {
            best = s;
            k = i;
        }
    }
    std::rotate(pts.begin(), pts.begin() + k, pts.end());
}

// ============================================================================
//  预处理：二值化 -> 闭运算 -> 高斯（同时留作梯度图）-> Canny
// ============================================================================
void preprocess(const cv::Mat &bgrFrame, cv::Mat &mask, cv::Mat &maskBlur, cv::Mat &edges)
{
    cv::Mat hsv;
    cv::cvtColor(bgrFrame, hsv, cv::COLOR_BGR2HSV);
    cv::inRange(hsv, kHsvLow, kHsvHigh, mask);

    const cv::Mat kernel = cv::getStructuringElement(
        cv::MORPH_ELLIPSE, cv::Size(kMorphKernel, kMorphKernel));
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);

    cv::GaussianBlur(mask, maskBlur, cv::Size(kBlurKernel, kBlurKernel), 0);
    cv::Canny(maskBlur, edges, kCannyLow, kCannyHigh, kCannyAperture, true);
}

// ============================================================================
//  单根光条：轮廓 -> 4 个规范化角点
// ============================================================================
bool extractBarCorners(const std::vector<cv::Point> &contour,
                       const cv::Mat &mag, const cv::Mat &angle,
                       std::array<cv::Point2f, 4> &outCorners, double &cx)
{
    const int n = static_cast<int>(contour.size());
    if (n < kMinContourPoints)
        return false;

    // ---- 1) 每个轮廓点的边缘方向 + 方向直方图 ----
    std::vector<float> ea(n, -1.f); // -1 表示梯度太弱，方向不可信
    std::array<int, 180> votes{};
    for (int i = 0; i < n; ++i)
    {
        const cv::Point &p = contour[i];
        if (mag.at<float>(p) < kMinGradientMag)
            continue;
        float a = std::fmod(angle.at<float>(p) + 90.f, 180.f);
        if (a < 0.f)
            a += 180.f;
        ea[i] = a;
        votes[static_cast<int>(a) % 180] += 1;
    }

    int bestBin = -1, bestScore = 0;
    for (int i = 0; i < 180; ++i)
    {
        int s = 0;
        for (int d = -kHistHalfWindow; d <= kHistHalfWindow; ++d)
            s += votes[(i + d + 180) % 180];
        if (s > bestScore)
        {
            bestScore = s;
            bestBin = i;
        }
    }
    if (bestBin < 0 || bestScore < kMinBestScore)
        return false;

    // ---- 2) 用 rho 把主方向的点分成两组（两条长边）----
    //  theta 取法向 (bestBin+90°)，投到切向得到的是弧长，两条边分不开
    const double theta0 = (bestBin + 90.0) * CV_PI / 180.0;
    const double ct = std::cos(theta0), st = std::sin(theta0);
    std::vector<std::pair<float, cv::Point>> onLong;
    for (int i = 0; i < n; ++i)
    {
        if (ea[i] < 0.f)
            continue;
        float d = std::abs(ea[i] - bestBin);
        d = std::min(d, 180.f - d);
        if (d > kEdgeAngleTolDeg)
            continue;
        onLong.emplace_back(static_cast<float>(contour[i].x * ct + contour[i].y * st),
                            contour[i]);
    }
    if (static_cast<int>(onLong.size()) < kMinLongPoints)
        return false;

    std::sort(onLong.begin(), onLong.end(),
              [](const std::pair<float, cv::Point> &A, const std::pair<float, cv::Point> &B)
              { return A.first < B.first; });

    // 2 类均值(k-means)在 rho 上分两组。
    // 不能取"最大间隙"：两条长边的圆角会把间隙填满
    const float rmin = onLong.front().first;
    const float rmax = onLong.back().first;
    float c1 = rmin + (rmax - rmin) * 0.25f;
    float c2 = rmin + (rmax - rmin) * 0.75f;
    for (int it = 0; it < 50; ++it)
    {
        double s1 = 0, s2 = 0;
        int cnt1 = 0, cnt2 = 0;
        const float midRho = (c1 + c2) * 0.5f;
        for (const auto &rp : onLong)
        {
            if (rp.first < midRho)
            {
                s1 += rp.first;
                ++cnt1;
            }
            else
            {
                s2 += rp.first;
                ++cnt2;
            }
        }
        if (cnt1 == 0 || cnt2 == 0)
            break;
        const float n1 = static_cast<float>(s1 / cnt1);
        const float n2 = static_cast<float>(s2 / cnt2);
        const bool done = std::abs(n1 - c1) < 1e-3f && std::abs(n2 - c2) < 1e-3f;
        c1 = n1;
        c2 = n2;
        if (done)
            break;
    }
    const float lineGap = std::abs(c2 - c1); // 两条长边之间的距离

    int split = 0;
    const float midRho = (c1 + c2) * 0.5f;
    while (split < static_cast<int>(onLong.size()) && onLong[split].first < midRho)
        ++split;

    if (lineGap < kMinLineGapPx || split < kMinPointsPerSide ||
        static_cast<int>(onLong.size()) - split < kMinPointsPerSide)
        return false;

    std::vector<cv::Point2f> sideA, sideB;
    for (int i = 0; i < split; ++i)
        sideA.emplace_back(onLong[i].second);
    for (int i = split; i < static_cast<int>(onLong.size()); ++i)
        sideB.emplace_back(onLong[i].second);

    // ---- 3) 拟合两条长边，取中线 ----
    cv::Vec4f fA, fB;
    cv::fitLine(sideA, fA, cv::DIST_L2, 0, 0.01, 0.01);
    cv::fitLine(sideB, fB, cv::DIST_L2, 0, 0.01, 0.01);
    Line2D LA = makeLine(fA), LB = makeLine(fB);

    if (std::abs(LA.a * LB.b - LB.a * LA.b) > kMaxParallelCross)
        return false; // 两条长边不平行

    if (LA.a * LB.a + LA.b * LB.b < 0) // 法向统一
    {
        LB.a = -LB.a;
        LB.b = -LB.b;
        LB.c = -LB.c;
    }

    Line2D Lmid{LA.a + LB.a, LA.b + LB.b, LA.c + LB.c};
    const double nn = std::hypot(Lmid.a, Lmid.b);
    if (nn < 1e-9)
        return false;
    Lmid.a /= nn;
    Lmid.b /= nn;
    Lmid.c /= nn;

    // ---- 4) 中线与轮廓求交（这两个点只是短边中点，用来找短边方向）----
    std::vector<cv::Point2f> ends;
    for (int i = 0; i < n; ++i)
    {
        const cv::Point2f p0 = contour[i];
        const cv::Point2f p1 = contour[(i + 1) % n];
        const double s0 = Lmid.a * p0.x + Lmid.b * p0.y + Lmid.c;
        const double s1 = Lmid.a * p1.x + Lmid.b * p1.y + Lmid.c;
        if (s0 * s1 > 0)
            continue;
        if (std::abs(s0 - s1) < 1e-9)
            continue;
        const double t = s0 / (s0 - s1);
        if (t < 0.0 || t > 1.0)
            continue;
        const cv::Point2f h = p0 + static_cast<float>(t) * (p1 - p0);
        bool dup = false; // 相邻两段会给出同一个交点
        for (const auto &e : ends)
            if (cv::norm(h - e) < kMinPointSeparationPx)
            {
                dup = true;
                break;
            }
        if (!dup)
            ends.push_back(h);
    }
    if (ends.size() < 2)
        return false;

    int i0 = 0, i1 = 1;
    double far = -1;
    for (int i = 0; i < static_cast<int>(ends.size()); ++i)
        for (int j = i + 1; j < static_cast<int>(ends.size()); ++j)
        {
            const double d = cv::norm(ends[i] - ends[j]);
            if (d > far)
            {
                far = d;
                i0 = i;
                i1 = j;
            }
        }

    // ---- 5) 沿轮廓两侧各走一小段，估出短边方向 ----
    auto sideAngle = [&](int near, int dir, double &ang) -> bool
    {
        double sx = 0, sy = 0;
        int cnt = 0;
        for (int k = 2; k <= 8; ++k)
        {
            const int idx = ((near + dir * k) % n + n) % n;
            if (ea[idx] < 0.f)
                continue;
            const double r = ea[idx] * CV_PI / 180.0;
            sx += std::cos(2 * r); // 2θ 平均，避开 0/180 回绕
            sy += std::sin(2 * r);
            ++cnt;
        }
        if (cnt < 3)
            return false;
        ang = 0.5 * std::atan2(sy, sx) * 180.0 / CV_PI;
        if (ang < 0)
            ang += 180.0;
        return true;
    };
    auto devFromLong = [&](double a)
    {
        const double d = std::abs(a - bestBin);
        return std::min(d, 180.0 - d);
    };

    std::vector<Line2D> shortLines;
    for (int e : {i0, i1})
    {
        int near = 0;
        double bd = 1e18;
        for (int i = 0; i < n; ++i)
        {
            const double d = cv::norm(cv::Point2f(contour[i]) - ends[e]);
            if (d < bd)
            {
                bd = d;
                near = i;
            }
        }

        double aF = 0, aB = 0;
        const bool hF = sideAngle(near, +1, aF);
        const bool hB = sideAngle(near, -1, aB);
        double ang = -1;
        if (hF && hB)
            ang = (devFromLong(aF) > devFromLong(aB)) ? aF : aB;
        else if (hF)
            ang = aF;
        else if (hB)
            ang = aB;
        if (ang < 0 || devFromLong(ang) < kMinShortEdgeDevDeg)
            continue;

        const double r = ang * CV_PI / 180.0;
        const cv::Point2f dir(static_cast<float>(std::cos(r)),
                              static_cast<float>(std::sin(r)));
        // 法向量 = 方向逆时针转 90° = (-dir.y, dir.x)
        shortLines.push_back(Line2D{-dir.y, dir.x,
                                    -(double(-dir.y) * ends[e].x + double(dir.x) * ends[e].y)});
    }

    // ---- 6) 两条短边 × 两条长边 = 4 个亚像素角点 ----
    std::vector<cv::Point2f> corners;
    if (shortLines.size() == 2)
    {
        for (const auto &Ls : shortLines)
        {
            for (const Line2D *Ll : {&LA, &LB})
            {
                cv::Point2f c;
                if (!lineIntersect(Ls, *Ll, c))
                    continue;
                double bd = 1e18; // 离轮廓太远说明某条线跑飞了
                for (int i = 0; i < n; ++i)
                    bd = std::min(bd, cv::norm(c - cv::Point2f(contour[i])));
                if (bd > kMaxCornerDistPx)
                    continue;
                bool dup = false;
                for (const auto &q : corners)
                    if (cv::norm(c - q) < kMinPointSeparationPx)
                    {
                        dup = true;
                        break;
                    }
                if (!dup)
                    corners.push_back(c);
            }
        }
    }

    orderCornersCCW(corners);
    if (corners.size() != 4)
        return false;

    for (int k = 0; k < 4; ++k)
        outCorners[k] = corners[k];

    cx = 0.25 * (corners[0].x + corners[1].x + corners[2].x + corners[3].x);
    return true;
}

// ============================================================================
//  整帧检测：图像 -> 8 个角点
// ============================================================================
bool detectImpl(const cv::Mat &bgrFrame, std::vector<cv::Point2f> &points2D,
                std::string *reason)
{
    points2D.clear();
    if (bgrFrame.empty())
    {
        if (reason)
            *reason = "输入帧为空";
        return false;
    }

    cv::Mat mask, maskBlur, edges;
    preprocess(bgrFrame, mask, maskBlur, edges);

    // 必须用 CHAIN_APPROX_NONE：SIMPLE 会把直边上的点抽稀成两个端点，
    // 那样直边方向直方图里几乎没有点可统计
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(edges, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

    // 梯度在灰度 maskBlur 上算，比在二值 edges 上稳定得多
    cv::Mat gx, gy, mag, angle;
    cv::Sobel(maskBlur, gx, CV_32F, 1, 0, 3);
    cv::Sobel(maskBlur, gy, CV_32F, 0, 1, 3);
    cv::magnitude(gx, gy, mag);
    cv::phase(gx, gy, angle, true); // 0~360

    struct Bar
    {
        double cx = 0.0;
        std::array<cv::Point2f, 4> pts{};
    };
    std::vector<Bar> bars;

    for (const auto &contour : contours)
    {
        Bar b;
        if (extractBarCorners(contour, mag, angle, b.pts, b.cx))
            bars.push_back(b);
    }

    if (bars.size() != 2)
    {
        if (reason)
            *reason = "有效光条数 = " + std::to_string(bars.size()) + "（需要 2）";
        return false;
    }

    // 按质心 x 从左到右排序。不能依赖 findContours 的顺序（它没有顺序保证），
    // 这样 [0..3] 一定是左条、[4..7] 一定是右条
    std::sort(bars.begin(), bars.end(),
              [](const Bar &a, const Bar &b) { return a.cx < b.cx; });

    if (bars[1].cx - bars[0].cx < kMinBarSpacingPx)
    {
        if (reason)
            *reason = "两光条质心间距只有 " +
                      std::to_string(static_cast<int>(bars[1].cx - bars[0].cx)) +
                      " px，左右布局不成立";
        return false;
    }

    points2D.reserve(8);
    for (const auto &b : bars)
        for (const auto &p : b.pts)
            points2D.push_back(p);

    return true;
}

// ============================================================================
//  位姿：8 个角点 -> 车体系下的目标点与目标向量
// ============================================================================
// 内部诊断量：只在调参辅助功能里用，不进入对外返回值
struct PoseDetail
{
    std::vector<cv::Point2f> points2D;
    std::vector<cv::Point2f> reproj;
    cv::Matx33d R_armor_to_cam = cv::Matx33d::eye();
    cv::Vec3d t_armor_in_cam = cv::Vec3d(0, 0, 0);
    double errMeanPx = 0.0;
};

CarTarget poseImpl(const std::vector<cv::Point2f> &points2D, const PnPDate &data,
                   const CarTargetOptions &opt, PoseDetail *detail = nullptr)
{
    CarTarget out;
    PoseDetail local;
    PoseDetail &det = detail ? *detail : local;
    det.points2D = points2D;

    if (points2D.size() != 8 || data.points3D.size() != 8)
    {
        out.reason = "点数不匹配：2D = " + std::to_string(points2D.size()) +
                     "，3D = " + std::to_string(data.points3D.size()) + "（都需要 8）";
        return out;
    }

    const cv::Matx33d K = data.cameraMatrix;
    const std::vector<double> &D = data.distCoeffs;

    // 8 点全部共面(z=0) → IPPE：解析解，不需初值。
    // 代价是平面位姿天然有两个解，用重投影误差挑
    std::vector<cv::Mat> rvecs, tvecs;
    const int nSol = cv::solvePnPGeneric(data.points3D, points2D, K, D,
                                         rvecs, tvecs, false, cv::SOLVEPNP_IPPE);
    if (nSol < 1)
    {
        out.reason = "IPPE 无解";
        return out;
    }

    int best = 0;
    double bestErr = 1e18;
    std::vector<std::vector<cv::Point2f>> projs(static_cast<size_t>(nSol));
    for (int i = 0; i < nSol; ++i)
    {
        cv::projectPoints(data.points3D, rvecs[i], tvecs[i], K, D, projs[i]);
        double e = 0.0;
        for (size_t k = 0; k < projs[i].size(); ++k)
            e += cv::norm(projs[i][k] - points2D[k]);
        e /= static_cast<double>(projs[i].size());
        if (e < bestErr)
        {
            bestErr = e;
            best = i;
        }
    }

    cv::Mat rvec = rvecs[best];
    cv::Mat tvec = tvecs[best];
    if (opt.refineWithLM)
        cv::solvePnPRefineLM(data.points3D, points2D, K, D, rvec, tvec);

    cv::Mat R;
    cv::Rodrigues(rvec, R);
    cv::projectPoints(data.points3D, rvec, tvec, K, D, det.reproj);

    det.R_armor_to_cam = cv::Matx33d(R);
    det.t_armor_in_cam = cv::Vec3d(tvec.at<double>(0), tvec.at<double>(1),
                                   tvec.at<double>(2));

    det.errMeanPx = 0.0;
    for (size_t k = 0; k < det.reproj.size(); ++k)
        det.errMeanPx += cv::norm(det.reproj[k] - points2D[k]);
    det.errMeanPx /= static_cast<double>(det.reproj.size());

    // ---- 装甲板系 -> 车体系 ----
    //   P_car = R_car_from_cam * (R * P_armor + t) + t_cam_in_car
    // 展开后：
    //   R_armor_to_car = R_car_from_cam * R
    //   t_armor_in_car = R_car_from_cam * t + t_cam_in_car
    const cv::Matx33d R_armor_to_car = opt.R_car_from_cam * det.R_armor_to_cam;
    const cv::Vec3d t_armor_in_car =
        cv::Vec3d(opt.R_car_from_cam * det.t_armor_in_cam) + opt.t_cam_in_car;

    // ★ 目标点：点要平移
    const cv::Vec3d pointArmor(data.m_point.x, data.m_point.y, data.m_point.z);
    out.point = cv::Vec3d(R_armor_to_car * pointArmor) + t_armor_in_car;

    // ★ 目标向量：自由向量，只旋转不平移
    const cv::Vec3d vecArmor(data.v[0], data.v[1], data.v[2]);
    out.vec = cv::Vec3d(R_armor_to_car * vecArmor);

    out.errMeanPx = det.errMeanPx;
    out.ok = true;
    return out;
}

} // namespace

// ============================================================================
//  对外接口
// ============================================================================

bool detectCorners(const cv::Mat &bgrFrame, std::vector<cv::Point2f> &points2D)
{
    return detectImpl(bgrFrame, points2D, nullptr);
}

CarTarget solveCarTargetFromCorners(const std::vector<cv::Point2f> &points2D,
                                    const PnPDate &data,
                                    const CarTargetOptions &opt)
{
    return poseImpl(points2D, data, opt);
}

CarTarget solveCarTarget(const cv::Mat &bgrFrame, const PnPDate &data,
                         const CarTargetOptions &opt)
{
    std::vector<cv::Point2f> points2D;
    std::string reason;
    if (!detectImpl(bgrFrame, points2D, &reason))
    {
        CarTarget out;
        out.reason = reason;
        return out;
    }
    return poseImpl(points2D, data, opt);
}

// ============================================================================
//  PnPTuner：累计误差 + 给出 points3D 调整建议
// ============================================================================

CarTarget PnPTuner::add(const cv::Mat &bgrFrame, const PnPDate &data)
{
    std::vector<cv::Point2f> points2D;
    CarTarget out;
    if (!detectImpl(bgrFrame, points2D, &out.reason))
        return out;

    PoseDetail det;
    out = poseImpl(points2D, data, CarTargetOptions(), &det);
    if (!out.ok)
        return out;

    const cv::Matx33d K = data.cameraMatrix;
    const cv::Matx33d R = det.R_armor_to_cam;

    for (size_t k = 0; k < points2D.size(); ++k)
    {
        const cv::Point3f &pw = data.points3D[k];
        // 该点在相机系的深度
        const double zc = R(2, 0) * pw.x + R(2, 1) * pw.y + R(2, 2) * pw.z +
                          det.t_armor_in_cam[2];
        if (zc <= 1.0)
            continue;
        // 像素差 -> 相机系横向偏移 -> 装甲板系偏移（R 正交阵，转置即逆）
        const double du = points2D[k].x - det.reproj[k].x;
        const double dv = points2D[k].y - det.reproj[k].y;
        const double dcx = du * zc / K(0, 0);
        const double dcy = dv * zc / K(1, 1);
        dSum_[k][0] += R(0, 0) * dcx + R(1, 0) * dcy;
        dSum_[k][1] += R(0, 1) * dcx + R(1, 1) * dcy;
        dSum_[k][2] += R(0, 2) * dcx + R(1, 2) * dcy;
    }

    ++frames_;
    errSum_ += out.errMeanPx;
    return out;
}

std::string PnPTuner::advice(const PnPDate &data) const
{
    if (frames_ == 0)
        return "\n[提示] 还没有成功解出过位姿，没有调整建议\n";

    std::string out;
    char buf[512];

    std::snprintf(buf, sizeof(buf),
                  "\n=== 三维坐标调整建议（累计 %d 帧平均，单位 mm）===\n", frames_);
    out += buf;
    out += "点   角点       dX      dY      dZ    当前坐标(mm)          建议坐标(mm)\n";

    for (int k = 0; k < 8 && k < static_cast<int>(data.points3D.size()); ++k)
    {
        const cv::Vec3d d(dSum_[k][0] / frames_, dSum_[k][1] / frames_,
                          dSum_[k][2] / frames_);
        const cv::Point3f &p = data.points3D[static_cast<size_t>(k)];
        std::snprintf(buf, sizeof(buf),
                      "p%d  %-5s  %+7.2f %+7.2f %+7.2f  (%7.1f,%7.1f,%5.1f)  (%7.1f,%7.1f,%5.1f)\n",
                      k, kCornerName[k], d[0], d[1], d[2],
                      p.x, p.y, p.z, p.x + d[0], p.y + d[1], p.z + d[2]);
        out += buf;
    }

    std::snprintf(buf, sizeof(buf), "累计平均反投影误差 %.2f px\n", avgErrMeanPx());
    out += buf;
    out += "提示：建议只应用 50%，改完重跑再迭代；一次改满会过拟合单帧\n";
    out += "=================================================\n";
    return out;
}

void PnPTuner::reset()
{
    frames_ = 0;
    errSum_ = 0.0;
    for (auto &d : dSum_)
        d = cv::Vec3d(0.0, 0.0, 0.0);
}
