#include "PnPSolver.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <mutex>
#include <cstdio>
#include <cstdlib>

namespace
{

// ============================================================================
//  调试输出：只有环境变量 PNP_DEBUG 非空且不为 "0" 时才打印，默认完全静默
//  （本库在相机线程里逐帧调用，无条件打印会把终端刷爆）
// ============================================================================
bool debugOn()
{
    static const bool on = []
    {
        const char *v = std::getenv("PNP_DEBUG");
        return v != nullptr && *v != '\0' && std::string(v) != "0";
    }();
    return on;
}

// 一次检测的统计量；析构时打印一行，提前 return 也能打出当前进度
struct DetectTrace
{
    const bool on = debugOn();
    size_t contours = 0; // findContours 得到的原始轮廓数
    size_t usable = 0;   // 点数够、参与梯度取景的轮廓数
    size_t bars = 0;     // 成功提取出 4 个角点的光条数
    double spacing = -1; // 两光条质心间距 (px)，-1 表示尚未计算
    const char *result = "未完成";

    ~DetectTrace()
    {
        if (!on)
            return;
        std::fprintf(stderr, "[PnP] 轮廓=%zu 可用=%zu 光条=%zu 间距=%.0fpx -> %s\n",
                     contours, usable, bars, spacing, result);
    }
};

// ============================================================================
//  固定的算法参数（要调就改这里）
// ============================================================================
// 红蓝通道相减二值化的差分阈值：运行时用滑条可改，下面是默认值
//   mask = (R - B) > 差分阈值
const int kDefaultDiffThreshold = 70; // R - B 差分阈值默认值

const double kCannyLow = 50.0;
const double kCannyHigh = 150.0;
const int kCannyAperture = 3;
const int kGradMargin = 8; // 梯度 ROI 外扩像素：Sobel 3x3 需要邻域，采样点必须离 ROI 边界足够远

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
const double kMaxShortEdgeOffPx = 10.0;   // 拟合出的短边直线到短边中点的最大偏离
const int kMinShortFitPoints = 2;         // 拟合短边直线至少要几个等分点

const double kMinBarSpacingPx = 20.0; // 两光条质心最小水平间距

// 滑条可改的差分阈值（相机线程每帧 load 一次，开销可忽略）
std::atomic<int> gDiffThreshold{kDefaultDiffThreshold};

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

// Line2D -> 方向角 [0,180)，与 ea/votes 的角度口径一致
double lineAngleDeg(const Line2D &L)
{
    double ang = std::atan2(L.a, -L.b) * 180.0 / CV_PI;
    if (ang < 0)
        ang += 180.0;
    return ang;
}

// ============================================================================
//  调试绘制：半透明文字面板（左上角显示实时数值）
// ============================================================================
//  先铺一块半透明黑底再写字，亮背景/暗背景都能看清
void drawTextPanel(cv::Mat &img, const std::vector<std::string> &lines,
                   const cv::Point &origin, double fontScale, int lineHeight,
                   const cv::Scalar &color)
{
    if (lines.empty())
        return;

    int maxW = 0;
    for (const auto &t : lines)
    {
        int base = 0;
        const cv::Size sz = cv::getTextSize(t, cv::FONT_HERSHEY_SIMPLEX, fontScale, 1, &base);
        maxW = std::max(maxW, sz.width);
    }
    const int pad = 6;
    const int h = lineHeight * static_cast<int>(lines.size()) + 2 * pad;
    const int w = maxW + 2 * pad;
    const cv::Rect box(origin.x, origin.y - lineHeight + pad, w, h);

    // 裁剪到图像内再填，避免越界
    const cv::Rect imgRect(0, 0, img.cols, img.rows);
    const cv::Rect safe = box & imgRect;
    if (safe.area() > 0)
    {
        cv::Mat roi = img(safe);
        cv::addWeighted(roi, 0.30, cv::Mat(roi.size(), roi.type(), cv::Scalar(0, 0, 0)), 0.70, 0, roi);
    }

    for (size_t i = 0; i < lines.size(); ++i)
        cv::putText(img, lines[i], origin + cv::Point(pad, lineHeight * static_cast<int>(i)),
                    cv::FONT_HERSHEY_SIMPLEX, fontScale, color, 1, cv::LINE_AA);
}

// ============================================================================
//  调试绘制：坐标系三轴
// ============================================================================
//  颜色固定为 X=红、Y=绿、Z=蓝（BGR 下分别是 (0,0,255)/(0,255,0)/(255,0,0)）
const cv::Scalar kAxisBGR[3] = {cv::Scalar(0, 0, 255), cv::Scalar(0, 220, 0),
                                cv::Scalar(255, 0, 0)};
const char *const kAxisName[3] = {"x", "y", "z"};

// 带黑描边的文字：直接压在箭头/角点上也能看清，不用铺半透明底板
void putTextOutline(cv::Mat &img, const std::string &text, const cv::Point &org,
                    double fontScale, const cv::Scalar &color)
{
    cv::putText(img, text, org, cv::FONT_HERSHEY_SIMPLEX, fontScale,
                cv::Scalar(0, 0, 0), 3, cv::LINE_AA);
    cv::putText(img, text, org, cv::FONT_HERSHEY_SIMPLEX, fontScale, color, 1,
                cv::LINE_AA);
}

// 轴向的深度标签：相机 +z 指向"屏幕里"（远离观察者）
//   d.z > 0 -> 指向纸面内(away)   d.z < 0 -> 指向纸面外(朝向观察者)
const char *axisDepthTag(const cv::Vec3d &d)
{
    if (d[2] > 0.2)
        return " in";   // into screen
    if (d[2] < -0.2)
        return " out";  // toward viewer
    return "";          // 基本在图像平面内
}

// 画"方位指示器"：原点不在视野里时用。
//  R_cam_from_frame 的三列 = 该系三轴在相机系下的方向；
//  在这里做正交投影（相当于站在相机位置看过去）：图像 u 轴 = 相机 +x，
//  图像 v 轴 = 相机 +y（向下）。因此：
//    轴基本垂直纸面（|x|,|y| 都很小）→ 画成圆点并标注"⊙/⊗"
//    否则 → 按方向画一根定长箭头
void drawOrientationGizmo(cv::Mat &img, const cv::Matx33d &R_cam_from_frame,
                          const cv::Point &anchor, int size, const char *label)
{
    cv::circle(img, anchor, 3, cv::Scalar(255, 255, 255), -1, cv::LINE_AA);
    cv::putText(img, label, anchor + cv::Point(-size, -size - 6),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);

    for (int c = 0; c < 3; ++c)
    {
        const cv::Vec3d d(R_cam_from_frame(0, c), R_cam_from_frame(1, c),
                          R_cam_from_frame(2, c));
        const double n = std::hypot(d[0], d[1]); // 在图像平面上的投影长度
        if (n < 0.25)
        {
            // 几乎沿光轴：正对/背对观察者，画圆点
            const bool toward = d[2] < 0.0; // 相机 +z 指向"屏幕里"，所以 d[2]<0 是朝向观察者
            cv::circle(img, anchor, 7, kAxisBGR[c], 1, cv::LINE_AA);
            if (toward)
                cv::circle(img, anchor, 2, kAxisBGR[c], -1, cv::LINE_AA);
            const std::string t = std::string(kAxisName[c]) + axisDepthTag(d);
            cv::putText(img, t, anchor + cv::Point(10, -10),
                        cv::FONT_HERSHEY_SIMPLEX, 0.45, kAxisBGR[c], 1, cv::LINE_AA);
            continue;
        }
        const cv::Point tip(anchor.x + cvRound(d[0] / n * size),
                            anchor.y + cvRound(d[1] / n * size));
        cv::arrowedLine(img, anchor, tip, kAxisBGR[c], 2, cv::LINE_AA, 0, 0.28);
        const std::string t = std::string(kAxisName[c]) + axisDepthTag(d);
        cv::putText(img, t, tip + cv::Point(5, -5),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, kAxisBGR[c], 1, cv::LINE_AA);
    }
}

// 虚线箭头：杆画成虚线段，末端留一段实心箭头。
// 用来画"次要"的那个坐标系，和实线箭头一眼可分。
void drawDashedArrow(cv::Mat &img, const cv::Point2f &from, const cv::Point2f &to,
                     const cv::Scalar &color, int thickness)
{
    const cv::Point2f d = to - from;
    const float len = cv::norm(d);
    if (len < 3.f)
        return;
    const cv::Point2f u = d / len;
    const float shaft = len * 0.72f;   // 杆占 72%，剩下给箭头
    for (float t = 0.f; t < shaft; t += 9.f)
        cv::line(img, from + u * t, from + u * std::min(t + 5.f, shaft), color,
                 thickness, cv::LINE_AA);
    cv::arrowedLine(img, from + u * (len * 0.5f), to, color, thickness, cv::LINE_AA, 0, 0.3);
}

// 画"真实三维三轴"：原点在相机系下已知，三列方向也已知，走透视投影。
//  端点跑到相机后方（z<=1）的轴直接跳过。
void drawAxesAtPoint(cv::Mat &img, const cv::Matx33d &K, const std::vector<double> &D,
                     const cv::Point3d &originCam, const cv::Matx33d &R_cam_from_frame,
                     double len, int thickness, double fontScale, bool withLabels = true,
                     bool dashed = false, bool outlined = false)
{
    std::vector<cv::Point3f> pts;
    pts.emplace_back((float)originCam.x, (float)originCam.y, (float)originCam.z);
    for (int c = 0; c < 3; ++c)
        pts.emplace_back((float)(originCam.x + R_cam_from_frame(0, c) * len),
                         (float)(originCam.y + R_cam_from_frame(1, c) * len),
                         (float)(originCam.z + R_cam_from_frame(2, c) * len));

    std::vector<cv::Point2f> uv;
    cv::projectPoints(pts, cv::Vec3d(0, 0, 0), cv::Vec3d(0, 0, 0), cv::Mat(K), D, uv);

    cv::circle(img, uv[0], 3, cv::Scalar(255, 255, 255), -1, cv::LINE_AA);
    for (int c = 0; c < 3; ++c)
    {
        if (pts[c + 1].z <= 1.0f)
            continue; // 端点跑到相机后方

        // 几乎沿光轴的轴（比如正对相机时的板法向）透视投影只剩几个像素，
        // 箭头看不出朝向，改用和 base_link 指示器一致的 ⊙/⊗ 标记。
        const cv::Vec3d dir(R_cam_from_frame(0, c), R_cam_from_frame(1, c),
                            R_cam_from_frame(2, c));
        if (std::hypot(dir[0], dir[1]) < 0.25)
        {
            // 半径要比原点那个白色实心点(3)大，否则标记会被它吞掉
            cv::circle(img, uv[0], 9, kAxisBGR[c], 1, cv::LINE_AA);
            if (dir[2] < 0.0) // 相机 +z 指向屏幕里，所以 z<0 是朝向观察者
                cv::circle(img, uv[0], 4, kAxisBGR[c], -1, cv::LINE_AA);
            else
            {
                cv::line(img, uv[0] + cv::Point2f(-5.f, -5.f), uv[0] + cv::Point2f(5.f, 5.f),
                         kAxisBGR[c], 1, cv::LINE_AA);
                cv::line(img, uv[0] + cv::Point2f(-5.f, 5.f), uv[0] + cv::Point2f(5.f, -5.f),
                         kAxisBGR[c], 1, cv::LINE_AA);
            }
            // 这个标记必须带 in/out 文字，否则 ⊙/⊗ 本身看不出朝向
            putTextOutline(img, std::string(kAxisName[c]) + axisDepthTag(dir),
                           uv[0] + cv::Point2f(12.f, -12.f), fontScale, kAxisBGR[c]);
            continue;
        }
        if (outlined)   // 先画一遍略粗的黑箭头当描边，再压上彩色：彩色仍是主体
            cv::arrowedLine(img, uv[0], uv[c + 1], cv::Scalar(0, 0, 0),
                            thickness + 1, cv::LINE_AA, 0, 0.25);
        if (dashed)
            drawDashedArrow(img, uv[0], uv[c + 1], kAxisBGR[c], thickness);
        else
            cv::arrowedLine(img, uv[0], uv[c + 1], kAxisBGR[c], thickness, cv::LINE_AA, 0, 0.25);
        const bool inside = uv[c + 1].x >= 0 && uv[c + 1].y >= 0 &&
                            uv[c + 1].x < img.cols && uv[c + 1].y < img.rows;
        if (inside && withLabels)
        {
            const cv::Vec3d d(R_cam_from_frame(0, c), R_cam_from_frame(1, c),
                              R_cam_from_frame(2, c));
            const std::string t = std::string(kAxisName[c]) + axisDepthTag(d);
            putTextOutline(img, t, uv[c + 1] + cv::Point2f(5.f, -5.f), fontScale,
                           kAxisBGR[c]);
        }
    }
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
//  预处理：直接做红蓝通道差分，阈值二值化，不做开闭运算
//  适合“红灯条 / 蓝灯条颜色特征明显”的装甲板。
//  这里默认按红灯条处理：diff = R - B，阈值取 gDiffThreshold（滑条可改）。
//  若目标实际为蓝色灯条，只需把 diff 方向改为 B - R 即可。
// ============================================================================
void preprocess(const cv::Mat &bgrFrame, cv::Mat &mask, cv::Mat &maskBlur, cv::Mat &edges)
{
    std::array<cv::Mat, 3> ch{};
    cv::split(bgrFrame, ch);

    const int diffTh = gDiffThreshold.load(std::memory_order_relaxed);

    // 红灯条：差分增强（8U 下 cv::subtract 饱和，R < B 直接截成 0）
    cv::Mat diff;
    cv::subtract(ch[2], ch[0], diff); // R - B

    // 直接二值化：保留强红偏差目标
    cv::threshold(diff, mask, diffTh, 255, cv::THRESH_BINARY);

    // 不做开闭运算；直接保留整块二值图用于后续轮廓和梯度计算。
    maskBlur = mask.clone();
    cv::Canny(mask, edges, kCannyLow, kCannyHigh, kCannyAperture, true);
}

// ============================================================================
//  单根光条：轮廓 -> 4 个规范化角点
// ============================================================================
bool extractBarCorners(const std::vector<cv::Point> &contour,
                       const cv::Mat &mag, const cv::Mat &angle,
                       const cv::Point gradOrigin,
                       std::array<cv::Point2f, 4> &outCorners, double &cx)
{
    const int n = static_cast<int>(contour.size());
    if (n < kMinContourPoints)
        return false;

    // ---- 1) 每个轮廓点的边缘方向 + 方向直方图 ----
    // mag/angle 是"轮廓包围盒内"的局部梯度图，采样前要减去包围盒左上角坐标
    std::vector<float> ea(n, -1.f); // -1 表示梯度太弱，方向不可信
    std::array<int, 180> votes{};
    for (int i = 0; i < n; ++i)
    {
        const cv::Point gp = contour[i] - gradOrigin;
        if (gp.x < 0 || gp.y < 0 || gp.x >= mag.cols || gp.y >= mag.rows)
            continue; // 理论上不会发生（ROI 已覆盖所有轮廓），防御性检查
        if (mag.at<float>(gp) < kMinGradientMag)
            continue;
        float a = std::fmod(angle.at<float>(gp) + 90.f, 180.f);
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

    // ---- 3b) 与两条长边平行、位于 1/4 / 2/4 / 3/4 处的三条线 ----
    //  f 是"从 LA 到 LB 的相对位置"：f=0 就是 LA，f=1 就是 LB，f=0.5 就是中线
    auto lerpLine = [&](double f)
    {
        Line2D L{(1.0 - f) * LA.a + f * LB.a,
                 (1.0 - f) * LA.b + f * LB.b,
                 (1.0 - f) * LA.c + f * LB.c};
        const double nn = std::hypot(L.a, L.b);
        if (nn < 1e-9)
            return Line2D{0.0, 0.0, 0.0}; // 退化成点，a==b==0，调用方据此判失败
        L.a /= nn;
        L.b /= nn;
        L.c /= nn;
        return L;
    };

    // 直线与轮廓的所有交点（亚像素，去重）
    auto intersectContour = [&](const Line2D &L, std::vector<cv::Point2f> &hits)
    {
        for (int i = 0; i < n; ++i)
        {
            const cv::Point2f p0 = contour[i];
            const cv::Point2f p1 = contour[(i + 1) % n];
            const double s0 = L.a * p0.x + L.b * p0.y + L.c;
            const double s1 = L.a * p1.x + L.b * p1.y + L.c;
            if (s0 * s1 > 0)
                continue;
            if (std::abs(s0 - s1) < 1e-9)
                continue;
            const double t = s0 / (s0 - s1);
            if (t < 0.0 || t > 1.0)
                continue;
            const cv::Point2f h = p0 + static_cast<float>(t) * (p1 - p0);
            bool dup = false; // 相邻两段会给出同一个交点
            for (const auto &d : hits)
                if (cv::norm(h - d) < kMinPointSeparationPx)
                {
                    dup = true;
                    break;
                }
            if (!dup)
                hits.push_back(h);
        }
    };

    // ---- 4) 三条平行线 × 轮廓 = 每条短边上的三个等分点 ----
    //  只用中线交点 + 该点附近的梯度方向定短边，等于"一个点定一条直线"，
    //  点的噪声会整块变成方向误差；改成 1/4、2/4、3/4 三条线的交点共 3 个点拟合，
    //  思路仍是"先拟合长边、再由长边推出短边"，只是把短边的约束从 1 点变成 3 点。
    const std::array<double, 3> kFrac{0.25, 0.5, 0.75};
    std::array<std::vector<cv::Point2f>, 3> fracHits;
    std::array<Line2D, 3> fracLines;
    for (int i = 0; i < 3; ++i)
    {
        fracLines[i] = lerpLine(kFrac[i]);
        if (fracLines[i].a == 0.0 && fracLines[i].b == 0.0)
            return false;
        intersectContour(fracLines[i], fracHits[i]);
    }

    // 中线（2/4）与轮廓的两个交点 = 上下两条短边的中点
    const std::vector<cv::Point2f> &ends = fracHits[1];
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

    // ---- 5) 短边直线：优先用三个等分点拟合，退化时回退到邻域梯度方向 ----
    //  5a 用 1/4、2/4、3/4 三个交点拟合（默认路径）
    //  5b 回到原来的做法：中线交点 + 该点前后几个轮廓点的梯度方向
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
        // 三条平行线在这一端各给一个交点，正好是短边上的 1/4、2/4、3/4 点
        std::vector<cv::Point2f> onEdge;
        for (int li = 0; li < 3; ++li)
        {
            int best = -1;
            double bdHit = 1e18;
            for (int k = 0; k < static_cast<int>(fracHits[li].size()); ++k)
            {
                const double d = cv::norm(fracHits[li][k] - ends[e]);
                if (d < bdHit)
                {
                    bdHit = d;
                    best = k;
                }
            }
            if (best >= 0)
                onEdge.push_back(fracHits[li][best]);
        }

        // ---- 5a) 首选用这 3 个等分点直接拟合短边直线 ----
        Line2D Ls{};
        bool haveLine = false;
        if (static_cast<int>(onEdge.size()) >= kMinShortFitPoints)
        {
            cv::Vec4f fs;
            cv::fitLine(onEdge, fs, cv::DIST_L2, 0, 0.01, 0.01);
            const Line2D cand = makeLine(fs);
            // 拟合出的方向不能和长边平行；且必须穿过该短边中点（否则是挑错了点）
            const double off = std::abs(cand.a * ends[e].x + cand.b * ends[e].y + cand.c);
            if (devFromLong(lineAngleDeg(cand)) >= kMinShortEdgeDevDeg &&
                off <= kMaxShortEdgeOffPx)
            {
                Ls = cand;
                haveLine = true;
            }
        }

        // ---- 5b) 兜底：等分点不够时仍按原来的"单点 + 邻域梯度方向"定线 ----
        if (!haveLine)
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
            Ls = Line2D{-dir.y, dir.x,
                        -(double(-dir.y) * ends[e].x + double(dir.x) * ends[e].y)};
        }

        shortLines.push_back(Ls);
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
                std::string *reason, DetectDebug *dbg = nullptr)
{
    points2D.clear();
    DetectTrace trace;

    // 退出时统一把中间量同步给调用方（生产路径 dbg == nullptr，零开销）
    struct DebugSync
    {
        DetectDebug *dbg = nullptr;
        const std::vector<cv::Point2f> &pts;
        std::string *reason = nullptr;
        bool ok = false;
        ~DebugSync()
        {
            if (!dbg)
                return;
            dbg->ok = ok;
            dbg->points2D = pts;
            if (!ok && reason)
                dbg->reason = *reason;
        }
    } sync{dbg, points2D, reason};
    if (dbg)
    {
        dbg->ok = false;
        dbg->reason.clear();
        dbg->bars.clear();
        dbg->contours.clear();
        dbg->mask.release();
    }

    if (bgrFrame.empty())
    {
        if (reason)
            *reason = "输入帧为空";
        trace.result = "输入帧为空";
        return false;
    }

    cv::Mat mask, maskBlur, edges;
    preprocess(bgrFrame, mask, maskBlur, edges);
    if (dbg)
        dbg->mask = mask;

    // 必须用 CHAIN_APPROX_NONE：SIMPLE 会把直边上的点抽稀成两个端点，
    // 那样直边方向直方图里几乎没有点可统计
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(edges, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
    trace.contours = contours.size();
    if (dbg)
        dbg->contours = contours;

    // ---- 梯度只在轮廓包围盒内计算 ----
    // 整帧算 Sobel×2 + magnitude + phase 约占整条检测链 50% 的时间，
    // 但真正被用到的只有轮廓上的几百个点。这里先求候选轮廓包围盒的并集，
    // 只在这块区域算梯度（外扩 kGradMargin 以覆盖 Sobel 的 3x3 邻域，
    // 保证轮廓点上的值与整帧计算完全一致）。
    cv::Rect gradRoi;
    for (const auto &c : contours)
    {
        if (static_cast<int>(c.size()) < kMinContourPoints)
            continue; // 噪点轮廓不参与取景，也省掉后面的无效计算
        ++trace.usable;
        const cv::Rect r = cv::boundingRect(c);
        gradRoi = (gradRoi.area() == 0) ? r : (gradRoi | r);
    }
    if (gradRoi.area() == 0)
    {
        if (reason)
            *reason = "没有有效轮廓";
        trace.result = "没有有效轮廓";
        return false;
    }
    gradRoi = cv::Rect(gradRoi.x - kGradMargin, gradRoi.y - kGradMargin,
                       gradRoi.width + 2 * kGradMargin,
                       gradRoi.height + 2 * kGradMargin) &
              cv::Rect(0, 0, bgrFrame.cols, bgrFrame.rows);

    // 梯度在灰度 maskBlur 上算，比在二值 edges 上稳定得多
    cv::Mat gx, gy, mag, angle;
    const cv::Mat blurRoi = maskBlur(gradRoi);
    cv::Sobel(blurRoi, gx, CV_32F, 1, 0, 3);
    cv::Sobel(blurRoi, gy, CV_32F, 0, 1, 3);
    cv::magnitude(gx, gy, mag);
    cv::phase(gx, gy, angle, true); // 0~360

    struct Bar
    {
        double cx = 0.0;
        std::array<cv::Point2f, 4> pts{};
    };
    std::vector<Bar> bars;
    bars.reserve(contours.size());

    for (const auto &contour : contours)
    {
        Bar b;
        if (extractBarCorners(contour, mag, angle, gradRoi.tl(), b.pts, b.cx))
            bars.push_back(b);
    }
    trace.bars = bars.size();

    // 按质心 x 从左到右排序。不能依赖 findContours 的顺序（它没有顺序保证），
    // 这样 [0..3] 一定是左条、[4..7] 一定是右条
    std::sort(bars.begin(), bars.end(),
              [](const Bar &a, const Bar &b) { return a.cx < b.cx; });

    // 调参视图用：光条没凑齐 2 根也把已拟合的梯形带出去
    if (dbg)
    {
        dbg->bars.reserve(bars.size());
        for (const auto &b : bars)
            dbg->bars.push_back(b.pts);
    }

    if (bars.size() != 2)
    {
        if (reason)
            *reason = "有效光条数 = " + std::to_string(bars.size()) + "（需要 2）";
        trace.result = "光条数不足";
        return false;
    }

    if (bars[1].cx - bars[0].cx < kMinBarSpacingPx)
    {
        trace.spacing = bars[1].cx - bars[0].cx;
        if (reason)
            *reason = "两光条质心间距只有 " +
                      std::to_string(static_cast<int>(bars[1].cx - bars[0].cx)) +
                      " px，左右布局不成立";
        trace.result = "间距过小";
        return false;
    }

    trace.spacing = bars[1].cx - bars[0].cx;
    trace.result = "OK";

    points2D.reserve(8);
    for (const auto &b : bars)
        for (const auto &p : b.pts)
            points2D.push_back(p);

    sync.ok = true;
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
    cv::Matx33d R_cam_from_armor = cv::Matx33d::eye();
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

    // ---- 选解：误差优先 + 物理约束 + 时序连续性 ----
    // IPPE 对共面点必然给 2 个解，两解姿态差 27~30°，而重投影误差只差 0.3~0.5 px。
    // 只按误差最小挑，余量太薄，噪声一变就翻解（姿态瞬间跳几十度）。
    struct Cand
    {
        int idx = -1;
        double err = 0.0;   // 累加器，必须从 0 开始（从 1e18 起会让两解误差差不出区别）
        cv::Matx33d R = cv::Matx33d::eye();
        bool normalOk = true;   // 板法向是否朝相机（物理可行）
    };
    std::vector<Cand> cands;
    cands.reserve(static_cast<size_t>(nSol));
    for (int i = 0; i < nSol; ++i)
    {
        Cand c;
        c.idx = i;
        std::vector<cv::Point2f> proj;
        cv::projectPoints(data.points3D, rvecs[i], tvecs[i], K, D, proj);
        for (size_t k = 0; k < proj.size(); ++k)
            c.err += cv::norm(proj[k] - points2D[k]);
        c.err /= static_cast<double>(proj.size());

        cv::Mat Ri;
        cv::Rodrigues(rvecs[i], Ri);
        c.R = cv::Matx33d(Ri);
        // 装甲板 +Z = 板法向；正面朝相机时，其在相机系下的 z 分量必须为负
        c.normalOk = c.R(2, 2) < 0.0;
        cands.push_back(c);
    }

    // 上一帧的姿态（用于打平时判定连续性）。跨调用共享，加锁保护；
    // 超过 priorMaxAgeSec 未更新则视为失效，避免旧姿态把结果锁死。
    static std::mutex priorMtx;
    static cv::Matx33d priorR = cv::Matx33d::eye();
    static bool havePrior = false;
    static std::chrono::steady_clock::time_point priorTp;
    cv::Matx33d prevR;
    bool usePrev = false;
    if (opt.useTemporalPrior)
    {
        std::lock_guard<std::mutex> lk(priorMtx);
        const auto age = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - priorTp).count();
        usePrev = havePrior && age <= opt.priorMaxAgeSec;
        prevR = priorR;
    }

    auto angleDeg = [](const cv::Matx33d &A, const cv::Matx33d &B)
    {
        const cv::Matx33d d = A.t() * B;
        const double tr = d(0, 0) + d(1, 1) + d(2, 2);
        return std::acos(std::max(-1.0, std::min(1.0, (tr - 1.0) * 0.5))) * 180.0 / CV_PI;
    };

    // ① 物理约束：只保留法向朝相机的解（若全都被剔除则不约束，交给误差）
    std::vector<Cand> pool;
    for (const auto &c : cands)
        if (!opt.requireNormalTowardCamera || c.normalOk)
            pool.push_back(c);
    if (pool.empty())
        pool = cands;

    // ② 误差排序
    std::sort(pool.begin(), pool.end(),
              [](const Cand &a, const Cand &b) { return a.err < b.err; });

    int best = pool.front().idx;
    double gap = -1.0;
    bool ambiguous = false;

    if (pool.size() >= 2)
    {
        gap = pool[1].err - pool[0].err;
        ambiguous = gap <= opt.solErrMarginPx;

        // ③ 误差打平时，用上一帧姿态决出（迟滞，防在两簇之间来回跳）
        if (ambiguous && usePrev)
        {
            int pick = pool.front().idx;
            double bestAng = angleDeg(prevR, pool.front().R);
            for (size_t i = 1; i < pool.size(); ++i)
            {
                const double a = angleDeg(prevR, pool[i].R);
                if (a < bestAng)
                {
                    bestAng = a;
                    pick = pool[i].idx;
                }
            }
            best = pick;
        }
    }

    cv::Mat rvec = rvecs[best];
    cv::Mat tvec = tvecs[best];
    if (opt.refineWithLM)
        cv::solvePnPRefineLM(data.points3D, points2D, K, D, rvec, tvec);

    cv::Mat R;
    cv::Rodrigues(rvec, R);
    cv::projectPoints(data.points3D, rvec, tvec, K, D, det.reproj);

    det.R_cam_from_armor = cv::Matx33d(R);
    det.t_armor_in_cam = cv::Vec3d(tvec.at<double>(0), tvec.at<double>(1),
                                   tvec.at<double>(2));

    det.errMeanPx = 0.0;
    for (size_t k = 0; k < det.reproj.size(); ++k)
        det.errMeanPx += cv::norm(det.reproj[k] - points2D[k]);
    det.errMeanPx /= static_cast<double>(det.reproj.size());

    // ---- 装甲板系 -> 车体系 ----
    //   P_car = R_car_from_cam * (R * P_armor + t) + t_cam_in_car
    // 展开后：
    //   R_car_from_armor = R_car_from_cam * R
    //   t_armor_in_car = R_car_from_cam * t + t_cam_in_car
    const cv::Matx33d R_car_from_armor = opt.R_car_from_cam * det.R_cam_from_armor;
    const cv::Vec3d t_armor_in_car =
        cv::Vec3d(opt.R_car_from_cam * det.t_armor_in_cam) + opt.t_cam_in_car;

    // ★ 目标点：点要平移
    const cv::Vec3d pointArmor(data.m_point.x, data.m_point.y, data.m_point.z);
    out.point = cv::Vec3d(R_car_from_armor * pointArmor) + t_armor_in_car;

    // ★ 目标向量：自由向量，只旋转不平移
    const cv::Vec3d vecArmor(data.v[0], data.v[1], data.v[2]);
    out.vec = cv::Vec3d(R_car_from_armor * vecArmor);

    out.R_car_from_armor = R_car_from_armor;
    out.solGapPx = gap;
    out.solAmbiguous = ambiguous;
    // ★ 目标系：X 目标 = -X 装甲板，Y 目标 = -Z 装甲板，Z 目标 = -Y 装甲板
    out.R_car_from_target = R_car_from_armor * opt.R_armor_from_target;

    out.errMeanPx = det.errMeanPx;
    out.ok = true;

    // 更新"上一帧姿态"，供下一次选解使用
    if (opt.useTemporalPrior)
    {
        std::lock_guard<std::mutex> lk(priorMtx);
        priorR = det.R_cam_from_armor;
        havePrior = true;
        priorTp = std::chrono::steady_clock::now();
    }
    return out;
}

// ============================================================================
//  旋转矩阵 -> ZYX 欧拉角（R = Rz(yaw) * Ry(pitch) * Rx(roll)）
// ============================================================================
cv::Vec3d rpyFromR(const cv::Matx33d &R)
{
    // pitch 只能取 [-90°, 90°]，R(2,0) = -sin(pitch)
    double sp = -R(2, 0);
    sp = std::min(1.0, std::max(-1.0, sp));
    const double pitch = std::asin(sp);

    double roll = 0.0, yaw = 0.0;
    if (std::abs(sp) > 1.0 - 1e-9)
    {
        // 万向锁：pitch = ±90°，roll 与 yaw 只有和/差有意义，这里令 roll = 0
        roll = 0.0;
        yaw = std::atan2(-R(0, 1), R(1, 1));
    }
    else
    {
        roll = std::atan2(R(2, 1), R(2, 2));
        yaw = std::atan2(R(1, 0), R(0, 0));
    }
    return cv::Vec3d(roll, pitch, yaw);
}

} // namespace

// ============================================================================
//  对外接口
// ============================================================================

bool detectCorners(const cv::Mat &bgrFrame, std::vector<cv::Point2f> &points2D)
{
    return detectImpl(bgrFrame, points2D, nullptr);
}

bool detectCornersDebug(const cv::Mat &bgrFrame, DetectDebug &dbg)
{
    std::string reason;
    const bool ok = detectImpl(bgrFrame, dbg.points2D, &reason, &dbg);
    if (!ok)
        dbg.reason = reason;
    return ok;
}

int binaryDiffThreshold()
{
    return gDiffThreshold.load(std::memory_order_relaxed);
}

void setBinaryDiffThreshold(int diffThreshold)
{
    gDiffThreshold.store(std::clamp(diffThreshold, 0, 255), std::memory_order_relaxed);
}

CarTarget solveCarTargetFromCorners(const std::vector<cv::Point2f> &points2D,
                                    const PnPDate &data,
                                    const CarTargetOptions &opt)
{
    return poseImpl(points2D, data, opt);
}

void drawFrameOverlay(cv::Mat &bgrFrame, const PnPDate &data, const CarTarget &target,
                      const CarTargetOptions &opt, double targetAxisLenMm, int gizmoSize,
                      const ExtraFrame &extra)
{
    if (bgrFrame.empty())
        return;

    const cv::Matx33d K = data.cameraMatrix;
    const std::vector<double> &D = data.distCoeffs;

    // 车体系 -> 相机系
    const cv::Matx33d R_cam_from_car = opt.R_car_from_cam.t();
    const cv::Vec3d t_car_in_cam = -(R_cam_from_car * opt.t_cam_in_car);

    // ---- ① base_link 方位指示器：放右下角 ----
    // base_link 原点在相机上方几百 mm、前方只有几百 mm，投影落在画面外，
    // 所以这里不画它"在哪里"，只画"三根轴朝哪"（正交投影 = 从相机看过去的样子）。
    const cv::Point anchor(bgrFrame.cols - gizmoSize - 70, bgrFrame.rows - gizmoSize - 45);
    drawOrientationGizmo(bgrFrame, R_cam_from_car, anchor, gizmoSize, "base_link");

    // ---- ② 目标点处的目标系三轴（真实透视投影，能直接看出方位对不对）----
    if (!target.ok)
        return;

    const cv::Vec3d p_car(target.point[0], target.point[1], target.point[2]);
    const cv::Vec3d p_cam = R_cam_from_car * p_car + t_car_in_cam;
    const cv::Matx33d R_cam_from_target = R_cam_from_car * target.R_car_from_target;

    // 装甲板原点（目标点减去装甲板系下的偏置，反算回来）
    const cv::Vec3d mPoint(data.m_point.x, data.m_point.y, data.m_point.z);
    const cv::Vec3d armorOrigin_car =
        cv::Vec3d(target.point[0], target.point[1], target.point[2]) -
        cv::Vec3d(target.R_car_from_armor * mPoint);
    const cv::Vec3d armorOrigin_cam = R_cam_from_car * armorOrigin_car + t_car_in_cam;

    std::vector<cv::Point3f> two{
        cv::Point3f((float)armorOrigin_cam[0], (float)armorOrigin_cam[1], (float)armorOrigin_cam[2]),
        cv::Point3f((float)p_cam[0], (float)p_cam[1], (float)p_cam[2])};
    std::vector<cv::Point2f> uvo;
    cv::projectPoints(two, cv::Vec3d(0, 0, 0), cv::Vec3d(0, 0, 0), cv::Mat(K), D, uvo);

    // 装甲板原点十字 + 到目标点的连线（虚线），方便看偏置是往哪个方向甩出去的
    const int cross = 8;
    cv::line(bgrFrame, uvo[0] - cv::Point2f(cross, 0), uvo[0] + cv::Point2f(cross, 0),
             cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
    cv::line(bgrFrame, uvo[0] - cv::Point2f(0, cross), uvo[0] + cv::Point2f(0, cross),
             cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
    // 装甲板系的名字不写在原点上（会和近轴向轴的 in/out 标签撞），
    // 靠左下角图例的 "solid = target frame / dashed = armor frame" 区分
    for (int k = 0; k < 12; k += 2)   // 虚线
    {
        const cv::Point2f a = uvo[0] + (uvo[1] - uvo[0]) * (k / 12.f);
        const cv::Point2f b = uvo[0] + (uvo[1] - uvo[0]) * ((k + 1) / 12.f);
        cv::line(bgrFrame, a, b, cv::Scalar(0, 255, 255), 1, cv::LINE_AA);
    }
    // 装甲板系三轴：虚线 + 比目标系更长（两系原点才差 160mm，等长会完全重叠），
    // 也不标逐轴字母，只用一个带描边的名字说明"这一组虚线是装甲板系"
    const cv::Matx33d R_cam_from_armor = R_cam_from_car * target.R_car_from_armor;
    drawAxesAtPoint(bgrFrame, K, D,
                    cv::Point3d(armorOrigin_cam[0], armorOrigin_cam[1], armorOrigin_cam[2]),
                    R_cam_from_armor, targetAxisLenMm * 1.7, 2, 0.45, false, true);

    // 目标系三轴：粗线，画在装甲板系之上
    drawAxesAtPoint(bgrFrame, K, D, cv::Point3d(p_cam[0], p_cam[1], p_cam[2]),
                    R_cam_from_target, targetAxisLenMm, 2, 0.6);

    // ---- ④ 末端系（带黑描边），并显示它和目标系的差值 ----
    // 用途：把"视觉想让它摆成什么样（目标系）"和"它现在实际是什么样（末端系）"
    // 放在同一张图上，姿态差多少一眼能看出来。
    double dPosMm = -1.0, dAngDeg = -1.0;
    if (extra.ok)
    {
        // 差值先算：即使末端原点跑到画面外（或相机后方），这两个数依然有参考价值
        {
            const cv::Vec3d d = extra.t_frame_in_car -
                                cv::Vec3d(target.point[0], target.point[1], target.point[2]);
            dPosMm = std::sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
            // 姿态夹角：两个系之间的最小旋转角（欧拉角不能逐轴相减，只能这样算）
            const cv::Matx33d dR = extra.R_car_from_frame.t() * target.R_car_from_target;
            const double tr = dR(0, 0) + dR(1, 1) + dR(2, 2);
            dAngDeg = std::acos(std::max(-1.0, std::min(1.0, (tr - 1.0) * 0.5))) * 180.0 / CV_PI;
        }

        const cv::Matx33d R_cam_from_extra = R_cam_from_car * extra.R_car_from_frame;
        const cv::Vec3d e_cam = R_cam_from_car * extra.t_frame_in_car + t_car_in_cam;
        if (e_cam[2] > 1.0)   // 原点在相机后方就不画三轴（投影会翻）
        {
            drawAxesAtPoint(bgrFrame, K, D, cv::Point3d(e_cam[0], e_cam[1], e_cam[2]),
                            R_cam_from_extra, targetAxisLenMm * 1.25, 4, 0.5,
                            false, false, true);

            // 末端原点到目标点的连线，直观看出还差多远
            std::vector<cv::Point3f> seg{
                cv::Point3f((float)e_cam[0], (float)e_cam[1], (float)e_cam[2]),
                cv::Point3f((float)p_cam[0], (float)p_cam[1], (float)p_cam[2])};
            std::vector<cv::Point2f> segUV;
            cv::projectPoints(seg, cv::Vec3d(0, 0, 0), cv::Vec3d(0, 0, 0), cv::Mat(K), D, segUV);
            for (int k = 0; k < 16; k += 2)   // 白色虚线
            {
                const cv::Point2f a = segUV[0] + (segUV[1] - segUV[0]) * (k / 16.f);
                const cv::Point2f b = segUV[0] + (segUV[1] - segUV[0]) * ((k + 1) / 16.f);
                cv::line(bgrFrame, a, b, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
            }

            // 标签用半透明底板，压在箭头上也读得清
            drawTextPanel(bgrFrame, {extra.label + " O"}, cv::Point(cvRound(segUV[0].x) + 10,
                          cvRound(segUV[0].y) + 18), 0.5, 22, cv::Scalar(255, 255, 255));
        }
    }

    // 偏置量放到左下角固定位置，避免和装甲板 +Y 轴（朝图像下方）的箭头撞在一起
    char off[160];
    std::snprintf(off, sizeof(off), "offset in armor frame: (%.1f, %.1f, %.1f) mm",
                  mPoint[0], mPoint[1], mPoint[2]);
    cv::putText(bgrFrame, off, cv::Point(10, bgrFrame.rows - 34),
                cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(0, 255, 255), 1, cv::LINE_AA);

    // ---- ③ 左上角：实时欧拉角 + 与上一帧的变化量 ----
    {
        const cv::Vec3d rpyRad = rpyFromR(target.R_car_from_target);
        const double k = 180.0 / CV_PI;
        const double roll = rpyRad[0] * k, pitch = rpyRad[1] * k, yaw = rpyRad[2] * k;

        // 与上一帧比较：帧间抖动/漂移一眼可见；超过 1s 未更新或上次失败则不算变化量
        static cv::Vec3d prev;
        static bool havePrev = false;
        static auto prevTp = std::chrono::steady_clock::now();
        const auto nowTp = std::chrono::steady_clock::now();
        const bool fresh = havePrev &&
                           (nowTp - prevTp) < std::chrono::milliseconds(1000);
        const cv::Vec3d d = fresh ? cv::Vec3d(roll - prev[0], pitch - prev[1], yaw - prev[2])
                                  : cv::Vec3d(0, 0, 0);

        char l0[128], l1[128], l2[128];
        std::snprintf(l0, sizeof(l0), "roll  %+8.3f deg", roll);
        std::snprintf(l1, sizeof(l1), "pitch %+8.3f deg", pitch);
        std::snprintf(l2, sizeof(l2), "yaw   %+8.3f deg", yaw);
        char l3[128], l4[128];
        std::snprintf(l3, sizeof(l3), "d/frame %+6.2f %+6.2f %+6.2f",
                      d[0], d[1], d[2]);
        std::snprintf(l4, sizeof(l4), "err   %.2f px", target.errMeanPx);

        drawTextPanel(bgrFrame, {l0, l1, l2, l3, l4}, cv::Point(10, 26), 0.62, 26,
                      cv::Scalar(0, 255, 255));

        // 双解诊断：gap 小 = 两解难分，此时姿态最容易在两簇之间跳
        char l5[128];
        if (target.solGapPx < 0.0)
            std::snprintf(l5, sizeof(l5), "sol   single solution");
        else
            std::snprintf(l5, sizeof(l5), "sol   gap %.2f px  %s", target.solGapPx,
                          target.solAmbiguous ? "AMBIGUOUS(temporal)" : "clear");
        const cv::Scalar solColor = target.solAmbiguous ? cv::Scalar(0, 0, 255)
                                                       : cv::Scalar(0, 255, 0);
        drawTextPanel(bgrFrame, {l5}, cv::Point(10, 26 + 26 * 5), 0.62, 26, solColor);

        // 末端系 vs 目标系：位置差(mm) + 姿态夹角(deg)，小于阈值显示绿、否则黄
        char l6[160];
        if (dAngDeg >= 0.0)
        {
            std::snprintf(l6, sizeof(l6), "%s vs target: dpos %.1f mm  dang %.2f deg",
                          extra.label.c_str(), dPosMm, dAngDeg);
            const cv::Scalar c6 = (dAngDeg < 5.0 && dPosMm < 20.0) ? cv::Scalar(0, 255, 0)
                                                                  : cv::Scalar(0, 220, 255);
            drawTextPanel(bgrFrame, {l6}, cv::Point(10, 26 + 26 * 6), 0.62, 26, c6);
        }

        prev = cv::Vec3d(roll, pitch, yaw);
        havePrev = true;
        prevTp = nowTp;
    }

    // 图例：说明 in/out 与两种线宽的含义（Hershey 字体不能写中文，这里用英文）
    cv::putText(bgrFrame, "axis: X=red  Y=green  Z=blue   'in' = into screen (away)   'out' = toward viewer",
                cv::Point(10, bgrFrame.rows - 12), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                cv::Scalar(230, 230, 230), 1, cv::LINE_AA);
    cv::putText(bgrFrame, "solid = target frame    dashed = armor frame",
                cv::Point(10, bgrFrame.rows - 60), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                cv::Scalar(230, 230, 230), 1, cv::LINE_AA);

    // 目标点名 + 车系坐标：放到左下角，不再压在目标系三轴上
    char info[160];
    std::snprintf(info, sizeof(info), "target frame in car: (%.0f, %.0f, %.0f) mm",
                  p_car[0], p_car[1], p_car[2]);
    cv::putText(bgrFrame, info, cv::Point(10, bgrFrame.rows - 78),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
}

// ============================================================================
//  CarTargetItem / CarTargetPose：两级索引 + 姿态换算 + 6 自由度封包
// ============================================================================

// 内层下标：位姿取 0..5，向量取 0..2
double CarTargetItem::operator[](int k) const
{
    CV_Assert(k >= 0 && k < n);
    return n == 6 ? pose[k] : vec[k];
}

// cv::Vec6d p = r[0];
CarTargetItem::operator cv::Vec6d() const
{
    CV_Assert(n == 6);
    return pose;
}

// cv::Vec3d v = r[1];
CarTargetItem::operator cv::Vec3d() const
{
    CV_Assert(n == 3);
    return vec;
}

// 外层只用两个索引：0 = 六自由度位姿，1 = 三自由度向量
CarTargetItem CarTargetPose::operator[](int i) const
{
    CV_Assert(i == 0 || i == 1);

    CarTargetItem item;
    if (i == 0)
    {
        item.pose = pose;
        item.n = 6;
    }
    else
    {
        item.vec = vec;
        item.n = 3;
    }
    return item;
}

cv::Vec3d CarTargetPose::rpyDeg() const
{
    const double kRad2Deg = 180.0 / CV_PI;
    return cv::Vec3d(pose[3] * kRad2Deg, pose[4] * kRad2Deg,
                     pose[5] * kRad2Deg);
}

CarTargetPose toCarTargetPose(const CarTarget &t)
{
    CarTargetPose out;
    out.ok = t.ok;
    out.reason = t.reason;
    out.errMeanPx = t.errMeanPx;
    out.vec = t.vec;
    out.solGapPx = t.solGapPx;
    out.solAmbiguous = t.solAmbiguous;
    out.R = t.R_car_from_target;
    if (!t.ok)
        return out;

    // 目标系 = 装甲板系右乘 R_armor_from_target：X 取反，Y/Z 互换后取反
    out.R = t.R_car_from_target;

    // 平移部分：目标系原点就是目标点本身
    out.pose[0] = t.point[0];
    out.pose[1] = t.point[1];
    out.pose[2] = t.point[2];

    const cv::Vec3d rpy = rpyFromR(out.R);
    out.pose[3] = rpy[0];
    out.pose[4] = rpy[1];
    out.pose[5] = rpy[2];

    return out;
}

CarTargetPose solveCarTargetPose(const cv::Mat &bgrFrame, const PnPDate &data,
                                 const CarTargetOptions &opt)
{
    std::vector<cv::Point2f> points2D;
    std::string reason;
    if (!detectImpl(bgrFrame, points2D, &reason))
    {
        CarTargetPose out;
        out.reason = reason;
        return out;
    }
    return toCarTargetPose(poseImpl(points2D, data, opt));
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
    const cv::Matx33d R = det.R_cam_from_armor;

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
