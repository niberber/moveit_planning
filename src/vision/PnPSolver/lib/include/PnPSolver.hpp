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
//      r[1]        // ★ 目标向量 1（= PnPDate::v1，车体系）
//      r[2]        // ★ 目标向量 2（= PnPDate::v2，车体系）
//
//  注：v1 是后来插到中间的新量；【原来的】目标向量（旧 PnPDate::v）现在是 r[2]，
//      按老代码语义调用第二返回值的调用方要改成取 r[2]。
//
//      r[0][0..2]  // 目标点在车体系下的坐标 (mm)
//      r[0][3..5]  // 姿态 roll / pitch / yaw (rad)
//      r[1][0..2]  // 目标向量 1 在车体系下的表示
//      r[2][0..2]  // 目标向量 2 在车体系下的表示
//
//      cv::Vec6d p6 = r[0];     // 也可以直接转成定长向量
//      cv::Vec3d p3 = r[1];
//      cv::Vec3d p3b = r[2];
//      r.rpyDeg();              // 姿态换成角度 (deg)
//
//  ★ 本帧解无效（机械臂遮挡、光条没凑齐、重投影误差/内点数不达标）时：
//      r.ok       仍为 true（调用方照常可用，规划不会因此中断）
//      r.held     true，表示 r[0] / r[1] / r[2] 全部取自最近一次有效解
//      r.heldAgeSec  那份有效解距今多少秒（只作诊断，不限制沿用）
//      r.stampSec    仍是【当前帧】的时间戳
//    沿用没有时间上限：一直用到出现新的有效解，或调用 resetLastValidPose()；
//    只有在"从来没有过有效解"时才退回 r.ok == false。
//
//  目标系（"目标坐标系"）定义：
//      原点 = 目标点（即 CarTarget::point，不是装甲板原点）
//      三个轴以装甲板系为参考（B = R_armor_from_target，见 CarTargetOptions）：
//          X 轴 = -装甲板系 X 轴
//          Y 轴 = -装甲板系 Z 轴
//          Z 轴 = -装甲板系 Y 轴
//    也就是装甲板的 Y/Z 互换后都取反，X 也取反；等价于绕装甲板系
//    (0, 1, -1)/√2 轴转 180°（det = +1，仍是右手系）。
//    合成关系：R_car_from_target = R_car_from_armor * R_armor_from_target
//    原点不变，因此目标朝向相对装甲板整体翻转 180°。
//
//  ★ 旋转矩阵统一命名规则：R_<输出系>_from_<输入系>
//      语义：把【输入系】下的向量变到【输出系】下。左乘作用，可链式相乘，
//            相邻的中间系会抵消：
//                R_car_from_target = R_car_from_armor * R_armor_from_target
//                                    └── 输出 car ──┘ └ 中间 armor 抵消 ┘
//      三列含义：R_A_from_B 的三列 = B 系三轴在 A 系下的方向。
//      上一版用的是 <来源>_to_<去向>（如 R_armor_to_car），前缀一会儿是来源、
//      一会儿是去向，在同一行里混用极易写反，已全部统一到本规则。
// ============================================================================
#pragma once

#include <opencv2/opencv.hpp>
#include <array>
#include <string>
#include <vector>

#include "PnPDate.hpp"

// ----------------------------------------------------------------------------
//  解算结果
// ----------------------------------------------------------------------------
struct CarTarget
{
    cv::Vec3d point = cv::Vec3d(0, 0, 0); // ★ 目标点     (mm, 车体系)
    cv::Vec3d vec1 = cv::Vec3d(0, 0, 0);  // ★ 目标向量 1 (车体系) = R_car_from_armor * PnPDate::v1
    cv::Vec3d vec2 = cv::Vec3d(0, 0, 0);  // ★ 目标向量 2 (车体系) = R_car_from_armor * PnPDate::v2

    // 目标点在装甲板系下的坐标：p_target_armor（即 PnPDate::m_point）
    // 目标点（目标系原点）在车体系下：
    //   point = R_car_from_armor * p_target_armor + t_armor_in_car
    // 装甲板系 -> 车体系 的旋转矩阵（列 = 装甲板三轴在车体系下的方向）
    cv::Matx33d R_car_from_armor = cv::Matx33d::eye();

    // 目标系 -> 车体系 的旋转矩阵（列 = 目标系三轴在车体系下的方向）
    //   R_car_from_target = R_car_from_armor * R_armor_from_target
    //   R_armor_from_target = [[-1,0,0],[0,0,-1],[0,-1,0]]：
    //   X 目标 = -X 装甲板，Y 目标 = -Z 装甲板，Z 目标 = -Y 装甲板
    cv::Matx33d R_car_from_target = cv::Matx33d::eye();

    double errMeanPx = -1.0; // 本帧反投影误差 (px)，未解出时为 -1
    bool ok = false;         // 是否解算成功
    std::string reason;      // ok == false 的原因

    // ---- IPPE 选解诊断 ----
    double solGapPx = -1.0;  // 最优与次优解的重投影误差差 (px)；只有一个解时为 -1
    bool solAmbiguous = false; // 两解误差落在 solErrMarginPx 之内（靠连续性判据决出）

    // ---- 有效性判定 ----
    int inliers = 0; // 内点数 0..8：单点重投影误差 <= inlierErrPx 记 1 个
};

// ----------------------------------------------------------------------------
//  可选参数，用默认值即可
// ----------------------------------------------------------------------------
struct CarTargetOptions
{
    // 相机系 -> 车体系（左乘在装甲板位姿上）
    // 车体系即 base_link，轴向：x 右、y 前、z 上（右手系，由 URDF 实测确认）
    // 按你的定义：
    //   cam_x -> car_x
    //   cam_y -> -car_z
    //   cam_z -> car_y
    // 因此本矩阵的三列 = 相机三个轴在车系下的方向：
    //   cam_x（图像向右） -> ( 1, 0, 0)  即车体 +x = 右
    //   cam_y（图像向下） -> ( 0, 0,-1)  即车体 -z = 下
    //   cam_z（光轴）     -> ( 0, 1, 0)  即车体 +y = 前 ← 光轴朝前
    // 这版矩阵满足你当前定义的右手系要求。
    cv::Matx33d R_car_from_cam = cv::Matx33d( 1, 0,  0,
                                               0, 0,  1,
                                               0, -1, 0);
    // 车体原点 -> 相机光心 (mm)，顺序与车体系 (x, y, z) 一致：
    // 右 72.83 / 后 492.4 / 上 726.45（y 轴朝前，取负表示光心在原点后方）
    cv::Vec3d t_cam_in_car = cv::Vec3d(-72.83, -492.4, 726.45);

    // 目标系 -> 装甲板系 的固定旋转（列 = 目标三轴在装甲板系下的方向）：
    //   X 目标 = -X 装甲板
    //   Y 目标 = -Z 装甲板
    //   Z 目标 = -Y 装甲板
    // 即 X 取反、Y/Z 互换后取反；等价于绕装甲板系 (0, 1, -1)/√2 轴转 180°，
    // det = +1，仍是右手系。
    cv::Matx33d R_armor_from_target = cv::Matx33d(-1,  0,  0,
                                                   0,  0, -1,
                                                   0, -1,  0);

    // ---- IPPE 平面二义性的选解约束 ----
    // IPPE 对共面点必然给出 2 个解，实测两解姿态差 27~30°，而重投影误差只差 0.3~0.5 px。
    // 只按"平均重投影误差最小"挑，余量太薄，噪声一变就可能翻解（姿态瞬间跳几十度）。
    // 规则：误差差距超过 solErrMarginPx 时按误差取（清晰胜出）；
    //       差距在 margin 之内视为"打平"，此时用上一帧姿态做连续性判据（迟滞），
    //       避免在两簇之间来回跳。
    bool useTemporalPrior = true;   // 打平时用上一帧姿态选解（有解就一直沿用）
    double solErrMarginPx = 1.0;    // 误差差多少 px 以内算"打平"

    // 物理约束：装甲板 +Z 是板法向，正面朝相机时其相机系 z 分量必须 < 0。
    // 只用来剔除"板翻到背面"这种物理上不可能的解；两个解都朝相机时不起作用。
    bool requireNormalTowardCamera = true;

    bool refineWithLM = false; // IPPE 之后再做一步 LM 精修

    // ---- 有效性判定：只看位姿的几何合理性（深度/距离） ----
    // 不用重投影误差/内点数判失效：误差偏大的帧照样用本帧解；
    // 真正解不出（检测失败 / IPPE 无解）时 ok = false，由 applyHold 沿用最近一次有效解。
    bool checkValidity = true;
    double inlierErrPx = 3.0;    // 单点误差小于它算内点（仅统计，不参与判失效）
    double minDepthMm = 200.0;   // 装甲板在相机前方的最短距离，防止解到相机背后
    double maxRangeMm = 8000.0;  // 目标点距离上限，防止解飞

    // ---- 本帧无效时保持最近一次有效解 ----
    // 沿用没有时间上限：一直用到出现新的有效解，或调用 resetLastValidPose()。
    // 换目标/换工位/重新标定时记得 reset，否则会拿上一工位的位姿顶上。
    bool holdLastValidPose = true;
};

// ----------------------------------------------------------------------------
//  ★ 一帧的完整结果：6 自由度位姿 + 两个目标向量
//
//  为什么不用 std::vector<double> 装？
//    1) 9 个以上数的含义只能靠下标记，容易错位；
//    2) 长度可变，编译器帮不上忙；
//    3) 位姿本身是"3 平移 + 3 姿态"，向量是"3 分量"，分组信息会丢。
//  => 用结构体 + 一个"项"视图：r[0] 是六自由度，r[1] / r[2] 是三自由度向量，
//     这个项既能再下标（r[0][3]），也能隐式转成 cv::Vec6d / cv::Vec3d。
// ----------------------------------------------------------------------------

// r[0] / r[1] / r[2] 取出来的东西（一个 6 维或 3 维的小数组）
struct CarTargetItem
{
    cv::Vec6d pose = cv::Vec6d(0, 0, 0, 0, 0, 0); // 仅当 n == 6 时有效
    cv::Vec3d vec = cv::Vec3d(0, 0, 0);           // 仅当 n == 3 时有效（向量 1 或 2）
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
    // 目标系原点 = 目标点，姿态 = 装甲板姿态 · R_armor_from_target：
    //   R_car_from_target = R_car_from_armor * [[-1,0,0],[0,0,-1],[0,-1,0]]
    // 此处的 pose 记录目标系原点在车体系下的位置，R 则记录目标系在车体系下的定向。
    cv::Vec6d pose = cv::Vec6d(0, 0, 0, 0, 0, 0);

    cv::Vec3d vec1 = cv::Vec3d(0, 0, 0); // ★ 目标向量 1（车体系）
    cv::Vec3d vec2 = cv::Vec3d(0, 0, 0); // ★ 目标向量 2（车体系）

    // 目标系 -> 车体系 的旋转矩阵，与 pose[3..5] 一定是同一个姿态
    //   R_car_from_target = R_car_from_armor * [[-1,0,0],[0,0,-1],[0,-1,0]]
    cv::Matx33d R = cv::Matx33d::eye();

    double errMeanPx = -1.0; // 本帧反投影误差 (px)，未解出时为 -1
    bool ok = false;         // 是否解算成功
    std::string reason;      // ok == false 的原因

    double solGapPx = -1.0;    // 最优/次优解重投影误差差 (px)，透传自 CarTarget
    bool solAmbiguous = false; // 本帧两解是否"打平"

    // ★ 有效性判定 / 保持上一帧 ----
    int inliers = 0;          // 本帧内点数 0..8（透传自 CarTarget）
    // ★ true = 本帧无效，pose / vec1 / vec2 / R 全部取自最近一次有效解
    //   此时 ok 仍为 true（调用方照常可用），但要靠 held 区分新旧。
    bool held = false;
    double heldAgeSec = 0.0;  // 沿用的那份有效解距今多久 (s)；held == false 时为 0
    double stampSec = -1.0;   // 本帧时间戳 (s)；held 时也是"当前帧"的时间戳

    // ★ 三级索引（只有三个索引号）：
    //     r[0]  -> 六自由度   [x, y, z, roll, pitch, yaw]
    //     r[1]  -> 目标向量 1 [x, y, z]（= PnPDate::v1）
    //     r[2]  -> 目标向量 2 [x, y, z]（= PnPDate::v2）
    //   再往下一层：
    //     r[0][0]  -> x (mm)       r[1][0] -> vec1.x     r[2][0] -> vec2.x
    //     r[0][3]  -> roll (rad)   r[1][2] -> vec1.z     r[2][2] -> vec2.z
    //   也可以直接赋给 cv::Vec6d / cv::Vec3d：
    //     cv::Vec6d p = r[0];      cv::Vec3d v1 = r[1];  cv::Vec3d v2 = r[2];
    //   索引不是 0/1/2，或内层越界，都会触发 CV_Assert
    CarTargetItem operator[](int i) const;

    cv::Vec3d rpyDeg() const; // 姿态换成角度，方便打印
};

// ----------------------------------------------------------------------------
//  ★ 主函数：一帧图像 -> 6 自由度位姿 + 目标向量
// ----------------------------------------------------------------------------
//  stampSec：本帧时间戳 (s)，传 < 0 表示用库内时钟（system_clock）。
//  本帧无效时返回最近一次有效解的位姿/向量，held = true，stampSec 仍是本帧时间戳。
CarTargetPose solveCarTargetPose(const cv::Mat &bgrFrame, const PnPDate &data,
                                 const CarTargetOptions &opt = CarTargetOptions(),
                                 double stampSec = -1.0);

// 清掉"最近一次有效解"缓存（换目标、重新标定、开始新一次定位时调用）
void resetLastValidPose();

// 查缓存里还有什么（ok == false 且 reason 说明为空，表示还没有有效解）
CarTargetPose lastValidPose();

// 由旧的 CarTarget 结果转换（内部就是"补一个 R 再转欧拉角"，不重新解算）
CarTargetPose toCarTargetPose(const CarTarget &t);

// ----------------------------------------------------------------------------

//  分步接口（一般用不到）
// ----------------------------------------------------------------------------

// 图像 -> 8 个角点。顺序：左条 左上→左下→右下→右上，右条同样（屏幕逆时针）
bool detectCorners(const cv::Mat &bgrFrame, std::vector<cv::Point2f> &points2D);

// ----------------------------------------------------------------------------
//  ★ 调参接口：红蓝通道相减二值化的差分阈值（运行时可用滑条实时改）
//
//      mask = (R - B) > 差分阈值
//
//  默认值即库里的固定常量（差分 70）；滑条调好后把结果写回默认值即可。
//  线程安全：相机线程每帧读一次，滑条回调在 GUI 线程写，互不阻塞。
// ----------------------------------------------------------------------------
int binaryDiffThreshold();                    // 当前 R-B 差分阈值（0~255）
void setBinaryDiffThreshold(int diffThreshold); // 入参自动夹到 0~255

// ----------------------------------------------------------------------------
//  一次检测的全部中间量，给调参/可视化用（生产路径不产生任何额外开销）
// ----------------------------------------------------------------------------
struct DetectDebug
{
    cv::Mat mask;                                 // 红蓝差分二值图（CV_8UC1）
    std::vector<std::vector<cv::Point>> contours; // 二值图上的外轮廓（与检测内部同一份）
    std::vector<cv::Point2f> points2D;            // 8 个角点，顺序同 points3D
    std::vector<std::array<cv::Point2f, 4>> bars; // 每根光条"梯形拟合"出的 4 个角点
    bool ok = false;                              // 是否凑齐 2 根光条并给出 8 个角点
    std::string reason;                           // ok == false 的原因
};

// 图像 -> 8 个角点，并把二值图 / 轮廓 / 梯形拟合结果一并带出来
bool detectCornersDebug(const cv::Mat &bgrFrame, DetectDebug &dbg);

// ----------------------------------------------------------------------------
//  ★ 调试：在图像上画两个坐标系，用来肉眼核对 xyz 方向
//
//    ① base_link：画在画面【右下角】的方位指示器。
//       base_link 原点在相机上方几百 mm、前方仅几百 mm，投影落在画面外，
//       所以用正交投影画"三根轴朝哪"（等价于站在相机位置看过去）。
//       轴基本垂直纸面时画成圆点：实心=朝向观察者，空心=背离观察者。
//    ② 目标系：画在【识别出的目标点】上，走真实透视投影，轴长 targetAxisLenMm。
//
//    ③ 装甲板系：虚线，画在装甲板原点
//    ④ 末端系（可选）：给 ok=true 才画，带黑描边，并额外显示它和目标系的
//       位置差 / 姿态夹角，用来直接对比"视觉解算的目标姿态"和"末端实际姿态"。
//
//  颜色固定：X=红、Y=绿、Z=蓝。target.ok == false 时只画 ①。
// ----------------------------------------------------------------------------
// 额外要叠加到画面上的一个坐标系（典型用法：末端 TCP）
struct ExtraFrame
{
    bool ok = false;                                  // false = 不画
    cv::Matx33d R_car_from_frame = cv::Matx33d::eye(); // 该系在车系下的姿态（列 = 三轴在车系下的方向）
    cv::Vec3d t_frame_in_car = cv::Vec3d(0, 0, 0);     // 该系原点在车系下的坐标 (mm)
    std::string label = "tcp";                         // 原点旁的标签文字
};

void drawFrameOverlay(cv::Mat &bgrFrame, const PnPDate &data, const CarTarget &target,
                      const CarTargetOptions &opt = CarTargetOptions(),
                      double targetAxisLenMm = 100.0, int gizmoSize = 55,
                      const ExtraFrame &extra = ExtraFrame());

// 8 个角点 -> 目标点 + 目标向量
CarTarget solveCarTargetFromCorners(const std::vector<cv::Point2f> &points2D,
                                    const PnPDate &data,
                                    const CarTargetOptions &opt = CarTargetOptions());

// ----------------------------------------------------------------------------
//  调试工具：累计多帧误差，退出时给出 points3D 调整建议
//
//      PnPTuner tuner;
//      while (cap.read(frame)) {
//          const CarTarget t = tuner.add(frame, data);   // 等价 solveCarTargetPose
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
