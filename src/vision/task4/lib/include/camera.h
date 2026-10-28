#pragma once
#include "MvCameraControl.h"
#include <string>
#include <opencv2/opencv.hpp>
#include <atomic>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <thread>

class CameraControl
{
private:
    // 所有对象共享的设备列表
    static MV_CC_DEVICE_INFO_LIST m_stDeviceList;
    // 相机句柄
    void *m_handle = nullptr;
    // 输出帧数据
    MV_FRAME_OUT stOutFrame{};
    // 上一次取帧的错误码，只在错误码变化时打印，避免断线时刷屏
    int m_lastGrabErr = 0;
public:
    //用于神经网络推理的图像，以及后续展示图像（RGB 三通道）
    cv::Mat m_rgb;

    // 禁止拷贝构造和赋值
    CameraControl(const CameraControl &) = delete;
    CameraControl &operator=(const CameraControl &) = delete;

    // 默认构造函数
    CameraControl();

    // SDK初始化
    static void sdk_initialize();

    // 枚举设备
    static void enumerate_devices();

    // 创建相机句柄,输入参数为设备索引号
    void create_camerahandle(int i);

    // 打开相机，如果打开失败，则销毁相机句柄
    void open_camera();
    // 设置曝光时间和增益
    void set_exposuregain(int exposureTime, int gain);

    // 关闭相机
    void close_camera();

    // 销毁相机句柄
    void destroy_camerahandle();

    // 缓存设置
    void set_buffer(int bufferNum);

    // 开始取流
    void start_streaming();

    // 采集一帧图像、展示并释放图像缓存
    // 返回 false 表示结束采集（按 ESC 或采集失败）
    // 按 Q/q 将当前帧保存为 images/ 目录下带时间戳的 png
    bool img_collection_free_buffer();

    // ★ 只取一帧并转成 RGB 三通道的 cv::Mat（不显示、不保存、不碰 GUI）
    //   成功返回 true；1s 超时、句柄无效、像素格式不支持都返回 false。
    //   出帧顺序严格是 RGB：送神经网络可直接用；而 PnPSolver::solveCarTargetPose()
    //   内部按 BGR 处理（COLOR_BGR2HSV），调用处要自己转一次 COLOR_RGB2BGR。
    //   这个函数是线程安全的用法前提：在后台线程里循环调用（见 CameraStream）。
    bool grab(cv::Mat &out);

    //停止取流
    void stop_stream();

    // 反初始化SDK
    static void sdk_finalize();
    // 析构函数，关闭相机并销毁句柄
    ~CameraControl();
    /**
     * @brief 一键运行相机采集程序（封装了完整流程，调用一次即可完成全部工作）
     *
     * 内部按顺序自动完成：
     *   1. SDK 初始化            MV_CC_Initialize
     *   2. 枚举设备              MV_CC_EnumDevices（GigE + USB）
     *   3. 创建相机句柄          使用 cameraIndex 指定的设备
     *   4. 打开相机
     *   5. 设置曝光时间和增益
     *   6. 设置图像缓存节点数
     *   7. 开始取流并循环采集    窗口实时显示画面
     *   8. 停止取流、关闭窗口、销毁句柄
     *   9. SDK 反初始化          MV_CC_Finalize
     *
     * 运行期间的操作：
     *   - 按 ESC 键：结束采集，函数返回 0
     *   - 按 Q/q 键：把当前帧保存到 images/ 目录（文件名带毫秒时间戳）
     *
     * 使用示例：
     * @code
     *   #include "camera.h"
     *
     *   int main()
     *   {
     *       // 相机0，曝光 8000us，增益 15dB，缓存节点数 2
     *       return CameraControl::run_capture(0, 8000, 15, 2);
     *   }
     * @endcode
     *
     * @param cameraIndex  相机序列索引（枚举到的设备序号，从 0 开始；0 表示第一台相机）
     * @param exposureTime 曝光时间，单位：微秒（us），必须大于 0
     * @param gain         信号放大增益，单位：dB，必须大于等于 0
     * @param bufferNum    图像缓存节点数，必须大于 0（常用 2~8，越大越不易丢帧）
     *
     * @return 0 表示正常结束（用户按 ESC 或采集结束后退出）；
     *         负值表示失败：
     *           -1 参数非法
     *           -2 未找到相机设备
     *           -3 cameraIndex 超出设备数量范围
     *           -4 创建句柄 / 打开相机 / 开始取流失败
     */
    static int run_capture(int cameraIndex = 0,
                           int exposureTime = 8000,
                           int gain = 15,
                           int bufferNum = 2);
};

// ============================================================================
//  CameraStream —— 后台取帧线程 + "最新一帧"缓存
//
//  为什么需要它：
//      相机取图是一个死循环（不停 GetImageBuffer），而解算/规划在同一进程里
//      要按自己的节奏跑。两者放一个线程里会互相拖住，所以拆成两个线程：
//        线程 A（本类内部）：只做 grab()，把最新帧塞进缓存；
//        线程 B（你主程序）：随时取缓存里的最新帧去解算。
//
//  用法（开相机 -> 起线程 -> 主线程解算 -> 收尾）：
//      CameraControl cam;
//      CameraControl::sdk_initialize();
//      CameraControl::enumerate_devices();
//      cam.create_camerahandle(0);          // 相机序列索引
//      cam.open_camera();
//      cam.set_exposuregain(8000, 15);      // 曝光(us) / 增益(dB)
//      cam.set_buffer(2);                   // 缓存节点数
//      cam.start_streaming();
//
//      CameraStream stream(cam);
//      stream.start();                      // ★ 相机线程开始不停取图
//
//      while (true)
//      {
//          cv::Mat frame;
//          if (stream.latest(frame))        // ★ 有新帧才进来，非阻塞
//          {
//              cv::Mat bgr;
//              cv::cvtColor(frame, bgr, cv::COLOR_RGB2BGR);  // ★ 出帧是 RGB
//              CarTargetPose r = solveCarTargetPose(bgr, data);
//              if (r.ok) { cv::Vec6d p6 = r[0]; cv::Vec3d p3 = r[1]; }
//              cv::imshow("PnP", bgr);      // imshow/waitKey 必须在主线程
//          }
//          if (cv::waitKey(1) == 27) break;
//      }
//
//      cam.stop_stream();                   // 先停流，让阻塞的 grab 立刻返回
//      stream.stop();                       // 再回收线程
//      CameraControl::sdk_finalize();
//
//  注意：
//    - 相机线程只负责取图；imshow / waitKey / 解算 / 规划都留在主线程。
//    - latest() 拿到的是 cv::Mat 的浅拷贝（只加引用计数）。之后相机线程覆盖
//      缓存里的 latest_ 不会影响你已经取走的那一帧，所以解算期间不需要加锁。
//    - 帧是 RGB；OpenCV 的 imshow/imwrite 和 PnPSolver 都按 BGR 解释，
//      用之前自己转一次 COLOR_RGB2BGR。
//    - 析构函数会自动 stop()，但 CameraControl 必须在 stream 之后销毁
//      （即先 stream.stop()，再让 cam 出作用域）。
// ============================================================================
class CameraStream
{
public:
    // cam 必须已经 open_camera() + start_streaming()；本类不负责开关相机
    explicit CameraStream(CameraControl &cam) : cam_(cam) {}
    ~CameraStream() { stop(); }

    CameraStream(const CameraStream &) = delete;
    CameraStream &operator=(const CameraStream &) = delete;

    // 启动取帧线程，重复调用无副作用
    void start();

    // 停止取帧线程并 join；重复调用无副作用。
    // 想立刻返回就先把 cam.stop_stream()，否则最多等一次 GetImageBuffer 超时(1s)
    void stop();

    // 非阻塞：有新帧就取走并返回 true；没有新帧返回 false
    bool latest(cv::Mat &out);

    // 阻塞：最多等 timeoutMs 毫秒，等到新帧返回 true
    bool wait(cv::Mat &out, int timeoutMs = 200);

    // ★ 每帧回调：在【相机线程】里调用，参数是最新帧（RGB）
    //   用途：在采集线程里直接做解算/统计，主线程只取结果。
    //   注意：
    //     1) 回调里不要做 GUI（imshow/waitKey）；
    //     2) 回调耗时会计入采集周期（5ms 回调会拖慢帧率）；
    //     3) 建议在 start() 之前调用本函数设置。
    void setCallback(std::function<void(const cv::Mat &)> cb);

    bool running() const { return running_.load(); } // 线程是否在跑
    unsigned long long frames() const { return frames_.load(); } // 累计收到帧数
    unsigned long long misses() const { return misses_.load(); } // 累计取帧失败次数

private:
    void loop(); // 线程体：不停 grab() 并更新缓存

    CameraControl &cam_;

    std::thread th_;
    std::atomic<bool> running_{false};
    std::atomic<unsigned long long> frames_{0};
    std::atomic<unsigned long long> misses_{0};

    std::mutex mtx_;
    std::condition_variable cv_;
    cv::Mat latest_;
    bool hasNew_ = false;
    std::function<void(const cv::Mat &)> cb_; // 每帧回调（在相机线程里执行）
};
