#include "camera.h"
#include <iostream>
#include <cstring>
#include "MvCameraControl.h"
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <string>
#include <filesystem>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <ctime>

MV_CC_DEVICE_INFO_LIST CameraControl::m_stDeviceList{};

namespace
{
    // 把图像保存到 dir 目录下，文件名带毫秒时间戳，返回保存后的路径；失败返回空串
    std::string save_image(const cv::Mat &image, const std::string &dir)
    {
        std::error_code ec;
        std::filesystem::create_directories(dir, ec);
        if (ec)
        {
            std::cerr << "Failed to create directory " << dir << ": " << ec.message() << std::endl;
            return {};
        }

        auto now = std::chrono::system_clock::now();
        std::time_t seconds = std::chrono::system_clock::to_time_t(now);
        auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(
                          now.time_since_epoch())
                          .count() %
                      1000;

        std::tm tm_buf{};
        localtime_r(&seconds, &tm_buf);

        std::ostringstream oss;
        oss << dir << "/image_"
            << std::put_time(&tm_buf, "%Y%m%d_%H%M%S") << "_"
            << std::setfill('0') << std::setw(3) << millis << ".png";

        const std::string path = oss.str();
        if (!cv::imwrite(path, image))
        {
            std::cerr << "Failed to save image: " << path << std::endl;
            return {};
        }
        return path;
    }

    // 把 SDK 输出的一帧转成 BGR 三通道的 cv::Mat（out 自己持有内存，
    // 调用方随后 FreeImageBuffer 也不会影响它）。
    // ★ 注意：Bayer RG 转出来必须是 BGR，不能是 RGB，
    //   PnPSolver 内部按 COLOR_BGR2HSV 做颜色阈值，通道一反就检不到光条。
    bool to_bgr(const MV_FRAME_OUT_INFO_EX &info, const void *data, cv::Mat &out)
    {
        const int w = static_cast<int>(info.nWidth);
        const int h = static_cast<int>(info.nHeight);
        if (w <= 0 || h <= 0 || data == nullptr)
            return false;

        auto src1 = [&] { return cv::Mat(h, w, CV_8UC1, const_cast<void *>(data)); };
        auto src3 = [&] { return cv::Mat(h, w, CV_8UC3, const_cast<void *>(data)); };

        switch (info.enPixelType)
        {
        case PixelType_Gvsp_BayerRG8:
            // 实测该相机上报 BayerRG8 但实际相位是 BG，按 RG 转会导致红蓝互换
            // （红色灯条在画面里显示成蓝色 → PnPSolver 的红色阈值检不到），故用 BG 相位
            cv::cvtColor(src1(), out, cv::COLOR_BayerBG2BGR);
            return true;
        case PixelType_Gvsp_BayerGB8:
            cv::cvtColor(src1(), out, cv::COLOR_BayerGB2BGR);
            return true;
        case PixelType_Gvsp_BayerGR8:
            cv::cvtColor(src1(), out, cv::COLOR_BayerGR2BGR);
            return true;
        case PixelType_Gvsp_BayerBG8:
            cv::cvtColor(src1(), out, cv::COLOR_BayerBG2BGR);
            return true;
        case PixelType_Gvsp_RGB8_Packed:
            cv::cvtColor(src3(), out, cv::COLOR_RGB2BGR);
            return true;
        case PixelType_Gvsp_BGR8_Packed:
            out = src3().clone();
            return true;
        case PixelType_Gvsp_Mono8:
            cv::cvtColor(src1(), out, cv::COLOR_GRAY2BGR);
            return true;
        default:
            return false;
        }
    }
}

void CameraControl::sdk_initialize()
{
    MV_CC_Initialize();
}
CameraControl::CameraControl()
{
}
void CameraControl::enumerate_devices()
{
    memset(&m_stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
    int nRet1 = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &m_stDeviceList);
    if (nRet1 != MV_OK)
    {
        std::cerr << "Failed to enumerate devices, error code: " << nRet1 << std::endl;
    }
    else
    {
        std::cout << "Number of devices found: " << m_stDeviceList.nDeviceNum << std::endl;
    }
}
// 创建一个相机实例的句柄
void CameraControl::create_camerahandle(int i)
{
    if (i < 0 || i >= static_cast<int>(m_stDeviceList.nDeviceNum))
    {
        std::cerr << "Invalid camera index: " << i << ", available devices: "
                  << m_stDeviceList.nDeviceNum << std::endl;
        return;
    }
    if (m_stDeviceList.nDeviceNum > 0)
    {
        MV_CC_CreateHandle(&m_handle, m_stDeviceList.pDeviceInfo[i]);
        if (m_handle == nullptr)
        {
            std::cerr << "Failed to create camera handle." << std::endl;
        }
    }
}

// 打开相机
void CameraControl::open_camera()
{
    int nRet2 = MV_CC_OpenDevice(m_handle);
    if (nRet2 != MV_OK)
    {
        std::cerr << "Failed to open camera, error code: " << nRet2 << std::endl;
        destroy_camerahandle();
    }
    else
    {
        std::cout << "Camera opened successfully." << std::endl;
    }
}

// 曝光增益设置
void CameraControl::set_exposuregain(int exposureTime, int gain)
{
    int nRet3 = MV_CC_SetExposureTime(m_handle, exposureTime);
    if (nRet3 != MV_OK)
    {
        std::cerr << "Failed to set exposure time, error code: " << nRet3 << std::endl;
    }
    else
    {
        std::cout << "Exposure time set successfully." << std::endl;
    }

    int nRet4 = MV_CC_SetGain(m_handle, gain);
    if (nRet4 != MV_OK)
    {
        std::cerr << "Failed to set gain, error code: " << nRet4 << std::endl;
    }
    else
    {
        std::cout << "Gain set successfully." << std::endl;
    }
}

// 关闭相机
void CameraControl::close_camera()
{
    int nRet3 = MV_CC_CloseDevice(m_handle);
    if (nRet3 != MV_OK)
    {
        std::cerr << "Failed to close camera, error code: " << nRet3 << std::endl;
    }
    else
    {
        std::cout << "Camera closed successfully." << std::endl;
    }
}

// 销毁相机句柄
void CameraControl::destroy_camerahandle()
{
    int nRet4 = MV_CC_DestroyHandle(m_handle);
    if (nRet4 != MV_OK)
    {
        std::cerr << "Failed to destroy camera handle, error code: " << nRet4 << std::endl;
    }
    else
    {
        std::cout << "Camera handle destroyed successfully." << std::endl;
        m_handle = nullptr; // 句柄已失效，避免析构函数重复销毁
    }
}

// 缓存设置
void CameraControl::set_buffer(int bufferNum)
{
    int nRet5 = MV_CC_SetImageNodeNum(m_handle, bufferNum);
    if (nRet5 != MV_OK)
    {
        std::cerr << "Failed to set image buffer count, error code: " << nRet5 << std::endl;
    }
    else
    {
        std::cout << "Image buffer count set successfully." << std::endl;
    }
}

// 开始取流
void CameraControl::start_streaming()
{
    int nRet6 = MV_CC_StartGrabbing(m_handle);
    if (nRet6 != MV_OK)
    {
        std::cerr << "Failed to start streaming, error code: " << nRet6 << std::endl;
        MV_CC_CloseDevice(m_handle);
        MV_CC_DestroyHandle(m_handle);
        m_handle = nullptr; // 避免析构函数重复销毁
    }
    else
    {
        std::cout << "Streaming started successfully." << std::endl;
    }
}

// 采集一帧图像、显示并释放图像缓存
// 按下 ESC 返回 false（结束采集）；按下 Q/q 把当前帧保存到 images/ 目录
bool CameraControl::img_collection_free_buffer()
{
    // 取帧 + 转 BGR 都在 grab() 里完成，这里只负责显示和按键
    if (!grab(m_bgr))
        return false;

    cv::imshow("Camera", m_bgr);
    const int key = cv::waitKey(1) & 0xFF;

    if (key == 27) // ESC 退出
    {
        return false;
    }

    if (key == 'q' || key == 'Q') // Q/q 保存图片
    {
        const std::string path = save_image(m_bgr, "images");
        if (!path.empty())
        {
            std::cout << "Image saved: " << path << std::endl;
        }
    }

    return true;
}

// 只取一帧：GetImageBuffer -> 转 BGR -> FreeImageBuffer
// 全程不碰显示/按键，所以可以放在后台线程里反复调用
bool CameraControl::grab(cv::Mat &out)
{
    if (m_handle == nullptr)
        return false;

    std::memset(&stOutFrame, 0, sizeof(stOutFrame));
    const int ret = MV_CC_GetImageBuffer(m_handle, &stOutFrame, 1000);
    if (ret != MV_OK || stOutFrame.pBufAddr == nullptr)
    {
        // 只在错误码变化时打印一次，断线/停流时不会刷屏
        if (ret != m_lastGrabErr)
        {
            std::cerr << "GetImageBuffer failed: 0x"
                      << std::hex << ret << std::dec << std::endl;
            m_lastGrabErr = ret;
        }
        return false;
    }
    m_lastGrabErr = MV_OK;

    const unsigned int pixelType = stOutFrame.stFrameInfo.enPixelType;
    const bool ok = to_bgr(stOutFrame.stFrameInfo, stOutFrame.pBufAddr, out);

    // 释放相机缓存，之后 pBufAddr 失效；out 已由 cvtColor/clone 分配了独立内存
    const int nRet = MV_CC_FreeImageBuffer(m_handle, &stOutFrame);
    if (nRet != MV_OK)
        std::cerr << "Failed to free image buffer." << std::endl;
    stOutFrame.pBufAddr = nullptr;

    if (!ok)
        std::cerr << "Unsupported pixel format: 0x"
                  << std::hex << pixelType << std::dec << std::endl;

    return ok;
}

// 停止取流
void CameraControl::stop_stream()
{
    int n = MV_CC_StopGrabbing(m_handle);
    if (n != MV_OK)
    {
        std::cout << "Failed to stop streaming,erroe code:" << n << std::endl;
    }
}
void CameraControl::sdk_finalize()
{
    int nRet9 = MV_CC_Finalize();
    if (nRet9 != MV_OK)
    {
        std::cerr << "Failed to finalize SDK, error code: " << nRet9 << std::endl;
    }
    else
    {
        std::cout << "SDK finalized successfully." << std::endl;
    }
}
// 一键运行相机采集程序：封装 main() 中的完整流程
// 参数：相机索引、曝光时间(us)、增益(dB)、缓存节点数
// 返回：0 正常结束；-1 参数非法；-2 未找到设备；-3 索引越界；-4 创建/打开/取流失败
int CameraControl::run_capture(int cameraIndex, int exposureTime, int gain, int bufferNum)
{
    // 1. 参数检查
    if (cameraIndex < 0 || exposureTime <= 0 || gain < 0 || bufferNum <= 0)
    {
        std::cerr << "Invalid parameters: cameraIndex=" << cameraIndex
                  << ", exposureTime=" << exposureTime
                  << ", gain=" << gain
                  << ", bufferNum=" << bufferNum << std::endl;
        return -1;
    }

    // 2. SDK 初始化 + 枚举设备
    sdk_initialize();
    enumerate_devices();

    if (m_stDeviceList.nDeviceNum <= 0)
    {
        std::cerr << "No camera found." << std::endl;
        sdk_finalize();
        return -2;
    }
    if (cameraIndex >= static_cast<int>(m_stDeviceList.nDeviceNum))
    {
        std::cerr << "Camera index " << cameraIndex << " out of range, only "
                  << m_stDeviceList.nDeviceNum << " device(s) found." << std::endl;
        sdk_finalize();
        return -3;
    }

    // 3. 创建句柄 -> 打开 -> 设置曝光/增益 -> 设置缓存 -> 开始取流
    {
        CameraControl camera;
        camera.create_camerahandle(cameraIndex);
        if (camera.m_handle == nullptr)
        {
            std::cerr << "Failed to create camera handle." << std::endl;
            sdk_finalize();
            return -4;
        }

        camera.open_camera();
        if (camera.m_handle == nullptr) // 打开失败时 open_camera 内部已销毁句柄
        {
            sdk_finalize();
            return -4;
        }

        camera.set_exposuregain(exposureTime, gain);
        camera.set_buffer(bufferNum);
        camera.start_streaming();
        if (camera.m_handle == nullptr) // 取流失败时 start_streaming 内部已关闭并销毁句柄
        {
            sdk_finalize();
            return -4;
        }

        // 4. 循环采集并显示：ESC 退出，Q/q 保存当前帧到 images/
        while (camera.img_collection_free_buffer())
        {
        }

        // 5. 停止取流；对象离开作用域时析构函数会关闭相机并销毁句柄
        camera.stop_stream();
        cv::destroyAllWindows();
    }

    // 6. SDK 反初始化
    sdk_finalize();
    return 0;
}

CameraControl::~CameraControl()
{
    if (m_handle != nullptr)
    {
        int nRet10 = MV_CC_CloseDevice(m_handle);
        int nRet11 = MV_CC_DestroyHandle(m_handle);
        if (nRet10 != MV_OK)
        {
            std::cerr << "Failed to close camera in destructor, error code: " << nRet10 << std::endl;
        }
        else if (nRet11 != MV_OK)
        {
            std::cerr << "Failed to destroy camera handle in destructor, error code: " << nRet11 << std::endl;
        }
        else
        {
            std::cout << "Camera closed and handle destroyed successfully in destructor." << std::endl;
        }
        m_handle = nullptr;
    }
}

// ============================================================================
//  CameraStream：后台线程不停取帧，主线程随时取"最新一帧"
// ============================================================================
void CameraStream::start()
{
    if (running_.load())
        return; // 已经在跑了
    running_.store(true);
    th_ = std::thread(&CameraStream::loop, this);
}

void CameraStream::stop()
{
    running_.store(false);

    // 线程此刻可能正阻塞在 GetImageBuffer(超时 1s) 里；
    // 上层若先调 cam.stop_stream()，这里能立刻 join，否则最多等 1s
    if (th_.joinable())
        th_.join();
}

void CameraStream::loop()
{
    while (running_.load())
    {
        cv::Mat frame;
        if (!cam_.grab(frame))
        {
            ++misses_;
            // 退避 5ms：grab 立刻失败时（句柄为空/相机掉线）不能空转烧 CPU
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            continue; // 单帧失败（超时等）不结束线程，继续取下一帧
        }

        ++frames_;
        {
            std::lock_guard<std::mutex> lk(mtx_);
            latest_ = frame; // 浅拷贝，只加一次引用计数，开销极小
            hasNew_ = true;
        }
        cv_.notify_all();

        // ★ 每帧回调：在锁外执行，避免耗时回调阻塞 wait()
        std::function<void(const cv::Mat &)> cb;
        {
            std::lock_guard<std::mutex> lk(mtx_);
            cb = cb_;
        }
        if (cb)
            cb(frame);
    }
}

bool CameraStream::latest(cv::Mat &out)
{
    std::lock_guard<std::mutex> lk(mtx_);
    if (!hasNew_ || latest_.empty())
        return false;

    out = latest_;
    hasNew_ = false;
    return true;
}

bool CameraStream::wait(cv::Mat &out, int timeoutMs)
{
    std::unique_lock<std::mutex> lk(mtx_);
    const bool signalled = cv_.wait_for(
        lk, std::chrono::milliseconds(timeoutMs),
        [this] { return hasNew_ || !running_.load(); });

    if (!signalled || !hasNew_ || latest_.empty())
        return false;

    out = latest_;
    hasNew_ = false;
    return true;
}

// 设置每帧回调（在相机线程里执行）
void CameraStream::setCallback(std::function<void(const cv::Mat &)> cb)
{
    std::lock_guard<std::mutex> lk(mtx_);
    cb_ = std::move(cb);
}