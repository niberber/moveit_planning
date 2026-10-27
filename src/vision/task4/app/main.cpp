#include "camera.h"
// 一键运行：相机序列索引 = 0，曝光时间 = 8000us，增益 = 15dB，缓存节点数 = 2
// 运行中按 ESC 退出，按 Q/q 保存当前帧到 images/
int main()
{
    return CameraControl::run_capture(0, 8000, 15, 2);
}

