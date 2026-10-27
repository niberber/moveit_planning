add_library(HKCamera ./src/camera.cpp)

find_package(OpenCV REQUIRED)
# camera.h 的公有成员暴露了 cv::Mat，所以 OpenCV 需要 PUBLIC 传递
target_include_directories(HKCamera
    PUBLIC
    ${OpenCV_INCLUDE_DIRS}
)
    
target_link_libraries(HKCamera
    PUBLIC
    ${OpenCV_LIBS}
)

target_include_directories(HKCamera 
                           PUBLIC
                           ${CMAKE_CURRENT_SOURCE_DIR}/include)

# 海康 MVS SDK：默认系统级安装路径 /opt/MVS，也可用 -DMVS_ROOT=... 指定（例如免 root 解压的目录）
set(MVS_ROOT "/opt/MVS" CACHE PATH "海康 MVS SDK 根目录")
find_path(MVS_INCLUDE_DIR
          NAMES MvCameraControl.h
          HINTS "${MVS_ROOT}/include"
          NO_DEFAULT_PATH)
find_library(MVS_LIBRARY
             NAMES MvCameraControl
             HINTS "${MVS_ROOT}/lib/64" "${MVS_ROOT}/lib/arm64" "${MVS_ROOT}/lib/aarch64"
             NO_DEFAULT_PATH)
if(NOT MVS_INCLUDE_DIR OR NOT MVS_LIBRARY)
    message(FATAL_ERROR
            "未找到海康 MVS SDK（当前 MVS_ROOT=${MVS_ROOT}）。\n"
            "  系统级安装: sudo bash scripts/install_mvs.sh\n"
            "  免 root 安装: bash scripts/install_mvs.sh --user\n"
            "  或手动指定: cmake -DMVS_ROOT=<SDK 根目录> ...")
endif()

get_filename_component(MVS_LIBRARY_DIR "${MVS_LIBRARY}" DIRECTORY)

target_include_directories(HKCamera PUBLIC ${MVS_INCLUDE_DIR})
target_link_libraries(HKCamera PUBLIC ${MVS_LIBRARY})
# 把 MVS 库目录写进可执行文件的 rpath，运行时无需再设 LD_LIBRARY_PATH
target_link_options(HKCamera INTERFACE "-Wl,-rpath,${MVS_LIBRARY_DIR}")

