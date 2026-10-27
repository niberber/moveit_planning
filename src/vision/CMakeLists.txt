cmake_minimum_required(VERSION 3.22)
project(PnP LANGUAGES CXX)

# ============================================================================
#  构建配置：视觉解算是"每帧都要跑"的热路径，必须开优化
#  说明：CMAKE_BUILD_TYPE 为空时 CMake 不会加任何 -O（等价 -O0），
#        解算库在这种构建下比 Release 慢 3~10 倍。
#        实测旧构建命令只有 -g，没有 -O，属于这种情况。
# ============================================================================
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type (Release/Debug)" FORCE)
    message(STATUS "vision: 未指定 CMAKE_BUILD_TYPE，默认使用 Release")
endif()

# -O3 覆盖可能存在的 -O2；Debug 构建时不加，保证可调试
add_compile_options($<$<NOT:$<CONFIG:Debug>>:-O3>)

# -march=native 只在本机编译且本机运行时开启（换机器可能触发非法指令）
option(VISION_MARCH_NATIVE "编译时使用 -march=native（仅限本机编译并在同一台机器运行）" OFF)
if(VISION_MARCH_NATIVE)
    add_compile_options(-march=native)
    message(STATUS "vision: 已启用 -march=native")
endif()

add_executable(${PROJECT_NAME} PnP.cpp)

add_subdirectory(PnPDate)
add_subdirectory(PnPSolver)

target_link_libraries(${PROJECT_NAME} PRIVATE PnPSolver)
