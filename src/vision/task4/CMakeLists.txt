cmake_minimum_required(VERSION 3.22)
project(MyProject)

set(CMAKE_CXX_STANDARD 17)

# 与 src/vision/CMakeLists.txt 保持一致：未指定构建类型时默认 Release，并开 -O3
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type (Release/Debug)" FORCE)
    message(STATUS "task4: 未指定 CMAKE_BUILD_TYPE，默认使用 Release")
endif()
add_compile_options($<$<NOT:$<CONFIG:Debug>>:-O3>)

add_subdirectory(lib)

add_executable(Camera_Demo ./app/main.cpp)

target_link_libraries(Camera_Demo PRIVATE HKCamera)

