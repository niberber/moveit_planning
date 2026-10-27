add_library(PnPSolver SHARED lib/src/PnPSolver.cpp)

find_package(OpenCV REQUIRED)

# 该库是逐帧调用的热路径：无论从哪个工程 add_subdirectory 进来
# （src/vision 本工程，或 engineer_moveit2），都保证优化级别，不受外层 CMAKE_BUILD_TYPE 影响
# 例：外层以空构建类型配置时默认是 -O0，解算会慢数倍
# 注：OpenCV 的 CV_Assert 不受 -DNDEBUG 影响，越界检查仍然生效
target_compile_options(PnPSolver PRIVATE $<$<NOT:$<CONFIG:Debug>>:-O3>)

target_include_directories(PnPSolver PUBLIC lib/include)
target_link_libraries(PnPSolver PUBLIC PnPDate ${OpenCV_LIBS})
target_include_directories(PnPSolver PUBLIC ${OpenCV_INCLUDE_DIRS})
