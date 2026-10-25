add_library(PnPSolver SHARED lib/src/PnPSolver.cpp)

find_package(OpenCV REQUIRED)

target_include_directories(PnPSolver PUBLIC lib/include)
target_link_libraries(PnPSolver PUBLIC PnPDate ${OpenCV_LIBS})
target_include_directories(PnPSolver PUBLIC ${OpenCV_INCLUDE_DIRS})
