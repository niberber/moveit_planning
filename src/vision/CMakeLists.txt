cmake_minimum_required(VERSION 3.22)
project(PnP LANGUAGES CXX)

add_executable(${PROJECT_NAME} PnP.cpp)

add_subdirectory(PnPDate)
add_subdirectory(PnPSolver)

target_link_libraries(${PROJECT_NAME} PRIVATE PnPSolver)
