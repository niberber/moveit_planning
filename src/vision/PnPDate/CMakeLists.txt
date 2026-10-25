add_library(PnPDate SHARED lib/src/PnPDate.cpp)

find_package(OpenCV REQUIRED)

target_include_directories(PnPDate PUBLIC lib/include)
target_link_libraries(PnPDate PUBLIC ${OpenCV_LIBS})
target_include_directories(PnPDate PUBLIC ${OpenCV_INCLUDE_DIRS})
