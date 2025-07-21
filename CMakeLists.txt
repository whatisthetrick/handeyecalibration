cmake_minimum_required(VERSION 3.10)
project(calibration)

# 设置C++标准
set(CMAKE_CXX_STANDARD 14)

# 找OpenCV
set(OpenCV_DIR "C:/opencv4.11_vs2022/x64/vc17/lib")
find_package(OpenCV REQUIRED)


# 编译可执行文件
add_executable(calibration calibration.cpp)

# 链接OpenCV库
target_link_libraries(calibration ${OpenCV_LIBS})

# 包含OpenCV头文件
include_directories(${OpenCV_INCLUDE_DIRS})
