cmake_minimum_required(VERSION 3.0)
project(SaliencyExtractor)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 14)

# --------------- 核心配置: 指向 DSI-SLAM 目录（可用 -D 覆盖） ---------------
get_filename_component(DSI_SLAM_ROOT_DIR_DEFAULT "${CMAKE_SOURCE_DIR}/../.." REALPATH)
set(DSI_SLAM_ROOT_DIR "${DSI_SLAM_ROOT_DIR_DEFAULT}" CACHE PATH "Path to DSI-SLAM root directory")
# 兼容旧变量名
set(ORB_SLAM3_ROOT_DIR ${DSI_SLAM_ROOT_DIR})
# ----------------------------------------------------------------------

add_definitions(-DUSE_SALIENCY)

# 查找依赖库
find_package(OpenCV 3.0 REQUIRED)
find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)

# 包含 ORB-SLAM3 的头文件
include_directories(
    ${DSI_SLAM_ROOT_DIR}
    ${DSI_SLAM_ROOT_DIR}/include
    ${DSI_SLAM_ROOT_DIR}/include/CameraModels
    ${DSI_SLAM_ROOT_DIR}/Thirdparty/Sophus
    ${EIGEN3_INCLUDE_DIR}
    ${Pangolin_INCLUDE_DIRS}
)

# 指向编译好的 ORB-SLAM3 库文件
# 通常在 ORB_SLAM3/lib/libORB_SLAM3.so
set(LIBS 
    ${OpenCV_LIBS} 
    ${EIGEN3_LIBS}
    ${Pangolin_LIBRARIES}
    ${DSI_SLAM_ROOT_DIR}/lib/libORB_SLAM3.so
    ${DSI_SLAM_ROOT_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so  # <--- 添加这一行
    ${DSI_SLAM_ROOT_DIR}/Thirdparty/g2o/lib/libg2o.so       # <--- 最好也加上这一行，防止g2o报错
)

# 创建可执行文件
add_executable(saliency_extractor saliency_view.cc)

# 链接库
target_link_libraries(saliency_extractor ${LIBS})
