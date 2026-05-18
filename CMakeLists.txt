cmake_minimum_required(VERSION 3.10)
project(nanodet_cam CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ============================================================
#  ncnn 库查找
# ============================================================
if(DEFINED ENV{NCNN_PATH})
    set(NCNN_ROOT $ENV{NCNN_PATH})
else()
    set(NCNN_ROOT "${CMAKE_SOURCE_DIR}/ncnn-src")
endif()

find_path(NCNN_INCLUDE_DIR net.h
    PATHS ${NCNN_ROOT}/src
          ${NCNN_ROOT}/include/ncnn
          /usr/local/include/ncnn
)
find_library(NCNN_LIB ncnn
    PATHS ${NCNN_ROOT}/build-arm/src
          ${NCNN_ROOT}/build/src
          ${NCNN_ROOT}/lib
          /usr/local/lib
)

if(NOT NCNN_INCLUDE_DIR OR NOT NCNN_LIB)
    message(FATAL_ERROR "未找到 ncnn! 请设置: export NCNN_PATH=/path/to/ncnn")
endif()

# 需要同时添加源码目录和构建目录的 include (platform.h 等生成头文件在构建目录)
set(NCNN_BUILD_INCLUDE_DIR "${NCNN_ROOT}/build-arm/src")
include_directories(${NCNN_INCLUDE_DIR} ${NCNN_BUILD_INCLUDE_DIR})
message(STATUS "NCNN_INCLUDE: ${NCNN_INCLUDE_DIR}")
message(STATUS "NCNN_LIB:     ${NCNN_LIB}")

# ============================================================
#  ARM 编译选项 (IMX6ULL Cortex-A7 + NEON + 硬浮点)
# ============================================================
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2 -ffast-math -fomit-frame-pointer")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mcpu=cortex-a7 -mfpu=neon-vfpv4 -mfloat-abi=hard")

# ============================================================
#  摄像头实时检测程序 (V4L2)
# ============================================================
# ============================================================
#  检测程序 (默认 YOLOv11n)
# ============================================================
option(BUILD_NANODET "Build NanoDet version" OFF)
option(BUILD_FASTEST "Build YOLO-FastestV2 version" OFF)

if(BUILD_NANODET)
    add_executable(detect
        models/nanodet_m_INT8/nanodet.cpp
        models/nanodet_m_INT8/nanodet_cam_detect.cpp
    )
elseif(BUILD_FASTEST)
    add_executable(detect
        models/YOLO-FastestV2_FP16/yolofastestv2.cpp
        models/YOLO-FastestV2_FP16/yolo_cam_detect.cpp
    )
else()
    add_executable(detect
        models/YOLOv11n_FP16/yolov11.cpp
        models/YOLOv11n_FP16/yolov11_cam_detect.cpp
    )
endif()
target_link_libraries(detect
    ${NCNN_LIB}
    pthread
    dl
)
set_target_properties(detect PROPERTIES
    LINK_FLAGS "-static-libstdc++ -static-libgcc"
)
