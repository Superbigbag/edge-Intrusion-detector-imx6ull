# ============================================================
#  ARM 交叉编译工具链 (IMX6ULL Cortex-A7)
#  用法: cmake -DCMAKE_TOOLCHAIN_FILE=toolchain.cmake ..
# ============================================================

set(CMAKE_SYSTEM_NAME      Linux)
set(CMAKE_SYSTEM_PROCESSOR arm)

# ---------- 编译器路径 (根据你的工具链修改) ----------
# 默认使用 PATH 中的 arm-linux-gnueabihf-g++/gcc
# 如需指定路径: export CROSS_PREFIX=/path/to/bin/arm-linux-gnueabihf

set(CROSS_PREFIX "$ENV{CROSS_PREFIX}")

if(NOT CROSS_PREFIX)
    find_program(CROSS_GCC arm-linux-gnueabihf-gcc)
    find_program(CROSS_GXX arm-linux-gnueabihf-g++)
    if(NOT CROSS_GCC OR NOT CROSS_GXX)
        message(FATAL_ERROR "未找到交叉编译器! 请设置: export CROSS_PREFIX=/path/to/bin/arm-linux-gnueabihf")
    endif()
    set(CMAKE_C_COMPILER   ${CROSS_GCC})
    set(CMAKE_CXX_COMPILER ${CROSS_GXX})
else()
    set(CMAKE_C_COMPILER   "${CROSS_PREFIX}-gcc")
    set(CMAKE_CXX_COMPILER "${CROSS_PREFIX}-g++")
endif()

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

# ---------- IMX6ULL CPU 特性 ----------
set(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS} -mcpu=cortex-a7 -mfpu=neon-vfpv4 -mfloat-abi=hard")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mcpu=cortex-a7 -mfpu=neon-vfpv4 -mfloat-abi=hard")
