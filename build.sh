#!/bin/bash
# ============================================================
#  NanoDet ncnn ARM 交叉编译脚本 (IMX6ULL)
#  用法:
#    ./build.sh         # 交叉编译 ARM 版本
#    ./build.sh clean   # 清理构建
# ============================================================
set -e

PROJECT_DIR="$(cd "$(dirname "$0")" && pwd)"
BUILD_DIR="${PROJECT_DIR}/build-arm"
NCNN_DIR="${NCNN_PATH:-${PROJECT_DIR}/ncnn-src}"

GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m'

info()  { echo -e "${GREEN}[INFO]${NC}  $1"; }
warn()  { echo -e "${YELLOW}[WARN]${NC}  $1"; }
error() { echo -e "${RED}[ERROR]${NC} $1"; exit 1; }

# ---------- 交叉编译器检测 ----------
CROSS_PREFIX="${CROSS_PREFIX:-arm-linux-gnueabihf}"
CROSS_CXX="${CROSS_PREFIX}-g++"

if ! command -v "${CROSS_CXX}" &>/dev/null; then
    error "未找到交叉编译器: ${CROSS_CXX}
    请设置环境变量:
      export CROSS_PREFIX=/path/to/toolchain/bin/arm-linux-gnueabihf"
fi
CROSS_VER=$("${CROSS_CXX}" --version 2>&1 | head -1)
info "交叉编译器: ${CROSS_CXX}  (${CROSS_VER})"

# ---------- 检查/编译 ncnn ARM 库 ----------
if [ ! -f "${NCNN_DIR}/build-arm/src/libncnn.a" ]; then
    warn "ncnn ARM 库未找到, 正在编译 ncnn..."
    if [ ! -d "${NCNN_DIR}" ]; then
        info "克隆 ncnn..."
        git clone https://github.com/Tencent/ncnn.git --depth 1 "${NCNN_DIR}"
    fi
    mkdir -p "${NCNN_DIR}/build-arm"
    cd "${NCNN_DIR}/build-arm"
    cmake .. \
        -DCMAKE_C_COMPILER="${CROSS_PREFIX}-gcc" \
        -DCMAKE_CXX_COMPILER="${CROSS_PREFIX}-g++" \
        -DCMAKE_C_FLAGS="-mcpu=cortex-a7 -mfpu=neon-vfpv4 -mfloat-abi=hard" \
        -DCMAKE_CXX_FLAGS="-mcpu=cortex-a7 -mfpu=neon-vfpv4 -mfloat-abi=hard" \
        -DCMAKE_SYSTEM_NAME=Linux \
        -DCMAKE_SYSTEM_PROCESSOR=arm \
        -DNCNN_VULKAN=OFF \
        -DNCNN_OPENMP=OFF \
        -DNCNN_RUNTIME_CPU=OFF \
        -DNCNN_BUILD_EXAMPLES=OFF \
        -DNCNN_BUILD_TOOLS=OFF
    make -j$(nproc)
    info "ncnn ARM 库编译完成"
fi

# ---------- 编译检测程序 ----------
info "编译 $1 摄像头检测程序..."
mkdir -p "${BUILD_DIR}"
cd "${BUILD_DIR}"

cmake .. \
    -DCMAKE_C_COMPILER="${CROSS_PREFIX}-gcc" \
    -DCMAKE_CXX_COMPILER="${CROSS_PREFIX}-g++" \
    -DCMAKE_SYSTEM_NAME=Linux \
    -DCMAKE_SYSTEM_PROCESSOR=arm \
    -DNCNN_INCLUDE_DIR="${NCNN_DIR}/src" \
    -DNCNN_LIB="${NCNN_DIR}/build-arm/src/libncnn.a"

make -j$(nproc)
info "编译完成: ${BUILD_DIR}/detect"

file "${BUILD_DIR}/detect" 2>/dev/null || true

echo ""
echo "部署到 IMX6ULL (YOLOv11n):"
echo "  scp ${BUILD_DIR}/detect \\"
echo "      ${PROJECT_DIR}/YOLOv11n_FP16/yolov11n.param \\"
echo "      ${PROJECT_DIR}/YOLOv11n_FP16/yolov11n.bin \\"
echo "      root@<板子IP>:/app/"
echo ""
echo "板端运行:"
echo "  cd /app && ./detect yolov11n.param yolov11n.bin /dev/video0 0.6"
echo ""
echo "切换版本: cmake -DBUILD_NANODET=ON ..     # NanoDet-m"
echo "         cmake -DBUILD_FASTEST=ON ..     # YOLO-FastestV2"

# ---------- 清理 ----------
if [ "${1}" = "clean" ]; then
    info "清理构建..."
    rm -rf "${BUILD_DIR}"
    info "清理完成"
fi
