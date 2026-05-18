#!/bin/bash
# YOLO-FastestV2 INT8 量化脚本
# 用法: ./quantize_yolo.sh
set -e

PROJECT_DIR="/mnt/hgfs/sharefile/ncnn"
TOOLS_DIR="${PROJECT_DIR}/ncnn-src/build-x64/tools"

CALIB_DIR="${PROJECT_DIR}/YOLO-FastestV2_INT8/calibration"
SRC_PARAM="${PROJECT_DIR}/YOLO-FastestV2_FP16/yolo-fastestv2.param"
SRC_BIN="${PROJECT_DIR}/YOLO-FastestV2_FP16/yolo-fastestv2.bin"
OUT_PARAM="${PROJECT_DIR}/YOLO-FastestV2_INT8/yolo-fastestv2.param"
OUT_BIN="${PROJECT_DIR}/YOLO-FastestV2_INT8/yolo-fastestv2.bin"
TABLE="${PROJECT_DIR}/YOLO-FastestV2_INT8/yolo-fastestv2.table"

# Step 1: calibration images → table
echo "[1/4] 生成标定表..."
ls "${CALIB_DIR}"/*.jpg > /tmp/calib_list.txt 2>/dev/null || {
    echo "标定图集为空! 请先用摄像头抓帧放到 ${CALIB_DIR}/"
    exit 1
}
N=$(wc -l < /tmp/calib_list.txt)
echo "  标定图片: ${N} 张"

"${TOOLS_DIR}/quantize/ncnn2table" \
    "${SRC_PARAM}" "${SRC_BIN}" /tmp/calib_list.txt "${TABLE}" \
    "mean=[0.0,0.0,0.0]" \
    "norm=[0.0039216,0.0039216,0.0039216]" \
    "shape=[352,352,3]" pixel=BGR method=kl

echo "  标定表: ${TABLE}"

# Step 2: quantize
echo "[2/4] INT8 量化..."
"${TOOLS_DIR}/quantize/ncnn2int8" \
    "${SRC_PARAM}" "${SRC_BIN}" "${OUT_PARAM}" "${OUT_BIN}" "${TABLE}"

echo "  输出: ${OUT_PARAM} / ${OUT_BIN}"

# Step 3: optimize
echo "[3/4] 优化..."
"${TOOLS_DIR}/ncnnoptimize" \
    "${OUT_PARAM}" "${OUT_BIN}" "${OUT_PARAM}" "${OUT_BIN}" 0

# Step 4: rebuild
echo "[4/4] 重新编译..."
cd "${PROJECT_DIR}"
rm -rf build-arm
./build.sh
cp build-arm/detect YOLO-FastestV2_INT8/

ls -lh "${OUT_PARAM}" "${OUT_BIN}"
echo "INT8 量化完成!"
