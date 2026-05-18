#!/bin/bash
# YOLOv11n INT8 量化脚本 (使用 Fastest 的标定图集)
set -e

PROJECT_DIR="/mnt/hgfs/sharefile/ncnn"
TOOLS_DIR="${PROJECT_DIR}/ncnn-src/build-x64/tools"

CALIB_DIR="${PROJECT_DIR}/models/YOLOv11n_INT8/calibration"
SRC_PARAM="${PROJECT_DIR}/models/YOLOv11n_FP16/yolov11n.param"
SRC_BIN="${PROJECT_DIR}/models/YOLOv11n_FP16/yolov11n.bin"
OUT_PARAM="${PROJECT_DIR}/models/YOLOv11n_INT8/yolov11n.param"
OUT_BIN="${PROJECT_DIR}/models/YOLOv11n_INT8/yolov11n.bin"
TABLE="${PROJECT_DIR}/models/YOLOv11n_INT8/yolov11n.table"

echo "[1/4] 生成标定表..."
ls "${CALIB_DIR}"/*.jpg > /tmp/calib_list.txt 2>/dev/null
N=$(wc -l < /tmp/calib_list.txt)
echo "  标定图片: ${N} 张"

"${TOOLS_DIR}/quantize/ncnn2table" \
    "${SRC_PARAM}" "${SRC_BIN}" /tmp/calib_list.txt "${TABLE}" \
    "mean=[0.0,0.0,0.0]" \
    "norm=[0.0039216,0.0039216,0.0039216]" \
    "shape=[320,320,3]" pixel=BGR method=kl

echo "[2/4] INT8 量化..."
"${TOOLS_DIR}/quantize/ncnn2int8" \
    "${SRC_PARAM}" "${SRC_BIN}" "${OUT_PARAM}" "${OUT_BIN}" "${TABLE}"

echo "[3/4] 优化..."
"${TOOLS_DIR}/ncnnoptimize" \
    "${OUT_PARAM}" "${OUT_BIN}" "${OUT_PARAM}" "${OUT_BIN}" 0

echo "[4/4] 重新编译..."
cd "${PROJECT_DIR}"
rm -rf build-arm
./build.sh
cp build-arm/detect models/YOLOv11n_INT8/

ls -lh "${OUT_PARAM}" "${OUT_BIN}"
echo "YOLOv11n INT8 量化完成!"
