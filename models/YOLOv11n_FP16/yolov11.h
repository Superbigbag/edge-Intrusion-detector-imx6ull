/* YOLOv11n 目标检测 C++ 推理类 (ARM/IMX6ULL 专用)
 * 输入: 320x320 BGR, mean=[0,0,0] norm=[1/255,...]
 * 输出: 单blob "out0" [144, N], reg_max=15
 */
#ifndef YOLOV11_H
#define YOLOV11_H

#include <vector>
#include "net.h"

class YoloV11
{
public:
    struct Object
    {
        float x, y, w, h;
        float prob;
        int label;
        int class_id;
    };

    struct BoxInfo
    {
        float x1, y1, x2, y2;
        float score;
        int label;
    };

    YoloV11();
    ~YoloV11();

    int loadModel(const char* param_path, const char* bin_path, int num_threads = 2);

    int detectFromMat(ncnn::Mat& in, int img_w, int img_h,
                      std::vector<Object>& objects,
                      double* out_model_ms = nullptr,
                      double* out_decode_ms = nullptr,
                      double* out_post_ms = nullptr);

    int input_size;
    float prob_threshold;
    float nms_threshold;

private:
    ncnn::Net net;
    int num_threads;

    static const int strides[3];
    static const int reg_max;

    const char* input_name  = "in0";
    const char* output_name = "out0";

    void decodeAndFilter(const float* pred, int img_w, int img_h,
                         std::vector<BoxInfo>& boxes);

    static void nmsSorted(const std::vector<BoxInfo>& boxes,
                          std::vector<int>& picked, float nms_threshold);
};

#endif
