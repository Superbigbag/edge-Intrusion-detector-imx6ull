/* YOLO-FastestV2 目标检测 C++ 推理类 (ARM/IMX6ULL 专用)
 * 输入: 352x352 BGR, 归一化 mean=[0,0,0] norm=[1/255,...]
 * 输出: 2 特征层 (22x22@stride16 + 11x11@stride32), 每层3锚框
 */
#ifndef YOLOFASTESTV2_H
#define YOLOFASTESTV2_H

#include <vector>
#include "net.h"

class YoloFastestV2
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

    YoloFastestV2();
    ~YoloFastestV2();

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

    static const int strides[2];
    static const int num_anchors;
    static const float anchors[6][2];

    const char* output_names[2] = { "794", "796" };

    void decodeAndFilter(const ncnn::Mat* outs, int img_w, int img_h,
                         std::vector<BoxInfo>& boxes);

    static void nmsSorted(const std::vector<BoxInfo>& boxes,
                          std::vector<int>& picked, float nms_threshold);
};

#endif
