/* NanoDet 目标检测 C++ 推理类 (ARM/IMX6ULL 专用)
 * 模型: NanoDet-m (320x320, COCO 80类)
 */
#ifndef NANODET_H
#define NANODET_H

#include <vector>
#include "net.h"

class NanoDet
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

    NanoDet();
    ~NanoDet();

    int loadModel(const char* param_path, const char* bin_path, int num_threads = 2);

    /* 首次调用前需调用 loadModel，之后可多次调用 detectFromMat */
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
    static const int reg_max = 7;

    const char* cls_outputs[3] = { "792", "814", "836" };
    const char* box_outputs[3] = { "795", "817", "839" };

    void decodeBoxes(const float* box_pred, int H, int W, int stride,
                     std::vector<BoxInfo>& boxes_out);

    static void nmsSorted(const std::vector<BoxInfo>& boxes,
                          std::vector<int>& picked, float nms_threshold);
};

#endif
