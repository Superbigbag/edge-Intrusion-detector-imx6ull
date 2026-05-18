/* YOLO-FastestV2 目标检测 C++ 实现 (ARM/IMX6ULL 专用)
 */
#include "yolofastestv2.h"

#include <algorithm>
#include <cmath>
#include <ctime>

const int   YoloFastestV2::strides[2]     = { 16, 32 };
const int   YoloFastestV2::num_anchors    = 3;
const float YoloFastestV2::anchors[6][2]  = {
    {12.64f, 19.39f},   {37.88f, 51.48f},  {55.71f, 138.31f},
    {126.91f, 78.23f},  {131.57f, 214.55f}, {279.92f, 258.87f}
};

YoloFastestV2::YoloFastestV2()
    : input_size(352),
      prob_threshold(0.35f),
      nms_threshold(0.25f),
      num_threads(2)
{
}

YoloFastestV2::~YoloFastestV2()
{
    net.clear();
}

int YoloFastestV2::loadModel(const char* param_path, const char* bin_path, int _num_threads)
{
    num_threads = _num_threads;
    net.opt.num_threads = num_threads;
    net.opt.use_vulkan_compute = false;
    net.opt.use_bf16_storage = false;
    net.opt.use_fp16_packed = true;
    net.opt.use_fp16_storage = true;
    net.opt.use_fp16_arithmetic = false;
    net.opt.use_packing_layout = true;
    net.opt.use_int8_inference = true;
    net.opt.use_int8_storage = true;
    net.opt.use_int8_arithmetic = true;
    net.opt.use_int8_packed = true;

    int ret = net.load_param(param_path);
    if (ret != 0)
        return -1;

    ret = net.load_model(bin_path);
    if (ret != 0)
        return -2;

    return 0;
}

static inline float fastSigmoid(float x)
{
    return 1.0f / (1.0f + std::exp(-x));
}

void YoloFastestV2::decodeAndFilter(const ncnn::Mat* outs, int img_w, int img_h,
                                    std::vector<BoxInfo>& boxes)
{
    boxes.clear();

    for (int i = 0; i < 2; ++i)
    {
        int outW = outs[i].h;  // grid width
        int outH = outs[i].c;  // grid height
        int outC = outs[i].w;  // 255 = 3 * 85
        int stride = strides[i];

        for (int h = 0; h < outH; ++h)
        {
            const float* values = outs[i].channel(h);

            for (int w = 0; w < outW; ++w)
            {
                const float* ptr = values + w * outC;

                for (int b = 0; b < num_anchors; ++b)
                {
                    float objScore = fastSigmoid(ptr[4 * num_anchors + b]);

                    int max_cls = 0;
                    float max_cls_score = 0.f;
                    for (int c = 0; c < 80; ++c)
                    {
                        float s = fastSigmoid(ptr[4 * num_anchors + num_anchors + c]);
                        if (s > max_cls_score)
                        {
                            max_cls_score = s;
                            max_cls = c;
                        }
                    }

                    float score = objScore * max_cls_score;
                    if (score < prob_threshold)
                        continue;

                    float bcx = ((ptr[b * 4 + 0] * 2.f - 0.5f) + w) * stride;
                    float bcy = ((ptr[b * 4 + 1] * 2.f - 0.5f) + h) * stride;
                    float bw  = std::pow(ptr[b * 4 + 2] * 2.f, 2.f) * anchors[i * num_anchors + b][0];
                    float bh  = std::pow(ptr[b * 4 + 3] * 2.f, 2.f) * anchors[i * num_anchors + b][1];

                    float x1 = (bcx - 0.5f * bw) * img_w / input_size;
                    float y1 = (bcy - 0.5f * bh) * img_h / input_size;
                    float x2 = (bcx + 0.5f * bw) * img_w / input_size;
                    float y2 = (bcy + 0.5f * bh) * img_h / input_size;

                    if (x1 < 0) x1 = 0;
                    if (y1 < 0) y1 = 0;
                    if (x2 > img_w) x2 = img_w;
                    if (y2 > img_h) y2 = img_h;

                    if (x2 - x1 < 2 || y2 - y1 < 2)
                        continue;

                    BoxInfo box;
                    box.x1 = x1;
                    box.y1 = y1;
                    box.x2 = x2;
                    box.y2 = y2;
                    box.score = score;
                    box.label = max_cls;
                    boxes.push_back(box);
                }
            }
        }
    }
}

int YoloFastestV2::detectFromMat(ncnn::Mat& in, int img_w, int img_h,
                                 std::vector<Object>& objects,
                                 double* out_model_ms, double* out_decode_ms, double* out_post_ms)
{
    objects.clear();

    struct timespec t0, t1, t2, t3;
    clock_gettime(CLOCK_MONOTONIC, &t0);

    ncnn::Extractor ex = net.create_extractor();
    ex.set_light_mode(true);
    ex.input("input.1", in);

    ncnn::Mat outs[2];
    ex.extract(output_names[0], outs[0]);
    ex.extract(output_names[1], outs[1]);

    clock_gettime(CLOCK_MONOTONIC, &t1);
    double model_ms = (t1.tv_sec - t0.tv_sec) * 1000.0 + (t1.tv_nsec - t0.tv_nsec) / 1e6;

    std::vector<BoxInfo> all_boxes;
    decodeAndFilter(outs, img_w, img_h, all_boxes);

    clock_gettime(CLOCK_MONOTONIC, &t2);
    double decode_ms = (t2.tv_sec - t1.tv_sec) * 1000.0 + (t2.tv_nsec - t1.tv_nsec) / 1e6;

    // NMS
    for (int c = 0; c < 80; ++c)
    {
        std::vector<BoxInfo> class_boxes;
        for (const auto& b : all_boxes)
            if (b.label == c)
                class_boxes.push_back(b);

        if (class_boxes.empty())
            continue;

        std::sort(class_boxes.begin(), class_boxes.end(),
                  [](const BoxInfo& a, const BoxInfo& b) { return a.score > b.score; });

        std::vector<int> picked;
        nmsSorted(class_boxes, picked, nms_threshold);

        for (int idx : picked)
        {
            const BoxInfo& b = class_boxes[idx];
            Object obj;
            obj.class_id = b.label;
            obj.label     = b.label;
            obj.prob      = b.score;
            obj.x = b.x1;
            obj.y = b.y1;
            obj.w = b.x2 - b.x1;
            obj.h = b.y2 - b.y1;
            objects.push_back(obj);
        }
    }

    clock_gettime(CLOCK_MONOTONIC, &t3);
    double post_ms = (t3.tv_sec - t2.tv_sec) * 1000.0 + (t3.tv_nsec - t2.tv_nsec) / 1e6;

    if (out_model_ms)  *out_model_ms  = model_ms;
    if (out_decode_ms) *out_decode_ms = decode_ms;
    if (out_post_ms)   *out_post_ms   = post_ms;

    return 0;
}

void YoloFastestV2::nmsSorted(const std::vector<BoxInfo>& boxes,
                               std::vector<int>& picked,
                               float nms_threshold)
{
    picked.clear();
    const int n = boxes.size();

    std::vector<float> areas(n);
    for (int i = 0; i < n; ++i)
    {
        float w = boxes[i].x2 - boxes[i].x1;
        float h = boxes[i].y2 - boxes[i].y1;
        areas[i] = (w > 0 && h > 0) ? w * h : 0.f;
    }

    for (int i = 0; i < n; ++i)
    {
        const BoxInfo& a = boxes[i];
        bool keep = true;
        for (int j = 0; j < (int)picked.size(); ++j)
        {
            const BoxInfo& b = boxes[picked[j]];
            float ix1 = std::max(a.x1, b.x1);
            float iy1 = std::max(a.y1, b.y1);
            float ix2 = std::min(a.x2, b.x2);
            float iy2 = std::min(a.y2, b.y2);
            float iw = ix2 - ix1;
            float ih = iy2 - iy1;
            if (iw <= 0 || ih <= 0) continue;
            float iou = iw * ih / (areas[i] + areas[picked[j]] - iw * ih);
            if (iou > nms_threshold) { keep = false; break; }
        }
        if (keep) picked.push_back(i);
    }
}
