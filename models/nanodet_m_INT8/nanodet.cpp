/* NanoDet 目标检测 C++ 实现 (ARM/IMX6ULL 专用)
 */
#include "nanodet.h"

#include <algorithm>
#include <cmath>
#include <ctime>

const int NanoDet::strides[3] = { 8, 16, 32 };

NanoDet::NanoDet()
    : input_size(320),
      prob_threshold(0.35f),
      nms_threshold(0.3f),
      num_threads(2)
{
}

NanoDet::~NanoDet()
{
    net.clear();
}

int NanoDet::loadModel(const char* param_path, const char* bin_path, int _num_threads)
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

static float* softmax(const float* x, int length, float* buf)
{
    float max_val = x[0];
    for (int i = 1; i < length; ++i)
        if (x[i] > max_val) max_val = x[i];

    float sum = 0.f;
    for (int i = 0; i < length; ++i)
    {
        buf[i] = std::exp(x[i] - max_val);
        sum += buf[i];
    }
    float inv = 1.f / sum;
    for (int i = 0; i < length; ++i)
        buf[i] *= inv;

    return buf;
}

void NanoDet::decodeBoxes(const float* box_pred, int H, int W, int stride,
                          std::vector<BoxInfo>& boxes_out)
{
    const int num_points = H * W;

    float softmax_buf[8];

    for (int i = 0; i < num_points; ++i)
    {
        int grid_y = i / W;
        int grid_x = i % W;

        float dis_pred[4] = { 0.f, 0.f, 0.f, 0.f };

        for (int j = 0; j < 4; ++j)
        {
            const float* dist_ptr = box_pred + i * 32 + j * 8;
            float* prob = softmax(dist_ptr, 8, softmax_buf);
            for (int k = 0; k < 8; ++k)
                dis_pred[j] += prob[k] * (float)k;
        }

        // 以 grid 中心为基准, distance 要乘 stride
        float pb_cx = (grid_x + 0.5f) * stride;
        float pb_cy = (grid_y + 0.5f) * stride;

        float x1 = pb_cx - dis_pred[0] * stride;
        float y1 = pb_cy - dis_pred[1] * stride;
        float x2 = pb_cx + dis_pred[2] * stride;
        float y2 = pb_cy + dis_pred[3] * stride;

        BoxInfo box;
        box.x1 = x1;
        box.y1 = y1;
        box.x2 = x2;
        box.y2 = y2;
        box.score = 0.f;
        box.label = -1;
        boxes_out.push_back(box);
    }
}

int NanoDet::detectFromMat(ncnn::Mat& in, int img_w, int img_h,
                           std::vector<Object>& objects,
                           double* out_model_ms, double* out_decode_ms, double* out_post_ms)
{
    objects.clear();

    struct timespec t0, t1, t2, t3;
    clock_gettime(CLOCK_MONOTONIC, &t0);

    ncnn::Extractor ex = net.create_extractor();
    ex.set_light_mode(true);
    ex.input("input.1", in);

    // 仅收集各层输出, 不做解码
    ncnn::Mat cls_preds[3], box_preds[3];
    for (int level = 0; level < 3; ++level)
    {
        ex.extract(cls_outputs[level], cls_preds[level]);
        ex.extract(box_outputs[level], box_preds[level]);
    }

    clock_gettime(CLOCK_MONOTONIC, &t1);
    double model_ms = (t1.tv_sec - t0.tv_sec) * 1000.0 + (t1.tv_nsec - t0.tv_nsec) / 1e6;

    // 纯解码 + 分数过滤
    std::vector<BoxInfo> all_boxes;
    for (int level = 0; level < 3; ++level)
    {
        int H = input_size / strides[level];
        int W = H;
        int num_points = H * W;

        std::vector<BoxInfo> level_boxes;
        level_boxes.reserve(num_points);
        decodeBoxes((const float*)box_preds[level].data, H, W, strides[level], level_boxes);

        const float* cls_data = (const float*)cls_preds[level].data;
        for (int i = 0; i < num_points; ++i)
        {
            const float* scores = cls_data + i * 80;
            int max_label = 0;
            float max_score = scores[0];
            for (int c = 1; c < 80; ++c)
            {
                if (scores[c] > max_score)
                {
                    max_score = scores[c];
                    max_label = c;
                }
            }

            if (max_score < prob_threshold)
                continue;

            level_boxes[i].score = max_score;
            level_boxes[i].label = max_label;
            all_boxes.push_back(level_boxes[i]);
        }
    }

    clock_gettime(CLOCK_MONOTONIC, &t2);
    double decode_ms = (t2.tv_sec - t1.tv_sec) * 1000.0 + (t2.tv_nsec - t1.tv_nsec) / 1e6;

    // NMS 后处理
    for (int c = 0; c < 80; ++c)
    {
        std::vector<BoxInfo> class_boxes;
        for (const auto& b : all_boxes)
            if (b.label == c)
                class_boxes.push_back(b);

        if (class_boxes.empty())
            continue;

        std::sort(class_boxes.begin(), class_boxes.end(),
                  [](const BoxInfo& a, const BoxInfo& b) {
                      return a.score > b.score;
                  });

        std::vector<int> picked;
        NanoDet::nmsSorted(class_boxes, picked, nms_threshold);

        for (int idx : picked)
        {
            const BoxInfo& b = class_boxes[idx];
            Object obj;
            obj.class_id = b.label;
            obj.label     = b.label;
            obj.prob      = b.score;

            float scale_x = (float)img_w / input_size;
            float scale_y = (float)img_h / input_size;

            obj.x = b.x1 * scale_x;
            obj.y = b.y1 * scale_y;
            obj.w = (b.x2 - b.x1) * scale_x;
            obj.h = (b.y2 - b.y1) * scale_y;

            if (obj.x < 0) { obj.w += obj.x; obj.x = 0; }
            if (obj.y < 0) { obj.h += obj.y; obj.y = 0; }
            if (obj.x + obj.w > img_w) obj.w = img_w - obj.x;
            if (obj.y + obj.h > img_h) obj.h = img_h - obj.y;

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

void NanoDet::nmsSorted(const std::vector<BoxInfo>& boxes,
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

            float inter_x1 = std::max(a.x1, b.x1);
            float inter_y1 = std::max(a.y1, b.y1);
            float inter_x2 = std::min(a.x2, b.x2);
            float inter_y2 = std::min(a.y2, b.y2);

            float inter_w = inter_x2 - inter_x1;
            float inter_h = inter_y2 - inter_y1;
            if (inter_w <= 0 || inter_h <= 0)
                continue;

            float inter_area = inter_w * inter_h;
            float iou = inter_area / (areas[i] + areas[picked[j]] - inter_area);

            if (iou > nms_threshold)
            {
                keep = false;
                break;
            }
        }
        if (keep)
            picked.push_back(i);
    }
}
