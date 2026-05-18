/* YOLOv11n 目标检测 C++ 实现 (ARM/IMX6ULL 专用)
 */
#include "yolov11.h"

#include <algorithm>
#include <cmath>
#include <ctime>

const int YoloV11::strides[3] = { 8, 16, 32 };
const int YoloV11::reg_max   = 15;

YoloV11::YoloV11()
    : input_size(320),
      prob_threshold(0.35f),
      nms_threshold(0.45f),
      num_threads(2)
{
}

YoloV11::~YoloV11()
{
    net.clear();
}

int YoloV11::loadModel(const char* param_path, const char* bin_path, int _num_threads)
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
    if (ret != 0) return -1;
    ret = net.load_model(bin_path);
    if (ret != 0) return -2;
    return 0;
}

static float* softmax16(const float* x, float* buf)
{
    float max_val = x[0];
    for (int i = 1; i < 16; ++i) if (x[i] > max_val) max_val = x[i];
    float sum = 0.f;
    for (int i = 0; i < 16; ++i) { buf[i] = std::exp(x[i] - max_val); sum += buf[i]; }
    float inv = 1.f / sum;
    for (int i = 0; i < 16; ++i) buf[i] *= inv;
    return buf;
}

static inline float sigmoid(float x) { return 1.f / (1.f + std::exp(-x)); }

void YoloV11::decodeAndFilter(const float* pred, int img_w, int img_h,
                               std::vector<BoxInfo>& boxes)
{
    boxes.clear();
    const int num_class = 80;
    const int num_bins  = reg_max + 1; // 16
    const int per_point = 4 * num_bins + num_class; // 144

    float softmax_buf[16];
    int offset = 0;

    for (int level = 0; level < 3; ++level)
    {
        int grid_w = input_size / strides[level];
        int grid_h = grid_w;
        int num_points = grid_w * grid_h;

        for (int i = 0; i < num_points; ++i)
        {
            const float* ptr = pred + (offset + i) * per_point;

            // 4 distance values from distribution
            float dis_pred[4] = { 0.f, 0.f, 0.f, 0.f };
            for (int j = 0; j < 4; ++j)
            {
                const float* dist_ptr = ptr + j * num_bins;
                float* prob = softmax16(dist_ptr, softmax_buf);
                for (int k = 0; k < num_bins; ++k)
                    dis_pred[j] += prob[k] * (float)k;
            }

            // classification
            const float* cls_ptr = ptr + 4 * num_bins;
            int max_cls = 0;
            float max_score = sigmoid(cls_ptr[0]);
            for (int c = 1; c < num_class; ++c)
            {
                float s = sigmoid(cls_ptr[c]);
                if (s > max_score) { max_score = s; max_cls = c; }
            }
            if (max_score < prob_threshold) continue;

            int gy = i / grid_w;
            int gx = i % grid_w;

            // decode: grid center + distance * stride
            float cx = (gx + 0.5f) * strides[level];
            float cy = (gy + 0.5f) * strides[level];

            float x1 = (cx - dis_pred[0] * strides[level]) * img_w / input_size;
            float y1 = (cy - dis_pred[1] * strides[level]) * img_h / input_size;
            float x2 = (cx + dis_pred[2] * strides[level]) * img_w / input_size;
            float y2 = (cy + dis_pred[3] * strides[level]) * img_h / input_size;

            if (x1 < 0) x1 = 0; if (y1 < 0) y1 = 0;
            if (x2 > img_w) x2 = img_w; if (y2 > img_h) y2 = img_h;
            if (x2 - x1 < 2 || y2 - y1 < 2) continue;

            BoxInfo box;
            box.x1 = x1; box.y1 = y1; box.x2 = x2; box.y2 = y2;
            box.score = max_score; box.label = max_cls;
            boxes.push_back(box);
        }
        offset += num_points;
    }
}

int YoloV11::detectFromMat(ncnn::Mat& in, int img_w, int img_h,
                           std::vector<Object>& objects,
                           double* out_model_ms, double* out_decode_ms, double* out_post_ms)
{
    objects.clear();

    struct timespec t0, t1, t2, t3;
    clock_gettime(CLOCK_MONOTONIC, &t0);

    ncnn::Extractor ex = net.create_extractor();
    ex.set_light_mode(true);
    ex.input(input_name, in);

    ncnn::Mat out;
    ex.extract(output_name, out);

    clock_gettime(CLOCK_MONOTONIC, &t1);
    double model_ms = (t1.tv_sec - t0.tv_sec)*1000.0 + (t1.tv_nsec - t0.tv_nsec)/1e6;

    std::vector<BoxInfo> all_boxes;
    decodeAndFilter((const float*)out.data, img_w, img_h, all_boxes);

    clock_gettime(CLOCK_MONOTONIC, &t2);
    double decode_ms = (t2.tv_sec - t1.tv_sec)*1000.0 + (t2.tv_nsec - t1.tv_nsec)/1e6;

    for (int c = 0; c < 80; ++c)
    {
        std::vector<BoxInfo> cboxes;
        for (const auto& b : all_boxes) if (b.label == c) cboxes.push_back(b);
        if (cboxes.empty()) continue;
        std::sort(cboxes.begin(), cboxes.end(),
                  [](const BoxInfo& a, const BoxInfo& b) { return a.score > b.score; });
        std::vector<int> picked;
        nmsSorted(cboxes, picked, nms_threshold);
        for (int idx : picked)
        {
            const BoxInfo& b = cboxes[idx];
            Object obj;
            obj.class_id = b.label; obj.label = b.label; obj.prob = b.score;
            obj.x = b.x1; obj.y = b.y1; obj.w = b.x2 - b.x1; obj.h = b.y2 - b.y1;
            objects.push_back(obj);
        }
    }

    clock_gettime(CLOCK_MONOTONIC, &t3);
    double post_ms = (t3.tv_sec - t2.tv_sec)*1000.0 + (t3.tv_nsec - t2.tv_nsec)/1e6;

    if (out_model_ms)  *out_model_ms  = model_ms;
    if (out_decode_ms) *out_decode_ms = decode_ms;
    if (out_post_ms)   *out_post_ms   = post_ms;
    return 0;
}

void YoloV11::nmsSorted(const std::vector<BoxInfo>& boxes,
                         std::vector<int>& picked, float nms_threshold)
{
    picked.clear();
    const int n = boxes.size();
    std::vector<float> areas(n);
    for (int i = 0; i < n; ++i) {
        float w = boxes[i].x2 - boxes[i].x1, h = boxes[i].y2 - boxes[i].y1;
        areas[i] = (w > 0 && h > 0) ? w * h : 0.f;
    }
    for (int i = 0; i < n; ++i) {
        const BoxInfo& a = boxes[i]; bool keep = true;
        for (int j = 0; j < (int)picked.size(); ++j) {
            const BoxInfo& b = boxes[picked[j]];
            float ix1 = std::max(a.x1, b.x1), iy1 = std::max(a.y1, b.y1);
            float ix2 = std::min(a.x2, b.x2), iy2 = std::min(a.y2, b.y2);
            float iw = ix2 - ix1, ih = iy2 - iy1;
            if (iw <= 0 || ih <= 0) continue;
            float iou = iw*ih / (areas[i] + areas[picked[j]] - iw*ih);
            if (iou > nms_threshold) { keep = false; break; }
        }
        if (keep) picked.push_back(i);
    }
}
