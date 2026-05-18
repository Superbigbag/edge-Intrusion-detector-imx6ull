/* YOLOv11n 摄像头实时入侵检测程序 (IMX6ULL / ARM Linux)
 *
 * 用法:
 *   ./detect model.param model.bin [dev] [prob] [width] [height] [gpio]
 */

#include "yolov11.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <csignal>
#include <ctime>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <linux/videodev2.h>

// ============================================================
//  简易 GPIO 控制 (sysfs)
// ============================================================
class GPIOCtrl
{
public:
    GPIOCtrl() : pin(-1), exported(false) {}

    bool init(int gpio_pin)
    {
        if (gpio_pin < 0) return false;
        pin = gpio_pin;

        // 导出 GPIO
        int fd = open("/sys/class/gpio/export", O_WRONLY);
        if (fd < 0) { perror("gpio export"); return false; }
        char buf[16];
        snprintf(buf, sizeof(buf), "%d", pin);
        if (write(fd, buf, strlen(buf)) < 0)
        {
            // 可能已经导出, 忽略错误
        }
        close(fd);

        // 设置为输出
        char path[128];
        snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/direction", pin);
        // 等待 sysfs 节点创建
        for (int retry = 0; retry < 100; ++retry)
        {
            fd = open(path, O_WRONLY);
            if (fd >= 0) break;
            usleep(10000); // 10ms
        }
        if (fd < 0) { perror("gpio direction"); return false; }
        if (write(fd, "out", 3) < 0) { perror("gpio direction write"); close(fd); return false; }
        close(fd);

        exported = true;
        return true;
    }

    void setValue(int value)
    {
        if (pin < 0 || !exported) return;
        char path[128];
        snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/value", pin);
        int fd = open(path, O_WRONLY);
        if (fd < 0) return;
        char c = (value != 0) ? '1' : '0';
        write(fd, &c, 1);
        close(fd);
    }

    void release()
    {
        if (!exported || pin < 0) return;
        int fd = open("/sys/class/gpio/unexport", O_WRONLY);
        if (fd >= 0)
        {
            char buf[16];
            snprintf(buf, sizeof(buf), "%d", pin);
            write(fd, buf, strlen(buf));
            close(fd);
        }
        exported = false;
    }

    ~GPIOCtrl() { release(); }

private:
    int pin;
    bool exported;
};

// ============================================================
//  V4L2 摄像头封装 (YUYV 格式, 内存映射)
// ============================================================
class V4L2Camera
{
public:
    V4L2Camera() : fd(-1), buffers(nullptr), nbufs(0), width(0), height(0) {}

    ~V4L2Camera() { closeDevice(); }

    bool openDevice(const char* dev, int w, int h)
    {
        width  = w;
        height = h;

        fd = ::open(dev, O_RDWR | O_NONBLOCK, 0);
        if (fd < 0)
        {
            perror("open video device");
            return false;
        }

        // 查询能力
        struct v4l2_capability cap;
        if (ioctl(fd, VIDIOC_QUERYCAP, &cap) < 0)
        {
            perror("VIDIOC_QUERYCAP");
            return false;
        }
        printf("[V4L2] 设备: %s, 驱动: %s\n", cap.card, cap.driver);
        if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE))
        {
            fprintf(stderr, "[V4L2] 设备不支持视频采集\n");
            return false;
        }
        if (!(cap.capabilities & V4L2_CAP_STREAMING))
        {
            fprintf(stderr, "[V4L2] 设备不支持流式IO\n");
            return false;
        }

        // 设置格式 YUYV 4:2:2
        struct v4l2_format fmt;
        memset(&fmt, 0, sizeof(fmt));
        fmt.type                = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        fmt.fmt.pix.width       = width;
        fmt.fmt.pix.height      = height;
        fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
        fmt.fmt.pix.field       = V4L2_FIELD_NONE;
        if (ioctl(fd, VIDIOC_S_FMT, &fmt) < 0)
        {
            perror("VIDIOC_S_FMT");
            return false;
        }
        printf("[V4L2] 格式: YUYV %dx%d, 每帧大小: %d bytes\n",
               fmt.fmt.pix.width, fmt.fmt.pix.height, fmt.fmt.pix.sizeimage);

        width  = fmt.fmt.pix.width;
        height = fmt.fmt.pix.height;

        // 设置帧率 (15 fps for IMX6ULL)
        struct v4l2_streamparm parm;
        memset(&parm, 0, sizeof(parm));
        parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        parm.parm.capture.timeperframe.numerator   = 1;
        parm.parm.capture.timeperframe.denominator  = 15;
        ioctl(fd, VIDIOC_S_PARM, &parm);

        // 请求缓冲区 (使用内存映射, 3个缓冲区)
        struct v4l2_requestbuffers req;
        memset(&req, 0, sizeof(req));
        req.count  = 3;
        req.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        req.memory = V4L2_MEMORY_MMAP;
        if (ioctl(fd, VIDIOC_REQBUFS, &req) < 0)
        {
            perror("VIDIOC_REQBUFS");
            return false;
        }
        nbufs = req.count;
        printf("[V4L2] 申请了 %d 个缓冲区\n", nbufs);

        // 映射缓冲区
        buffers = (Buffer*)calloc(nbufs, sizeof(Buffer));
        for (unsigned int i = 0; i < nbufs; ++i)
        {
            struct v4l2_buffer buf;
            memset(&buf, 0, sizeof(buf));
            buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buf.memory = V4L2_MEMORY_MMAP;
            buf.index  = i;
            if (ioctl(fd, VIDIOC_QUERYBUF, &buf) < 0)
            {
                perror("VIDIOC_QUERYBUF");
                return false;
            }
            buffers[i].length = buf.length;
            buffers[i].start  = mmap(NULL, buf.length,
                                     PROT_READ | PROT_WRITE,
                                     MAP_SHARED, fd, buf.m.offset);
            if (buffers[i].start == MAP_FAILED)
            {
                perror("mmap");
                return false;
            }
        }

        // 缓冲区入队
        for (unsigned int i = 0; i < nbufs; ++i)
        {
            struct v4l2_buffer buf;
            memset(&buf, 0, sizeof(buf));
            buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buf.memory = V4L2_MEMORY_MMAP;
            buf.index  = i;
            if (ioctl(fd, VIDIOC_QBUF, &buf) < 0)
            {
                perror("VIDIOC_QBUF");
                return false;
            }
        }

        // 开始采集
        enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        if (ioctl(fd, VIDIOC_STREAMON, &type) < 0)
        {
            perror("VIDIOC_STREAMON");
            return false;
        }
        printf("[V4L2] 摄像头已启动, 分辨率 %dx%d\n", width, height);
        return true;
    }

    /* 获取一帧 YUYV 数据 (阻塞),
     *   返回: 数据指针 + 长度, 由调用方用完调用 releaseFrame */
    bool grabFrame(unsigned char** data, unsigned int* len)
    {
        struct v4l2_buffer buf;
        memset(&buf, 0, sizeof(buf));
        buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;

        // 改为阻塞模式等待
        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(fd, &fds);
        struct timeval tv;
        tv.tv_sec  = 2;
        tv.tv_usec = 0;
        int r = select(fd + 1, &fds, NULL, NULL, &tv);
        if (r <= 0)
        {
            fprintf(stderr, "[V4L2] 等待帧超时\n");
            return false;
        }

        if (ioctl(fd, VIDIOC_DQBUF, &buf) < 0)
        {
            perror("VIDIOC_DQBUF");
            return false;
        }

        *data = (unsigned char*)buffers[buf.index].start;
        *len  = buf.bytesused;
        queued_index = buf.index;
        return true;
    }

    /* 释放帧 (将缓冲区重新入队) */
    void releaseFrame()
    {
        struct v4l2_buffer buf;
        memset(&buf, 0, sizeof(buf));
        buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index  = queued_index;
        if (ioctl(fd, VIDIOC_QBUF, &buf) < 0)
            perror("VIDIOC_QBUF");
    }

    int getWidth()  const { return width; }
    int getHeight() const { return height; }

    void closeDevice()
    {
        if (fd >= 0)
        {
            enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            ioctl(fd, VIDIOC_STREAMOFF, &type);
        }
        if (buffers)
        {
            for (unsigned int i = 0; i < nbufs; ++i)
                if (buffers[i].start && buffers[i].start != MAP_FAILED)
                    munmap(buffers[i].start, buffers[i].length);
            free(buffers);
            buffers = nullptr;
        }
        if (fd >= 0) { close(fd); fd = -1; }
    }

private:
    typedef struct { void* start; unsigned int length; } Buffer;
    int          fd;
    Buffer*      buffers;
    unsigned int nbufs;
    int          width, height;
    unsigned int queued_index;
};

// ============================================================
//  全局变量 (信号处理)
// ============================================================
static volatile int g_running = 1;

static void sig_handler(int sig)
{
    (void)sig;
    g_running = 0;
}

// ============================================================
//  打印时间戳
// ============================================================
static void printTimestamp()
{
    time_t now = time(NULL);
    struct tm* t = localtime(&now);
    printf("[%02d:%02d:%02d] ", t->tm_hour, t->tm_min, t->tm_sec);
}

// ============================================================
//  主函数
// ============================================================
int main(int argc, char** argv)
{
    // 参数解析
    const char* param_path  = (argc >= 2) ? argv[1] : "model.param";
    const char* bin_path    = (argc >= 3) ? argv[2] : "model.bin";
    const char* video_dev   = (argc >= 4) ? argv[3] : "/dev/video0";
    float       prob_thr    = (argc >= 5) ? atof(argv[4]) : 0.6f;
    int         cam_res_w   = (argc >= 6) ? atoi(argv[5]) : 640;
    int         cam_res_h   = (argc >= 7) ? atoi(argv[6]) : 480;
    int         gpio_pin    = (argc >= 8) ? atoi(argv[7]) : -1;

    printf("========================================\n");
    printf("  YOLOv11n 入侵检测 - V4L2 实时版\n");
    printf("  (IMX6ULL / ARM Linux 专用)\n");
    printf("========================================\n");
    printf("  模型:   %s / %s\n", param_path, bin_path);
    printf("  设备:   %s\n", video_dev);
    printf("  分辨率: %dx%d\n", cam_res_w, cam_res_h);
    printf("  置信度: %.2f\n", prob_thr);
    printf("  GPIO:   %d %s\n", gpio_pin, (gpio_pin >= 0) ? "(报警使能)" : "(未启用)");

    // 信号处理 (Ctrl+C 安全退出)
    signal(SIGINT,  sig_handler);
    signal(SIGTERM, sig_handler);

    // -------------------------------------------------------
    // 1. 加载模型
    // -------------------------------------------------------
    printf("\n[1/3] 加载模型...");
    fflush(stdout);

    YoloV11 detector;
    detector.prob_threshold = prob_thr;
    detector.nms_threshold  = 0.45f;

    int ret = detector.loadModel(param_path, bin_path, 1);
    if (ret != 0)
    {
        fprintf(stderr, "\n  [错误] 模型加载失败! (错误码=%d)\n", ret);
        return 1;
    }
    printf(" 成功\n");
    printf("      输入: %dx%d, 线程: 1\n", detector.input_size, detector.input_size);
    printf(" 成功\n");
    printf("      输入: %dx%d, 线程: 1\n", detector.input_size, detector.input_size);

    // -------------------------------------------------------
    // 2. 打开摄像头
    // -------------------------------------------------------
    printf("\n[2/3] 打开摄像头...\n");
    V4L2Camera cam;
    if (!cam.openDevice(video_dev, cam_res_w, cam_res_h))
    {
        fprintf(stderr, "  [错误] 摄像头初始化失败!\n");
        return 1;
    }

    // -------------------------------------------------------
    // 3. 初始化 GPIO
    // -------------------------------------------------------
    GPIOCtrl gpio;
    if (gpio_pin >= 0)
    {
        if (gpio.init(gpio_pin))
            printf("       GPIO%d 已配置为输出\n", gpio_pin);
        else
            fprintf(stderr, "       [警告] GPIO%d 配置失败\n", gpio_pin);
    }

    // -------------------------------------------------------
    // 4. 主循环
    // -------------------------------------------------------
    printf("\n[3/3] 开始实时检测 (Ctrl+C 退出)\n");
    printf("----------------------------------------\n");
    printf("  [提示] 检测到人 → GPIO 拉高报警\n");
    printf("----------------------------------------\n\n");

    // 预分配避免循环内反复分配内存 (防止OOM)
    ncnn::Mat in_resized(detector.input_size, detector.input_size, 3);

    int   frame_count     = 0;
    int   alarm_frames    = 0;
    bool  alarm_active    = false;
    const int ALARM_HOLD  = 3;   // 连续3帧无人才取消报警

    while (g_running)
    {
        // 抓帧
        unsigned char* yuyv_data = nullptr;
        unsigned int   yuyv_len  = 0;
        if (!cam.grabFrame(&yuyv_data, &yuyv_len))
        {
            usleep(10000);
            continue;
        }
        frame_count++;

        struct timespec t_yuv0, t_yuv1, t_resize1, t_model1, t_total;
        clock_gettime(CLOCK_MONOTONIC, &t_yuv0);

        // YUYV → BGR 查表转换
        int cam_w = cam.getWidth();
        int cam_h = cam.getHeight();
        int rgb_size = cam_w * cam_h * 3;
        unsigned char* rgb_data = (unsigned char*)malloc(rgb_size);
        if (!rgb_data) { cam.releaseFrame(); continue; }

        int half_w = cam_w / 2;
        for (int y = 0; y < cam_h; ++y)
        {
            int src_line = y * cam_w * 2;
            int dst_line = y * cam_w * 3;
            for (int x = 0; x < half_w; ++x)
            {
                int src = src_line + x * 4;
                int dst = dst_line + x * 6;

                int Y0 = yuyv_data[src];
                int U  = yuyv_data[src + 1] - 128;
                int Y1 = yuyv_data[src + 2];
                int V  = yuyv_data[src + 3] - 128;

                int rv = (359 * V) >> 8;
                int guv = -((88 * U) >> 8) - ((183 * V) >> 8);
                int bu = (454 * U) >> 8;

                int r0 = Y0 + rv;
                int g0 = Y0 + guv;
                int b0 = Y0 + bu;
                int r1 = Y1 + rv;
                int g1 = Y1 + guv;
                int b1 = Y1 + bu;

                rgb_data[dst + 0] = b0 < 0 ? 0 : (b0 > 255 ? 255 : (unsigned char)b0);
                rgb_data[dst + 1] = g0 < 0 ? 0 : (g0 > 255 ? 255 : (unsigned char)g0);
                rgb_data[dst + 2] = r0 < 0 ? 0 : (r0 > 255 ? 255 : (unsigned char)r0);
                rgb_data[dst + 3] = b1 < 0 ? 0 : (b1 > 255 ? 255 : (unsigned char)b1);
                rgb_data[dst + 4] = g1 < 0 ? 0 : (g1 > 255 ? 255 : (unsigned char)g1);
                rgb_data[dst + 5] = r1 < 0 ? 0 : (r1 > 255 ? 255 : (unsigned char)r1);
            }
        }

        clock_gettime(CLOCK_MONOTONIC, &t_yuv1);

        // BGR → 缩放 + 归一化
        ncnn::Mat in = ncnn::Mat::from_pixels_resize(
            rgb_data, ncnn::Mat::PIXEL_BGR,
            cam_w, cam_h,
            detector.input_size, detector.input_size);

        float mean_vals[3] = { 0.f, 0.f, 0.f };
        float norm_vals[3] = { 1.f/255.f, 1.f/255.f, 1.f/255.f };
        in.substract_mean_normalize(mean_vals, norm_vals);

        free(rgb_data);
        cam.releaseFrame();

        clock_gettime(CLOCK_MONOTONIC, &t_resize1);

        // 推理
        std::vector<YoloV11::Object> objects;
        double model_ms = 0, decode_ms = 0, post_ms = 0;
        detector.detectFromMat(in, cam.getWidth(), cam.getHeight(), objects, &model_ms, &decode_ms, &post_ms);

        clock_gettime(CLOCK_MONOTONIC, &t_total);

        double yuv_ms  = (t_yuv1.tv_sec - t_yuv0.tv_sec) * 1000.0 + (t_yuv1.tv_nsec - t_yuv0.tv_nsec) / 1e6;
        double res_ms  = (t_resize1.tv_sec - t_yuv1.tv_sec) * 1000.0 + (t_resize1.tv_nsec - t_yuv1.tv_nsec) / 1e6;
        double total_ms = (t_total.tv_sec - t_yuv0.tv_sec) * 1000.0 + (t_total.tv_nsec - t_yuv0.tv_nsec) / 1e6;

        // 判断是否有"入侵" (检测到人)
        bool person_detected = false;
        float max_conf = 0.f;
        for (const auto& obj : objects)
        {
            if (obj.class_id == 0 && obj.prob > prob_thr)
            {
                person_detected = true;
                if (obj.prob > max_conf) max_conf = obj.prob;
            }
        }

        // GPIO 报警逻辑
        if (person_detected)
        {
            alarm_frames = 0;
            if (!alarm_active)
            {
                alarm_active = true;
                gpio.setValue(1);
                printTimestamp();
                printf("ALARM! 检测到入侵者 (置信度: %.2f)\n", max_conf);
            }
        }
        else
        {
            alarm_frames++;
            if (alarm_frames >= ALARM_HOLD && alarm_active)
            {
                alarm_active = false;
                gpio.setValue(0);
                printTimestamp();
                printf("ALARM OFF - 目标消失\n");
            }
        }

        // 每 2 秒输出状态
        {
            static struct timespec last_print = {0, 0};
            struct timespec now;
            clock_gettime(CLOCK_MONOTONIC, &now);
            double since = (now.tv_sec - last_print.tv_sec)
                         + (now.tv_nsec - last_print.tv_nsec) / 1e9;
            if (since >= 2.0)
            {
                last_print = now;
                int pc = 0;
                for (const auto& obj : objects)
                    if (obj.class_id == 0) pc++;
                printTimestamp();
                if (alarm_active)
                    printf("【有人】%d人 | 总%.0f (Y%d R%d M%d D%d P%d)\n",
                           pc, total_ms, (int)yuv_ms, (int)res_ms, (int)model_ms, (int)decode_ms, (int)post_ms);
                else
                    printf("【无人】总%.0f (Y%d R%d M%d D%d P%d)\n",
                           total_ms, (int)yuv_ms, (int)res_ms, (int)model_ms, (int)decode_ms, (int)post_ms);
            }
        }
    }

    // -------------------------------------------------------
    // 5. 清理
    // -------------------------------------------------------
    printf("\n退出中...\n");
    if (alarm_active)
    {
        gpio.setValue(0);
        printf("GPIO 已复位\n");
    }
    gpio.release();
    cam.closeDevice();
    printf("共处理 %d 帧, 程序结束.\n", frame_count);

    return 0;
}
