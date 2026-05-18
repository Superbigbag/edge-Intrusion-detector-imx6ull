/* YOLO-FastestV2 摄像头实时入侵检测程序 (IMX6ULL / ARM Linux)
 *
 * 用法:
 *   ./detect model.param model.bin [dev] [prob] [width] [height] [gpio]
 *   默认: 640x480, 置信度0.6, GPIO=131
 */

#include "yolofastestv2.h"
#include "mqtt_client.h"

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
//  报警控制 (支持 GPIO sysfs / LED sysfs 两种方式)
//    GPIO: ./detect ... 1   (GPIO1)
//    LED:  ./detect ... led:sys-led  或 led:red
// ============================================================
class AlarmCtrl
{
public:
    AlarmCtrl() : gpio_pin(-1), gpio_exported(false), led_fd(-1) {}

    bool init(const char* arg)
    {
        if (!arg || arg[0] == '\0' || strcmp(arg, "-1") == 0)
            return false;

        // LED 模式: 参数以 "led:" 开头
        if (strncmp(arg, "led:", 4) == 0)
        {
            const char* led_name = arg + 4;
            char path[128];
            // 先设置 trigger 为 none (手动控制)
            snprintf(path, sizeof(path), "/sys/class/leds/%s/trigger", led_name);
            int fd = open(path, O_WRONLY);
            if (fd >= 0)
            {
                write(fd, "none", 4);
                close(fd);
            }
            // 打开 brightness 文件保持句柄
            snprintf(path, sizeof(path), "/sys/class/leds/%s/brightness", led_name);
            led_fd = open(path, O_WRONLY);
            if (led_fd < 0)
            {
                fprintf(stderr, "[LED] 无法打开 %s\n", path);
                return false;
            }
            printf("    [LED] %s 已就绪\n", led_name);
            return true;
        }

        // GPIO 模式: 纯数字
        gpio_pin = atoi(arg);
        if (gpio_pin < 0) return false;

        int fd = open("/sys/class/gpio/export", O_WRONLY);
        if (fd < 0) { perror("gpio export"); return false; }
        char buf[16];
        snprintf(buf, sizeof(buf), "%d", gpio_pin);
        write(fd, buf, strlen(buf));
        close(fd);

        char path[128];
        snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/direction", gpio_pin);
        for (int retry = 0; retry < 100; ++retry)
        {
            fd = open(path, O_WRONLY);
            if (fd >= 0) break;
            usleep(10000);
        }
        if (fd < 0) { perror("gpio direction"); return false; }
        write(fd, "out", 3);
        close(fd);
        gpio_exported = true;
        printf("    [GPIO] GPIO%d 已配置为输出\n", gpio_pin);
        return true;
    }

    void setValue(int value)
    {
        if (led_fd >= 0)
        {
            char c = (value != 0) ? '1' : '0';
            lseek(led_fd, 0, SEEK_SET);
            write(led_fd, &c, 1);
            return;
        }
        if (gpio_pin < 0 || !gpio_exported) return;
        char path[128];
        snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/value", gpio_pin);
        int fd = open(path, O_WRONLY);
        if (fd < 0) return;
        char c = (value != 0) ? '1' : '0';
        write(fd, &c, 1);
        close(fd);
    }

    void release()
    {
        if (led_fd >= 0) { close(led_fd); led_fd = -1; return; }
        if (!gpio_exported || gpio_pin < 0) return;
        int fd = open("/sys/class/gpio/unexport", O_WRONLY);
        if (fd >= 0)
        {
            char buf[16];
            snprintf(buf, sizeof(buf), "%d", gpio_pin);
            write(fd, buf, strlen(buf));
            close(fd);
        }
        gpio_exported = false;
    }

    ~AlarmCtrl() { release(); }

private:
    int gpio_pin;
    bool gpio_exported;
    int led_fd;
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
//  简易 INI 读取
// ============================================================
static char* iniGet(char* buf, const char* key, char* out, int out_len)
{
    for (char* p = buf; *p; ++p)
    {
        if (strncmp(p, key, strlen(key)) == 0)
        {
            char* eq = strchr(p, '=');
            if (!eq) return nullptr;
            char* val = eq + 1;
            while (*val == ' ' || *val == '\t') val++;
            int len = strcspn(val, "\r\n");
            if (len >= out_len) len = out_len - 1;
            memcpy(out, val, len);
            out[len] = '\0';
            return out;
        }
        p = strchr(p, '\n');
        if (!p) break;
    }
    return nullptr;
}

// ============================================================
//  SHA256 + HMAC-SHA256 + Base64 (零依赖, 用于华为云密码)
// ============================================================

static inline uint32_t SHROTR(uint32_t x, int n) { return (x >> n) | (x << (32 - n)); }

static void sha256_transform(uint32_t s[8], const uint8_t chunk[64])
{
    static const uint32_t K[64] = {
        0x428a2f98,0x71374491,0xb5c0fbcf,0xe9b5dba5,0x3956c25b,0x59f111f1,0x923f82a4,0xab1c5ed5,
        0xd807aa98,0x12835b01,0x243185be,0x550c7dc3,0x72be5d74,0x80deb1fe,0x9bdc06a7,0xc19bf174,
        0xe49b69c1,0xefbe4786,0x0fc19dc6,0x240ca1cc,0x2de92c6f,0x4a7484aa,0x5cb0a9dc,0x76f988da,
        0x983e5152,0xa831c66d,0xb00327c8,0xbf597fc7,0xc6e00bf3,0xd5a79147,0x06ca6351,0x14292967,
        0x27b70a85,0x2e1b2138,0x4d2c6dfc,0x53380d13,0x650a7354,0x766a0abb,0x81c2c92e,0x92722c85,
        0xa2bfe8a1,0xa81a664b,0xc24b8b70,0xc76c51a3,0xd192e819,0xd6990624,0xf40e3585,0x106aa070,
        0x19a4c116,0x1e376c08,0x2748774c,0x34b0bcb5,0x391c0cb3,0x4ed8aa4a,0x5b9cca4f,0x682e6ff3,
        0x748f82ee,0x78a5636f,0x84c87814,0x8cc70208,0x90befffa,0xa4506ceb,0xbef9a3f7,0xc67178f2
    };
    uint32_t w[64];
    int i;
    for (i=0;i<16;i++)
        w[i]=(uint32_t)chunk[i*4]<<24|(uint32_t)chunk[i*4+1]<<16|
             (uint32_t)chunk[i*4+2]<<8|chunk[i*4+3];
    for (i=16;i<64;i++)
    {
        uint32_t s0=SHROTR(w[i-15],7)^SHROTR(w[i-15],18)^(w[i-15]>>3);
        uint32_t s1=SHROTR(w[i-2],17)^SHROTR(w[i-2],19)^(w[i-2]>>10);
        w[i]=w[i-16]+s0+w[i-7]+s1;
    }
    uint32_t a=s[0],b=s[1],c=s[2],d=s[3],e=s[4],f=s[5],g=s[6],h=s[7];
    for (i=0;i<64;i++)
    {
        uint32_t S1=SHROTR(e,6)^SHROTR(e,11)^SHROTR(e,25);
        uint32_t ch=(e&f)^((~e)&g);
        uint32_t t1=h+S1+ch+K[i]+w[i];
        uint32_t S0=SHROTR(a,2)^SHROTR(a,13)^SHROTR(a,22);
        uint32_t maj=(a&b)^(a&c)^(b&c);
        h=g;g=f;f=e;e=d+t1;d=c;c=b;b=a;a=t1+S0+maj;
    }
    s[0]+=a;s[1]+=b;s[2]+=c;s[3]+=d;s[4]+=e;s[5]+=f;s[6]+=g;s[7]+=h;
}

static void sha256(const uint8_t* data, size_t len, uint8_t digest[32])
{
    uint32_t st[8]={0x6a09e667,0xbb67ae85,0x3c6ef372,0xa54ff53a,
                    0x510e527f,0x9b05688c,0x1f83d9ab,0x5be0cd19};
    uint8_t block[64];
    int blen=0;
    uint64_t bits=0;
    for (size_t i=0;i<len;i++){
        block[blen++]=data[i];
        if (blen==64){sha256_transform(st,block);blen=0;}
    }
    bits=(uint64_t)len*8;
    block[blen++]=0x80;
    if (blen>56){while(blen<64)block[blen++]=0;sha256_transform(st,block);blen=0;}
    while(blen<56)block[blen++]=0;
    for(int i=7;i>=0;i--){block[blen++]=(bits>>(i*8))&0xFF;}
    sha256_transform(st,block);
    for(int i=0;i<8;i++){digest[i*4]=(st[i]>>24)&0xFF;digest[i*4+1]=(st[i]>>16)&0xFF;
                          digest[i*4+2]=(st[i]>>8)&0xFF;digest[i*4+3]=st[i]&0xFF;}
}

static void hmac_sha256(const uint8_t* key, size_t key_len,
                        const uint8_t* msg, size_t msg_len,
                        uint8_t digest[32])
{
    uint8_t k[96];
    int i;
    for (i=0;i<64;i++) k[i]=(i<(int)key_len)?key[i]:0;
    for (i=0;i<64;i++) k[i]^=0x36;
    memcpy(k+64, msg, msg_len);
    sha256(k, 64+msg_len, digest);
    for (i=0;i<64;i++) k[i]^=0x36^0x5c;
    memcpy(k+64, digest, 32);
    sha256(k, 64+32, digest);
}

static int hex_encode(const uint8_t* data, int len, char* out)
{
    static const char H[]="0123456789abcdef";
    for(int i=0;i<len;i++){out[i*2]=H[data[i]>>4];out[i*2+1]=H[data[i]&0xF];}
    out[len*2]='\0';
    return len*2;
}

// ============================================================
//  截获图绘制 + BMP 写入 (零依赖)
// ============================================================

static const unsigned char FONT5X7[10][7] = {
    {0x0E,0x11,0x11,0x11,0x11,0x11,0x0E}, // 0
    {0x04,0x0C,0x04,0x04,0x04,0x04,0x0E}, // 1
    {0x0E,0x11,0x01,0x02,0x04,0x08,0x1F}, // 2
    {0x0E,0x11,0x01,0x06,0x01,0x11,0x0E}, // 3
    {0x02,0x06,0x0A,0x12,0x1F,0x02,0x02}, // 4
    {0x1F,0x10,0x1E,0x01,0x01,0x11,0x0E}, // 5
    {0x06,0x08,0x10,0x1E,0x11,0x11,0x0E}, // 6
    {0x1F,0x01,0x02,0x04,0x08,0x08,0x08}, // 7
    {0x0E,0x11,0x11,0x0E,0x11,0x11,0x0E}, // 8
    {0x0E,0x11,0x11,0x0F,0x01,0x11,0x0E}, // 9
};

static void drawCharBGR(uint8_t* bgr, int imgW, int imgH, int x, int y, char c, int r, int g, int b)
{
    if (x < 0 || x + 5 > imgW || y < 0 || y + 7 > imgH) return;
    unsigned char rows[7];
    if (c >= '0' && c <= '9')
        memcpy(rows, FONT5X7[c - '0'], 7);
    else if (c == '.')
        { for (int i = 0; i < 5; i++) rows[i] = 0; rows[5] = 0x06; rows[6] = 0x06; }
    else
        return;
    for (int dy = 0; dy < 7; dy++) {
        unsigned char bits = rows[dy];
        for (int dx = 0; dx < 5; dx++) {
            if (bits & (1 << (4 - dx))) {
                int idx = ((y + dy) * imgW + (x + dx)) * 3;
                bgr[idx] = b; bgr[idx + 1] = g; bgr[idx + 2] = r;
            }
        }
    }
}

static void drawTextBGR(uint8_t* bgr, int imgW, int imgH, int x, int y, const char* text, int r, int g, int b)
{
    for (int i = 0; text[i]; i++)
        drawCharBGR(bgr, imgW, imgH, x + i * 6, y, text[i], r, g, b);
}

static void drawRectBGR(uint8_t* bgr, int imgW, int imgH, int x, int y, int w, int h, int r, int g, int b, int thick)
{
    if (x < 0) { w += x; x = 0; }
    if (y < 0) { h += y; y = 0; }
    if (x + w > imgW) w = imgW - x;
    if (y + h > imgH) h = imgH - y;
    if (w <= 0 || h <= 0) return;
    if (thick < 1) thick = 1;
    if (thick > w / 2) thick = w / 2;
    if (thick > h / 2) thick = h / 2;

    for (int t = 0; t < thick; t++) {
        for (int dx = 0; dx < w; dx++) {
            int i1 = ((y + t) * imgW + (x + dx)) * 3;
            bgr[i1] = b; bgr[i1 + 1] = g; bgr[i1 + 2] = r;
            int i2 = ((y + h - 1 - t) * imgW + (x + dx)) * 3;
            bgr[i2] = b; bgr[i2 + 1] = g; bgr[i2 + 2] = r;
        }
    }
    for (int t = 0; t < thick; t++) {
        for (int dy = thick; dy < h - thick; dy++) {
            int i1 = ((y + dy) * imgW + (x + t)) * 3;
            bgr[i1] = b; bgr[i1 + 1] = g; bgr[i1 + 2] = r;
            int i2 = ((y + dy) * imgW + (x + w - 1 - t)) * 3;
            bgr[i2] = b; bgr[i2 + 1] = g; bgr[i2 + 2] = r;
        }
    }
}

static void saveBMP(const char* path, const uint8_t* bgr, int w, int h)
{
    FILE* fp = fopen(path, "wb");
    if (!fp) return;
    int row = (w * 3 + 3) & ~3;
    int isz = row * h;
    int fsz = 54 + isz;

    unsigned char hdr[54] = {0};
    hdr[0] = 'B'; hdr[1] = 'M';
    hdr[2] = fsz; hdr[3] = fsz >> 8; hdr[4] = fsz >> 16; hdr[5] = fsz >> 24;
    hdr[10] = 54;
    hdr[14] = 40;
    hdr[18] = w;       hdr[19] = w >> 8;       hdr[20] = w >> 16;       hdr[21] = w >> 24;
    hdr[22] = h;       hdr[23] = h >> 8;       hdr[24] = h >> 16;       hdr[25] = h >> 24;
    hdr[26] = 1;
    hdr[28] = 24;
    hdr[34] = isz;     hdr[35] = isz >> 8;     hdr[36] = isz >> 16;     hdr[37] = isz >> 24;
    fwrite(hdr, 1, 54, fp);

    unsigned char pad[3] = {0, 0, 0};
    for (int ry = h - 1; ry >= 0; ry--) {
        fwrite(bgr + ry * w * 3, 1, w * 3, fp);
        if (row > w * 3) fwrite(pad, 1, row - w * 3, fp);
    }
    fclose(fp);
}

// ============================================================
//  全局变量 (信号处理)
// ============================================================
static volatile int g_running = 1;
static int g_capture_idx = 1;

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
    int         gpio_pin    = -1;
    const char* gpio_arg    = (argc >= 8) ? argv[7] : "131";

    printf("========================================\n");
    printf("  YOLO-FastestV2 入侵检测 - V4L2 实时版\n");
    printf("  (IMX6ULL / ARM Linux 专用)\n");
    printf("========================================\n");
    printf("  模型:   %s / %s\n", param_path, bin_path);
    printf("  设备:   %s\n", video_dev);
    printf("  分辨率: %dx%d\n", cam_res_w, cam_res_h);
    printf("  置信度: %.2f\n", prob_thr);
    printf("  报警:   %s %s\n", gpio_arg ? gpio_arg : "未启用", gpio_arg ? "" : "(参数: GPIO号 或 led:名字)");

    // 信号处理 (Ctrl+C 安全退出)
    signal(SIGINT,  sig_handler);
    signal(SIGTERM, sig_handler);

    // -------------------------------------------------------
    // 1. 加载模型
    // -------------------------------------------------------
    printf("\n[1/5] 加载模型...");
    fflush(stdout);

    YoloFastestV2 detector;
    detector.prob_threshold = prob_thr;
    detector.nms_threshold  = 0.25f;

    int ret = detector.loadModel(param_path, bin_path, 1);
    if (ret != 0)
    {
        fprintf(stderr, "\n  [错误] 模型加载失败! (错误码=%d)\n", ret);
        return 1;
    }
    printf(" 成功\n");
    printf("      输入: %dx%d, 线程: 1\n", detector.input_size, detector.input_size);

    // -------------------------------------------------------
    // 2. 打开摄像头
    // -------------------------------------------------------
    printf("\n[2/5] 打开摄像头...\n");
    V4L2Camera cam;
    if (!cam.openDevice(video_dev, cam_res_w, cam_res_h))
    {
        fprintf(stderr, "  [错误] 摄像头初始化失败!\n");
        return 1;
    }

    // -------------------------------------------------------
    // 3. 初始化报警
    // -------------------------------------------------------
    printf("\n[3/5] 初始化报警...\n");
    AlarmCtrl alarm;
    if (gpio_arg && alarm.init(gpio_arg))
    {
        usleep(500000);  // 等 GPIO 稳定
        alarm.setValue(1);  // 初始熄灭
    }
    else if (gpio_arg)
    {
        fprintf(stderr, "       [警告] 报警初始化失败 (%s)\n", gpio_arg);
    }

    // -------------------------------------------------------
    // 4. MQTT 连接
    // -------------------------------------------------------
    MqttClient mqtt;
    char mqtt_topic[128];
    char device_id[64] = "01";  // 默认
    bool mqtt_ok = false;

    {
        FILE* fp = fopen("mqtt.conf", "r");
        if (fp)
        {
            char buf[4096] = {0};
            fread(buf, 1, sizeof(buf)-1, fp);
            fclose(fp);

            char host_buf[256]={0}, port_buf[16]={0}, user_buf[128]={0};
            char secret_buf[128]={0}, keep_buf[16]={0}, did_buf[64]={0};

            char* host = iniGet(buf, "host",     host_buf, sizeof(host_buf));
            char* port = iniGet(buf, "port",     port_buf, sizeof(port_buf));
            char* user = iniGet(buf, "username", user_buf, sizeof(user_buf));
            char* sec  = iniGet(buf, "secret",   secret_buf,sizeof(secret_buf));
            char* keep = iniGet(buf, "keepalive",keep_buf, sizeof(keep_buf));
            char* did  = iniGet(buf, "device_id",did_buf,  sizeof(did_buf));
            if (did[0]) strncpy(device_id, did, sizeof(device_id)-1);

            if (host && port && sec && user)
            {
                // 密码: hex(HMAC-SHA256(key=时间戳, msg=密钥))
                // 时间戳: UTC YYYYMMDDHH
                time_t now = time(NULL);
                struct tm* gmt = gmtime(&now);
                char ts_str[16];
                snprintf(ts_str, sizeof(ts_str), "%04d%02d%02d%02d",
                         gmt->tm_year+1900, gmt->tm_mon+1, gmt->tm_mday, gmt->tm_hour);

                uint8_t hmac_digest[32];
                // 华为云: key=时间戳, message=密钥 (与常规HMAC顺序相反!)
                hmac_sha256((const uint8_t*)ts_str, strlen(ts_str),
                            (const uint8_t*)sec, strlen(sec), hmac_digest);

                char pass_buf[65];
                hex_encode(hmac_digest, 32, pass_buf);

                char clid_buf[192];
                snprintf(clid_buf, sizeof(clid_buf), "%s_0_0_%s", device_id, ts_str);

                int pt = port ? atoi(port) : 1883;
                int ka = keep ? atoi(keep) : 60;
                printf("[4/5] 连接 MQTT %s:%d...\n", host, pt);
                printf("       ts=%s\n", ts_str);
                printf("       clientid=%s\n", clid_buf);
                printf("       password=%s\n", pass_buf);
                if (mqtt.connect(host, pt, clid_buf, user, pass_buf, ka))
                {
                    printf("       MQTT 已连接\n");
                    mqtt_ok = true;
                }
                else
                    fprintf(stderr, "       [警告] MQTT 连接失败 (检查网络/DNS/密钥)\n");
            }
            else
                fprintf(stderr, "       [警告] 配置不全 (host/port/secret/user/device_id)\n");
        }
        else
                fprintf(stderr, "       [警告] 未找到 mqtt.conf\n");
    }
    snprintf(mqtt_topic, sizeof(mqtt_topic), "$oc/devices/%s/sys/properties/report", device_id);
    printf("\n[5/5] 开始实时检测 (Ctrl+C 退出)\n");
    printf("----------------------------------------\n");
    printf("  [提示] 检测到人 → 灯光 + MQTT上传\n");
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

        cam.releaseFrame();

        clock_gettime(CLOCK_MONOTONIC, &t_resize1);

        // 推理
        std::vector<YoloFastestV2::Object> objects;
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
                alarm.setValue(0);  // 低电平点亮
                printTimestamp();
                printf("ALARM! 检测到入侵者 (置信度: %.2f)\n", max_conf);

                if (mqtt_ok && person_detected)
                {
                    // 找到置信度最高的人
                    float best_conf = 0.f;
                    const YoloFastestV2::Object* best = nullptr;
                    for (const auto& obj : objects)
                    {
                        if (obj.class_id == 0 && obj.prob > best_conf)
                        { best_conf = obj.prob; best = &obj; }
                    }
                    if (best)
                    {
                        char json[512];
                        char ts[32];
                        time_t now = time(NULL);
                        strftime(ts, sizeof(ts), "%Y-%m-%dT%H:%M:%S", localtime(&now));
                        snprintf(json, sizeof(json),
                            "{\"services\":[{\"service_id\":\"intrusion\",\"properties\":{"
                            "\"device_id\":\"%s\",\"timestamp\":\"%s\","
                            "\"event\":\"intrusion\","
                            "\"confidence\":%.2f}}]}",
                            device_id, ts, best->prob);
                        mqtt.publish(mqtt_topic, json);
                    }
                }

                // 保存截获图 (画框 + 写 BMP) — boxes 已在相机坐标空间
                mkdir("captures", 0755);
                for (const auto& obj : objects)
                {
                    if (obj.class_id == 0 && obj.prob > prob_thr)
                    {
                        int bx = (int)obj.x, by = (int)obj.y;
                        int bw = (int)obj.w, bh = (int)obj.h;
                        drawRectBGR(rgb_data, cam_w, cam_h, bx, by, bw, bh, 0, 255, 0, 2);
                        char conf[8];
                        snprintf(conf, sizeof(conf), "%.2f", obj.prob);
                        int ty = by - 9;
                        if (ty < 0) ty = by;
                        drawTextBGR(rgb_data, cam_w, cam_h, bx, ty, conf, 0, 255, 0);
                    }
                }
                char bmp_path[64];
                snprintf(bmp_path, sizeof(bmp_path), "captures/%02d.bmp", g_capture_idx);
                saveBMP(bmp_path, rgb_data, cam_w, cam_h);
                printf("        保存截获图: %s\n", bmp_path);
                g_capture_idx++;
                if (g_capture_idx > 20) g_capture_idx = 1;
            }
        }
        else
        {
            alarm_frames++;
            if (alarm_frames >= ALARM_HOLD && alarm_active)
            {
                alarm_active = false;
                alarm.setValue(1);  // 高电平熄灭
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

                if (mqtt_ok) mqtt.ping();  // 保活
            }
        }

        free(rgb_data);
    }

    // -------------------------------------------------------
    // 5. 清理
    // -------------------------------------------------------
    printf("\n退出中...\n");
    if (alarm_active) alarm.setValue(1);  // 退出时熄灭
    alarm.release();
    mqtt.disconnect();
    cam.closeDevice();
    printf("共处理 %d 帧, 程序结束.\n", frame_count);

    return 0;
}
