# NCNN 边缘端入侵检测

> 在 **NXP i.MX6ULL** (Cortex-A7 800MHz, 512MB RAM) 上实现实时人员检测，并通过 MQTT 上报华为云 IoT 平台。

**核心亮点：**
- **纯 CPU 推理** — 无 GPU、无 NPU，仅靠 ARM NEON 指令加速
- **INT8 量化** — 单核 Cortex-A7 跑出 400ms/帧，模型仅 272KB
- **MQTT 云端上报** — 零依赖 MQTT 3.1.1 客户端，HMAC-SHA256 自动签密

---

## 效果展示

| 项目 | 截图 |
|------|------|
| 实时检测画面 | <!-- 此处贴检测效果截图/gif --> |
| GPIO LED 报警 | <!-- 此处贴板子 LED 亮灯照片 --> |
| MQTT 云端上报 | <!-- 此处贴华为云控制台截图 --> |
| 截获图（绿框+置信度） | <!-- 此处贴 captures/*.bmp 效果图 --> |

```
运行日志示例：

[18:20:15] ALARM! 检测到入侵者 (置信度: 0.72)
           保存截获图: captures/01.bmp
[18:20:17] 【有人】1人 | 总398 (Y15 R22 M336 D23 P0)
[18:20:22] ALARM OFF - 目标消失
[18:20:24] 【无人】总389 (Y15 R22 M327 D23 P0)
```

---

## 模型对比

| 模型 | FP16 | **INT8** | 体积 | 算子类型 |
|------|------|----------|------|----------|
| **YOLO-FastestV2** ★ | 450ms | **400ms** | 272KB | 非标算子 |
| NanoDet-m | 1000ms | 800ms | — | 非标算子 |
| YOLOv11n | 2000ms | 1450ms | — | 标准算子（融合最优） |

> ★ 主选模型 — Cortex-A7 上速度/精度/体积综合最优

### 每帧耗时拆解（YOLO-FastestV2 INT8）

```
YUV→BGR 15ms | 缩放+归一化 22ms | 推理 336ms | 解码+NMS 23ms
总计：~400ms（约 2.5 FPS）
```

---

## 快速开始

**1. 部署到板端**

```bash
scp models/YOLO-FastestV2_INT8/detect \
    models/YOLO-FastestV2_INT8/yolo-fastestv2.param \
    models/YOLO-FastestV2_INT8/yolo-fastestv2.bin \
    models/YOLO-FastestV2_INT8/mqtt.conf \
    root@<板子IP>:/app/
```

**2. 编辑 `mqtt.conf`** — 填入设备密钥即可：

```ini
host      = <华为云 IoTDA 地址>
port      = 1883
username  = <设备ID>
secret    = <设备密钥>            # 密码由程序根据时间戳自动生成
device_id = <设备ID>
keepalive = 60
```

**3. 运行**

```bash
cd /app
./detect yolo-fastestv2.param yolo-fastestv2.bin /dev/video1 0.45
```

检测到人时：**GPIO LED 亮 → MQTT 上报 → 存 BMP 截获图**（`captures/01.bmp` ~ `20.bmp` 循环覆盖）。

---

## 系统架构

```
USB 摄像头 (V4L2 YUYV 640×480)
    │
    ├─ YUYV→BGR（整数查表，无浮点运算）
    ├─ 缩放至 352×352，归一化 (1/255)
    ├─ ncnn::Net 推理（NEON 加速）
    ├─ Anchor 解码 + NMS（IoU 阈值 0.25）
    │
    ├─ GPIO 报警（sysfs 接口，3 帧消抖）
    ├─ MQTT 上报（QoS 1，HMAC-SHA256 签密）
    └─ BMP 截获图（画绿框 + 像素字体置信度，20 张循环）
```

### 零外部依赖

不依赖 OpenCV、libjpeg、OpenSSL、mosquitto 等任何第三方库，全部手写：

| 模块 | 代码量 | 替代物 |
|------|--------|--------|
| `mqtt_client.h` | 211 行 | libmosquitto / paho |
| HMAC-SHA256 | 139 行 | OpenSSL / mbedtls |
| BMP 写入 | 45 行 | libjpeg / OpenCV |
| 5×7 像素字体 | 40 行 | freetype / OpenCV |
| YUYV→BGR 转换 | 35 行 | OpenCV cvtColor |
| V4L2 采集 | 200 行 | OpenCV VideoCapture |

---

## 从源码编译

```bash
# 1. 设置交叉编译器
export CROSS_PREFIX=/path/to/arm-linux-gnueabihf/bin/arm-linux-gnueabihf

# 2. 编译 ncnn 库 + 检测程序
./build.sh                          # 默认编译 YOLOv11n

# 3. 切换模型（CMake 参数）
cmake -DBUILD_FASTEST=ON ..         # YOLO-FastestV2
cmake -DBUILD_NANODET=ON ..         # NanoDet-m

# 4. INT8 量化
./quantize_yolo.sh                  # YOLO-FastestV2 → INT8
```

编译选项：`-mcpu=cortex-a7 -mfpu=neon-vfpv4 -mfloat-abi=hard -O2 -ffast-math`  
工具链：Linaro GCC 6.2.1 `arm-linux-gnueabihf`

---

## 硬件平台

| 组件 | 规格 |
|------|------|
| 开发板 | 百问网 IMX6ULL Pro |
| CPU | NXP i.MX6ULL, Cortex-A7 @ 800MHz |
| 内存 | 512MB DDR3 |
| 摄像头 | USB UVC (YUYV 640×480) |
| 指示灯 | GPIO131 板载 USR LED（低电平亮） |

---

## MQTT 上报格式

Topic：`$oc/devices/{device_id}/sys/properties/report`

```json
{
  "services": [{
    "service_id": "intrusion",
    "properties": {
      "device_id": "69feddd8e094d615923affd3_00013f2d3e4d",
      "timestamp": "2026-05-09T18:20:00",
      "event": "intrusion",
      "confidence": 0.72
    }
  }]
}
```

---

## Debug 事项

- [x] Box 解码漏 stride 乘法 → 框尺寸缩小 8-32 倍，修复后正确框住单人
- [x] `free(rgb_data)` 提前释放 → 截获图无法访问 BGR 缓冲，改为循环末尾释放
- [x] MQTT 密码算法错误 → 先 Base64 后 Hex；先 key=密钥 后 key=时间戳（华为云 HMAC 顺序相反）
- [x] MQTT 时间戳格式错误 → Unix epoch → UTC `YYYYMMDDHH`
- [x] 告警不退出 → MQTT 代码块 `}` 误吞 else 分支，`alarm_frames++` 丢失
- [x] 截获图框过大 → 解码器已将 boxes 缩放到相机分辨率，画图时重复缩放
- [x] YUYV→BGR 整数运算改用移位代替除法（`(359*V)>>8` 替代 `1.402*V`）
- [x] `ncnn::Extractor` 每帧局部重建 → 不复用，避免结果缓存导致检测异常
