# ncnn边缘端入侵检测

> 在 **NXP iMX6ULL** (Cortex-A7 800MHz, 512MB RAM) 上实现实时人员检测，并通过 MQTT 上报华为云 IoT 平台。

**核心亮点：**
- **纯 CPU 推理** — 无 GPU、无 NPU，仅靠 ARM NEON 指令加速
- **INT8 量化** — 单核 Cortex-A7 跑出 400ms/帧，模型仅 272KB
- **MQTT 云端上报** — 零依赖 MQTT 3.1.1 客户端

---

## 系统架构
```
USB 摄像头 (V4L2 YUYV 640×480)
    │
    ├─ YUYV→BGR（整数查表，无浮点运算）
    ├─ 缩放至 352×352，归一化 (1/255)
    ├─ ncnn:Net 推理（NEON 加速）
    ├─ Anchor 解码 + NMS（IoU 阈值 0.25）
    │
    ├─ GPIO 报警（sysfs 接口，3 帧消抖）
    ├─ MQTT 上报（QoS 1，HMAC-SHA256 签密）
    └─ BMP 截获图（画绿框 + 像素字体置信度，20 张循环）
```
 V4L2 采集、BMP写入、YUYV→BGR 转换不依赖 OpenCV、libjpeg等库


---

## 模型对比

| 模型 | FP16 | INT8 | 量化后体积 | 算子类型 |
|------|------|----------|------|----------|
| YOLO-FastestV2 | 450ms/帧 | 400ms/帧 | 272KB | 非标算子 |
| NanoDet-m | 1000ms/帧 | 800ms/帧 | 981KB | 非标算子 |
| YOLOv11n | 2000ms/帧 | 1450ms/帧 | 2613KB | 标准算子 |


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
    root@<板子IP>:/root/
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
cd /root
./detect yolo-fastestv2.param yolo-fastestv2.bin /dev/video1 0.45
```

检测到人时：**GPIO LED 亮 → MQTT 上报 → 存 BMP 截获图**（`captures/01.bmp` ~ `20.bmp` 循环覆盖）。

---

## 运行效果
### 初始化设备
<img src="https://github.com/Superbigbag/edge-Intrusion-detector-imx6ull/blob/main/images/%E5%88%9D%E5%A7%8B%E5%8C%96%E8%BF%87%E7%A8%8B.png" width="400" alt="系统实物图">

### 板端截获图
<img src="https://github.com/Superbigbag/edge-Intrusion-detector-imx6ull/blob/main/images/captures/02.bmp" width="400" alt="系统实物图">
<img src="https://github.com/Superbigbag/edge-Intrusion-detector-imx6ull/blob/main/images/captures/04.bmp" width="400" alt="系统实物图">

### MQTT云端上报（时间 2026-05-18 22:31:16）
<img src="https://github.com/Superbigbag/edge-Intrusion-detector-imx6ull/blob/main/images/mqtt%E4%B8%8A%E4%BC%A0%E7%BB%93%E6%9E%9C.png" width="400" alt="系统实物图">
<img src="https://github.com/Superbigbag/edge-Intrusion-detector-imx6ull/blob/main/images/%E4%BA%91%E7%AB%AF%E8%A1%A8%E6%A0%BC%E8%AE%B0%E5%BD%95.png" width="400" alt="系统实物图">

### PC同步打印（时间 2026-05-18 22:31:16）
<img src="https://github.com/Superbigbag/edge-Intrusion-detector-imx6ull/blob/main/images/pc%E7%9A%84%E5%90%8C%E6%AD%A5%E6%89%93%E5%8D%B0%E7%BB%93%E6%9E%9C.png" width="400" alt="系统实物图">

### LED报警
<img src="https://github.com/Superbigbag/edge-Intrusion-detector-imx6ull/blob/main/images/LED%E4%BA%AE.jpg" width="400" alt="系统实物图">




## 运行环境

| 组件 | 规格 |
|------|------|
| 开发板 | 百问网 IMX6ULL Pro |
| CPU | NXP i.MX6ULL, Cortex-A7 @ 800MHz |
| 内存 | 512MB DDR3 |
| 摄像头 | USB UVC (YUYV 640×480) |
| 指示灯 | GPIO131 板载 USR LED（低电平亮） |
|编译选项|-mcpu=cortex-a7 -mfpu=neon-vfpv4 -mfloat-abi=hard -O2 -ffast-math|
|工具链|Linaro GCC 6.2.1|arm-linux-gnueabihf|
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
## 参考文献
csdn上的

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
