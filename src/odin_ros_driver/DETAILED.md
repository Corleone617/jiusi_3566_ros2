# odin_ros_driver 变更记录

## 2026-07-02: 修复解耦遗漏 — 硬件 RGB 流激活条件未同步

### 问题

2026-06-24 解耦修改仅放宽了回调入口的守卫（line 987 `g_sendrgb` → `g_sendrgb || g_sendrgb_compressed`），但遗漏了**设备端 RGB 硬件流的激活/停用条件**（line 1833）。

```cpp
// line 1833: 仍只判断 g_sendrgb
if (g_sendrgb) {
    lidar_activate_stream_type(odinDevice, LIDAR_DT_RAW_RGB);
} else {
    lidar_deactivate_stream_type(odinDevice, LIDAR_DT_RAW_RGB);  // ← 直接关流
}
```

当 `sendrgb=0, sendrgbcompressed=1` 时：
1. 硬件流被 `lidar_deactivate_stream_type` 关闭 → 设备不发送任何 RGB 数据
2. 回调 `case LIDAR_DT_RAW_RGB:` 永不被触发
3. `publishRgb()` 从未调用 → 压缩图发布失效
4. 前次解耦修改形同虚设

### 修改 4: `src/host_sdk_sample.cpp` 行 1833

硬件流激活条件与回调守卫保持一致：

```diff
-        if (g_sendrgb) {
+        if (g_sendrgb || g_sendrgb_compressed) {
             lidar_activate_stream_type(odinDevice, LIDAR_DT_RAW_RGB);
         } else {
             lidar_deactivate_stream_type(odinDevice, LIDAR_DT_RAW_RGB);
         }
```

### 修改 5: `src/host_sdk_sample.cpp` 行 1825-1830

日志补充 `g_sendrgb_compressed` 输出，方便调试：

```diff
-                "Stream config: RGB=%d, IMU=%d, ODOM=%d, DTOF=%d, CLOUD_SLAM=%d",
-                g_sendrgb, g_sendimu, g_sendodom, g_senddtof, g_sendcloudslam);
+                "Stream config: RGB=%d, RGB_COMP=%d, IMU=%d, ODOM=%d, DTOF=%d, CLOUD_SLAM=%d",
+                g_sendrgb, g_sendrgb_compressed, g_sendimu, g_sendodom, g_senddtof, g_sendcloudslam);
```

ROS1 / ROS2 分支均已同步修改。

### 编译验证

```bash
colcon build --packages-select odin_ros_driver --cmake-args -DBUILD_SYSTEM=ROS2
# 编译通过
```

---

## 2026-06-24: CPU 功耗优化 — 上游更新 + 流控 + 解耦 raw/comprressed RGB

### 一、拉取上游更新

```bash
git pull origin main
# 7c97ac2 (0.10.4) → 388440c (0.12.0)
```

**更新内容** (22 files changed):

| 类别 | 文件 | 说明 |
|------|------|------|
| SDK 静态库 | `lib/liblydHostApi_arm.a` (517→592KB), `lib/liblydHostApi_amd.a` (481→554KB) | SDK 更新 |
| API 头文件 | `include/lidar_api.h` (+196), `include/lidar_api_type.h` (+305) | 新增 AE/AWB 等接口 |
| 核心源码 | `src/host_sdk_sample.cpp` (+281) | 功能增强 |
| 新服务 | `srv/SetAe.srv`, `srv/GetAe.srv`, `srv/SetAwb.srv`, `srv/GetAwb.srv` | 自动曝光/白平衡控制 |
| 构建/脚本 | `CMakeLists.txt` (+51), `script/build_ros2.sh`, `script/rosbag2_qos.yaml` | 构建更新 |
| 文档 | `README.md` (+262), `CHANGELOG.md`, `package.xml` | — |

---

### 二、配置变更 (`config/control_command.yaml`)

#### 变更清单

| 参数 | 原值 | 新值 | 目的 |
|------|------|------|------|
| `devstatuslog` | 1 | **0** | 关闭设备状态 CSV 写入（减少磁盘 I/O） |
| `sendcloudrender` | 1 | **0** | 关闭点云 RGB 渲染（跳过 `try_process_pair()` PCL 运算） |
| `sendrgb` | 1 | **0** | 关闭 raw RGB (BGR8) 解码，保留压缩图像 |

#### 生效原理

```
sendcloudrender: 0
  → 主循环中 g_ros_object->try_process_pair() 不再执行
  → 跳过 PCL 点云着色 + RGB 队列匹配

devstatuslog: 0
  → 设备状态回调中 fprintf + fflush 跳过
  → 状态 CSV 文件不再写入

sendrgb: 0 (配合代码变更)
  → publishRgb() 仍被调用（因为 sendrgbcompressed=1）
  → 跳过 cv::imdecode (JPEG→BGR8)
  → 压缩图像直通保留
```

---

### 三、代码变更：解耦 raw RGB 与压缩图像

#### 问题背景

原始代码中 `sendrgb` 开关同时控制 raw RGB 解码和压缩图像发布：

```
host_sdk_sample.cpp:987  if (g_sendrgb) → publishRgb(stream)
host_sdk_sample.h:802    publishRgb() {
                             cv::imdecode()           // CPU 密集
                             rgb_pub_->publish()      // /odin1/image (raw BGR8)
                             compressed_rgb_pub_->publish()  // /odin1/image/compressed
                         }
```

关闭 `sendrgb` 会连带丢失 bag_logd 需要的压缩图像。需要将二者解耦。

#### 修改 1: `src/host_sdk_sample.cpp` 行 987

**调用守卫扩展**：压缩图像开关也能触发 `publishRgb` 调用

```diff
-            if (g_sendrgb) {
+            if (g_sendrgb || g_sendrgb_compressed) {
                 g_ros_object->publishRgb((capture_Image_List_t *)&data->stream);
             }
```

#### 修改 2: `include/host_sdk_sample.h` 行 83-84

**添加 extern 声明**（`g_sendrgb` 和 `g_sendrgb_compressed` 此前仅在 .cpp 中定义）：

```diff
 extern int g_sendcloudrender;
+extern int g_sendrgb;
+extern int g_sendrgb_compressed;
```

#### 修改 3: `include/host_sdk_sample.h` 函数 `publishRgb()` (行 812-911)

**重构**：将压缩图像发布提到 `cv::imdecode` 之前，所有 RGB 处理用 `if (g_sendrgb)` 守卫。

重构后的执行流：

```
publishRgb(stream) {
    jpeg_data = 从设备缓冲包装 vector<uint8_t>   // 便宜：仅指针包装

    // ── 压缩图像直通（无解码，始终执行）──
    compressed_rgb_pub_->publish(jpeg_msg)         // /odin1/image/compressed

    // ── Raw RGB 处理（仅 sendrgb=1 时执行）──
    if (g_sendrgb) {
        cv::imdecode(jpeg_data)                    // JPEG→BGR8（CPU 密集）
        cv_bridge 转换
        rgb_pub_->publish()                        // /odin1/image
        undistort + undistort_rgb_pub_->publish()  // /odin1/image/undistorted
        data_logger_ 入队                          // 二进制日志
        cloud render 队列推送                      // sendcloudrender 辅助
    }
}
```

#### 效果

| 配置 | `/odin1/image` | `/odin1/image/compressed` | cv::imdecode |
|------|:---:|:---:|:---:|
| `sendrgb: 1, sendrgbcompressed: 1` | ✅ | ✅ | 执行 |
| `sendrgb: 0, sendrgbcompressed: 1` | ❌ | ✅ | **跳过** |
| `sendrgb: 0, sendrgbcompressed: 0` | ❌ | ❌ | 跳过 |

---

### 四、编译验证

```bash
colcon build --symlink-install --packages-select odin_ros_driver
# 编译通过，仅 libtbb 版本警告（不影响功能）
```

---

### 五、当前完整配置快照

```yaml
# control_command.yaml 关键开关
sendrgbcompressed: 1    # 压缩图像 (JPEG 直通) → bag_logd 记录
sendrgb: 0              # raw RGB 解码关闭
sendcloudrender: 0      # 点云 RGB 渲染关闭
senddepth: 0            # 深度补全关闭
sendreprojection: 0     # 点云重投影关闭
sendoverlay: 0          # 图像叠加关闭
devstatuslog: 0         # 设备状态 CSV 关闭
recorddata: 0           # MindCloud 录制关闭
pubintensitygray: 0     # 灰度强度图关闭
showpath: 0             # 轨迹可视化关闭
showcamerapose: 0       # 相机姿态可视化关闭
sendimagemask: 0        # 图像掩码传输关闭
resetalgo: 0            # 算法重置关闭
```

所有 CPU 密集型可选功能已关闭，仅保留核心 SLAM 传感器流（IMU、odom、dtof、cloud_slam）和压缩图像。

---

## 2026-06-24: image_mask 功能启用 + 路径解析 + RPATH 修复

### sendimagemask 启用

**文件**: `config/control_command.yaml`
- `sendimagemask: 1` — 启动时将 mask.png 传输到 Odin1 设备
- `image_mask_abs_path: "config/mask.png"` — 相对路径

### 代码变更：运行时路径解析

**文件**: `src/host_sdk_sample.cpp` 行 1712-1735

相对路径（不以 `/` 开头）自动通过 `ament_index_cpp::get_package_share_directory("odin_ros_driver")` 解析为绝对路径。部署到 rk3566 时路径自适配，无需手动修改。

### CMakeLists.txt RPATH 修复

**文件**: `CMakeLists.txt:475`

```diff
- LINK_FLAGS "-Wl,--no-as-needed -Wl,--rpath=${LIB_DIR}"
+ LINK_FLAGS "-Wl,--no-as-needed"
```

移除硬编码到源码树 (`/home/jiusi/jiusi_ws/src/odin_ros_driver/lib`) 的 RUNPATH。SDK 为静态库 (.a)，不需要 RPATH。

