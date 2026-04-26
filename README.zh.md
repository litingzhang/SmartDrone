# SmartDrone 项目说明

SmartDrone 是一个面向双目/双目惯性无人机运行时系统，包含：

- 机端原生运行时（`src/native`）
- Android 控制端（`src/android`）
- 标定与离线回放工具

## 1. 目录结构

```text
.
|- src/
|  |- native/        # 机端运行时
|  |- android/       # Android App
|- config/           # 运行配置
|- ORB_SLAM3/        # SLAM 依赖
|- third_party/      # 第三方依赖
`- output/           # 构建缓存与发布产物
```

运行时入口文件是 [src/native/main.cpp](/d:/SmartDrone/src/native/main.cpp)。

## 2. 构建

统一入口：

```bash
./scripts/build.sh smart_drone
./scripts/build.sh android
./scripts/build.sh all
./scripts/build.sh test
./scripts/build.sh replay
```

构建目标：

- `smart_drone`：只构建主运行时可执行文件
- `android`：构建 Android App，默认任务为 `:app:assembleDebug`
- `all`：先构建 `ORB_SLAM3`，再构建本地 C++ 目标和 Android App
- `test`：构建并运行主机侧单元测试
- `replay`：构建主机侧离线回放工具

构建模式参数：

- `--clean`：构建前清理对应构建目录
- `--reconfigure`：即使已有缓存也强制重新执行 CMake 配置

执行示例：

```bash
./scripts/build.sh smart_drone --clean --reconfigure
./scripts/build.sh all --clean
```

Android 构建参数：

```bash
ANDROID_GRADLE_TASK=assembleRelease ./scripts/build.sh android
BUILD_JOBS=8 ./scripts/build.sh all
```

其中 `ANDROID_GRADLE_TASK` 会把 Android 默认的 `assembleDebug` 替换成你指定的 Gradle 任务。

如果设置 `BUILD_JOBS`，会覆盖默认并行度；默认情况下 `scripts/build.sh` 使用 `$(nproc)`。

## 2B. 工作区定制说明

当前工作区在基础版本之上增加了面向 `Jetson Orin NX + 单路 UVC 拼接双目 + XFeat` 的专项适配，主要包括：

- `uvc_stereo_opencv` 按“单个 UVC 设备输出左右拼接帧”工作，当前约定总图尺寸例如 `1280x480` 表示单目 `640x480`。
- UVC 路径优先按 `YUYV/YUV2` 采集，运行时会转灰度，并在机端把单帧切成左右目后分别送入 SLAM / XFeat。
- packed stereo 模式下不再做左右时间戳配对；左右图共享同一次抓帧后的单调时钟时间戳。
- 为保证实时性，packed-UVC 相机队列被强制压到 `1`，优先保留最新帧，避免因 SLAM/XFeat 变慢导致旧图堆积。
- XFeat worker 已适配 `auto/cpu/cuda`，Jetson 上可配合 CUDA 版 `torch` 走 GPU，并启用 `fp16/autocast/cudnn.benchmark`。
- `slam_dfx` 已增加 XFeat 分段耗时与数据量统计，便于看清瓶颈是否在预处理、IPC、worker 推理或双目匹配。
- UDP 图传目标不再要求长期写死；运行时可根据当前手机连接的 active peer 动态切换图像发送目的地址。
- `UdpImageSender` 当前图像发送上限已提升到 `30 FPS`，同时手机端 `slam.input_fps` 上限也已经抬高到适合高帧率 UVC 模式的范围。
- Android 端源码已同步补充 UVC 相关曝光/增益/自动曝光、packed stereo 能力判断、XFeat 能力显示与更高的 `slam fps` 上限。

如仅维护机端而不构建手机端，可优先关注原生运行时与脚本改动；Android 侧修改主要用于与新的 UVC / XFeat 行为保持一致。

## 2A. 在 CM5 / Jetson Orin NX 上从 0 开始准备

当前仓库的主要使用方式为：

- 在一台 `x86_64 Linux` 主机构建
- 将产物部署到 `Raspberry Pi CM5` 或 `Jetson Orin NX`

执行顺序：

1. 先在目标机安装运行时依赖
2. 从目标机导出 sysroot
3. 在主机上用 `scripts/build.sh` 交叉编译
4. 把 `output/artifacts/<platform>` 上传到目标机
5. 在目标机上准备词袋、配置文件以及 XFeat Python 环境（如启用 XFeat）后运行

### 2A.1 主机构建依赖

使用 Ubuntu / Debian 类 `x86_64` 主机，并安装：

```bash
sudo apt-get update
sudo apt-get install -y \
  git cmake build-essential pkg-config rsync \
  gcc-aarch64-linux-gnu g++-aarch64-linux-gnu
```

辅助工具：

- `ninja-build`：用于手动执行 CMake + Ninja 工作流
- `adb`：用于 Android 构建与安装流程
- `python3`：用于标定转换、离线回放及辅助脚本

### 2A.2 目标机依赖

先在 CM5 或 Jetson 目标机上安装依赖，再导出 sysroot。

默认 `libcamera_stereo_ov9281` provider 需要：

```bash
sudo apt-get update
sudo apt-get install -y \
  libopencv-dev \
  libcamera-dev \
  libgpiod-dev \
  python3 python3-pip python3-venv
```

如果使用 `uvc_stereo_opencv` provider：

- 仍然需要 `OpenCV`
- 如果使用 IMU 相关模式，仍然需要 `libgpiod`
- `libcamera` 对这个 provider 本身不是必需的；如后续需要切回 libcamera 路径，应继续保留

运行前置条件：

- 目标机上存在 `ORBvoc.txt`，或者启动时通过 `--vocab /path/to/ORBvoc.txt` 指定
- 目标机上存在对应的 `config/*.yaml`
- 如果运行 `stereo-imu` / `mono-imu`，SPI IMU 和 GPIO DRDY 必须可用
- 如果走默认 MAVLink 串口链路，对应串口设备必须存在且权限正确

### 2A.3 XFeat 运行环境

XFeat 不是必需的；默认 ORB 前端不需要它。

启用 `slam.feature_frontend=xfeat` 时，需要在目标机准备 Python 环境：

```bash
./scripts/install_xfeat_cm5.sh
```

这个脚本会安装：

- Python 虚拟环境
- CPU 版 PyTorch
- `accelerated_features` 仓库
- 启用时附带安装的 OpenCV / tqdm demo 依赖

之后需要根据你的部署路径设置：

- `--xfeat-python`
- `--xfeat-repo`
- `--xfeat-worker`

### 2A.4 导出 sysroot

构建脚本默认约定：

- CM5 sysroot 默认在 `../sysroots/cm5`
- Jetson sysroot 默认在 `../sysroots/jetson-orin-nx`

可在目标机上按下列方式导出：

```bash
mkdir -p ~/sysroot-export
sudo rsync -a --delete \
  /lib /usr/lib /usr/include /usr/share/pkgconfig /usr/lib/aarch64-linux-gnu /lib/aarch64-linux-gnu \
  ~/sysroot-export/
```

然后把这份导出的目录复制到主机，例如：

- `../sysroots/cm5`
- `../sysroots/jetson-orin-nx`

sysroot 至少应包含：

- `usr/include`
- `usr/lib/aarch64-linux-gnu`
- `usr/lib`
- `lib/aarch64-linux-gnu`
- 目标机上的 `pkgconfig`

### 2A.5 为 CM5 构建

如果 sysroot 在默认位置：

```bash
./scripts/build.sh smart_drone
./scripts/build.sh all
```

如果 sysroot 在其他位置：

```bash
export SYSROOT=/opt/sysroots/cm5
./scripts/build.sh smart_drone --reconfigure
./scripts/build.sh all --clean --reconfigure
```

产物位于：

- `output/artifacts/cm5/bin/smart_drone`
- `output/artifacts/cm5/lib/`
- `output/artifacts/cm5/config/`
- `output/artifacts/cm5/scripts/`

### 2A.6 为 Jetson Orin NX 构建

如果 sysroot 在默认位置：

```bash
./scripts/build.sh smart_drone --jetson-orin-nx
./scripts/build.sh all --jetson-orin-nx
```

如果 sysroot 在其他位置：

```bash
export JETSON_SYSROOT=/opt/sysroots/jetson-orin-nx
./scripts/build.sh smart_drone --jetson-orin-nx --reconfigure
./scripts/build.sh all --jetson-orin-nx --clean --reconfigure
```

产物位于：

- `output/artifacts/jetson-orin-nx/bin/smart_drone`
- `output/artifacts/jetson-orin-nx/lib/`
- `output/artifacts/jetson-orin-nx/config/`
- `output/artifacts/jetson-orin-nx/scripts/`

当前 `scripts/build.sh` 已针对 Jetson 增加以下归一化能力：

- 支持 `orb` 模式：可单独构建 `ORB_SLAM3` 及其共享库。
- `--jetson-orin-nx` 会自动探测常见 sysroot 目录、交叉编译前缀和 host libdir，不再要求每次手工填全环境变量。
- 若设置 `SMART_DRONE_CAMERA_PROVIDER=uvc_stereo_opencv`，构建脚本会将 provider 选择直接透传给 CMake。
- 打包产物时会一并尝试带上 `ORBvoc.txt`、`scripts/xfeat_keypoint_worker.py` 和本地 `accelerated_features/` 仓库。

Jetson 版本建议采用如下方式在编译服务器上构建：

```bash
BUILD_JOBS=4 SMART_DRONE_CAMERA_PROVIDER=uvc_stereo_opencv \
  ./scripts/build.sh smart_drone --jetson-orin-nx --reconfigure
```

### 2A.7 部署到目标机

直接用内置上传脚本：

```bash
./scripts/upload.sh --platform cm5 --restart
./scripts/upload.sh --platform jetson-orin-nx --restart
```

或者显式指定目标机：

```bash
TARGET_HOST=ltz@192.168.0.105 REMOTE_DIR=/home/ltz \
  ./scripts/upload.sh --platform cm5 --restart
```

当前 `scripts/upload.sh` 已归一化为同时覆盖 CM5 / Jetson：

- `--jetson-orin-nx` 默认上传到 `nvidia@192.168.0.103:/home/nvidia/SmartDrone_cross`
- Jetson 默认采用 `artifact-root` 布局，把整个 `output/artifacts/jetson-orin-nx` 原子替换到远端目录
- 支持 `SSH_PASSWORD=...` 配合 `sshpass`，用于无人值守上传及 `sudo systemctl restart`
- artifact-root 模式会同时携带 `bin/`、`lib/`、`config/`、`scripts/`，以及打包进来的 `ORBvoc.txt` / `accelerated_features`

Jetson 建议部署命令：

```bash
SSH_PASSWORD=nvidia ./scripts/upload.sh --jetson-orin-nx --restart
```

上传脚本默认要求 `output/artifacts/<platform>` 下存在：

- `bin/smart_drone`
- `lib/libORB_SLAM3.so`
- `lib/libDBoW2.so`
- `lib/libg2o.so`
- `config/stereo.yaml`
- `config/stereo_inertial.yaml`
- `config/mono_right.yaml`
- `config/mono_inertial_right.yaml`
- 当打包后的 worker 脚本存在时，包含 `scripts/xfeat_keypoint_worker.py`

### 2A.8 目标机初次运行

不通过 systemd 启动时，执行：

```bash
cd /home/ltz
export LD_LIBRARY_PATH=/home/ltz
./smart_drone --auto-mode idle --settings config/stereo.yaml --vocab /path/to/ORBvoc.txt
```

按模式切换配置文件：

- `config/stereo.yaml`：纯双目
- `config/stereo_inertial.yaml`：双目 + IMU
- `config/mono_right.yaml`：单目
- `config/mono_inertial_right.yaml`：单目 + IMU

使用仓库中的 `smart_drone.service` 时，应先按实际环境修改：

- 其中的 `User=user` 应修改为实际用户名
- `WorkingDirectory=~`
- `LD_LIBRARY_PATH=~`
- `ExecStart=~/smart_drone --auto-mode idle`

### 2A.9 相机 provider 选择

默认构建时 provider：

```bash
-DSMART_DRONE_CAMERA_PROVIDER=libcamera_stereo_ov9281
```

当前支持：

- `libcamera_stereo_ov9281`
- `uvc_stereo_opencv`

Provider 语义：

- `libcamera_stereo_ov9281` 假设左右是两路独立 libcamera 流
- `uvc_stereo_opencv` 假设单个 UVC 设备输出左右拼接图
- 使用 `uvc_stereo_opencv` 时应配置：
  - `camera.uvc_device_index`
  - `camera.uvc_eye_width`
  - `camera.uvc_eye_height`
  - `camera.uvc_packed_stereo=true`

当前 `uvc_stereo_opencv` 的运行语义补充如下：

- 请求给 UVC 设备的实际宽度为 `2 * camera.uvc_eye_width`
- 例如总输出 `1280x480` 时，单目输入尺寸实际是 `640x480`
- 如果设备协商到 `CV_8UC2`，运行时按 `YUYV/YUV2 -> Gray` 进行转换
- 左右图直接从同一帧切分，不依赖 `camera.pair_window_ms` 做软件配对
- 为降低排队延迟，packed-UVC 路径会将 frame queue 强制收敛到 `1`

### 2A.10 当前仍需手工准备的部分

以下部分当前仍需手工完成：

- 从目标机导出 / 更新 sysroot
- 确保目标机上有 `ORBvoc.txt`
- 如果用 XFeat，单独准备 Python 运行环境
- 配置相机、串口、SPI、GPIO 的权限与设备节点
- 如果 Jetson 不走现有 `libcamera` 或 packed-UVC 路径，仍需自己接入新的 camera provider

Android 构建行为：

- 优先使用 `src/android/gradlew`
- 如果仓库里只有 `src/android/gradlew.bat`，脚本会通过 `cmd.exe` 调用
- 如果仓库里没有 Gradle Wrapper，则回退到系统 `gradle`
- 构建前会删除 `src/android/app/.cxx`，避免仓库移动或重命名后遗留旧 CMake 缓存路径

### 2.1 Jetson Orin NX 交叉编译

目录规范：

- 构建缓存统一放到 `output/build/...`
- 可上传/可分发产物统一放到 `output/artifacts/...`

Jetson Orin NX 已收编进统一构建入口：

```bash
./scripts/build.sh smart_drone --jetson-orin-nx
./scripts/build.sh all --jetson-orin-nx
```

相关文件：

- `toolchain/toolchain-jetson-orin-nx-aarch64.cmake`
- `scripts/build.sh`

默认配置：

- `JETSON_SYSROOT` 默认指向 `../sysroots/jetson-orin-nx`
- `JETSON_TOOLCHAIN_PREFIX` 默认值为 `aarch64-linux-gnu`
- native 构建目录为 `output/build/jetson-orin-nx/smart_drone`
- ORB_SLAM3 构建目录为 `output/build/jetson-orin-nx/orbslam3`
- 产物目录为 `output/artifacts/jetson-orin-nx`

执行示例：

```bash
export JETSON_SYSROOT=/opt/sysroots/jetson-orin-nx
./scripts/build.sh smart_drone --jetson-orin-nx --reconfigure
./scripts/build.sh all --jetson-orin-nx --clean --reconfigure
```

初次验证交叉工具链时应先关闭 ARM 定向调优；基础构建确认通过后，再启用目标 CPU 调优参数：

```bash
cmake -S . -B output/build/jetson-orin-nx/smart_drone \
  -DSYSROOT="$JETSON_SYSROOT" \
  -DCMAKE_TOOLCHAIN_FILE=toolchain/toolchain-jetson-orin-nx-aarch64.cmake \
  -DENABLE_ARM_CPU_TUNING=OFF
```

如果直接手动使用统一后的目录规范，可写成：

```bash
cmake -S . -B output/build/jetson-orin-nx/smart_drone \
  -DSYSROOT="$JETSON_SYSROOT" \
  -DCMAKE_TOOLCHAIN_FILE=toolchain/toolchain-jetson-orin-nx-aarch64.cmake \
  -DENABLE_ARM_CPU_TUNING=OFF
```

当前原生运行时已把相机 provider 选择提升到 CMake 变量：

```bash
-DSMART_DRONE_CAMERA_PROVIDER=libcamera_stereo_ov9281
```

当前已预留两个 provider 入口：

- `libcamera_stereo_ov9281`
- `uvc_stereo_opencv`

其中 `uvc_stereo_opencv` 当前按“单个 UVC 设备输出左右拼接图”处理：

- 用 `camera.uvc_device_index` 作为这个 UVC 设备索引
- 用 `camera.uvc_eye_width / camera.uvc_eye_height` 表示单目分辨率
- 实际向 UVC 设备请求的采集宽度为 `2 * camera.uvc_eye_width`
- 用 `camera.uvc_packed_stereo=true` 显式声明当前是左右拼接输出
- 左右图共享同一帧采集时间戳，减少软件时间戳抖动

Jetson sysroot 至少需要包含：

- `usr/include`
- `usr/lib/aarch64-linux-gnu`
- `usr/lib`
- `lib/aarch64-linux-gnu`
- 目标机上的 `pkgconfig`
- 目标机实际安装版本的 `OpenCV`、`libcamera`、`gpiod`

Jetson 支持边界：

- 原生 `aarch64 + sysroot + pkg-config` 交叉编译链路已接好
- Python/XFeat worker 仍需在目标机单独准备运行环境
- 当前已支持 `libcamera_stereo_ov9281` 和标准 `uvc_stereo_opencv` 两种编译入口；如果 Jetson 相机不走这两条链路，仍需新增 provider
- 交叉编译成功不表示 Jetson 端相机链路可直接运行

## 3. 手动构建

### 3.1 构建 CM5 运行时

```bash
cd ~/SmartDrone
./scripts/build.sh smart_drone
./scripts/build.sh all
```

### 3.2 构建 Android App

```bash
cd ~/SmartDrone/src/android
rm -rf app/.cxx app/build
./gradlew :app:assembleDebug --no-daemon
adb -d install -r app/build/outputs/apk/debug/app-debug.apk
```

Android 构建完成后，脚本还会把最新 APK 同步到 `output/artifacts/android/latest.apk`。

## 4. Android 安装

```bash
cd src/android
./gradlew :app:assembleDebug
adb install -r app/build/outputs/apk/debug/app-debug.apk
```

## 5. 上传部署

使用 `scripts/upload.sh` 把运行时可执行文件、`ORB_SLAM3` 共享库和运行配置上传到目标机：

```bash
./scripts/upload.sh
./scripts/upload.sh --restart
./scripts/upload.sh --platform jetson-orin-nx --restart
TARGET_HOST=ltz@192.168.0.103 REMOTE_DIR=/home/ltz ./scripts/upload.sh --restart
./scripts/upload.sh --adb-ip 192.168.0.100 --adb-port 33707
./scripts/upload.sh --restart --adb-ip 192.168.0.100 --adb-port 33707
```

仅安装 Android APK：

```bash
./scripts/upload.sh --adb-only --adb-ip 192.168.0.100 --adb-port 33707 \
  --apk ./src/android/app/build/outputs/apk/debug/app-debug.apk
```

上传行为：

- 上传 `output/artifacts/cm5/bin/smart_drone`
- 上传 `libORB_SLAM3.so`、`libDBoW2.so`、`libg2o.so`
- 需要时会在远端创建 `~/config`
- 上传 `config/stereo.yaml`、`config/stereo_inertial.yaml`
- 上传 `config/mono_right.yaml`、`config/mono_inertial_right.yaml`
- 当 `output/artifacts/<platform>/scripts/xfeat_keypoint_worker.py` 存在时一并上传
- 远端先写入 `*.new` 再 `mv` 原子替换

上传脚本环境变量：

- `TARGET_HOST`：默认 `ltz@192.168.0.105`
- `REMOTE_DIR`：默认 `/home/ltz`
- `REMOTE_SERVICE`：默认 `smart_drone`
- `RESTART_SERVICE`：设为 `1` 时上传后重启 systemd 服务
- `DEPLOY_PLATFORM`：默认 `cm5`，用于选择 `output/artifacts/<platform>` 下的发布件
- `UPLOAD_LAYOUT`：`flat` 或 `artifact-root`；Jetson 默认 `artifact-root`
- `SSH_PASSWORD`：设置后通过 `sshpass` 执行 `ssh/scp`，并可自动喂给 `sudo -S`
- `ADB_IP` / `ADB_PORT`：如果两者都设置了（或通过 `--adb-ip` / `--adb-port` 传入），脚本会执行 Android `adb connect` 并安装 APK

Android 部署参数：

- `--apk <path>`：用于 `adb install -r` 的 APK 路径，默认是 `output/artifacts/android/latest.apk`
- `--adb-only`：跳过 SSH/运行时上传，只执行 Android `adb connect` + APK 安装

发布平台参数：

- `--platform <name>`：显式选择 `output/artifacts/<name>` 下的发布件，例如 `cm5` 或 `jetson-orin-nx`
- `--jetson-orin-nx`：等价于 `--platform jetson-orin-nx`，并启用 Jetson 默认目标机与目录

ADB 约束：

- 启用 ADB 安装时必须同时提供 `--adb-ip` 和 `--adb-port`（或环境变量 `ADB_IP` + `ADB_PORT`）
- `--apk` 默认路径：`output/artifacts/android/latest.apk`

## 6. 运行测试

```bash
./scripts/build.sh test
```

这会构建 `smart_drone_unit_tests`，并在 `output/build/host/unit-test` 下运行 `ctest`。

覆盖范围：

- ModeManager
- RuntimeConfigService
- PerceptionPipeline
- FrameTimingTracker
- 离线回放数据集加载与 IMU 窗口提取
- 使用 fake SLAM engine 的 replay runner 主路径

## 7. 离线回放

```bash
./scripts/build.sh replay
./output/artifacts/host/offline-replay/smart_drone_offline_replay \
  --dataset tests/data \
  --out build/offline_replay_pose.csv \
  --summary-json build/offline_replay_summary.json
```

回放参数示例：

```bash
./output/artifacts/host/offline-replay/smart_drone_offline_replay \
  --dataset tests/data \
  --out build/offline_replay_pose.csv \
  --summary-json build/offline_replay_summary.json \
  --sensor-mode stereo-imu
```

当数据集运动不足以初始化双目惯性模式时，可使用纯双目回放：

```bash
./output/artifacts/host/offline-replay/smart_drone_offline_replay \
  --stereo-only \
  --out build/offline_replay_pose_stereo.csv \
  --summary-json build/offline_replay_stereo_summary.json
```

`--summary-json` 会输出一份简要统计，包含：

- `frames_out`
- `pose_valid_frames`
- `tracking_ok_frames`
- `tracking_lost_frames`
- `identity_pose_frames`

默认词袋路径会解析到 `ORB_SLAM3/Vocabulary/ORBvoc.txt`。

## 8. 离线回放基线

回放构建还注册了一个 CTest 里的纯双目集成基线：

```bash
cd output/build/host/offline-replay
ctest -R OfflineReplayStereoOnly -V
```

这条基线会运行带 `--stereo-only --summary-json ...` 的回放，然后检查：

- `tracking_ok_frames == frames_out`
- `identity_pose_frames <= 1`

这项检查用于在不过度依赖 IMU 初始化质量的前提下验证当前本地双目回放数据集是否发生回归。

也可以用 EuRoC MAV 序列做轨迹回归。`SMART_DRONE_EUROC_DATASET` 指向序列目录或其中的 `mav0` 目录，例如 `MH_01_easy`：

```bash
cd output/build/host/offline-replay
SMART_DRONE_EUROC_DATASET=/data/EuRoC/MH_01_easy \
SMART_DRONE_EUROC_MAX_FRAMES=600 \
ctest -R OfflineReplayEurocRegression -V
```

该用例会运行纯双目 offline replay，然后用 `mav0/state_groundtruth_estimate0/data.csv` 计算 SE(3) 对齐后的 ATE/RPE。默认阈值为 `ATE RMSE <= 2.5m`、`RPE translation RMSE <= 1.0m`，可通过 `SMART_DRONE_EUROC_MAX_ATE_RMSE` 和 `SMART_DRONE_EUROC_MAX_RPE_RMSE` 覆盖。未设置 `SMART_DRONE_EUROC_DATASET` 时测试自动跳过。

## 9. 标定流程

### 9.1 启动 Kalibr 环境

Docker 命令示例：

```bash
sudo docker run -it --rm \
  -v ~/workspace/kalibr_data:/data \
  -w /data \
  swr.cn-north-4.myhuaweicloud.com/ddn-k8s/docker.io/ulong2/ie_kalibr_image:latest \
  bash

export MPLBACKEND=Agg
source /opt/ros/noetic/setup.bash
source /data/kalibr_ws/devel/setup.bash

apt-get install -y --no-install-recommends python3-wxgtk4.0
apt-get install -y --no-install-recommends python3-igraph
apt-get install -y --no-install-recommends python3-scipy
```

### 9.2 生成双目标定输入

仓库根目录下的 [scripts/make_rosbag.py](/d:/SmartDrone/scripts/make_rosbag.py) 现在会一次生成两份 bag：

```bash
python3 scripts/make_rosbag.py
```

默认读取：

- `/data/calib_data_0/cam0`、`/data/calib_data_0/cam1`、`/data/calib_data_0/imu.csv`
- `/data/calib_data_1/cam0`、`/data/calib_data_1/cam1`、`/data/calib_data_1/imu.csv`

默认输出：

- `/data/calib_data_0.bag`
- `/data/calib_data_1.bag`

bag 生成器还会：

- 自动去掉尾部无效相机图像
- 裁掉最后一帧有效图像之后的 IMU 样本

然后对 `calib_data_0.bag` 做双目标定：

```bash
rosrun kalibr kalibr_calibrate_cameras \
  --bag /data/calib_runs/calib_data_0.bag \
  --target /data/aprilgrid.yaml \
  --models pinhole-radtan pinhole-radtan \
  --topics /cam0/image_raw /cam1/image_raw \
  --approx-sync 0.002
```

### 9.3 生成双目-IMU 标定参数

然后对 `calib_data_1.bag` 做双目-IMU 标定：

```bash
rosrun kalibr kalibr_calibrate_imu_camera \
  --bag /data/calib_runs/calib_data_1.bag \
  --cam /data/calib_runs/calib_data_0-camchain.yaml \
  --imu /data/imu.yaml \
  --target /data/aprilgrid.yaml
```

### 9.4 将 Kalibr 输出转换为运行时 YAML

Kalibr 完成后，使用 [scripts/convert_kalibr_to_smartdrone_yaml.py](/d:/SmartDrone/scripts/convert_kalibr_to_smartdrone_yaml.py) 将输出转换成 SmartDrone 运行时配置：

```bash
python3 scripts/convert_kalibr_to_smartdrone_yaml.py \
  --camchain /data/calib_A-camchain-imucam.yaml \
  --imu /data/imu.yaml
```

会生成：

- `config/stereo.yaml`
- `config/stereo_inertial.yaml`
- `config/mono_inertial_right.yaml`

转换器也支持直接指向远端 Kalibr 结果目录：

```bash
python3 scripts/convert_kalibr_to_smartdrone_yaml.py \
  --kalibr-root ltz@192.168.0.50:~/workspace/kalibr_data/calib_runs
```

转换器会尝试：

- 在该目录下寻找最新的 `*camchain-imucam.yaml` 或 `*camchain.yaml`
- 从上一级 Kalibr 数据目录读取 `imu.yaml`
- 在本地写出 SmartDrone 运行时 YAML

`T_b_c1` 使用说明（重要）：

- `config/stereo.yaml` 中的 `T_b_c1` 表示 `body -> c1(左目相机)` 外参。
- 纯双目模式（`SensorMode::Stereo`）下，系统会用它把 SLAM 左目位姿转换为机体系位姿再发布。
- 变换关系为：`T_w_b = T_w_c1 * (T_b_c1)^-1`。
- 若未配置 `T_b_c1`（或 `IMU.T_b_c1`），纯双目位姿将保持在相机坐标系，导致与机体坐标不一致。

## 10. 关键链路

- 发现链路：UDP 广播 `15000`，消息前缀 `smartdrone_discovery`
- 控制链路：UDP TLV（命令/ACK/状态/心跳）
- 飞控链路：MAVLink（串口 `/dev/ttyAMA0`）

## 11. 手机端运行时调参

`CMD_RUNTIME_CONFIG` 现支持以下两类 SLAM 相关调参（由 Android 控制端下发）：

- `T_b_c1` 运行时覆盖：`slam.tbc_override_enabled`（默认关闭）、`slam.tbc_tx_m / ty_m / tz_m`、`slam.tbc_roll_deg / pitch_deg / yaw_deg`（手机端范围：roll/yaw `-10.0~10.0`，pitch `-10.0~100.0`，步进 `0.1`）。
  在纯双目且 YAML 已加载 `T_b_c1` 时，手机端 `pitch_deg` 现在表示“在 YAML 基准外参上的动态俯仰增量”，适合做 `0~90°` 前视到下视切换；关闭覆盖时 UI 会回到 `pitch=0` 的中性位。
- ORB 提取器参数：
- `slam.orb_nfeatures`
- `slam.orb_scale_factor`
- `slam.orb_nlevels`
- `slam.orb_ini_th_fast`
- `slam.orb_min_th_fast`

与当前 UVC/XFeat 适配相关的补充说明：

- `camera.auto_exposure` 在 UVC packed stereo 路径下表示“是否交还给 UVC 相机固件/ISP 自动曝光”，而非 libcamera 的 AE 控制。
- `camera.pair_window_ms` 在单路 packed-UVC 路径下仅作为兼容字段保留，当前不参与左右目软件配对。
- 手机端 `slam.input_fps` 上限已提高，以适配 `1280x400@120`、`1280x480@100`、`1600x600@100` 等高帧率模式。
- 图传开启 `stream.send_image=true` 时，当前单路 UVC 帧会先切成左右目，再分别发送预览与特征点叠加结果。

参数生效方式：

- ORB 参数变更后会触发 SLAM 会话重启。
- 启动 SLAM 前，运行时会基于当前 `settings` 生成 `*.runtime_orb.yaml`，覆盖 `ORBextractor.*` 后再初始化 ORB-SLAM3。
- `slam.orb_min_th_fast` 必须小于等于 `slam.orb_ini_th_fast`，否则配置会被拒绝。

## 12. 相关文档

- 架构文档（中文）：`docs/architecture.zh.md`
- 架构文档（英文）：`docs/architecture.en.md`
- README（英文）：`README.en.md`
