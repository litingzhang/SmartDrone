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
`- build/            # 构建产物
```

## 2. 构建

统一入口：

```bash
./scripts/build.sh smart_drone
./scripts/build.sh android
./scripts/build.sh all
./scripts/build.sh test
./scripts/build.sh replay
```

可选参数：

```bash
ANDROID_GRADLE_TASK=assembleRelease ./scripts/build.sh android
BUILD_JOBS=8 ./scripts/build.sh all
```

## 3. Android 安装

```bash
cd src/android
./gradlew :app:assembleDebug
adb install -r app/build/outputs/apk/debug/app-debug.apk
```

## 4. 上传部署

使用根目录脚本：

```bash
./scripts/upload.sh
./scripts/upload.sh --restart
TARGET_HOST=ltz@192.168.0.103 REMOTE_DIR=/home/ltz ./scripts/upload.sh --restart
```

仅安装 Android APK：

```bash
./scripts/upload.sh --adb-only --adb-ip 192.168.0.100 --adb-port 33707 \
  --apk ./src/android/app/build/outputs/apk/debug/app-debug.apk
```

## 5. 运行测试

```bash
./scripts/build.sh test
```

包含：

- ModeManager
- RuntimeConfigService
- PerceptionPipeline
- FrameTimingTracker
- Offline Replay 关键路径

## 6. 离线回放

```bash
./scripts/build.sh replay
./build/offline-replay/tests/smart_drone_offline_replay \
  --dataset tests/data \
  --out build/offline_replay_pose.csv \
  --summary-json build/offline_replay_summary.json
```

## 7. 标定流程

1. 采集数据并生成 rosbag（`scripts/make_rosbag.py`）
2. 使用 Kalibr 完成双目与双目惯性标定
3. 使用 `scripts/convert_kalibr_to_smartdrone_yaml.py` 生成运行时 YAML：

- `config/stereo.yaml`
- `config/stereo_inertial.yaml`
- `config/mono_inertial_right.yaml`

## 8. 关键链路

- 发现链路：UDP 广播 `15000`，消息前缀 `smartdrone_discovery`
- 控制链路：UDP TLV（命令/ACK/状态/心跳）
- 飞控链路：MAVLink（串口 `/dev/ttyAMA0`）

## 9. 相关文档

- 架构文档（中文）：`docs/architecture.zh.md`
- 架构文档（英文）：`docs/architecture.en.md`
- README（英文）：`README.en.md`

