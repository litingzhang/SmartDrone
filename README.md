# SmartDrone

## Project Layout

```
.
├─ src/
│  ├─ smart_drone/    # CM5 flight runtime entry and device-facing modules
│  ├─ common/tlv/     # shared UDP/TLV control protocol and server helpers
│  └─ android/        # Android app project
├─ third_party/       # MAVLink and other external code
├─ ORB_SLAM3/         # SLAM dependency
└─ build/             # generated build outputs
```

**Build CM5 main program**

```
cd ~/SmartDrone
./build.sh smart_drone   # only build the runtime executable
./build.sh all           # build ORB-SLAM3 first, then smart_drone
```

**Build Android APP**

```
cd ~/SmartDrone/src/android
rm -rf app/.cxx app/build
./gradlew :app:assembleDebug --no-daemon
adb -d install -r src/android/app/build/outputs/apk/debug/app-debug.apk
```

`smart_drone` source entry is now at `src/smart_drone/main.cpp`.
Shared TLV control code now lives under `src/common/tlv/`.

**Calibration**

1. Start a docker env, eg.

   ```
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

2. generate stereo calib param

   ```
   python3 make_rosbag.py;
   rosrun kalibr kalibr_calibrate_cameras \
     --bag /data/calib_A.bag \
     --target /data/aprilgrid.yaml \
     --models pinhole-radtan pinhole-radtan \
     --topics /cam0/image_raw /cam1/image_raw \
     --approx-sync 0.002
   ```

3. generate stereo-imu calib param for genarating stereo_imu.yaml

   ```
   python3 make_rosbag.py;
   rosrun kalibr kalibr_calibrate_imu_camera \
     --bag /data/calib_B.bag \
     --cam /data/calib_A-camchain.yaml \
     --imu /data/imu.yaml \
     --target /data/aprilgrid.yaml
   ```

## Build

统一构建入口使用仓库根目录下的 `build.sh`：

```bash
./build.sh smart_drone
./build.sh android
./build.sh all
```

参数说明：

- `smart_drone`：编译主运行程序
- `android`：编译 Android App，默认执行 `:app:assembleDebug`
- `all`：先编译 `ORB_SLAM3`，再编译本地 C++ 目标和 Android App

Android 构建规则：

- 优先使用 `src/android/gradlew`
- 如果只有 `src/android/gradlew.bat`，脚本会通过 `cmd.exe` 调用
- 如果仓库里没有 Gradle Wrapper，则回退到系统安装的 `gradle`
- 构建前会自动清理 `src/android/app/.cxx`，避免仓库改名或移动后残留旧的 CMake 绝对路径缓存

可选环境变量：

```bash
ANDROID_GRADLE_TASK=assembleRelease ./build.sh android
```

这会把 Android 构建任务从默认的 `assembleDebug` 切换成你指定的 Gradle task。

## Upload

可以使用根目录下的 `upload.sh` 把可执行文件、ORB-SLAM3 相关动态库和标定文件上传到远端：

```bash
./upload.sh
./upload.sh --restart
TARGET_HOST=ltz@192.168.0.103 REMOTE_DIR=/home/ltz ./upload.sh --restart
```

默认行为：

- 上传 `build/cmake/smart_drone`
- 上传 `libORB_SLAM3.so`、`libDBoW2.so`、`libg2o.so`
- 上传 `src/ov9281_icm42688/stereo_inertial.yaml`
- 先传到远端 `*.new`，再通过 `mv` 原子替换，避免直接覆盖正在使用的文件

可选环境变量：

- `TARGET_HOST`：默认 `ltz@192.168.0.105`
- `REMOTE_DIR`：默认 `/home/ltz`
- `REMOTE_SERVICE`：默认 `smart_drone`
- `RESTART_SERVICE`：设为 `1` 时，上传后执行 `sudo systemctl restart smart_drone`
