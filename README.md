# SmartDrone

SmartDrone is a stereo / stereo-inertial drone runtime built around `ORB_SLAM3`, IMU input, UDP preview streaming, MAVLink pose publishing, and a small Android control app.

## Project Layout

```text
.
|- src/
|  |- smart_drone/   # CM5 flight runtime entry and device-facing modules
|  |- common/tlv/    # Shared UDP/TLV control protocol and helpers
|  `- android/       # Android app project
|- third_party/      # MAVLink and other external code
|- ORB_SLAM3/        # SLAM dependency
`- build/            # Generated build outputs
```

The runtime entry point is [`src/smart_drone/main.cpp`](/d:/SmartDrone/src/smart_drone/main.cpp).

## Build

Use the repository root `build.sh` script as the unified build entry point:

```bash
./build.sh smart_drone
./build.sh android
./build.sh all
```

Build targets:

- `smart_drone`: build the main runtime executable only
- `android`: build the Android app, defaulting to `:app:assembleDebug`
- `all`: build `ORB_SLAM3` first, then the local C++ targets and Android app

Android build behavior:

- Prefer `src/android/gradlew`
- If only `src/android/gradlew.bat` exists, the script uses `cmd.exe`
- If no Gradle wrapper exists in the repo, the script falls back to the system `gradle`
- Before building, the script removes `src/android/app/.cxx` to avoid stale CMake cache paths after repo moves or renames

Optional environment variable:

```bash
ANDROID_GRADLE_TASK=assembleRelease ./build.sh android
```

This overrides the default Android Gradle task from `assembleDebug` to the value you provide.

## Build Manually

### Build the CM5 Runtime

```bash
cd ~/SmartDrone
./build.sh smart_drone
./build.sh all
```

### Build the Android App

```bash
cd ~/SmartDrone/src/android
rm -rf app/.cxx app/build
./gradlew :app:assembleDebug --no-daemon
adb -d install -r app/build/outputs/apk/debug/app-debug.apk
```

## Calibration

### 1. Start a Kalibr Environment

Example Docker command:

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

### 2. Generate Stereo Calibration Parameters

```bash
python3 make_rosbag.py
rosrun kalibr kalibr_calibrate_cameras \
  --bag /data/calib_A.bag \
  --target /data/aprilgrid.yaml \
  --models pinhole-radtan pinhole-radtan \
  --topics /cam0/image_raw /cam1/image_raw \
  --approx-sync 0.002
```

### 3. Generate Stereo-IMU Calibration Parameters

Use this step to generate `stereo_imu.yaml`:

```bash
python3 make_rosbag.py
rosrun kalibr kalibr_calibrate_imu_camera \
  --bag /data/calib_B.bag \
  --cam /data/calib_A-camchain.yaml \
  --imu /data/imu.yaml \
  --target /data/aprilgrid.yaml
```

## Upload

Use the root `upload.sh` script to upload the runtime executable, `ORB_SLAM3` shared libraries, and calibration files to the target device:

```bash
./upload.sh
./upload.sh --restart
TARGET_HOST=ltz@192.168.0.103 REMOTE_DIR=/home/ltz ./upload.sh --restart
```

Default behavior:

- Upload `build/cmake/smart_drone`
- Upload `libORB_SLAM3.so`, `libDBoW2.so`, and `libg2o.so`
- Upload the runtime calibration file used by the device
- Upload to temporary `*.new` files first, then atomically replace the final files with `mv`

Optional environment variables:

- `TARGET_HOST`: default `ltz@192.168.0.105`
- `REMOTE_DIR`: default `/home/ltz`
- `REMOTE_SERVICE`: default `smart_drone`
- `RESTART_SERVICE`: if set to `1`, run `sudo systemctl restart smart_drone` after upload
