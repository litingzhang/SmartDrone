# SmartDrone

SmartDrone is a stereo / stereo-inertial drone runtime built around `ORB_SLAM3`, IMU input, UDP preview streaming, MAVLink pose publishing, and a small Android control app.

## Project Layout

```text
.
|- src/
|  |- main.cpp       # CM5 flight runtime entry point
|  |- core/          # Runtime orchestration, sessions, modes, shared models
|  |- adapters/      # Camera / IMU / SLAM / telemetry concrete implementations
|  |- platform/      # Linux and board-specific access layers
|  |- common/tlv/    # Shared UDP/TLV control protocol and helpers
|  `- android/       # Android app project
|- config/           # Runtime settings files deployed with the app
|- third_party/      # MAVLink and other external code
|- ORB_SLAM3/        # SLAM dependency
`- build/            # Generated build outputs
```

The runtime entry point is [`src/main.cpp`](/d:/SmartDrone/src/main.cpp).

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

The repository root [`make_rosbag.py`](/d:/SmartDrone/make_rosbag.py) now generates both bags in one run:

```bash
python3 make_rosbag.py
```

By default it reads:

- `/data/calib_A/cam0`, `/data/calib_A/cam1`, `/data/calib_A/imu.csv`
- `/data/calib_B/cam0`, `/data/calib_B/cam1`, `/data/calib_B/imu.csv`

And writes:

- `/data/calib_A.bag`
- `/data/calib_B.bag`

The bag generator also:

- removes trailing invalid camera images automatically
- trims trailing IMU samples after the last valid camera frame

Then run stereo camera calibration on `calib_A.bag`:

```bash
rosrun kalibr kalibr_calibrate_cameras \
  --bag /data/calib_A.bag \
  --target /data/aprilgrid.yaml \
  --models pinhole-radtan pinhole-radtan \
  --topics /cam0/image_raw /cam1/image_raw \
  --approx-sync 0.002
```

### 3. Generate Stereo-IMU Calibration Parameters

Then run stereo-IMU calibration on `calib_B.bag`:

```bash
rosrun kalibr kalibr_calibrate_imu_camera \
  --bag /data/calib_B.bag \
  --cam /data/calib_A-camchain.yaml \
  --imu /data/imu.yaml \
  --target /data/aprilgrid.yaml
```

### 4. Convert Kalibr Results To Runtime YAML

After Kalibr finishes, use [`convert_kalibr_to_smartdrone_yaml.py`](/d:/SmartDrone/convert_kalibr_to_smartdrone_yaml.py) to convert the Kalibr output into SmartDrone runtime configs:

```bash
python3 convert_kalibr_to_smartdrone_yaml.py \
  --camchain /data/calib_A-camchain-imucam.yaml \
  --imu /data/imu.yaml
```

This generates:

- `config/stereo.yaml`
- `config/stereo_inertial.yaml`
- `config/mono_inertial_right.yaml`

You can also point the converter directly at a remote Kalibr result directory:

```bash
python3 convert_kalibr_to_smartdrone_yaml.py \
  --kalibr-root ltz@192.168.0.50:~/workspace/kalibr_data/calib_runs
```

The converter will try to:

- find the latest `*camchain-imucam.yaml` or `*camchain.yaml` under that directory
- read `imu.yaml` from the parent Kalibr data directory
- write the SmartDrone YAML files locally

## Upload

Use the root `upload.sh` script to upload the runtime executable, `ORB_SLAM3` shared libraries, and runtime config files to the target device:

```bash
./upload.sh
./upload.sh --restart
TARGET_HOST=ltz@192.168.0.103 REMOTE_DIR=/home/ltz ./upload.sh --restart
```

Default behavior:

- Upload `build/cmake/smart_drone`
- Upload `libORB_SLAM3.so`, `libDBoW2.so`, and `libg2o.so`
- Create `~/config` on the target when needed
- Upload `config/stereo.yaml` and `config/stereo_inertial.yaml`
- Upload `config/mono_right.yaml` and `config/mono_inertial_right.yaml`
- Upload to temporary `*.new` files first, then atomically replace the final files with `mv`

Optional environment variables:

- `TARGET_HOST`: default `ltz@192.168.0.105`
- `REMOTE_DIR`: default `/home/ltz`
- `REMOTE_SERVICE`: default `smart_drone`
- `RESTART_SERVICE`: if set to `1`, run `sudo systemctl restart smart_drone` after upload
