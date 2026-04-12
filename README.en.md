# SmartDrone

SmartDrone is a stereo / stereo-inertial drone runtime built around `ORB_SLAM3`, IMU input, UDP preview streaming, MAVLink pose publishing, and a small Android control app.

## Project Layout

```text
.
|- src/
|  |- native/        # Native runtime sources and CMake target
|  |  |- main.cpp    # CM5 flight runtime entry point
|  |  |- core/       # Runtime orchestration, sessions, modes, shared models
|  |  |- adapters/   # Camera / IMU / SLAM / telemetry concrete implementations
|  |  |- platform/   # Linux and board-specific access layers
|  |  `- common/tlv/ # Shared UDP/TLV control protocol and helpers
|  `- android/       # Android app project
|- config/           # Runtime settings files deployed with the app
|- third_party/      # MAVLink and other external code
|- ORB_SLAM3/        # SLAM dependency
`- build/            # Generated build outputs
```

The runtime entry point is [`src/native/main.cpp`](/d:/SmartDrone/src/native/main.cpp).

## Build

Use `scripts/build.sh` as the unified build entry point:

```bash
./scripts/build.sh smart_drone
./scripts/build.sh android
./scripts/build.sh all
```

Build targets:

- `smart_drone`: build the main runtime executable only
- `android`: build the Android app, defaulting to `:app:assembleDebug`
- `all`: build `ORB_SLAM3` first, then the local C++ targets and Android app
- `test`: build and run host-side unit tests
- `replay`: build the host-side offline replay tool

Android build behavior:

- Prefer `src/android/gradlew`
- If only `src/android/gradlew.bat` exists, the script uses `cmd.exe`
- If no Gradle wrapper exists in the repo, the script falls back to the system `gradle`
- Before building, the script removes `src/android/app/.cxx` to avoid stale CMake cache paths after repo moves or renames

Optional environment variable:

```bash
ANDROID_GRADLE_TASK=assembleRelease ./scripts/build.sh android
```

This overrides the default Android Gradle task from `assembleDebug` to the value you provide.

Optional build parallelism override:

```bash
BUILD_JOBS=8 ./scripts/build.sh replay
```

By default `scripts/build.sh` uses `$(nproc)`.

## Build Manually

### Build the CM5 Runtime

```bash
cd ~/SmartDrone
./scripts/build.sh smart_drone
./scripts/build.sh all
```

### Build the Android App

```bash
cd ~/SmartDrone/src/android
rm -rf app/.cxx app/build
./gradlew :app:assembleDebug --no-daemon
adb -d install -r app/build/outputs/apk/debug/app-debug.apk
```

## Tests

Run the host-side unit tests with:

```bash
./scripts/build.sh test
```

This builds the `smart_drone_unit_tests` target and runs `ctest` in `build/unit-test`.

Current host-side coverage includes:

- `FrameTimingTracker`
- `ModeManager`
- `PerceptionPipeline`
- `RuntimeConfigService`
- replay dataset loading and IMU window extraction
- replay runner plumbing with a fake SLAM engine

## Offline Replay

Use the host-side offline replay tool to feed recorded stereo images and IMU data into the local replay pipeline and export a pose CSV:

```bash
./scripts/build.sh replay
./build/offline-replay/tests/smart_drone_offline_replay
```

Useful options:

```bash
./build/offline-replay/tests/smart_drone_offline_replay \
  --dataset tests/data \
  --out build/offline_replay_pose.csv \
  --summary-json build/offline_replay_summary.json \
  --sensor-mode stereo-imu
```

Stereo-only replay is often useful when a dataset does not contain enough motion to initialize stereo-inertial mode:

```bash
./build/offline-replay/tests/smart_drone_offline_replay \
  --stereo-only \
  --out build/offline_replay_pose_stereo.csv \
  --summary-json build/offline_replay_stereo_summary.json
```

`--summary-json` writes a small machine-readable summary with:

- `frames_out`
- `pose_valid_frames`
- `tracking_ok_frames`
- `tracking_lost_frames`
- `identity_pose_frames`

The default vocabulary path resolves to `ORB_SLAM3/Vocabulary/ORBvoc.txt`.

## Offline Replay Baseline

The replay build also registers a stereo-only integration baseline in CTest:

```bash
cd build/offline-replay
ctest -R OfflineReplayStereoOnly -V
```

The baseline runs the replay tool with `--stereo-only --summary-json ...` and then checks:

- `tracking_ok_frames == frames_out`
- `identity_pose_frames <= 1`

This gives us a quick regression check for the current local stereo replay dataset without depending on IMU initialization quality.

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

The repository root [`scripts/make_rosbag.py`](/d:/SmartDrone/scripts/make_rosbag.py) now generates both bags in one run:

```bash
python3 scripts/make_rosbag.py
```

By default it reads:

- `/data/calib_data_0/cam0`, `/data/calib_data_0/cam1`, `/data/calib_data_0/imu.csv`
- `/data/calib_data_1/cam0`, `/data/calib_data_1/cam1`, `/data/calib_data_1/imu.csv`

And writes:

- `/data/calib_data_0.bag`
- `/data/calib_data_1.bag`

The bag generator also:

- removes trailing invalid camera images automatically
- trims trailing IMU samples after the last valid camera frame

Then run stereo camera calibration on `calib_data_0.bag`:

```bash
rosrun kalibr kalibr_calibrate_cameras \
  --bag /data/calib_runs/calib_data_0.bag \
  --target /data/aprilgrid.yaml \
  --models pinhole-radtan pinhole-radtan \
  --topics /cam0/image_raw /cam1/image_raw \
  --approx-sync 0.002
```

### 3. Generate Stereo-IMU Calibration Parameters

Then run stereo-IMU calibration on `calib_data_1.bag`:

```bash
rosrun kalibr kalibr_calibrate_imu_camera \
  --bag /data/calib_runs/calib_data_1.bag \
  --cam /data/calib_runs/calib_data_0-camchain.yaml \
  --imu /data/imu.yaml \
  --target /data/aprilgrid.yaml
```

### 4. Convert Kalibr Results To Runtime YAML

After Kalibr finishes, use [`scripts/convert_kalibr_to_smartdrone_yaml.py`](/d:/SmartDrone/scripts/convert_kalibr_to_smartdrone_yaml.py) to convert the Kalibr output into SmartDrone runtime configs:

```bash
python3 scripts/convert_kalibr_to_smartdrone_yaml.py \
  --camchain /data/calib_A-camchain-imucam.yaml \
  --imu /data/imu.yaml
```

This generates:

- `config/stereo.yaml`
- `config/stereo_inertial.yaml`
- `config/mono_inertial_right.yaml`

You can also point the converter directly at a remote Kalibr result directory:

```bash
python3 scripts/convert_kalibr_to_smartdrone_yaml.py \
  --kalibr-root ltz@192.168.0.50:~/workspace/kalibr_data/calib_runs
```

The converter will try to:

- find the latest `*camchain-imucam.yaml` or `*camchain.yaml` under that directory
- read `imu.yaml` from the parent Kalibr data directory
- write the SmartDrone YAML files locally

## Upload

Use `scripts/upload.sh` to upload the runtime executable, `ORB_SLAM3` shared libraries, and runtime config files to the target device:

```bash
./scripts/upload.sh
./scripts/upload.sh --restart
TARGET_HOST=ltz@192.168.0.103 REMOTE_DIR=/home/ltz ./scripts/upload.sh --restart
./scripts/upload.sh --adb-ip 192.168.0.100 --adb-port 33707
./scripts/upload.sh --restart --adb-ip 192.168.0.100 --adb-port 33707
./scripts/upload.sh --adb-only --adb-ip 192.168.0.100 --adb-port 33707
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
- `ADB_IP` / `ADB_PORT`: if both are set (or provided with `--adb-ip` / `--adb-port`), run Android `adb connect` and install APK

Optional Android deploy argument:

- `--apk <path>`: APK path for `adb install -r`, default `src/android/app/build/outputs/apk/debug/app-debug.apk`
- `--adb-only`: skip SSH/runtime upload and run Android `adb connect` + APK install only

