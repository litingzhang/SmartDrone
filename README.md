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
adb -d install -r app/build/outputs/apk/debug/app-debug.apk
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

