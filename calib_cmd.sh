sudo docker run -it --rm \
  -v /home/ltz/workspace/kalibr_data:/data \
  -w /data \
  swr.cn-north-4.myhuaweicloud.com/ddn-k8s/docker.io/ulong2/ie_kalibr_image:latest \
  bash

export MPLBACKEND=Agg
source /opt/ros/noetic/setup.bash
source /data/kalibr_ws/devel/setup.bash

# Record both calib_A and calib_B with 640x400 / 30fps / 500Hz to match the
# actual OV9281 RAW mode selected by libcamera on this platform.
# Example recorder command:
# ./calib_recorder --out /data/calib_A --w 640 --h 400 --fps 30 --imu-hz 500 --ae-disable --exp-us 6000 --gain 4 --pair-tol-us 5000

apt-get install -y --no-install-recommends python3-wxgtk4.0
apt-get install -y --no-install-recommends python3-igraph
apt-get install -y --no-install-recommends python3-scipy

export KALIBR_MANUAL_FOCAL_LENGTH_INIT=390
rosrun kalibr kalibr_calibrate_cameras \
  --bag /data/calib_A.bag \
  --target /data/aprilgrid.yaml \
  --models pinhole-radtan pinhole-radtan \
  --topics /cam0/image_raw /cam1/image_raw \
  --approx-sync 0.005


rosrun kalibr kalibr_calibrate_imu_camera \
  --bag /data/calib_B.bag \
  --cam /data/calib_A-camchain.yaml \
  --imu /data/imu.yaml \
  --target /data/aprilgrid.yaml
