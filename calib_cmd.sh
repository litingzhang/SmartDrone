docker run -it --rm \
  -v ~/workspace/kalibr_data:/data \
  osrf/ros:noetic-desktop-full


source /opt/ros/noetic/setup.bash
catkin_make -DCMAKE_BUILD_TYPE=Release
source /opt/ws_kalibr/devel/setup.bash
which kalibr_calibrate_cameras
which kalibr_calibrate_imu_camera

cd /data
source /opt/ros/noetic/setup.bash
source /data/kalibr_ws/devel/setup.bash

sudo docker run -it --rm \
  -v /home/ltz/workspace/kalibr_data:/data \
  -w /data \
  swr.cn-north-4.myhuaweicloud.com/ddn-k8s/docker.io/ulong2/ie_kalibr_image:latest \
  bash

export MPLBACKEND=Agg
source /opt/ros/noetic/setup.bash
source /data/kalibr_ws/devel/setup.bash

apt install -y python3-wxgtk4.0
apt install -y python3-igraph
apt install -y python3-scipy

rosrun kalibr kalibr_calibrate_cameras \
  --bag /data/calib_A.bag \
  --target /data/aprilgrid.yaml \
  --models pinhole-radtan pinhole-radtan \
  --topics /cam0/image_raw /cam1/image_raw \
  --approx-sync 0.005

rosrun kalibr kalibr_calibrate_imu_camera \
  --bag /data/calib_B_fixed.bag \
  --cam /data/calib_A-camchain.yaml \
  --imu /data/imu.yaml \
  --target /data/aprilgrid.yaml
