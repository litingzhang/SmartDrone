export LD_LIBRARY_PATH=$PWD:$LD_LIBRARY_PATH;

# Stereo only
sudo ./smart_drone \
  --sensor-mode stereo \
  --vocab ORBvoc.txt \
  --settings stereo_inertial.yaml \
  --w 640 --h 400 --fps 30 \
  --exp-us 6000 --gain 4 \
  --pair-ms 20 --keep-ms 200 \
  --cam1-ts-offset-ms -16 \
  --cmd-port 14550 \
  --udp --udp-ip 10.42.0.109 --udp-port 5000 --udp-jpeg-q 60

# Stereo + IMU
sudo ./smart_drone \
  --sensor-mode stereo-imu \
  --vocab ORBvoc.txt \
  --settings stereo_inertial.yaml \
  --w 640 --h 400 --fps 30 \
  --exp-us 6000 --gain 4 \
  --pair-ms 5 --keep-ms 200 \
  --auto-cam-offset --auto-offset-samples 60 --auto-offset-timeout-ms 3000 \
  --spi /dev/spidev0.0 --gpiochip /dev/gpiochip0 --drdy 24 \
  --imu-hz 500 --imu-start-reg 0x1F --accel-fs 16 --gyro-fs 2000 \
  --off-reject-ns 10000000 \
  --cmd-port 14550 \
  --udp --udp-ip 192.168.0.101 --udp-port 5000 --udp-jpeg-q 60
