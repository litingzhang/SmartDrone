export LD_LIBRARY_PATH=$PWD:$LD_LIBRARY_PATH;

./stereo_vslam --vocab ORBvoc.txt --settings stereo_inertial.yaml \
  --w 640 --h 480 --fps 50 --exp-us 3000 --gain 4 \
  --spi /dev/spidev0.0 --gpiochip /dev/gpiochip0 --drdy 24 --imu-hz 200 \
  --imu-start-reg 0x1F --accel-fs 16 --gyro-fs 2000 \
  --pair-ms 5 --keep-ms 200 --auto-cam-offset --auto-offset-samples 60 --auto-offset-timeout-ms 3000 \
  --udp --udp-ip 192.168.0.101 --udp-port 5000 --udp-jpeg-q 75