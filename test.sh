# 开机自启动
sudo chmod +x ./register_service.sh
sudo ./register_service.sh

systemctl status rm
systemctl stop rm
systemctl disable rm

# 自瞄包运行
ros2 launch pb2025_infantry_bringup cuhk_bringup.launch.py use_rviz:=True

# 编译
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers 10

# 相机测试
ros2 launch hik_camera_ros2_driver hik_camera_launch.py

ros2 run camera_calibration cameracalibrator --size 7x10 --square 0.015 image:=/camera/image camera:=/camera
