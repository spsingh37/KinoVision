 #!/bin/bash

# install ROS2 Camera Calibration package
# https://navigation.ros.org/tutorials/docs/camera_calibration.html

# downgrade setuptools version to 58.2.0
# which is the last version to work with ros2 python packages without any warnings
# https://robotics.stackexchange.com/questions/101255
pip install setuptools==58.2.0

sudo apt install ros-humble-camera-calibration-parsers
sudo apt install ros-humble-camera-info-manager
sudo apt install ros-humble-launch-testing-ament-cmake

cd ~
git clone -b humble https://github.com/ros-perception/image_pipeline.git
cd ~/image_pipeline
colcon build --symlink-install
