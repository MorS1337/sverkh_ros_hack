#!/bin/bash
echo "Начинаем настройку среды"

sudo apt update
sudo apt install -y python3-pip python3-colcon-common-extensions git

echo "Ставим Python-зависимости"
pip3 install ultralytics opencv-python setuptools==58.2.0

echo "📦 Читаем зависимости ROS..."
if [ ! -d "/etc/ros/rosdep" ]; then
    sudo rosdep init
fi
rosdep update
rosdep install --from-paths src --ignore-src -r -y

echo "Готово! Теперь делай: colcon build && source install/setup.bash"
