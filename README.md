
# 2. Установка ROS

Алгоритм установки и обустройства ros-noetic на чистой Kubuntu 20.04
(более новые версии kubuntu не поддерживают ROS Noetic )

```bash
# Requirements
sudo apt install python3-pip python3-venv python3-dev libpq-dev git curl ros-noetic-rosserial-python  -y

# --- Part from here https://wiki.ros.org/noetic/Installation/Ubuntu ---
# Setup your computer to accept software from packages.ros.org
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

# Set up your keys
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

# Installation
sudo apt update
sudo apt install ros-noetic-desktop-full


# --- Ros setup ---
# somehow get file like "ros-kmn-2025-06-17" and put its content into ~/ros
# it is probably at somewhere like /YandexDisk/Projects/YARP/YARP-13/ros-yarp13/
# i will use my github repo here
git clone https://github.com/Nepike/remote_lab ~/remote_lab
mv ~/remote_lab/ros ~/

cd ~/ros
chmod a+x setmode.sh
./setmode.sh
catkin_make

# Bashrc settings
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
echo "source ~/ros/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc


# --- Some usefull stuff ---
sudo apt install net-tools nmap

# Joystick
sudo apt-get install joystick ros-noetic-joy

```

# 3. Remote Lab
```bash

# Get files (If you installed ros via my repo - skip this)
git clone https://github.com/Nepike/remote_lab ~/remote_lab

cd ~/remote_lab/progs/remote_lab

# Virtual env setup
python3 -m venv virtualenv
. virtualenv/bin/activate
pip install -r requirements.txt
```