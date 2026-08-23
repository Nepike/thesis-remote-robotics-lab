# Setting up the server machine

The manager talks to the robots through ROS Noetic, which pins the host to **Ubuntu / Kubuntu
20.04** — later releases do not carry Noetic packages.

## 1. ROS Noetic

```bash
sudo apt install python3-pip python3-venv python3-dev libpq-dev git curl -y

# https://wiki.ros.org/noetic/Installation/Ubuntu
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

sudo apt update
sudo apt install ros-noetic-desktop-full ros-noetic-rosserial-python
```

## 2. The catkin workspace

The workspace holding the robot packages belongs to the laboratory — it is not my code, so it
is not tracked in this repository. It is published as a release asset instead, which keeps it
out of `git clone`:

**[Releases](https://github.com/Nepike/thesis-remote-robotics-lab/releases) → `ros-workspace.tar.gz`**

```bash
cd ~
tar -xzf ~/Downloads/ros-workspace.tar.gz     # unpacks into ./ros
cd ~/ros
chmod a+x setmode.sh
./setmode.sh

source /opt/ros/noetic/setup.bash
catkin_make
```

The drivers expect the built workspace at `~/ros` — `DeviceDrivers.py` sources
`~/ros/devel/setup.bash` when launching ROS nodes, so this path is not optional.

## 3. Shell setup

```bash
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
echo "source ~/ros/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## 4. Useful extras

```bash
sudo apt install net-tools nmap          # finding robots on the network
sudo apt install joystick ros-noetic-joy # manual joystick control
```

## 5. The manager itself

```bash
git clone https://github.com/Nepike/thesis-remote-robotics-lab ~/remote-lab
cd ~/remote-lab/manager

python3 -m venv virtualenv
. virtualenv/bin/activate
pip install -r requirements.txt

python network/server.py --host 0.0.0.0 --port 8000
```

Camera and ArUco configuration is covered separately in
[`../manager/navigation/ARUCO_SETUP.md`](../manager/navigation/ARUCO_SETUP.md).
