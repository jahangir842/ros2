Here's your shell history converted into a **Markdown** file with clear explanations and without numbering:

---

# ROS 2 Humble Compile History on ROCK5

This document records a sequence of Linux commands executed while setting up and compiling ROS 2 Humble on a ROCK5 device, along with system maintenance and environment configuration.

---

## 📦 Disk Usage & Inspection

```bash
lsblk
```
List all block devices to inspect disks and partitions.

```bash
du
```
Check disk usage of current directory without any formatting.

```bash
man du
```
Open the manual page for the `du` command to understand its options.

```bash
du -sh -d=2
du -sh -d 2
du -d=2
du -d2
du -shd2
du -sh -d2
du -sh -d
du -d
du -d=3
du -d3
du -s -d3
du -h -d3
du -h -d2
```
These are various attempts to summarize disk usage in human-readable format (`-h`), show depth of directories (`-d`), and summarize totals (`-s`). Correct syntax is:

```bash
du -h -d 2
```

---

## 🔁 System Update and Reboot

```bash
sudo apt update
sudo apt upgrade
sudo apt-get update && sudo apt-get full-upgrade
```
Update package lists and upgrade all packages to their latest versions.

```bash
reboot
systemctl reboot
```
Restart the system.

---

## 🌐 Environment Variables

```bash
history | grep export
export MAKEFLAGS="-j 4"
```
Set the number of threads for make jobs to improve compile speed.

```bash
tzselect
```
Set system timezone interactively.

```bash
less .profile
echo $TZ
```
Check timezone and inspect `.profile` file.

```bash
echo -e "export LC_ALL=en_US.UTF-8\nexport LANG=en_US.UTF-8\n" >> $HOME/.bashrc
source $HOME/.bashrc
```
Set locale environment variables for proper encoding support.

---

## 🧪 ROS 2 Build Environment

```bash
sudo apt-get install -y \
  python3-flake8-blind-except \
  python3-flake8-class-newline \
  python3-flake8-deprecated \
  python3-mypy \
  python3-pip \
  python3-pytest \
  python3-pytest-cov \
  python3-pytest-mock \
  python3-pytest-repeat \
  python3-pytest-rerunfailures \
  python3-pytest-runner \
  python3-pytest-timeout \
  python3-rosdep2 \
  python3-colcon-core \
  vcstool \
  build-essential
```
Install required Python packages, ROS tools, and build dependencies.

---

## 📁 ROS 2 Source Setup

```bash
mkdir -p ros2_humble/src
cd ros2_humble
wget https://raw.githubusercontent.com/ros2/ros2/humble/ros2.repos
vcs import src < ros2.repos
```
Set up the ROS 2 Humble workspace and import repositories using `vcs`.

---

## 🔧 ROS Dependency Management

```bash
sudo rosdep init
sudo rm /etc/ros/rosdep/sources.list.d/20-default.list
sudo rosdep init
rosdep update
```
Initialize and update rosdep, the ROS dependency manager.

```bash
rosdep install --from-paths src --ignore-src -y --skip-keys "fastcdr rti-connext-dds-6.0.1 urdfdom_headers python3-vcstool"
```
Install missing dependencies while skipping specified keys.

---

## ⚙️ Compilation and Sourcing

```bash
echo $MAKEFLAGS
history | grep make
export MAKEFLAGS="-j 4"
history | grep MAKE
echo $MAKEFLAGS
colcon build --symlink-install
```
Verify and set `MAKEFLAGS`, then build ROS 2 using `colcon`.

```bash
echo -e "source ${path_to_ros2_source}/ros2_humble/install/setup.bash" >> $HOME/.bashrc
source $HOME/.bashrc
```
Add sourcing to `.bashrc` to load ROS 2 environment on each shell start.

---

## 🚀 Run Demo Nodes

```bash
ros2 run demo_nodes_py listener
ros2 run demo_nodes_cpp talker
ros2 run demo_nodes_cpp listener
```
Run the ROS 2 demo nodes to test communication.

```bash
ros2
ros2 topic list
```
Verify `ros2` command and list available topics.

```bash
history | grep ros
```
Search for previous ROS commands in history.

---

## 🔍 File & System Utilities

```bash
cd /
ls -l
cd
ls
source .bashrc
less .bashrc
htop
sudo passwd radxa
clear
```
System exploration, process monitoring, setting password for `radxa` user, and cleaning screen.

---

## 🔍 Miscellaneous

```bash
cd /opt
ls
cd
find / -name ros
```
Navigate directories and search for files related to ROS.

---

## 🧾 Save Command History

```bash
history > ros2_humble_compile_history_on_rock5.txt
```
Export entire shell history to a file for documentation or debugging.

---

Let me know if you'd like this saved to a file or want this turned into a GitHub README!
