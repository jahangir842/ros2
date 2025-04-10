    1  lsblk
    2  du
    3  man du
    4  man du -sh -d=2
    5  du -sh -d=2
    6  du -sh -d 2
    7  man du
    8  du -d=2
    9  du -d2
   10  du -shd2
   11  du -sh -d2
   12  du -sh -d
   13  du -d
   14  du -d=3
   15  du -d3
   16  du -s -d3
   17  du -h -d3
   18  du -h -d2
   19  sudo apt update
   20  sudo apt upgrade
   21  reboot
   22  systemctl reboot
   23  history | grep export
   24  export MAKEFLAGS="-j 4"
   25  history | grep export
   26  tzselect
   27  less .profile
   28  echo $TZ
   29  less .profile
   30  echo -e "export LC_ALL=en_US.UTF-8\nexport LANG=en_US.UTF-8\n" >> $HOME/.bashrc
   31  source $HOME/.bashrc
   32  sudo apt-get update && sudo apt-get full-upgrade
   33  sudo apt-get install -y     python3-flake8-blind-except     python3-flake8-class-newline     python3-flake8-deprecated     python3-mypy     python3-pip     python3-pytest     python3-pytest-cov     python3-pytest-mock     python3-pytest-repeat     python3-pytest-rerunfailures     python3-pytest-runner     python3-pytest-timeout     python3-rosdep2     python3-colcon-core     vcstool     build-essential
   34  mkdir -p ros2_humble/src
   35  cd ros2_humble
   36  wget https://raw.githubusercontent.com/ros2/ros2/humble/ros2.repos
   37  vcs import src < ros2.repos
   38  sudo rosdep init
   39  sudo rm  /etc/ros/rosdep/sources.list.d/20-default.list
   40  sudo rosdep init
   41  rosdep update
   42  rosdep install --from-paths src --ignore-src -y --skip-keys "fastcdr rti-connext-dds-6.0.1 urdfdom_headers python3-vcstool"
   43  echo $MAKEFLAGS
   44  history | grep make
   45  export MAKEFLAGS="-j 4"
   46  history | grep make
   47  history | grep MAKE
   48  echo $MAKEFLAGS
   49  colcon build --symlink-install
   50  echo -e "source ${path_to_ros2_source}/ros2_humble/install/setup.bash" >> $HOME/.bashrc
   51  source $HOME/.bashrc
   52  ros2 run demo_nodes_py listener
   53  cd /
   54  ls -l
   55  cd
   56  ls
   57  source .bashrc
   58  less .bashrc 
   59  source .bashrc
   60  ros2 run demo_nodes_cpp talker
   61  htop
   62  sudo passwd radxa
   63  clear
   64  ros2 run demo_nodes_cpp listener
   65  ros2
   66  ros2 topic list 
   67  history | grep ros
   68  ros2 run demo_nodes_cpp talker
   69  history
   70  sudo poweroff
   71  cd /opt
   72  ls
   73  cd 
   74  find / -name ros
   75  history
   76  history > ros2_humble_compile_history_on_rock5.txt
