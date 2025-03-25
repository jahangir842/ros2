### Installing Ubuntu 24.04 on Raspberry Pi 5  

1. **Get the Raspberry Pi Imager**:  
   - Download the [Raspberry Pi Imager](https://www.raspberrypi.org/software/) for your OS (Windows, macOS, or Linux).
     
2. Alternatively, install it on Linux using:  
     ```bash
     sudo apt install rpi-imager
     ```

3. **Insert a microSD card**:  
   Insert your microSD card (at least 16GB recommended) into your computer using a card reader.

4. **Open Raspberry Pi Imager**:  
   Launch the Raspberry Pi Imager after installation.

5. **Choose Ubuntu 24.04 LTS**:  
   In the Raspberry Pi Imager, click on **CHOOSE OS** and select **General Purpose OS** and select **Ubuntu**. Then, choose **Ubuntu 24.04 LTS (Server)** for your architecture (usually ARM for Raspberry Pi 5).

6. **Select the microSD card**:  
   Click on **CHOOSE STORAGE** and select your microSD card.

7. **Write the image**:  
   After selecting the microSD card, click **WRITE** to begin writing the Ubuntu 24.04 LTS image to the card. This will take a few minutes.

8. **Eject and Insert the microSD card**:  
   Once the writing process is complete, safely eject the microSD card from your computer and insert it into the Raspberry Pi 5.

9. **Power on Raspberry Pi**:  
   Connect your Raspberry Pi to a monitor, keyboard, and mouse, and then power it on. Follow the on-screen instructions to complete the initial setup, including setting up your language, region, and network.

---

### Install ROS2 on Ubuntu 24.04

Ubuntu 24.04 is compatible with ROS2 **Humbl**e Hawksbill, so you should install that version. Here are the steps to install ROS2 Jazzy:

- **Official Documentation**: https://docs.ros.org/en/jazzy/index.html

#### Steps to install ROS2 Jazzy:

**Set up the ROS2 GPG key**:  
   Add the ROS2 repository's GPG key to your system with the following command:

   ```bash
   sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
   ```

**Add the ROS2 repository**:  
   Next, add the ROS2 repository to your APT sources list. This will allow you to download ROS2 packages directly from the official repository.

   ```bash
   echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
   ```

**Install ROS2 Jazzy desktop version**:  
   Install the ROS2 Jazzy desktop version (including the ROS2 core libraries and additional tools):

   ```bash
   sudo apt install ros-dev-tools
   sudo apt update
   sudo apt upgrade
   sudo apt install ros-rolling-desktop
   source /opt/ros/rolling/setup.bash
   ros2 run demo_nodes_cpp talker
   ```

**Source ROS2 environment**:  
   After installing ROS2, source the environment setup script to make the ROS2 tools available:

   ```bash
   source /opt/ros/Jazzy/setup.bash
   ```

**Run a demo node**:  
   You can verify your installation by running a demo node:

   ```bash
   ros2 run demo_nodes_cpp talker
   ```

This will start a basic "talker" node that sends out messages in a ROS2 communication channel.

---

