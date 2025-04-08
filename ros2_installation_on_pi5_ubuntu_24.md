### Installing Ubuntu 24.04 on Raspberry Pi 5  

1. **Get the Raspberry Pi Imager**:  
   - Download the **Raspberry Pi Imager:** [https://www.raspberrypi.org/software/](https://www.raspberrypi.org/software/) for your OS (Windows, macOS, or Linux).
     
2. Alternatively, install it on Linux using:  
     ```bash
     sudo apt install rpi-imager
     ```

3. **Insert a microSD card**:  
   Insert your microSD card (at least 16GB recommended) into your computer using a card reader.

4. **Open Raspberry Pi Imager**:  
   Launch the Raspberry Pi Imager after installation.

5. **Choose Ubuntu 24.04 LTS**:  
   In the Raspberry Pi Imager, click on **CHOOSE OS** and select **General Purpose OS** and select **Ubuntu**. Then, choose **Ubuntu 24.04 LTS (Desktop)** for your architecture (usually ARM for Raspberry Pi 5).

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

- **Official Documentation**: [https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)

**Source ROS2 environment**:  
   After installing ROS2, source the environment setup script to make the ROS2 tools available:

   ```bash
   source /opt/ros/Jazzy/setup.bash
   ```

---

### **Run a Demo Node**:  
You can verify your ROS 2 installation by running both a **talker** and **listener** node. The **talker** node sends messages, while the **listener** node receives them.

1. **Start the talker node**:  
   Open one terminal and run the following command to start the **talker** node, which will send messages on a ROS 2 topic.

   ```bash
   ros2 run demo_nodes_cpp talker
   ```

   This node will start sending messages in a ROS 2 communication channel.

2. **Start the listener node**:  
   Open another terminal and run the following command to start the **listener** node, which listens for the messages sent by the **talker**.

   ```bash
   ros2 run demo_nodes_cpp listener
   ```

   The listener will print the received messages to the terminal as they arrive.

---

This will set up a basic communication system in ROS 2, where the **talker** node publishes messages, and the **listener** node subscribes to and prints those messages. This is a simple way to verify that your ROS 2 installation is working correctly.

