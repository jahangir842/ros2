### Install ROS2 with Debian 12 on Rock5

This guide will walk you through the process of installing Debian 12 on your Rock5 device by writing the Debian image to an SD card using Etcher, then booting the system on your Rock5.

#### Prerequisites:
1. **Rock5 device** (e.g., Rock5B)
2. **SD card** (at least 16GB, recommended)
3. **Linux computer**
4. **Debian 12 Image**: `rock-5b_bookworm_kde_b5.output.img`
5. **Etcher Software**: A tool to write the image to the SD card.

#### Steps to Install Debian 12 on Rock5:

##### Step 1: Download the Debian Image
Ensure you have the correct Debian 12 image for your Rock5 device. You can download it from the official source. The file should be named something like `rock-5b_bookworm_kde_b5.output.img`.

#### Debian Image from Radxa Official Website:
- https://docs.radxa.com/en/rock5/rock5b/download
- https://docs.radxa.com/en/rock5/rock5b

**Note:** download image : rock-5b_bookworm_kde_b5 (`rock-5b_bookworm_kde_b5.output.img.xz`)


### Extract the Image
Before flashing, extract the `.xz` file to get the raw `.img` file.

#### **On Linux/macOS**
```sh
xz -d rock-5b_bookworm_kde_b5.output.img.xz
```
or using `unxz`:
```sh
unxz rock-5b_bookworm_kde_b5.output.img.xz
```

#### **On Windows**
Use **7-Zip** or **WinRAR** to extract the `.img` file.

---

#### Debian arm64 images:
- Image: [https://www.debian.org/distrib/netinst](https://cdimage.debian.org/debian-cd/current/arm64/iso-cd/debian-12.10.0-arm64-netinst.iso)
- Download Page: https://www.debian.org/distrib/netinst

---

##### Step 2: Install Etcher on Linux
Etcher is a simple tool that you can use to burn operating system images to SD cards or USB drives. Follow these steps to install Etcher on your Linux machine:

**Download Etcher**:
   You can download the Etcher from the official site:
   - Go to [Etcher's official website](https://www.balena.io/etcher/).
   - Download the etcher for Linux.
   - Once downloaded, extract it, open it and run `balena-etcher`

---

##### Step 3: Insert the SD Card
Insert your SD card (16GB or larger) into your computer using an SD card reader. Make sure to back up any data on the SD card, as it will be erased during the writing process. https://github.com/jahangir842/linux-notes/blob/main/file-system/wipefs.md

##### Step 4: Open Etcher and Select the Image
1. Launch Etcher from your application menu or by running the `.AppImage` if you used that method.
2. In Etcher, click on **Flash from file**.
3. Browse to and select the downloaded Debian 12 image file, `rock-5b_bookworm_kde_b5.output.img`.

##### Step 5: Select the SD Card
1. Etcher should automatically detect the SD card you inserted. Double-check that you’ve selected the correct drive (be careful if you have other drives connected).
2. Confirm that the SD card is selected as the target drive.

##### Step 6: Write the Image to the SD Card
1. Once you’ve selected the correct image and SD card, click the **Flash!** button to start the process.
2. Etcher will begin writing the Debian image to your SD card. This can take several minutes depending on the speed of your SD card and computer.
3. After the writing process is complete, Etcher will automatically verify the image to ensure no errors occurred during the process.

##### Step 7: Eject the SD Card
Once the flashing process is finished, and verification is complete, Etcher will notify you that the process has been successfully completed. Eject the SD card safely from your computer.

##### Step 8: Insert the SD Card into the Rock5
Take the SD card that you just flashed and insert it into your Rock5 device.

##### Step 9: Boot the Rock5
1. Connect your Rock5 device to a monitor, keyboard, and mouse.
2. Power on the Rock5. It should automatically boot into the Debian 12 system.
3. Follow the on-screen prompts to complete the initial setup and configuration of your Debian installation.

Congratulations! You've successfully installed Debian 12 on your Rock5 device using Etcher.

#### Initial Credentials:
- Username: radxa
- Password: radxa 

##### Additional Tips:
- If the Rock5 doesn’t boot, ensure the SD card is securely inserted, and try a different SD card if necessary.
- You can refer to the official Debian and Rock5 documentation for advanced configurations and troubleshooting. 

---

### Install ROS2 HUMBLE on Debian 12 BookWorm
- **Official Link:** https://docs.radxa.com/en/rock5/rock5t/app-development/ros2_humble

#### System Requirements:
- **OS**: Debian 12 Bookworm

#### Set Up the Locale:
To set up the locale for the system, use the following commands:

```bash
echo -e "export LC_ALL=en_US.UTF-8\nexport LANG=en_US.UTF-8\n" >> $HOME/.bashrc
source $HOME/.bashrc
```

#### Install the Required Tools:
Update your system and install the required dependencies:

```bash
sudo apt-get update && sudo apt-get full-upgrade
```

Then install the necessary packages:

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

#### Capacity:
Ensure that there is at least **15 GB** of free space in the workspace for compiling the source code.

#### Compile ROS2 Humble:

1. **Create a Working Directory**:
   ```bash
   mkdir -p ros2_humble/src
   cd ros2_humble
   ```

2. **Get the Source Code**:
   Download the ROS2 Humble source repository:

   ```bash
   wget https://raw.githubusercontent.com/ros2/ros2/humble/ros2.repos
   vcs import src < ros2.repos
   ```

3. **Install Dependencies**:
   Initialize and update `rosdep`, then install the dependencies:

   ```bash
   sudo rosdep init
   rosdep update
   rosdep install --from-paths src --ignore-src -y --skip-keys "fastcdr rti-connext-dds-6.0.1 urdfdom_headers python3-vcstool"
   ```

4. **Compile the Source Code**:
   Compile ROS2 Humble with the following command:

   ```bash
   colcon build --symlink-install
   ```

#### How to Use:

1. **Set Up the Environment**:
   To source the ROS2 setup script every time you open a terminal, add the following line to your `.bashrc`:

   ```bash
   echo -e "source ${path_to_ros2_source}/ros2_humble/install/setup.bash" >> $HOME/.bashrc
   source $HOME/.bashrc
   ```

   **Tip**: Replace `${path_to_ros2_source}` with the actual path to your `ros2_humble` directory.

2. **Example: Running a C++ Talker Node**:
   Open a terminal and run the C++ talker example:

   ```bash
   ros2 run demo_nodes_cpp talker
   ```

3. **Example: Running a Python Listener Node**:
   Open another terminal and run the Python listener example:

   ```bash
   ros2 run demo_nodes_py listener
   ```

#### Authentication:
After executing these two commands, you should see the following output from the talker side:

```bash
ros2 run demo_nodes_cpp talker
[INFO] [1728966840.691346935] [talker]: Publishing: 'Hello World: 1'
[INFO] [1728966841.691537928] [talker]: Publishing: 'Hello World: 2'
[INFO] [1728966842.691572879] [talker]: Publishing: 'Hello World: 3'
[INFO] [1728966843.691563207] [talker]: Publishing: 'Hello World: 4'
[INFO] [1728966844.691568120] [talker]: Publishing: 'Hello World: 5'
```

And the following output from the listener side:

```bash
ros2 run demo_nodes_py listener
[INFO] [1728966840.716456921] [listener]: I heard: [Hello World: 1]
[INFO] [1728966841.698002748] [listener]: I heard: [Hello World: 2]
[INFO] [1728966842.697675740] [listener]: I heard: [Hello World: 3]
[INFO] [1728966843.697643318] [listener]: I heard: [Hello World: 4]
[INFO] [1728966844.697551980] [listener]: I heard: [Hello World: 5]
```

This verifies that both the C++ and Python APIs are working as expected.
