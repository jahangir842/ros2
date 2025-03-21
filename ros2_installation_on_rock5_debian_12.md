### Guide to Install Debian 12 on Rock5 Using Etcher (Linux Only)

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

##### Step 2: Install Etcher on Linux
Etcher is a simple tool that you can use to burn operating system images to SD cards or USB drives. Follow these steps to install Etcher on your Linux machine:

1. **Using Snap Package (Recommended)**:
   If your system supports Snap, you can install Etcher by running the following command:
   ```bash
   sudo snap install etcher
   ```

2. **Using the .AppImage**:
   Alternatively, you can download the Etcher `.AppImage` from the official site:
   - Go to [Etcher's official website](https://www.balena.io/etcher/).
   - Download the `.AppImage` file for Linux.
   - Once downloaded, give it execute permission:
     ```bash
     chmod +x balena-etcher-electron-*.AppImage
     ```
   - Run it with:
     ```bash
     ./balena-etcher-electron-*.AppImage
     ```

##### Step 3: Insert the SD Card
Insert your SD card (16GB or larger) into your computer using an SD card reader. Make sure to back up any data on the SD card, as it will be erased during the writing process.

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

##### Additional Tips:
- If the Rock5 doesn’t boot, ensure the SD card is securely inserted, and try a different SD card if necessary.
- You can refer to the official Debian and Rock5 documentation for advanced configurations and troubleshooting. 

---

### Install ROS2 HUMBLE on Debian 12 BookWorm
- **Official Link:** https://docs.radxa.com/en/rock5/rock5t/app-development/ros2_humble
