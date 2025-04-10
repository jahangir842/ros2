## 🔄 SD Card Image Backup Procedure

### ✅ Prerequisites
Make sure the device (`/dev/sda`) is not mounted before starting the backup.

```bash
lsblk
```

### ⚙️ PI-5 Backup

```bash
# Unmount all partitions of the device
sudo umount /dev/sda*

# Create a backup image of the entire drive
sudo dd if=/dev/sda of=/media/jahangir/disk2/raspberry_pi_backup.img bs=4M status=progress
```

#### ✅ Sample Output:
```
63862472704 bytes (64 GB, 59 GiB) copied, 11258 s, 5.7 MB/s
15226+1 records in
15226+1 records out
63864569856 bytes (64 GB, 59 GiB) copied, 11258.5 s, 5.7 MB/s
```

---

### ⚙️ Rock-5 Backup

```bash
# Unmount all partitions of the device
sudo umount /dev/sda*

# Create a backup image of the entire drive
sudo dd if=/dev/sda of=/media/jahangir/disk2/rock-5_backup.img bs=4M status=progress
```

#### ✅ Sample Output:
```
31914459136 bytes (32 GB, 30 GiB) copied, 5623 s, 5.7 MB/s
7609+1 records in
7609+1 records out
31914983424 bytes (32 GB, 30 GiB) copied, 5623.74 s, 5.7 MB/s
```

### 📝 Notes

- **`/dev/sda`**: Ensure this is the correct source drive.
- **`bs=4M`**: Uses a 4MB block size for faster copying.
- **`status=progress`**: Shows real-time progress.
- **Unmount First**: Never image a mounted drive to avoid data corruption.

---

## 🔁 Writing Backup Image to SD Card

### ✅ Prerequisites

- Ensure the target device (e.g., `/dev/sda`) is **correct and unmounted**.
- Double-check the device path to avoid overwriting your main disk!

### ⚠️ Warning  
> Be **very careful** when using `dd`. Writing to the wrong device can erase your data!

---

## 💾 Restore Raspberry Pi / Rock-5 Image

### Step 1: Unmount the SD Card

Before writing the image, unmount all mounted partitions on the target device:

```bash
sudo umount /dev/sda*
```

### Step 2: Write the Image Back

#### 📥 For Raspberry Pi:
```bash
sudo dd if=/media/jahangir/disk2/raspberry_pi_backup.img of=/dev/sda bs=4M status=progress
```

#### 📥 For Rock-5:
```bash
sudo dd if=/media/jahangir/disk2/rock-5_backup.img of=/dev/sda bs=4M status=progress
```

### Step 3: Flush Write Cache (Recommended)

```bash
sync
```

This ensures all write operations are completed before you remove the SD card.

---

## ✅ Post-Restore Tips

- After writing, you can mount the SD card and inspect files to verify the restore.
- If restoring to a **larger SD card**, consider using `gparted` to expand the root partition.
- If restoring on **another system**, adjust device paths accordingly (e.g., `/dev/mmcblk0`, `/dev/sdb`, etc.).

---
