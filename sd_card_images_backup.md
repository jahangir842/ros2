## 🔄 SD Card Image Backup Procedure

### ✅ Prerequisites
Make sure the device (`/dev/sda`) is not mounted before starting the backup.

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

---

### 📝 Notes

- **`/dev/sda`**: Ensure this is the correct source drive.
- **`bs=4M`**: Uses a 4MB block size for faster copying.
- **`status=progress`**: Shows real-time progress.
- **Unmount First**: Never image a mounted drive to avoid data corruption.

---

Let me know if you want the restore steps too or how to compress the image after backup (e.g., using `gzip` or `xz`).





---

before rewrite

---

## PI-5
```bash
sudo umount /dev/sda*
```

```bash
sudo dd if=/dev/sda of=/media/jahangir/disk2/raspberry_pi_backup.img bs=4M status=progress
63862472704 bytes (64 GB, 59 GiB) copied, 11258 s, 5.7 MB/s
15226+1 records in
15226+1 records out
63864569856 bytes (64 GB, 59 GiB) copied, 11258.5 s, 5.7 MB/s
```

---


## Rock-5:

```bash
sudo umount /dev/sda*
```

```bash
sudo dd if=/dev/sda of=/media/jahangir/disk2/rock-5_backup.img bs=4M status=progress
31914459136 bytes (32 GB, 30 GiB) copied, 5623 s, 5.7 MB/s 
7609+1 records in
7609+1 records out
31914983424 bytes (32 GB, 30 GiB) copied, 5623.74 s, 5.7 MB/s
```
