# 🚤 Simple Boat ROS 2 Controller

![ROS 2 Humble](https://img.shields.io/badge/ROS_2-Humble-blue?logo=ros&logoColor=white)
![Platform](https://img.shields.io/badge/Platform-ArduSub%20%7C%20Pixhawk-orange)

Paket ROS 2 ini dibuat oleh **Ozza (Malang)** untuk mengendalikan robot kapal/ROV (Frame Vectored) secara otonom.

## 📋 Misi Pergerakan
Robot akan melakukan manuver berikut dengan durasi masing-masing **10 Detik**:

| No | Waktu | Gerakan | Channel (PWM) |
| :--- | :--- | :--- | :--- |
| 1 | 3-13s | **Maju** | Ch 4 (1700) |
| 2 | 13-23s | **Yaw Kanan** | Ch 3 (1300) *Inverted* |
| 3 | 23-33s | **Yaw Kiri** | Ch 3 (1700) *Inverted* |
| 4 | 33-43s | **Naik** | Ch 2 (1700) |
| 5 | 43-53s | **Turun** | Ch 2 (1300) |
| 6 | 53-63s | **Geser Kanan** | Ch 5 (1300) *Inverted* |
| 7 | 63-73s | **Geser Kiri** | Ch 5 (1700) *Inverted* |
| 8 | 73-83s | **Mundur** | Ch 4 (1300) |

## ⚙️ Wajib Disetting di QGroundControl
* `FRAME_CONFIG` = **Vectored**
* `BRD_SAFETY_MASK` = **16383**
* `FS_GCS_ENABLE` = **0**
* `SERVO1-6_FUNCTION` = **33, 34, 35, 36, 37, 38**

## 🚀 Cara Menjalankan

1. **Build Code:**
   ```bash
   cd ~/ros2_ws
   colcon build --symlink-install
   source install/setup.bash