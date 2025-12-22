# 🚤 Simple Boat ROS 2 Controller

![ROS 2 Humble](https://img.shields.io/badge/ROS_2-Humble-blue?logo=ros\&logoColor=white)
![C++](https://img.shields.io/badge/C++-17-blue?logo=c%2B%2B\&logoColor=white)
![Platform](https://img.shields.io/badge/Platform-ArduSub%20%7C%20Pixhawk-orange)
![Status](https://img.shields.io/badge/Status-Working%20Prototype-success)

**Simple Boat ROS 2 Controller** adalah paket ROS 2 untuk mengendalikan **robot kapal / ROV** berbasis **Pixhawk + ArduSub** secara **full autonomous** menggunakan **MAVROS**.
Node ini mengirim sinyal **RC Override (PWM)** ke Pixhawk untuk menggerakkan thruster berdasarkan skenario waktu yang telah ditentukan.

> 🎯 Cocok untuk:
>
> * Autonomous Surface Vehicle (ASV)
> * ROV / BlueROV-style robot
> * Riset robotika laut
> * Project kampus & portofolio robotika

👤 **Author:** Ozza (Indonesia 🇮🇩)

---

## 📋 Fitur Utama

* 🚀 **Full Autonomous Control**
  Robot bergerak otomatis tanpa RC fisik.

* 🎮 **RC Override via MAVROS**
  Menggunakan topik:

  ```
  /mavros/rc/override
  ```

* 🧭 **Vectored Thruster Support**
  Mendukung:

  * Surge (maju / mundur)
  * Yaw (rotasi)
  * Heave (naik / turun)
  * Sway (geser kiri / kanan)

* 🛡️ **Failsafe Prevention**
  Mengirim heartbeat RC agar Pixhawk tidak masuk failsafe.

---

### Penjelasan

* **boat_mover_node**
  Node ROS 2 yang menghasilkan PWM berdasarkan timeline
* **MAVROS**
  Bridge ROS 2 ↔ MAVLink
* **Pixhawk + ArduSub**
  Menerjemahkan RC override ke output motor
* **Thruster Mixer**
  Distribusi gaya ke thruster vectored

---

## ⚙️ Spesifikasi Hardware

| Komponen           | Keterangan                       |
| ------------------ | -------------------------------- |
| Companion Computer | Jetson Orin Nano                 |
| Flight Controller  | Pixhawk                          |
| Firmware           | ArduSub by QgroundControl        |
| Frame              | 6 Thruster (Vectored / BlueROV2) |
| Koneksi            | USB (`/dev/ttyACM0`)             |

---

## 🛠️ Konfigurasi Pixhawk (WAJIB)

Set melalui **QGroundControl** sebelum menjalankan ROS 2.

### Parameter ArduSub

| Parameter         | Nilai    | Fungsi                 |
| ----------------- | -------- | ---------------------- |
| `FRAME_CONFIG`    | Vectored | Konfigurasi 6 thruster |
| `BRD_SAFETY_MASK` | 16383    | Bypass safety switch   |
| `FS_GCS_ENABLE`   | 0        | Disable GCS failsafe   |
| `ARMING_CHECK`    | 0        | Arming tanpa GPS       |
| `SERVO1_FUNCTION` | 33       | Motor 1                |
| `SERVO2_FUNCTION` | 34       | Motor 2                |
| `SERVO3_FUNCTION` | 35       | Motor 3                |
| `SERVO4_FUNCTION` | 36       | Motor 4                |
| `SERVO5_FUNCTION` | 37       | Motor 5                |
| `SERVO6_FUNCTION` | 38       | Motor 6                |

⚠️ **WAJIB reboot Pixhawk setelah setting parameter**

---

## 🚀 Instalasi

Masuk ke workspace ROS 2:

```bash
cd ~/ros2_ws/src
git clone https://github.com/username-kamu/simple_boat.git
```

Build package:

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

---

## 🎮 Cara Menjalankan (Step-by-Step)

Jalankan **3 terminal terpisah**.

---

### 🖥️ Terminal 1 — MAVROS Connection

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch mavros apm.launch fcu_url:=/dev/ttyACM0:115200 gcs_url:=udp://@
```

Tunggu:

```
CON: Got HEARTBEAT
```

---

### 🖥️ Terminal 2 — Mode & Arming

```bash
source install/setup.bash
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{custom_mode: 'MANUAL'}"

ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: True}"
```

Pastikan:

```
success: True
```

---

### 🖥️ Terminal 3 — Jalankan Node Autonomous

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run simple_boat boat_mover_node
```

---

## 💃 Skenario Gerakan Robot

| No | Waktu (detik) | Gerakan     | Channel / PWM     |
| -- | ------------- | ----------- | ----------------- |
| 0  | 0 – 3         | Standby     | Semua 1500        |
| 1  | 3 – 13        | Maju        | Ch 4 → 1700       |
| 2  | 13 – 23       | Yaw Kanan   | Ch 3 → 1300 (Inv) |
| 3  | 23 – 33       | Yaw Kiri    | Ch 3 → 1700 (Inv) |
| 4  | 33 – 43       | Naik        | Ch 2 → 1700       |
| 5  | 43 – 53       | Turun       | Ch 2 → 1300       |
| 6  | 53 – 63       | Geser Kanan | Ch 5 → 1300 (Inv) |
| 7  | 63 – 73       | Geser Kiri  | Ch 5 → 1700 (Inv) |
| 8  | 73 – 83       | Mundur      | Ch 4 → 1300       |
| 9  | > 83          | Stop        | Semua 1500        |

---

## 🔧 Troubleshooting

### Motor tidak bergerak (`rc/out = 0`)

* Cek `BRD_SAFETY_MASK = 16383`
* Pastikan Pixhawk reboot

### Log jalan tapi robot diam

* Pastikan **ARMED**
* `ARMING_CHECK = 0`

### Arah terbalik

* Tukar PWM (1700 ↔ 1300)
* Atau reverse motor di QGroundControl

---

## 📌 Catatan Pengembangan

* Kontrol masih **open-loop (time-based)**
* Belum menggunakan sensor feedback
* Cocok sebagai **baseline autonomous controller**

---

## 🧪 Rencana Pengembangan

* FSM (Finite State Machine)
* Sensor IMU & Depth
* PID control
* ArduSub SITL + Gazebo
* Waypoint navigation

---

## 📜 Lisensi

MIT License — bebas digunakan untuk edukasi & riset.

---

✨ **Happy Coding & Happy Sailing!** 🚤
