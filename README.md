# C9 Magang Banyubramanta - Hadryan Rizky Dimas Saputra

Repositori ini berisi kumpulan tugas magang untuk divisi Programming.

## 📚 Daftar Isi

Tugas 1: Controller & Interfaces

Tugas 2: Deteksi Warna OpenCV

Tugas 3: Integrasi OpenCV & ROS 2

Tugas 4: Integrasi ROS 2 & OpenVINO

Tugas 5: ROS 2 ke Serial Bridge

Tugas 6: Simulasi Robot Gazebo

---

# [Tugas Magang 1 - Controller]

Paket ini berisi implementasi node kendali dasar yang menerjemahkan input joystick menjadi pesan kustom ROS 2.

----

## 🚀 Fungsionalitas

controller_node membaca data mentah dari topik /joy dan mempublikasikan data yang telah diproses ke topik /cmd_vel menggunakan custom message interfaces/msg/ControllerCommand.

### Pemetaan Kontrol (Xbox 360):

Stik Kiri (Vertikal): x_cmd (Maju/Mundur) - Stateless

Stik Kiri (Horizontal): y_cmd (Geser Kiri/Kanan) - Stateless

Stik Kanan (Vertikal): depth (Kedalaman) - Stateful [0.0, 10.0]

Stik Kanan (Horizontal): yaw (Putar) - Stateful [-180.0, 180.0]

---

## ⚙️ Cara Menjalankan

### 1. Build Interface & Controller
```
colcon build --packages-select interfaces
source install/setup.bash
colcon build --packages-select controller
source install/setup.bash
```

### 2. Terminal 1: Jalankan Joy Node (Pastikan Joystick tercolok)
```
ros2 run joy joy_node
```

### 3. Terminal 2: Jalankan Controller
```
ros2 run controller controller_node
```

### 4. Terminal 3: Verifikasi Output
```
ros2 topic echo /cmd_vel
```

---

# [Tugas Magang 2 - Deteksi Warna OpenCV]

Program C++ standalone (non-ROS) yang berfungsi sebagai HSV Color Tuner untuk mendeteksi objek berdasarkan warna dalam video.

---

## 🚀 Fungsionalitas

Menampilkan video asli, video hasil mask (hitam-putih), dan panel kontrol.

Menyediakan 8 slider (Trackbars) untuk mengatur Hue, Saturation, Value secara real-time.

---

## ⚙️ Cara Menjalankan

Prasyarat: libopencv-dev, build-essential.

### 1. Kompilasi Program
```
g++ deteksi_warna.cpp -o deteksi_app $(pkg-config --cflags --libs opencv4)
```

### 2. Jalankan Program
```
./deteksi_app second.mp4
```

Tekan 'q' atau ESC untuk keluar.

---

# [Tugas Magang 3 - Integrasi OpenCV dan ROS 2]

Integrasi program deteksi warna (Tugas 2) ke dalam ekosistem ROS 2 sebagai Publisher Node.

---

## 🚀 Fungsionalitas

Node video_publisher_node membaca file video dan mempublikasikan dua topik gambar secara terus-menerus:

/raw_image: Video asli berwarna.

/mask_image: Video hasil masking HSV (hitam-putih) dengan nilai HSV yang sudah di-hardcode.

---

## ⚙️ Cara Menjalankan

### 1. Build Package
```
colcon build --packages-select opencv_integration
source install/setup.bash
```

### 2. Jalankan Node (Ganti path video sesuai lokasi file Anda)
```
ros2 run opencv_integration video_publisher_node --ros-args -p video_file_path:="/home/hadryan/fourth.mp4"
```

### 3. Visualisasi (Buka rqt_image_view di terminal baru)
```
ros2 run rqt_image_view rqt_image_view
```

---

# [Tugas Magang 4 - Integrasi ROS 2 & OpenVINO]

Implementasi deteksi objek berbasis Deep Learning (YOLOv5) menggunakan OpenVINO Runtime dalam ROS 2.

---

## 🚀 Fungsionalitas

Modular: Memisahkan inference engine dan logika ROS.

High Performance: Akselerasi inferensi CPU dengan OpenVINO.

Input Fleksibel: Path model .onnx dan video diatur via parameter.

Output: Mempublikasikan gambar visualisasi (debug_image) dan data deteksi (detections).

---

## ⚙️ Cara Menjalankan

### 1. Build Interface & Package
```
colcon build --packages-select yolo_msgs vision_ov_yolo
source install/setup.bash
```

### 2. Jalankan Node Deteksi (Gunakan Absolute Path!)
```
ros2 run vision_ov_yolo yolo_detector --ros-args -p model_path:="/home/hadryan/ros2_ws/src/vision_ov_yolo/models/best.onnx" -p video_path:="/home/hadryan/ros2_ws/src/vision_ov_yolo/data/fourth.mp4"
```

### 3. Visualisasi
```
ros2 run rqt_image_view rqt_image_view  # Topik: /yolo/debug_image
ros2 topic echo /yolo/detections        # Lihat data teks
```

---

# [Tugas Magang 5 - ROS 2 ke Serial Bridge]

Jembatan komunikasi antara topik ROS 2 /cmd_vel dan perangkat keras (mikrokontroler) melalui Serial Port (ASIO).

---

## 🚀 Fungsionalitas

Mengonversi pesan geometry_msgs/Twist menjadi string ASCII ringkas.

Format: <linear.x,angular.z>\n

Contoh: <0.50,-0.20>

---

## ⚙️ Cara Menjalankan

Penting: Berikan hak akses port serial ```sudo usermod -a -G dialout $USER``` lalu logout/login.

### 1. Build Package
```
colcon build --packages-select serial_bridge
source install/setup.bash
```

#3# 2. Jalankan Bridge (Contoh port custom)
```
ros2 run serial_bridge serial_bridge_node --ros-args -p serial_port:=/dev/ttyUSB0 -p baud_rate:=9600
```

### 3. Kirim Data Test
```
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: -0.2}}" -r 1
```

---

## 🧪 Pengujian Virtual (socat)

Tanpa hardware, gunakan socat untuk membuat port virtual:

### Terminal 1: Buat port virtual
```
socat -d -d pty,raw,echo=0 pty,raw,echo=0
```

### Terminal 2: Jalankan node ke salah satu port (misal /dev/pts/2)
```
ros2 run serial_bridge serial_bridge_node --ros-args -p serial_port:=/dev/pts/2
```

### Terminal 3: Baca output di port pasangan (misal /dev/pts/3)
```
cat /dev/pts/1
```

---

# [Tugas Magang 6 - Simulasi Robot Gazebo]

Repositori ini berisi paket simulasi untuk robot 6 DoF (Degree of Freedom) sederhana yang dijalankan di lingkungan Gazebo.

---

## 🚀 Fungsionalitas Simulasi

Simulasi ini bertujuan untuk memvalidasi logika kontrol dan visi komputer tanpa menggunakan perangkat keras fisik.

### Fitur Utama:

Robot 6 DoF: Robot didesain dengan kebebasan gerak penuh di dalam simulasi (Surge, Sway, Heave, Roll, Pitch, Yaw).

Kamera Terintegrasi: Robot dilengkapi dengan sensor kamera virtual yang mensimulasikan pengambilan gambar dunia nyata.

Topik Output: /camera_sensor/image_raw

Frame Rate: 30 FPS (default)

Kontrol Joystick (Stateful): Robot dikendalikan menggunakan Xbox 360 Controller dengan logika khusus:

Gerak X/Y: Stateless (Lepas stik = Berhenti).

Gerak Depth/Yaw: Stateful (Lepas stik = Posisi terkunci/bertahan).

Deteksi Objek Real-time: Terintegrasi dengan node YOLO (dari Tugas 4) untuk mendeteksi objek "Flare" (Silinder Oranye) dan "Baskom" di dalam simulasi.

---

## ⚙️ Cara Menjalankan

### 1. Persiapan Workspace

Pastikan paket submarine_sim, controller, dan vision_ov_yolo sudah berada di dalam folder src. Lakukan build ulang workspace:
```
cd ~/ros2_ws
colcon build --packages-select submarine_sim controller vision_ov_yolo
source install/setup.bash
```

### 2. Menjalankan Simulasi & Kontrol

Jalankan file launch utama. File ini akan otomatis membuka Gazebo, memunculkan robot, dan menjalankan node kontroler joystick.

### Terminal 1:
```
ros2 launch submarine_sim sim.launch.py
```
Gazebo akan terbuka dan robot muncul. Pastikan Joystick Xbox 360 sudah terhubung.


### 3. Menyiapkan Objek Target (Flare/Baskom)

Agar deteksi objek bekerja, kita perlu membuat objek dummy di dalam Gazebo (karena dunia default mungkin kosong):

1. Di menu atas Gazebo, klik icon Cylinder (Tabung) atau Sphere (Bola).

2. Klik di lantai simulasi (pastikan berada di jangkauan kamera robot).

3. Klik kanan pada objek tersebut -> Edit Model.

4. Pilih tab Material (atau Link Inspector -> Visual -> Material).

5. Ubah warna menjadi Orange (untuk simulasi Flare) atau warna lain sesuai model YOLO Anda.

6. Exit Model Editor / Save.


### 4. Menjalankan Node Deteksi Objek (YOLO)

Jalankan node deteksi dengan mode Subscriber. Kita perlu mengarahkan node untuk mendengarkan topik kamera Gazebo (/camera_sensor/image_raw), bukan membaca file video.

### Terminal 2:

Ganti path model_path sesuai lokasi file best.onnx di komputer Anda
Perhatikan penggunaan --ros-args -p (parameter)

```
ros2 run vision_ov_yolo yolo_detector --ros-args -p model_path:="/home/hadryan/ros2_ws/src/vision_ov_yolo/models/best.onnx" -p image_topic:="/camera_sensor/image_raw"
```

### 5. Visualisasi Hasil Deteksi

Lihat apa yang dilihat oleh robot beserta kotak deteksi (Bounding Box) yang dihasilkan oleh YOLO.

### Terminal 3:
```
ros2 run rqt_image_view rqt_image_view
```
Di jendela RQT, klik dropdown di kiri atas.

Pilih topik: /yolo/debug_image.

Arahkan robot mendekati objek silinder oranye yang Anda buat, dan kotak deteksi akan muncul.

--- 

## 🧪 Validasi Sistem

Untuk memastikan semua sistem berjalan dengan benar, lakukan pengecekan berikut:

### 1. Cek Topik Kamera:

Pastikan topik kamera simulasi tersedia.
```
ros2 topic list
```
Output wajib ada: /camera_sensor/image_raw

### 2. Cek Respon Kontrol:

Saat joystick digerakkan, pastikan data velocity dikirim ke namespace robot yang benar.
```
ros2 topic echo /rov/cmd_vel
```

Cek Data Deteksi:
Lihat data teks hasil deteksi (koordinat dan label) yang dipublish oleh node YOLO.
```
ros2 topic echo /yolo/detections
```

---

:D
