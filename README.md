# Rancang-Bangun-Sistem-Monitoring-Pergeseran-Tanah-menggunakan-Modifikasi-Digital-Caliper-

Proyek ini adalah sistem yang dirancang untuk memantau pergerakan tanah (displacement) dan kelembaban tanah secara *real-time*, serta memublikasikannya melalui protokol MQTT. Data yang diterima juga dicatat ke kartu SD secara lokal dan ditampilkan pada antarmuka pengguna grafis (GUI) melalui aplikasi desktop Python.

![Diagram menunjukkan dua modul ESP (Main & Stake) berkomunikasi via MQTT ke broker pusat. Modul Main terhubung ke SD Card, OLED, RTC, Caliper, dan Sensor Tanah. PC menjalankan Aplikasi Python yang terhubung ke broker MQTT.](docs/system_diagram.png) 
*(Catatan: Anda dapat menambahkan diagram alir sistem Anda ke folder `docs/` dan mengubah nama filenya)*

## 📋 Ikhtisar Komponen Sistem

Sistem ini terdiri dari tiga komponen utama yang bekerja sama:

1.  **Alat Utama (Main Unit) - ESP8266:** Berfungsi sebagai **Client MQTT Subscriber** untuk menerima data orientasi dari Pasak, dan **Client MQTT Publisher** untuk mengirim data lokal (Displacement & Kelembaban Tanah). Unit ini juga mengelola pencatatan data ke SD Card dan menampilkan status pada layar OLED.
2.  **Sistem Pasak (Stake Unit) - ESP32-C3:** Berfungsi sebagai **Client MQTT Publisher** untuk mengukur dan mengirim data orientasi (Pitch, Roll, Yaw) dari sensor MPU6050.
3.  **Aplikasi Monitoring (Desktop App) - Python:** Aplikasi berbasis Python (Tkinter + Matplotlib) yang berfungsi sebagai **Client MQTT Subscriber** untuk menampilkan dan mencatat semua data sensor secara *real-time*.

## 📁 Struktur Repositori
