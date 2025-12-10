import tkinter as tk
from tkinter import ttk, messagebox, filedialog
import serial.tools.list_ports
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.animation import FuncAnimation
import pandas as pd
import numpy as np
import threading
import time
import json
from datetime import datetime
import paho.mqtt.client as mqtt
import re
from collections import deque

# Set matplotlib style
plt.style.use('dark_background')

class SensorMonitorApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Real-time Sensor Data Monitor")
        self.root.geometry("1200x800")
        self.root.configure(bg='#2c3e50')
        
        # Variables
        self.serial_connection = None
        self.mqtt_client = None
        self.is_connected = False
        self.connection_type = tk.StringVar(value="COM")
        self.data_points = 100
        self.sensor_data = {
            'Sensor1': deque([0] * self.data_points, maxlen=self.data_points),
            'Sensor2': deque([0] * self.data_points, maxlen=self.data_points),
            'Sensor3': deque([0] * self.data_points, maxlen=self.data_points),
            'Sensor4': deque([0] * self.data_points, maxlen=self.data_points),
            'Sensor5': deque([0] * self.data_points, maxlen=self.data_points)
        }
        # Tambahkan deque untuk timestamp
        self.sensor_timestamps = {
            'Sensor1': deque([datetime.now()] * self.data_points, maxlen=self.data_points),
            'Sensor2': deque([datetime.now()] * self.data_points, maxlen=self.data_points),
            'Sensor3': deque([datetime.now()] * self.data_points, maxlen=self.data_points),
            'Sensor4': deque([datetime.now()] * self.data_points, maxlen=self.data_points),
            'Sensor5': deque([datetime.now()] * self.data_points, maxlen=self.data_points)
        }
        self.sensor_names = {
            'Sensor1': tk.StringVar(value="Distance"),
            'Sensor2': tk.StringVar(value="Humidity"),
            'Sensor3': tk.StringVar(value="Pitch"),
            'Sensor4': tk.StringVar(value="Roll"),
            'Sensor5': tk.StringVar(value="Yaw")
        }
        self.sensor_colors = ['#ff6b6b', '#48dbfb', '#1dd1a1', '#feca57', '#ff9ff3']
        self.sensor_units = {
            'Sensor1': tk.StringVar(value="mm"),
            'Sensor2': tk.StringVar(value="%"),
            'Sensor3': tk.StringVar(value="°"),
            'Sensor4': tk.StringVar(value="°"),
            'Sensor5': tk.StringVar(value="°")
        }
        self.logging_active = False
        self.log_data = []
        self.capture_duration = tk.IntVar(value=5)  # minutes
        self.mqtt_topics = {
            'Sensor1': tk.StringVar(value="soil_monitoring/displacement"),
            'Sensor2': tk.StringVar(value="soil_monitoring/soil_moisture"),
            'Sensor3': tk.StringVar(value="soil_monitoring/pitch"),
            'Sensor4': tk.StringVar(value="soil_monitoring/roll"),
            'Sensor5': tk.StringVar(value="soil_monitoring/yaw")
        }
        self.mqtt_broker = tk.StringVar(value="broker.hivemq.com")
        self.mqtt_port = tk.IntVar(value=1883)
        self.time_resolution = tk.StringVar(value="second")
        
        # Setup GUI
        self.setup_gui()
        
        # Start port detection
        self.detect_ports()
        
        # Start animation
        self.ani = FuncAnimation(self.fig, self.update_plot, interval=100, cache_frame_data=False)
        
    def setup_gui(self):
        # Create main frames
        # --- Tambahkan Canvas dan Scrollbar untuk control_frame ---
        control_outer_frame = tk.Frame(self.root, bg='#34495e', padx=0, pady=0)
        control_outer_frame.pack(side=tk.LEFT, fill=tk.Y, padx=5, pady=5)

        canvas = tk.Canvas(control_outer_frame, bg='#34495e', highlightthickness=0)
        scrollbar = tk.Scrollbar(control_outer_frame, orient="vertical", command=canvas.yview)
        canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

        # Frame yang akan discroll
        control_frame = tk.Frame(canvas, bg='#34495e', padx=10, pady=10)
        canvas.create_window((0, 0), window=control_frame, anchor='nw')
        canvas.configure(yscrollcommand=scrollbar.set)

        # Update scrollregion setiap ada perubahan ukuran
        def on_frame_configure(event):
            canvas.configure(scrollregion=canvas.bbox("all"))
        control_frame.bind("<Configure>", on_frame_configure)

        # Create plot frame
        plot_frame = tk.Frame(self.root, bg='#2c3e50')
        plot_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # --- Sisanya tetap seperti sebelumnya, ganti semua 'control_frame' menjadi frame baru ini ---
        # Connection settings
        conn_label = tk.Label(control_frame, text="Connection Settings", font=('Arial', 12, 'bold'), 
                             bg='#34495e', fg='white')
        conn_label.pack(pady=(0, 10))
        
        # Connection type
        type_frame = tk.Frame(control_frame, bg='#34495e')
        type_frame.pack(fill=tk.X, pady=5)
        
        tk.Radiobutton(type_frame, text="COM Port", variable=self.connection_type, 
                      value="COM", command=self.toggle_connection_type, bg='#34495e', fg='white',
                      selectcolor='#2c3e50').pack(side=tk.LEFT)
        tk.Radiobutton(type_frame, text="MQTT", variable=self.connection_type, 
                      value="MQTT", command=self.toggle_connection_type, bg='#34495e', fg='white',
                      selectcolor='#2c3e50').pack(side=tk.LEFT)
        
        # COM settings
        self.com_frame = tk.Frame(control_frame, bg='#34495e')
        self.com_frame.pack(fill=tk.X, pady=5)
        
        tk.Label(self.com_frame, text="Port:", bg='#34495e', fg='white').pack(anchor='w')
        self.port_combo = ttk.Combobox(self.com_frame, state="readonly")
        self.port_combo.pack(fill=tk.X, pady=2)
        
        tk.Label(self.com_frame, text="Baud Rate:", bg='#34495e', fg='white').pack(anchor='w')
        self.baud_combo = ttk.Combobox(self.com_frame, values=[9600, 19200, 38400, 57600, 115200])
        self.baud_combo.set(115200)
        self.baud_combo.pack(fill=tk.X, pady=2)
        
        # MQTT settings (initially hidden)
        self.mqtt_frame = tk.Frame(control_frame, bg='#34495e')
        
        tk.Label(self.mqtt_frame, text="Broker:", bg='#34495e', fg='white').pack(anchor='w')
        tk.Entry(self.mqtt_frame, textvariable=self.mqtt_broker).pack(fill=tk.X, pady=2)
        
        tk.Label(self.mqtt_frame, text="Port:", bg='#34495e', fg='white').pack(anchor='w')
        tk.Entry(self.mqtt_frame, textvariable=self.mqtt_port).pack(fill=tk.X, pady=2)
        
        # Sensor configuration
        sensor_label = tk.Label(control_frame, text="Sensor Configuration", font=('Arial', 12, 'bold'), 
                               bg='#34495e', fg='white')
        sensor_label.pack(pady=(20, 10))
        
        # Create sensor configuration frames
        self.sensor_frames = []
        self.mqtt_topic_labels = []
        self.mqtt_topic_entries = []
        for i in range(1, 6):
            sensor_frame = tk.Frame(control_frame, bg='#34495e', relief=tk.GROOVE, bd=1)
            sensor_frame.pack(fill=tk.X, pady=5)
            
            tk.Label(sensor_frame, text=f"Sensor {i}:", bg='#34495e', fg='white', 
                    font=('Arial', 10, 'bold')).pack(anchor='w')
            
            # Sensor name
            tk.Label(sensor_frame, text="Name:", bg='#34495e', fg='white').pack(anchor='w')
            tk.Entry(sensor_frame, textvariable=self.sensor_names[f'Sensor{i}']).pack(fill=tk.X, pady=2)
            
            # Sensor unit
            tk.Label(sensor_frame, text="Unit:", bg='#34495e', fg='white').pack(anchor='w')
            tk.Entry(sensor_frame, textvariable=self.sensor_units[f'Sensor{i}']).pack(fill=tk.X, pady=2)
            
            # MQTT topic (only for MQTT mode)
            mqtt_topic_label = tk.Label(sensor_frame, text="MQTT Topic:", bg='#34495e', fg='white')
            mqtt_topic_entry = tk.Entry(sensor_frame, textvariable=self.mqtt_topics[f'Sensor{i}'])
            self.mqtt_topic_labels.append(mqtt_topic_label)
            self.mqtt_topic_entries.append(mqtt_topic_entry)
            
            self.sensor_frames.append(sensor_frame)
        
        # Connection buttons
        button_frame = tk.Frame(control_frame, bg='#34495e')
        button_frame.pack(fill=tk.X, pady=10)
        
        self.connect_btn = tk.Button(button_frame, text="Connect", command=self.toggle_connection,
                                    bg='#2ecc71', fg='white', font=('Arial', 10, 'bold'))
        self.connect_btn.pack(fill=tk.X, pady=5)
        
        # Data capture settings
        capture_frame = tk.Frame(control_frame, bg='#34495e')
        capture_frame.pack(fill=tk.X, pady=10)
        
        tk.Label(capture_frame, text="Data Capture", font=('Arial', 12, 'bold'), 
                bg='#34495e', fg='white').pack(anchor='w')
        
        tk.Label(capture_frame, text="Duration (minutes):", bg='#34495e', fg='white').pack(anchor='w')
        tk.Entry(capture_frame, textvariable=self.capture_duration).pack(fill=tk.X, pady=2)
        
        tk.Label(capture_frame, text="Time Resolution:", bg='#34495e', fg='white').pack(anchor='w')
        ttk.Combobox(
            capture_frame, textvariable=self.time_resolution,
            values=["second", "minute", "hour"], state="readonly"
        ).pack(fill=tk.X, pady=2)
        
        self.capture_btn = tk.Button(capture_frame, text="Start Capture", command=self.toggle_capture,
                                    bg='#3498db', fg='white', font=('Arial', 10, 'bold'))
        self.capture_btn.pack(fill=tk.X, pady=5)
        
        self.export_btn = tk.Button(capture_frame, text="Export to Excel", command=self.export_to_excel,
                                   bg='#9b59b6', fg='white', font=('Arial', 10, 'bold'))
        self.export_btn.pack(fill=tk.X, pady=5)
        
        # Plot area
        self.fig, self.ax = plt.subplots(figsize=(10, 8))
        self.fig.patch.set_facecolor('#2c3e50')
        self.ax.set_facecolor('#34495e')
        
        self.lines = []
        for i in range(5):
            # Tambahkan marker='o' agar tiap data ada titik
            line, = self.ax.plot([], [], label=f"Sensor {i+1}", color=self.sensor_colors[i], linewidth=2, marker='o', markersize=5)
            self.lines.append(line)
        
        self.ax.set_xlabel('Time', color='white', fontsize=12)
        self.ax.set_ylabel('Value', color='white', fontsize=12)
        self.ax.set_title('Real-time Sensor Data', color='white', fontsize=14)
        self.ax.legend(loc='upper right')
        self.ax.grid(True, color='#5D6D7E')
        self.ax.tick_params(colors='white')
        
        self.canvas = FigureCanvasTkAgg(self.fig, plot_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
        # Status bar
        self.status_var = tk.StringVar(value="Disconnected")
        status_bar = tk.Label(self.root, textvariable=self.status_var, relief=tk.SUNKEN, 
                             anchor=tk.W, bg='#34495e', fg='white')
        status_bar.pack(side=tk.BOTTOM, fill=tk.X)
        
        # Initialize UI
        self.toggle_connection_type()
        
        # --- Di akhir setup_gui, tambahkan binding mousewheel agar scroll bisa dengan mouse ---
        def _on_mousewheel(event):
            canvas.yview_scroll(int(-1*(event.delta/120)), "units")
        canvas.bind_all("<MouseWheel>", _on_mousewheel)

    def toggle_connection_type(self):
        if self.connection_type.get() == "COM":
            self.com_frame.pack(fill=tk.X, pady=5)
            self.mqtt_frame.pack_forget()
            # Hide MQTT topic fields
            for label, entry in zip(self.mqtt_topic_labels, self.mqtt_topic_entries):
                label.pack_forget()
                entry.pack_forget()
        else:  # MQTT
            self.com_frame.pack_forget()
            self.mqtt_frame.pack(fill=tk.X, pady=5)
            # Show MQTT topic fields
            for label, entry in zip(self.mqtt_topic_labels, self.mqtt_topic_entries):
                label.pack(anchor='w')
                entry.pack(fill=tk.X, pady=2)

    def detect_ports(self):
        ports = serial.tools.list_ports.comports()
        port_list = [port.device for port in ports]
        self.port_combo['values'] = port_list
        # Otomatis pilih port pertama jika ada, atau kosongkan jika tidak ada
        if port_list:
            # Jika belum ada yang dipilih atau port yang dipilih sudah tidak ada, pilih port pertama
            if not self.port_combo.get() or self.port_combo.get() not in port_list:
                self.port_combo.set(port_list[0])
        else:
            self.port_combo.set('')
        # Schedule next detection
        self.root.after(2000, self.detect_ports)
    
    def toggle_connection(self):
        if self.is_connected:
            self.disconnect()
        else:
            self.connect()
    
    def connect(self):
        try:
            if self.connection_type.get() == "COM":
                port = self.port_combo.get()
                baud = int(self.baud_combo.get())
                
                if not port:
                    messagebox.showerror("Error", "No COM port detected. Please connect your device.")
                    return
                
                self.serial_connection = serial.Serial(port, baud, timeout=1)
                self.read_thread = threading.Thread(target=self.read_serial_data, daemon=True)
                self.read_thread.start()
                
            else:  # MQTT
                broker = self.mqtt_broker.get()
                port = self.mqtt_port.get()
                self.mqtt_client = mqtt.Client()
                self.mqtt_client.on_connect = self.on_mqtt_connect
                self.mqtt_client.on_message = self.on_mqtt_message
                try:
                    self.mqtt_client.connect(broker, port, 60)
                    self.mqtt_client.loop_start()
                    # Subscribe to all sensor topics (ambil dari entry masing-masing sensor)
                    for i in range(1, 6):
                        topic = self.mqtt_topics[f'Sensor{i}'].get()
                        if topic:
                            self.mqtt_client.subscribe(topic)
                except Exception as e:
                    messagebox.showerror("Error", f"Failed to connect to MQTT broker: {str(e)}")
                    return
            self.is_connected = True
            self.connect_btn.config(text="Disconnect", bg='#e74c3c')
            self.status_var.set("Connected")
        except Exception as e:
            messagebox.showerror("Error", f"Failed to connect: {str(e)}")
    
    def disconnect(self):
        if self.connection_type.get() == "COM" and self.serial_connection:
            self.serial_connection.close()
        elif self.mqtt_client:
            self.mqtt_client.loop_stop()
            self.mqtt_client.disconnect()
            
        self.is_connected = False
        self.connect_btn.config(text="Connect", bg='#2ecc71')
        self.status_var.set("Disconnected")
    
    def read_serial_data(self):
        buffer = ""
        while self.is_connected and self.serial_connection and self.serial_connection.is_open:
            try:
                data = self.serial_connection.readline().decode('utf-8').strip()
                if data:
                    print("Diterima:", data)
                    try:
                        # Coba parsing sebagai angka float langsung
                        value = float(data)
                        timestamp = datetime.now()
                        # Simpan ke Sensor1 (sesuai dengan Arduino yang hanya mengirim 1 nilai)
                        self.sensor_data['Sensor1'].append(value)
                        self.sensor_timestamps['Sensor1'].append(timestamp)
                        if self.logging_active:
                            row = {'timestamp': timestamp}
                            row['Sensor1'] = value
                            for i in range(2, 6):
                                row[f'Sensor{i}'] = None
                            self.log_data.append(row)
                    except ValueError:
                        # Try to parse JSON data
                        if data.startswith('{') and data.endswith('}'):
                            try:
                                json_data = json.loads(data)
                                self.process_sensor_data(json_data)
                            except json.JSONDecodeError:
                                # If not JSON, try to parse as key:value pairs
                                self.process_text_data(data)
                        elif ':' in data or '=' in data:
                            self.process_text_data(data)
                        else:
                            # Data tidak dapat diparsing, abaikan
                            pass
            except UnicodeDecodeError:
                # Handle decode error (skip garbage data)
                pass
            except Exception as e:
                print(f"Error reading serial data: {e}")
                time.sleep(0.1)
    
    def process_sensor_data(self, data):
        timestamp = datetime.now()
        row = {'timestamp': timestamp}
        for i in range(1, 6):
            sensor_key = f"sensor{i}"
            if sensor_key in data:
                value = float(data[sensor_key])
                self.sensor_data[f'Sensor{i}'].append(value)
                self.sensor_timestamps[f'Sensor{i}'].append(timestamp)
                row[f'Sensor{i}'] = value
            else:
                row[f'Sensor{i}'] = None
        if self.logging_active:
            self.log_data.append(row)

    def process_text_data(self, data):
        # Process text data in format like "temp:25.6" or "sensor1=123"
        timestamp = datetime.now()
        
        # Try different delimiters
        if ':' in data:
            parts = data.split(':')
            if len(parts) == 2:
                key, value = parts[0].strip(), parts[1].strip()
                self.match_sensor_data(key, value, timestamp)
        elif '=' in data:
            parts = data.split('=')
            if len(parts) == 2:
                key, value = parts[0].strip(), parts[1].strip()
                self.match_sensor_data(key, value, timestamp)
        elif ',' in data:
            # CSV format
            parts = data.split(',')
            for part in parts:
                if ':' in part:
                    key_value = part.split(':')
                    if len(key_value) == 2:
                        key, value = key_value[0].strip(), key_value[1].strip()
                        self.match_sensor_data(key, value, timestamp)
    
    def match_sensor_data(self, key, value, timestamp):
        key = key.lower()
        sensor_idx = None
        for i in range(1, 6):
            if key == f"sensor{i}" or key == self.sensor_names[f'Sensor{i}'].get().lower():
                sensor_idx = i
                break
        if sensor_idx is None:
            for i in range(1, 6):
                sensor_name = self.sensor_names[f'Sensor{i}'].get().lower()
                if sensor_name in key or key in sensor_name:
                    sensor_idx = i
                    break
        if sensor_idx:
            try:
                num_value = float(value)
                self.sensor_data[f'Sensor{sensor_idx}'].append(num_value)
                self.sensor_timestamps[f'Sensor{sensor_idx}'].append(timestamp)
                # Cari baris terakhir, update nilai sensor
                if self.logging_active:
                    if self.log_data and self.log_data[-1]['timestamp'] == timestamp:
                        self.log_data[-1][f'Sensor{sensor_idx}'] = num_value
                    else:
                        row = {'timestamp': timestamp}
                        for i in range(1, 6):
                            row[f'Sensor{i}'] = None
                        row[f'Sensor{sensor_idx}'] = num_value
                        self.log_data.append(row)
            except ValueError:
                pass

    def on_mqtt_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.status_var.set("MQTT Connected")
        else:
            self.status_var.set(f"MQTT Connection failed: {rc}")
    
    def on_mqtt_message(self, client, userdata, msg):
        try:
            payload = msg.payload.decode('utf-8')
            timestamp = datetime.now()
            try:
                value = float(payload)
                for i in range(1, 6):
                    if msg.topic == self.mqtt_topics[f'Sensor{i}'].get():
                        self.sensor_data[f'Sensor{i}'].append(value)
                        self.sensor_timestamps[f'Sensor{i}'].append(timestamp)
                        if self.logging_active:
                            # Cari baris terakhir, update nilai sensor
                            if self.log_data and self.log_data[-1]['timestamp'] == timestamp:
                                self.log_data[-1][f'Sensor{i}'] = value
                            else:
                                row = {'timestamp': timestamp}
                                for j in range(1, 6):
                                    row[f'Sensor{j}'] = None
                                row[f'Sensor{i}'] = value
                                self.log_data.append(row)
                        break
            except ValueError:
                if payload.startswith('{') and payload.endswith('}'):
                    try:
                        json_data = json.loads(payload)
                        self.process_sensor_data(json_data)
                    except json.JSONDecodeError:
                        pass
        except Exception as e:
            print(f"Error processing MQTT message: {e}")
    
    def update_plot(self, frame):
        # Pilih resolusi waktu
        res = self.time_resolution.get()
        start_time = getattr(self, "capture_start_time", None)
        for i, line in enumerate(self.lines):
            sensor_key = f'Sensor{i+1}'
            y_data = list(self.sensor_data[sensor_key])
            ts_data = list(self.sensor_timestamps[sensor_key])
            # Filter data setelah capture dimulai
            if start_time:
                # Hitung waktu relatif
                rel_times = []
                rel_y = []
                for y, ts in zip(y_data, ts_data):
                    if ts >= start_time:
                        delta = ts - start_time
                        if res == "hour":
                            rel_times.append(round(delta.total_seconds() / 3600, 3))  # jam
                        elif res == "minute":
                            rel_times.append(round(delta.total_seconds() / 60, 3))   # menit
                        else:
                            rel_times.append(round(delta.total_seconds(), 3))        # detik
                        rel_y.append(y)
                x_data = rel_times
                y_data = rel_y
            else:
                x_data = []
                y_data = []
            line.set_data(x_data, y_data)

        # Update axes limits
        all_y = []
        all_x = []
        for i in range(1, 6):
            sensor_key = f'Sensor{i}'
            ts_data = list(self.sensor_timestamps[sensor_key])
            y_data = list(self.sensor_data[sensor_key])
            if start_time:
                for y, ts in zip(y_data, ts_data):
                    if ts >= start_time:
                        all_y.append(y)
                        delta = ts - start_time
                        if res == "hour":
                            all_x.append(round(delta.total_seconds() / 3600, 3))
                        elif res == "minute":
                            all_x.append(round(delta.total_seconds() / 60, 3))
                        else:
                            all_x.append(round(delta.total_seconds(), 3))
        if all_y:
            min_val = min(all_y)
            max_val = max(all_y)
            margin = (max_val - min_val) * 0.1 if max_val != min_val else 1
            self.ax.set_ylim(min_val - margin, max_val + margin)
        if all_x:
            self.ax.set_xlim(min(all_x), max(all_x))

        # Label sumbu X sesuai resolusi
        if res == "hour":
            self.ax.set_xlabel("Time (hours)", color='white', fontsize=12)
        elif res == "minute":
            self.ax.set_xlabel("Time (minutes)", color='white', fontsize=12)
        else:
            self.ax.set_xlabel("Time (seconds)", color='white', fontsize=12)

        # Update legend
        legend_labels = []
        for i in range(5):
            sensor_name = self.sensor_names[f'Sensor{i+1}'].get()
            unit = self.sensor_units[f'Sensor{i+1}'].get()
            current_value = list(self.sensor_data[f'Sensor{i+1}'])[-1] if self.sensor_data[f'Sensor{i+1}'] else 0
            legend_labels.append(f"{sensor_name}: {current_value:.2f} {unit}")
        self.ax.legend(legend_labels, loc='upper right')

        return self.lines
    
    def toggle_capture(self):
        if self.logging_active:
            self.logging_active = False
            self.capture_btn.config(text="Start Capture", bg='#3498db')
            self.status_var.set("Data capture stopped")
        else:
            self.log_data = []
            self.logging_active = True
            self.capture_btn.config(text="Stop Capture", bg='#e74c3c')
            # Simpan waktu mulai capture
            self.capture_start_time = datetime.now()
            duration_minutes = self.capture_duration.get()
            self.root.after(duration_minutes * 60 * 1000, self.stop_capture)
            self.status_var.set(f"Capturing data for {duration_minutes} minutes")
    
    def stop_capture(self):
        if self.logging_active:
            self.logging_active = False
            self.capture_btn.config(text="Start Capture", bg='#3498db')
            self.status_var.set(f"Data capture completed. Captured {len(self.log_data)} records")
    
    def export_to_excel(self):
        if not self.log_data:
            messagebox.showwarning("Warning", "No data to export")
            return
        try:
            df = pd.DataFrame(self.log_data)
            # Kolom sensor diberi nama sesuai konfigurasi
            sensor_columns = [self.sensor_names[f'Sensor{i}'].get() for i in range(1, 6)]
            rename_dict = {f'Sensor{i}': sensor_columns[i-1] for i in range(1, 6)}
            df = df.rename(columns=rename_dict)
            # Tambahkan kolom waktu relatif
            if 'timestamp' in df.columns and hasattr(self, "capture_start_time"):
                res = self.time_resolution.get()
                start_time = self.capture_start_time
                def rel_time(ts):
                    delta = pd.to_datetime(ts) - start_time
                    if res == "hour":
                        return round(delta.total_seconds() / 3600, 3)
                    elif res == "minute":
                        return round(delta.total_seconds() / 60, 3)
                    else:
                        return round(delta.total_seconds(), 3)
                df['Relative Time'] = df['timestamp'].apply(rel_time)
                df = df.drop(columns=['timestamp'])
                # Tempatkan kolom waktu relatif di depan
                cols = ['Relative Time'] + [col for col in df.columns if col != 'Relative Time']
                df = df[cols]
            filename = filedialog.asksaveasfilename(
                defaultextension=".xlsx",
                filetypes=[("Excel files", "*.xlsx"), ("CSV files", "*.csv")]
            )
            if filename:
                if filename.endswith('.xlsx'):
                    df.to_excel(filename, index=False)
                else:
                    df.to_csv(filename,      index=False)
                messagebox.showinfo("Success", f"Data exported successfully to {filename}")
        except Exception as e:
            messagebox.showerror("Error", f"Failed to export data: {str(e)}")

if __name__ == "__main__":
    root = tk.Tk()
    app = SensorMonitorApp(root)
    root.mainloop()
