import tkinter as tk
from tkinter import ttk, messagebox
from pymavlink import mavutil
import threading
import time
import math

class MavlinkGroundControl:
    def __init__(self, root):
        self.root = root
        self.root.title("PyMAVLink Yer Kontrol İstasyonu")
        self.root.geometry("1000x700")
        
        # MAVLink bağlantı değişkenleri
        self.master = None
        self.connected = False
        self.vehicle_type = ""
        self.vehicle_system = 0
        self.vehicle_component = 0
        
        # Veri değişkenleri
        self.attitude_data = {'roll': 0, 'pitch': 0, 'yaw': 0}
        self.gps_data = {'lat': 0, 'lon': 0, 'alt': 0, 'satellites': 0}
        self.battery_data = {'voltage': 0, 'current': 0, 'remaining': 0}
        self.system_status = "Bağlı Değil"
        self.flight_mode = "N/A"
        self.heartbeat_received = False
        
        # Arayüz oluştur
        self.create_widgets()
        
        # Veri güncelleme thread'i
        self.update_thread = None
        self.running = False
        
    def create_widgets(self):
        # Ana frame'ler
        self.connection_frame = ttk.LabelFrame(self.root, text="Bağlantı", padding=10)
        self.connection_frame.grid(row=0, column=0, padx=10, pady=5, sticky="ew")
        
        self.data_frame = ttk.LabelFrame(self.root, text="Uçuş Verileri", padding=10)
        self.data_frame.grid(row=1, column=0, padx=10, pady=5, sticky="nsew")
        
        self.map_frame = ttk.LabelFrame(self.root, text="Harita", padding=10)
        self.map_frame.grid(row=0, column=1, rowspan=2, padx=10, pady=5, sticky="nsew")
        
        # Bağlantı kontrolleri
        ttk.Label(self.connection_frame, text="Bağlantı:").grid(row=0, column=0)
        self.connection_combo = ttk.Combobox(self.connection_frame, 
                                           values=["udp:127.0.0.1:14550", 
                                                    "udp:0.0.0.0:14550",
                                                    "com3:57600",
                                                    "tcp:127.0.0.1:5760",
                                                    'tcp:127.0.0.1:5762',
                                                    'tcp:10.36.245.41:5762'])
        self.connection_combo.grid(row=0, column=1, padx=5)
        self.connection_combo.set("udp:127.0.0.1:14550")
        
        self.connect_btn = ttk.Button(self.connection_frame, text="Bağlan", command=self.toggle_connection)
        self.connect_btn.grid(row=0, column=2, padx=5)
        
        self.status_label = ttk.Label(self.connection_frame, text="Durum: Bağlı Değil")
        self.status_label.grid(row=1, column=0, columnspan=3, pady=5)
        
        # Uçuş verileri
        self.create_data_widgets()
        
        # Harita (basit bir canvas)
        self.map_canvas = tk.Canvas(self.map_frame, width=400, height=600, bg='white')
        self.map_canvas.pack(fill=tk.BOTH, expand=True)
        
        # Komut gönderme
        self.command_frame = ttk.LabelFrame(self.root, text="Komutlar", padding=10)
        self.command_frame.grid(row=2, column=0, columnspan=2, padx=10, pady=5, sticky="ew")
        
        self.create_command_widgets()
        
        # Grid yapılandırması
        self.root.columnconfigure(0, weight=1)
        self.root.columnconfigure(1, weight=2)
        self.root.rowconfigure(1, weight=1)
        
    def create_data_widgets(self):
        # Pozisyon verileri
        pos_frame = ttk.LabelFrame(self.data_frame, text="Pozisyon")
        pos_frame.grid(row=0, column=0, padx=5, pady=5, sticky="nsew")
        
        ttk.Label(pos_frame, text="Enlem:").grid(row=0, column=0, sticky="w")
        self.lat_label = ttk.Label(pos_frame, text="0.0")
        self.lat_label.grid(row=0, column=1, sticky="w")
        
        ttk.Label(pos_frame, text="Boylam:").grid(row=1, column=0, sticky="w")
        self.lon_label = ttk.Label(pos_frame, text="0.0")
        self.lon_label.grid(row=1, column=1, sticky="w")
        
        ttk.Label(pos_frame, text="Yükseklik:").grid(row=2, column=0, sticky="w")
        self.alt_label = ttk.Label(pos_frame, text="0.0 m")
        self.alt_label.grid(row=2, column=1, sticky="w")
        
        ttk.Label(pos_frame, text="Uydu Sayısı:").grid(row=3, column=0, sticky="w")
        self.sat_label = ttk.Label(pos_frame, text="0")
        self.sat_label.grid(row=3, column=1, sticky="w")
        
        # Attitude verileri
        att_frame = ttk.LabelFrame(self.data_frame, text="Oryantasyon")
        att_frame.grid(row=0, column=1, padx=5, pady=5, sticky="nsew")
        
        ttk.Label(att_frame, text="Roll:").grid(row=0, column=0, sticky="w")
        self.roll_label = ttk.Label(att_frame, text="0.0°")
        self.roll_label.grid(row=0, column=1, sticky="w")
        
        ttk.Label(att_frame, text="Pitch:").grid(row=1, column=0, sticky="w")
        self.pitch_label = ttk.Label(att_frame, text="0.0°")
        self.pitch_label.grid(row=1, column=1, sticky="w")
        
        ttk.Label(att_frame, text="Yaw:").grid(row=2, column=0, sticky="w")
        self.yaw_label = ttk.Label(att_frame, text="0.0°")
        self.yaw_label.grid(row=2, column=1, sticky="w")
        
        # Sistem durumu
        sys_frame = ttk.LabelFrame(self.data_frame, text="Sistem")
        sys_frame.grid(row=1, column=0, columnspan=2, padx=5, pady=5, sticky="ew")
        
        ttk.Label(sys_frame, text="Durum:").grid(row=0, column=0, sticky="w")
        self.sys_status_label = ttk.Label(sys_frame, text="Bağlı Değil")
        self.sys_status_label.grid(row=0, column=1, sticky="w")
        
        ttk.Label(sys_frame, text="Uçuş Modu:").grid(row=1, column=0, sticky="w")
        self.flight_mode_label = ttk.Label(sys_frame, text="N/A")
        self.flight_mode_label.grid(row=1, column=1, sticky="w")
        
        ttk.Label(sys_frame, text="Pil Voltajı:").grid(row=2, column=0, sticky="w")
        self.volt_label = ttk.Label(sys_frame, text="0.0 V")
        self.volt_label.grid(row=2, column=1, sticky="w")
        
        ttk.Label(sys_frame, text="Pil Yüzdesi:").grid(row=3, column=0, sticky="w")
        self.batt_percent_label = ttk.Label(sys_frame, text="0%")
        self.batt_percent_label.grid(row=3, column=1, sticky="w")
        
        # Grid yapılandırması
        self.data_frame.columnconfigure(0, weight=1)
        self.data_frame.columnconfigure(1, weight=1)
        self.data_frame.rowconfigure(0, weight=1)
        
    def create_command_widgets(self):
        # Uçuş modu değiştirme
        ttk.Label(self.command_frame, text="Uçuş Modu:").grid(row=0, column=0)
        self.mode_combo = ttk.Combobox(self.command_frame, 
                                     values=["STABILIZE", "ALT_HOLD", "LOITER", "GUIDED", "AUTO", "RTL"])
        self.mode_combo.grid(row=0, column=1, padx=5)
        self.mode_combo.set("GUIDED")
        
        self.set_mode_btn = ttk.Button(self.command_frame, text="Modu Değiştir", 
                                     command=self.set_flight_mode, state=tk.DISABLED)
        self.set_mode_btn.grid(row=0, column=2, padx=5)
        
        # Arm/Disarm
        self.arm_btn = ttk.Button(self.command_frame, text="ARM", 
                                 command=self.arm_disarm, state=tk.DISABLED)
        self.arm_btn.grid(row=0, column=3, padx=5)
        
        # Takeoff
        ttk.Label(self.command_frame, text="Takeoff Yükseklik (m):").grid(row=1, column=0)
        self.takeoff_alt = ttk.Entry(self.command_frame, width=10)
        self.takeoff_alt.grid(row=1, column=1, padx=5)
        self.takeoff_alt.insert(0, "10")
        
        self.takeoff_btn = ttk.Button(self.command_frame, text="Takeoff", 
                                     command=self.takeoff, state=tk.DISABLED)
        self.takeoff_btn.grid(row=1, column=2, padx=5)
        
        # Waypoint gönderme
        ttk.Label(self.command_frame, text="Hedef Koordinatlar:").grid(row=2, column=0)
        ttk.Label(self.command_frame, text="Enlem:").grid(row=2, column=1)
        self.wp_lat = ttk.Entry(self.command_frame, width=10)
        self.wp_lat.grid(row=2, column=2)
        
        ttk.Label(self.command_frame, text="Boylam:").grid(row=2, column=3)
        self.wp_lon = ttk.Entry(self.command_frame, width=10)
        self.wp_lon.grid(row=2, column=4)
        
        ttk.Label(self.command_frame, text="Yükseklik:").grid(row=2, column=5)
        self.wp_alt = ttk.Entry(self.command_frame, width=10)
        self.wp_alt.grid(row=2, column=6)
        
        self.send_wp_btn = ttk.Button(self.command_frame, text="Waypoint Gönder", 
                                    command=self.send_waypoint, state=tk.DISABLED)
        self.send_wp_btn.grid(row=2, column=7, padx=5)
        
    def toggle_connection(self):
        if self.connected:
            self.disconnect()
        else:
            self.connect()
    
    def connect(self):
        connection_string = self.connection_combo.get()
        print("burda1")
        try:
            self.master = mavutil.mavlink_connection(connection_string)
            print("burda2")
            self.master.wait_heartbeat()
            print("burda3")
            self.connected = True
            print("burda4")
            self.heartbeat_received = True
            print("burda5")
            self.vehicle_type = mavutil.mavlink.enums['MAV_TYPE'][self.master.mav_type].name
            print("burda6")
            self.vehicle_system = self.master.sysid
            print("burda7")
            self.vehicle_component = self.master.param_sysid[1]
            print("burda8")
            
            self.status_label.config(text=f"Durum: Bağlı ({self.vehicle_type}, SYS:{self.vehicle_system}, COMP:{self.vehicle_component})")
            print("burda9")
            self.connect_btn.config(text="Bağlantıyı Kes")
            print("burda10")
            self.sys_status_label.config(text="Bağlandı")
            print("burda11")
            
            # Komut butonlarını aktif et
            self.set_mode_btn.config(state=tk.NORMAL)
            self.arm_btn.config(state=tk.NORMAL)
            self.takeoff_btn.config(state=tk.NORMAL)
            self.send_wp_btn.config(state=tk.NORMAL)
            
            # Veri güncelleme thread'ini başlat
            self.running = True
            print("burda12")
            self.update_thread = threading.Thread(target=self.update_data, daemon=True)
            print("burda13")
            self.update_thread.start()
            print("burda14")
            
        except Exception as e:
            messagebox.showerror("Bağlantı Hatası", f"Bağlantı kurulamadı: {str(e)}")
            self.connected = False
            self.status_label.config(text="Durum: Bağlantı Hatası")
    
    def disconnect(self):
        self.running = False
        if self.update_thread and self.update_thread.is_alive():
            self.update_thread.join()
        
        if self.master:
            self.master.close()
        
        self.connected = False
        self.status_label.config(text="Durum: Bağlı Değil")
        self.connect_btn.config(text="Bağlan")
        self.sys_status_label.config(text="Bağlı Değil")
        
        # Komut butonlarını devre dışı bırak
        self.set_mode_btn.config(state=tk.DISABLED)
        self.arm_btn.config(state=tk.DISABLED)
        self.takeoff_btn.config(state=tk.DISABLED)
        self.send_wp_btn.config(state=tk.DISABLED)
    
    def update_data(self):
        while self.running:
            try:
                # Mesajları oku
                msg = self.master.recv_match(blocking=False)
                if msg:
                    self.process_message(msg)
                
                # UI güncelleme
                self.update_ui()
                
                # Haritayı güncelle
                self.update_map()
                
                time.sleep(0.1)
                
            except Exception as e:
                print(f"Veri güncelleme hatası: {str(e)}")
                time.sleep(1)
    
    def process_message(self, msg):
        msg_type = msg.get_type()
        
        if msg_type == "HEARTBEAT":
            self.heartbeat_received = True
            self.system_status = mavutil.mavlink.enums['MAV_STATE'][msg.system_status].name
            self.flight_mode = mavutil.mavlink.enums['MAV_MODE_FLAG'][msg.base_mode].name
            
        elif msg_type == "ATTITUDE":
            self.attitude_data['roll'] = math.degrees(msg.roll)
            self.attitude_data['pitch'] = math.degrees(msg.pitch)
            self.attitude_data['yaw'] = math.degrees(msg.yaw)
            
        elif msg_type == "GPS_RAW_INT":
            self.gps_data['lat'] = msg.lat / 1e7
            self.gps_data['lon'] = msg.lon / 1e7
            self.gps_data['alt'] = msg.alt / 1000  # mm to m
            self.gps_data['satellites'] = msg.satellites_visible
            
        elif msg_type == "SYS_STATUS":
            self.battery_data['voltage'] = msg.voltage_battery / 1000  # mV to V
            self.battery_data['current'] = msg.current_battery / 100  # 10mA to A
            self.battery_data['remaining'] = msg.battery_remaining
            
        elif msg_type == "VFR_HUD":
            self.altitude = msg.alt
            self.airspeed = msg.airspeed
            self.groundspeed = msg.groundspeed
            self.heading = msg.heading
            self.throttle = msg.throttle
            
    def update_ui(self):
        # GPS verileri
        self.lat_label.config(text=f"{self.gps_data['lat']:.6f}")
        self.lon_label.config(text=f"{self.gps_data['lon']:.6f}")
        self.alt_label.config(text=f"{self.gps_data['alt']:.1f} m")
        self.sat_label.config(text=f"{self.gps_data['satellites']}")
        
        # Attitude verileri
        self.roll_label.config(text=f"{self.attitude_data['roll']:.1f}°")
        self.pitch_label.config(text=f"{self.attitude_data['pitch']:.1f}°")
        self.yaw_label.config(text=f"{self.attitude_data['yaw']:.1f}°")
        
        # Sistem verileri
        self.sys_status_label.config(text=f"{self.system_status}")
        self.flight_mode_label.config(text=f"{self.flight_mode}")
        self.volt_label.config(text=f"{self.battery_data['voltage']:.1f} V")
        self.batt_percent_label.config(text=f"{self.battery_data['remaining']}%")
        
        # Arm/Disarm butonu güncelleme
        if self.connected and self.heartbeat_received:
            if self.flight_mode and 'MAV_MODE_FLAG_SAFETY_ARMED' in self.flight_mode:
                self.arm_btn.config(text="DISARM")
            else:
                self.arm_btn.config(text="ARM")
    
    def update_map(self):
        self.map_canvas.delete("all")
        
        # Basit bir harita çizimi
        width = self.map_canvas.winfo_width()
        height = self.map_canvas.winfo_height()
        
        # Merkez noktası
        center_x = width // 2
        center_y = height // 2
        
        # Drone pozisyonu (basitçe merkezde gösteriyoruz)
        drone_size = 10
        self.map_canvas.create_oval(center_x - drone_size, center_y - drone_size,
                                   center_x + drone_size, center_y + drone_size,
                                   fill="red", outline="black")
        
        # Koordinat bilgisi
        self.map_canvas.create_text(10, 10, anchor="nw", 
                                   text=f"Lat: {self.gps_data['lat']:.6f}\nLon: {self.gps_data['lon']:.6f}",
                                   font=("Arial", 10))
    
    def set_flight_mode(self):
        mode = self.mode_combo.get()
        try:
            if mode in ["STABILIZE", "ALT_HOLD", "LOITER", "GUIDED", "AUTO", "RTL"]:
                mode_id = self.master.mode_mapping()[mode]
                self.master.set_mode(mode_id)
                messagebox.showinfo("Başarılı", f"Uçuş modu {mode} olarak ayarlandı")
            else:
                messagebox.showerror("Hata", "Geçersiz uçuş modu")
        except Exception as e:
            messagebox.showerror("Hata", f"Uçuş modu değiştirilemedi: {str(e)}")
    
    def arm_disarm(self):
        try:
            if self.arm_btn.cget("text") == "ARM":
                self.master.arducopter_arm()
                messagebox.showinfo("Başarılı", "ARM komutu gönderildi")
            else:
                self.master.arducopter_disarm()
                messagebox.showinfo("Başarılı", "DISARM komutu gönderildi")
        except Exception as e:
            messagebox.showerror("Hata", f"ARM/DISARM komutu gönderilemedi: {str(e)}")
    
    def takeoff(self):
        try:
            altitude = float(self.takeoff_alt.get())
            self.master.mav.command_long_send(
                self.master.sysid, self.master.param_sysid[1],
                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                0, 0, 0, 0, 0, 0, 0, altitude)
            messagebox.showinfo("Başarılı", f"Takeoff komutu gönderildi ({altitude}m)")
        except Exception as e:
            messagebox.showerror("Hata", f"Takeoff komutu gönderilemedi: {str(e)}")
    
    def send_waypoint(self):
        try:
            lat = float(self.wp_lat.get())
            lon = float(self.wp_lon.get())
            alt = float(self.wp_alt.get())
            
            self.master.mav.send(mavutil.mavlink.MAVLink_set_position_target_global_int_message(
                0, self.master.sysid, self.master.param_sysid[1],
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                int(0b110111111000),  # POSITION_TARGET_TYPEMASK
                int(lat * 1e7), int(lon * 1e7), alt,
                0, 0, 0, 0, 0, 0, 0, 0))
                
            messagebox.showinfo("Başarılı", f"Waypoint gönderildi\nLat: {lat}, Lon: {lon}, Alt: {alt}m")
        except Exception as e:
            messagebox.showerror("Hata", f"Waypoint gönderilemedi: {str(e)}")

if __name__ == "__main__":
    root = tk.Tk()
    app = MavlinkGroundControl(root)
    root.mainloop()