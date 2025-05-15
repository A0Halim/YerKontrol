import tkinter as tk
from tkinter import ttk, messagebox
from pymavlink import mavutil
from pyproj import Proj
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
        self.gps_data = {'lat': 40.2318256, 'lon': 29.00003, 'alt': 0, 'satellites': 0}
        self.battery_data = {'voltage': 0, 'current': 0, 'remaining': 0}
        self.system_status = "Bağlı Değil"
        self.flight_mode = "N/A"
        self.heartbeat_received = False
        self.altitude_agl = 0  # Yer seviyesine göre yükseklik
        self.home_lat = 41.3622186
        self.home_lon = 36.1826348
        self.last_update_time = 0
        self.other_planes = {
            "konumBilgileri" : [
                {
                "takim_numarasi": 1,
                "iha_enlem": 40.2318256,
                "iha_boylam": 29.00993,
                "iha_irtifa": 36.0,
                "iha_dikilme": -8.0,
                "iha_yonelme": 127,
                "iha_yatis": 19.0,
                "iha_hizi": 41.0,
                "zaman_farki": 467
                }
            ]
        }
        self.qrcode = {
            "qrEnlem": 40.2326882,
            "qrBoylam": 29.00675778
        }
        self.hss_kordinat = [
            {
            "id": 0,
            "hssEnlem": 40.23260922,
            "hssBoylam": 29.00573015,
            "hssYaricap": 50
            },
            {
            "id": 1,
            "hssEnlem": 40.23351019,
            "hssBoylam": 28.99976492,
            "hssYaricap": 50
            },
            {
            "id": 2,
            "hssEnlem": 40.23105297,
            "hssBoylam": 29.00744677,
            "hssYaricap": 75
            },
            {
            "id": 3,
            "hssEnlem": 40.23090554,
            "hssBoylam": 29.00221109,
            "hssYaricap": 150
            }
        ]
        
        # Arayüz bilgileri
        self.zoom_factor = 1  # Varsayılan değer
        self.view_offset_x = 0  # X ekseninde kaydırma miktarı (pixel)
        self.view_offset_y = 0  # Y ekseninde kaydırma miktarı (pixel)
        self.proj = Proj(proj='utm', zone=35, ellps='WGS84')
        
        # Arayüz oluştur
        self.create_widgets()
        
        # Veri güncelleme thread'i
        self.update_thread = None
        self.running = False
        
        # UI güncelleme thread'i
        self.update_thread_ui = None
        self.running_iu = False
        
        self.home_x, self.home_y = self.proj(self.home_lon, self.home_lat)
        
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
        self.connection_combo.set("tcp:127.0.0.1:5762")
        
        self.connect_btn = ttk.Button(self.connection_frame, text="Bağlan", command=self.toggle_connection)
        self.connect_btn.grid(row=0, column=2, padx=5)
        
        self.status_label = ttk.Label(self.connection_frame, text="Durum: Bağlı Değil")
        self.status_label.grid(row=1, column=0, columnspan=3, pady=5)
        
        # ttk.Label(self.connection_frame, text="Server :").grid(row=0, column=3)
        # self.connect_server = ttk.Entry(self.connection_frame, width=10)
        # self.connect_server.grid(row=0, column=4, padx=5)
        
        # Uçuş verileri
        self.create_data_widgets()
        
        # Harita (basit bir canvas)
        self.current_map = True
        self.map_canvas = {
            True : tk.Canvas(self.map_frame, width=400, height=600, bg='white'),
            False : tk.Canvas(self.map_frame, width=400, height=600, bg='white')    
        }
        for map in self.map_canvas.values():
            map.pack(fill=tk.BOTH, expand=True)
            map.bind("<MouseWheel>", self.on_mousewheel)
            map.bind("<ButtonPress-1>", self.on_drag_start)
            map.bind("<B1-Motion>", self.on_drag_move)

        
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
        
        ttk.Label(pos_frame, text="Yükseklik (AGL):").grid(row=2, column=0, sticky="w")
        self.alt_label = ttk.Label(pos_frame, text="0.0 m")
        self.alt_label.grid(row=2, column=1, sticky="w")
        
        ttk.Label(pos_frame, text="Yükseklik (MSL):").grid(row=3, column=0, sticky="w")
        self.alt_msl_label = ttk.Label(pos_frame, text="0.0 m")
        self.alt_msl_label.grid(row=3, column=1, sticky="w")
        
        # ttk.Label(pos_frame, text="Uydu Sayısı:").grid(row=4, column=0, sticky="w")
        # self.sat_label = ttk.Label(pos_frame, text="0")
        # self.sat_label.grid(row=4, column=1, sticky="w")
        
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
        
        ttk.Label(sys_frame, text="Son Güncelleme:").grid(row=4, column=0, sticky="w")
        self.last_update_label = ttk.Label(sys_frame, text="Hiç güncellenmedi")
        self.last_update_label.grid(row=4, column=1, sticky="w")
        
        
        capture_frame = ttk.LabelFrame(self.data_frame, text="Tekip ekranı")
        capture_frame.grid(row=2, column=0, columnspan=2, padx=5, pady=5, sticky="nsew")
        tk.Canvas(capture_frame, width=400, height=280, bg='white').grid(row=0, column=0, sticky="nsew")
        
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
        
        self.set_mode_btn1 = ttk.Button(self.command_frame, text="Otonom Kalkış", 
                                     command=self.set_flight_mode, state=tk.DISABLED)
        self.set_mode_btn1.grid(row=0, column=4, padx=5)
        
        self.set_mode_btn2 = ttk.Button(self.command_frame, text="Qr görev", 
                                     command=self.set_flight_mode, state=tk.DISABLED)
        self.set_mode_btn2.grid(row=0, column=5, padx=5)
        
        self.set_mode_btn3 = ttk.Button(self.command_frame, text="Test", 
                                     command=self.set_flight_mode, state=tk.DISABLED)
        self.set_mode_btn3.grid(row=0, column=6, padx=5)
        
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
        print(connection_string)
        try:
            self.master = mavutil.mavlink_connection(connection_string)
            
            # 3 saniye içinde heartbeat bekleyin
            start_time = time.time()
            while time.time() - start_time < 3:
                msg = self.master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
                if msg:
                    break
                else:
                    raise ConnectionError("Heartbeat alınamadı - cihaz bağlı mı?")
            
            self.connected = True
            self.heartbeat_received = True
            self.vehicle_type = mavutil.mavlink.enums['MAV_TYPE'][self.master.mav_type].name
            self.vehicle_system = self.master.sysid
            self.vehicle_component = self.master.param_sysid[1]
            
            self.status_label.config(text=f"Durum: Bağlı ({self.vehicle_type}, SYS:{self.vehicle_system}, COMP:{self.vehicle_component})")
            self.connect_btn.config(text="Bağlantıyı Kes")
            self.sys_status_label.config(text="Bağlandı")
            
            # Komut butonlarını aktif et
            self.set_mode_btn.config(state=tk.NORMAL)
            self.arm_btn.config(state=tk.NORMAL)
            self.takeoff_btn.config(state=tk.NORMAL)
            self.send_wp_btn.config(state=tk.NORMAL)
            
            # Veri güncelleme thread'ini başlat
            self.running = True
            self.update_thread = threading.Thread(target=self.update_data, daemon=True)
            self.update_thread.start()
            
            # UI güncelleme thread'ini başlat
            self.running_iu = True
            self.update_thread_ui = threading.Thread(target=self.update_ui, daemon=True)
            self.update_thread_ui.start()
            
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
        last_msg_time = time.time()
        while self.running:
            try:
                msg = self.master.recv_match(blocking=True, timeout=1.0)
                if msg:
                    last_msg_time = time.time()
                    self.process_message(msg)
                else:
                    # 5 saniyeden fazla mesaj gelmezse bağlantı sorunu uyarısı
                    if time.time() - last_msg_time > 5:
                        self.system_status = "Bağlantı Sorunu"                
            except Exception as e:
                print(f"Veri güncelleme hatası: {str(e)}")
                time.sleep(1)
    
    def process_message(self, msg):
        msg_type = msg.get_type()
        
        if msg_type == "HEARTBEAT":
            self.heartbeat_received = True
            try:
                self.system_status = mavutil.mavlink.enums['MAV_STATE'][msg.system_status].name
                # Uçuş modunu doğru şekilde almak için:
                custom_mode = msg.custom_mode
                mode_mapping = self.master.mode_mapping()
                if mode_mapping:
                    for mode_name, mode_value in mode_mapping.items():
                        if custom_mode == mode_value:
                            self.flight_mode = mode_name
                            break
                    else:
                        self.flight_mode = f"Bilinmeyen Mod ({custom_mode})"
                else:
                    self.flight_mode = "Mod Bilgisi Yok"
            except Exception as e:
                print(f"Heartbeat işleme hatası: {str(e)}")
            
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
            self.altitude_agl = msg.alt  # Yer seviyesine göre yükseklik
    
    def update_ui(self):
        while self.running_iu:
            try:
                time.sleep(0.1)
                self.update_data_ui()
                self.update_map()
            except Exception as e:
                print(f"hata: {str(e)}")
                time.sleep(1)
    
    def update_data_ui(self):
        self.last_update_time = time.time()
        
        # GPS verileri
        self.lat_label.config(text=f"{self.gps_data['lat']:.6f}")
        self.lon_label.config(text=f"{self.gps_data['lon']:.6f}")
        self.alt_label.config(text=f"{self.altitude_agl:.1f} m")
        self.alt_msl_label.config(text=f"{self.gps_data['alt']:.1f} m")
        # self.sat_label.config(text=f"{self.gps_data['satellites']}")
        
        # Attitude verileri
        self.roll_label.config(text=f"{self.attitude_data['roll']:.1f}°")
        self.pitch_label.config(text=f"{self.attitude_data['pitch']:.1f}°")
        self.yaw_label.config(text=f"{self.attitude_data['yaw']:.1f}°")
        
        # Sistem verileri
        self.sys_status_label.config(text=f"{self.system_status}")
        self.flight_mode_label.config(text=f"{self.flight_mode}")
        self.volt_label.config(text=f"{self.battery_data['voltage']:.1f} V")
        self.batt_percent_label.config(text=f"{self.battery_data['remaining']}%")
        self.last_update_label.config(text=time.strftime("%H:%M:%S", time.localtime()))
        
        # Arm/Disarm butonu güncelleme
        if self.connected and self.heartbeat_received:
            if self.flight_mode and 'MAV_MODE_FLAG_SAFETY_ARMED' in self.system_status:
                self.arm_btn.config(text="DISARM")
            else:
                self.arm_btn.config(text="ARM")
    
    def update_map(self):
        self.map_canvas[self.current_map].pack(fill=tk.BOTH, expand=True)
        self.current_map = not self.current_map
        self.map_canvas[self.current_map].pack_forget()
        
        self.map_canvas[self.current_map].delete("all")
        width = self.map_canvas[self.current_map].winfo_width()
        height = self.map_canvas[self.current_map].winfo_height()
        
        # Grid çiz
        for i in range(0, width, 50):
            line_x = i + self.view_offset_x % 50
            self.map_canvas[self.current_map].create_line(line_x, 0, line_x, height, fill="lightgray")
            # self.map_canvas.create_text(i, 10, anchor="nw", text=self.gps_data["lon"], font=("Arial", 10), fill="black")
        for i in range(0, height, 50):
            line_y = i + self.view_offset_y % 50
            self.map_canvas[self.current_map].create_line(0, line_y, width, line_y, fill="lightgray")
            # self.map_canvas.create_text(10, i, anchor="nw", text=self.gps_data["lat"], font=("Arial", 10), fill="black")
            
        for hss in self.hss_kordinat:
            x, y = self.proj(hss["hssBoylam"], hss["hssEnlem"])
            hss_x = width // 2 + int((x - self.home_x) * self.zoom_factor) + self.view_offset_x
            hss_y = height // 2 - int((y - self.home_y) * self.zoom_factor) + self.view_offset_y
            hss_size = max(3, hss["hssYaricap"] * self.zoom_factor)
            self.map_canvas[self.current_map].create_oval(hss_x - hss_size, hss_y - hss_size, hss_x + hss_size, hss_y + hss_size, fill="yellow")

        x, y = self.proj(self.qrcode["qrBoylam"], self.qrcode["qrEnlem"])
        qrsize = max(5, min(20, 10 * self.zoom_factor))  
        qr_x = width // 2 + int((x - self.home_x) * self.zoom_factor) + self.view_offset_x
        qr_y = height // 2 - int((y - self.home_y) * self.zoom_factor) + self.view_offset_y      
        self.map_canvas[self.current_map].create_rectangle(qr_x - qrsize, qr_y - qrsize, qr_x + qrsize, qr_y + qrsize, fill="black")
        
        # Drone pozisyonu (basit hareket simülasyonu)
        x, y = self.proj(self.gps_data['lon'], self.gps_data['lat'])
        if self.home_lat != 0 and self.home_lon != 0:
            drone_x = width // 2 + int((x - self.home_x) * self.zoom_factor) + self.view_offset_x
            drone_y = height // 2 - int((y - self.home_y) * self.zoom_factor) + self.view_offset_y
        else:
            drone_x = width // 2
            drone_y = height // 2
        
        # Sınır kontrolü
        drone_x = max(20, min(width-20, drone_x))
        drone_y = max(20, min(height-20, drone_y))
        
        # Drone çiz
        drone_size = max(5, min(20, 10 * self.zoom_factor))
        self.map_canvas[self.current_map].create_oval(drone_x - drone_size, drone_y - drone_size, drone_x + drone_size, drone_y + drone_size, 
                                  fill="red", outline="black")
        # Yönü göster
        arrow_len = 20
        end_x = drone_x + arrow_len * math.sin(math.radians(self.attitude_data['yaw']))
        end_y = drone_y - arrow_len * math.cos(math.radians(self.attitude_data['yaw']))
        self.map_canvas[self.current_map].create_line(drone_x, drone_y, end_x, end_y, arrow=tk.LAST, width=2)
        
        for drone_data in self.other_planes["konumBilgileri"]:
            # Diğer drone'un konumunu hesapla
            x, y = self.proj(drone_data['iha_boylam'], drone_data['iha_enlem'])
            otherdrone_x = width // 2 + int((x - self.home_x) * self.zoom_factor) + self.view_offset_x
            otherdrone_y = height // 2 - int((y - self.home_y) * self.zoom_factor) + self.view_offset_y
            # Diğer drone'u çiz
            otherdrone_size = max(5, min(20, 10 * self.zoom_factor))
            self.map_canvas[self.current_map].create_oval(
                otherdrone_x - otherdrone_size, otherdrone_y - otherdrone_size, 
                otherdrone_x + otherdrone_size, otherdrone_y + otherdrone_size, 
                fill="blue", outline="black"
            )
            # Diğer dronun yönünü göster
            arrow_len = 20
            end_x = otherdrone_x + arrow_len * math.cos(math.radians(drone_data['iha_yonelme'] - 90))
            end_y = otherdrone_y - arrow_len * math.sin(math.radians(drone_data['iha_yonelme'] - 90))
            self.map_canvas[self.current_map].create_line(otherdrone_x, otherdrone_y, end_x, end_y, arrow=tk.LAST, width=2)
        
        
        # Bilgiler
        # info_text = f"Lat: {self.gps_data['lat']:.6f}\nLon: {self.gps_data['lon']:.6f}\n"
        # info_text += f"Yükseklik: {self.altitude_agl:.1f}m\n"
        # info_text += f"Mod: {self.flight_mode}"
        # self.map_canvas.create_text(10, 10, anchor="nw", text=info_text, 
        #                           font=("Arial", 10), fill="black")
    
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

    def on_mousewheel(self, event):
        # Yukarı scroll (zoom in)
        if event.delta > 0:
            self.zoom_factor *= 1.1  # %10 büyüt
            self.view_offset_x *= 1.1
            self.view_offset_y *= 1.1
        # Aşağı scroll (zoom out)
        elif event.delta < 0:
            self.zoom_factor *= 0.9  # %10 küçült
            self.view_offset_x *= 0.9
            self.view_offset_y *= 0.9
        
        # Zoom'u güncelle (yeniden çiz)
        self.update_map()
        
    def on_drag_start(self, event):
        self.last_x = event.x
        self.last_y = event.y

    def on_drag_move(self, event):
        # Fare hareket miktarını hesapla
        dx = event.x - self.last_x
        dy = event.y - self.last_y
        
        # Ofseti güncelle
        self.view_offset_x += dx
        self.view_offset_y += dy
        
        # Son pozisyonu kaydet
        self.last_x = event.x
        self.last_y = event.y
        
        # Haritayı yeniden çiz
        self.update_map()
        
if __name__ == "__main__":
    root = tk.Tk()
    app = MavlinkGroundControl(root)
    root.mainloop()