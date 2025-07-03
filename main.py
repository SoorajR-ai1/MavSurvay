import sys
import os
import socket
import PyQt5.QtWidgets
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QHBoxLayout, QLabel, QWidget, QDockWidget,QPushButton, QTextEdit, QTableWidget, QTableWidgetItem, QScrollArea, QMessageBox, QFileDialog, QInputDialog, QTabWidget, QAction, QMenuBar
from PyQt5.QtWebEngineWidgets import QWebEngineView
from PyQt5.QtCore import QUrl, QTimer, Qt
import math
import pyproj
import csv
import subprocess
import serial.tools.list_ports
from tkinter import filedialog, messagebox
import re
import threading
from pymavlink import mavutil
import datetime
import json



UDP_IP = "0.0.0.0"
UDP_PORT = 10051

class SoftwareVeriables:
    def __init__(self):

        self.ASV_gps = {
            "latitude": 0.0,
            "longitude": 0.0,
            "altitude": 0.0,
            "easting": 0.0,
            "northing": 0.0,
            "zone": "",
            "hdop": 0.0,
            "vdop": 0.0,
            "satellites_visible": 0,
            "date_time": datetime.datetime.now().strftime("%Y-%m-%d,%H:%M:%S")
        }
        self.ASV_pingsonar = 0.0

        self.ASV_attitude = {
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 0.0,
            "quaternion": [0.0, 0.0, 0.0, 1.0]
        }
        self.ASV_atmosphere = {
            "temperature": 0.0,
            "humidity": 0.0,
            "pressure": 0.0
        }
        self.ASV_system_time = {
            "time_unix_usec": 0,
            "time_boot_ms": 0,
        }
        self.ASV_battery_status = {
            "voltage": 0.0,
            "current": 0.0,
            "remaining": 0.0,
            "temperature": 0.0
        }

        self.WaterQuality = {
            "temperature": 0.0,
            "ph": 0.0,
            "turbidity": 0.0,
            "dissolved_oxygen": 0.0,
            "conductivity": 0.0,
            "salinity": 0.0
        }

        self.nmea_data = {
            "SDDPT": {
                "depth": 0.0,
                "units": "m"
            },
            "SDDBT": {
                "depth": 0.0,
                "units": "m"
            }
        }

        self.soft_logrecord_path = ""   
        self.soft_logrecord_rate = 1  # Default log record rate in hertz
        self.soft_logrecord_flag = False  # Flag to indicate if logging is active

        



class MapWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.init_menu_bar()

        self.soft_vars = SoftwareVeriables()

        self.setWindowTitle("MavSurvay")
        self.setGeometry(100, 100, 1600, 900)

        # Class attributes
        self.survey_points = []
        self.timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        #self.survey_dir = f"Survey/{self.timestamp}"
        #os.makedirs(self.survey_dir, exist_ok=True)
        #self.MAP_FILE = f"{self.survey_dir}/map_{self.timestamp}.html"
        #self.CSV_FILE = f"{self.survey_dir}/survey_log_{self.timestamp}.csv"
        #self.depth_log_file = f"{self.survey_dir}/depth_{self.timestamp}.csv"
        #self.sonar_serial = None
        #soft_logrecord_flag = False  # Flag to indicate if logging is active

        # Setup UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((UDP_IP, UDP_PORT))
        self.sock.setblocking(False)  # Non-blocking mode


        # Setup WebEngine View
        self.browser = QWebEngineView()
        self.setCentralWidget(self.browser)

        # Timer to update UI
        self.ui_update_timer = QTimer()
        self.ui_update_timer.timeout.connect(self.update_ui)
        self.ui_update_timer.start(100)

        # Timer to update MAP
        self.map_update_timer = QTimer()
        self.map_update_timer.timeout.connect(self.update_MAP)
        self.map_update_timer.start(1000)  # Update every second

        #start MAVLink thread
        self.start_mavlink_thread(udp_ip="127.0.0.1", udp_port=14555)  # Or your MAVLink UDP port

        #start NMEA thread
        self.start_nmea_thread(self,serial_port="COM4", baudrate=4800)  # Or your NMEA serial port

        self.browser.load(QUrl.fromLocalFile(os.path.abspath("live_map.html")))

        self.init_ui()



    def closeEvent(self, event):
        """Ensure serial port is closed on exit."""
        if self.sonar_serial and self.sonar_serial.is_open:
            self.sonar_serial.close()
        event.accept()

    def init_ui(self):
        self.init_left_panel()
        self.init_right_panel()
        self.init_bottom_panel()
        

    def init_left_panel(self):
        dock = QDockWidget("Mission Config", self)
        panel = QWidget()
        layout = QVBoxLayout()

        # File info
        self.projectFolder = QLabel(f"No Project Selected")
        self.open_folder_btn = QPushButton("Open Folder")
        self.open_folder_btn.setDisabled(True)
        self.open_folder_btn.clicked.connect(self.open_survey_folder)
        layout.addWidget(self.projectFolder)
        layout.addWidget(self.open_folder_btn)
        layout.addSpacing(20)


        # Waypoint editor
        layout.addWidget(QLabel("Waypoint Editor:"))
        wpEdit_btn = QPushButton("Alter Waypoint")
        wpEdit_btn.clicked.connect(self.open_and_process_file)
        layout.addWidget(wpEdit_btn)
        layout.addSpacing(40)
        
        layout.addWidget(QLabel("Recording Controls:"))
        self.record_all_points_btn = QPushButton("Record Data")
        self.record_all_points_btn.clicked.connect(self.record_all_points)
        layout.addWidget(self.record_all_points_btn)
        layout.addSpacing(10)
        self.record_all_points_btn = QPushButton("Stop Recording")
        self.record_all_points_btn.clicked.connect(self.StropRecording)
        layout.addWidget(self.record_all_points_btn)
        layout.addStretch()

        panel.setLayout(layout)
        dock.setWidget(panel)
        #dock.setFeatures(QDockWidget.NoDockWidgetFeatures)
        self.addDockWidget(Qt.LeftDockWidgetArea, dock)

    def init_right_panel(self):
        dock = QDockWidget("Telemetry & Controls", self)
        rightpanel = QWidget()
        layout = QVBoxLayout()



        # Labels to store live telemetry data
        self.ASV_gps_label = QLabel("GPS Data")
        self.ASV_gps_datetime_label = QLabel("Date Time: --")
        self.ASV_gps_hdop_label = QLabel("HDOP: --")
        self.ASV_gps_vdop_label = QLabel("VDOP: --")
        self.ASV_gps_satellites_label = QLabel("Satellites: --")
        self.ASV_gps_altitude_label = QLabel("Altitude: --")
        self.ASV_gps_lat_label = QLabel("Latitude: --")
        self.ASV_gps_lon_label = QLabel("Longitude: --")
        self.ASV_gps_easting_label = QLabel("Easting: --")
        self.ASV_gps_northing_label = QLabel("Northing: --")
        self.ASV_gps_zone_label = QLabel("Zone: --")

        self.ASV_gps_label.setStyleSheet("font-weight: bold; font-size: 12px;")
        layout.addWidget(self.ASV_gps_label)
        layout.addWidget(self.ASV_gps_datetime_label)
        layout.addWidget(self.ASV_gps_hdop_label)
        layout.addWidget(self.ASV_gps_vdop_label)
        layout.addWidget(self.ASV_gps_satellites_label)
        layout.addWidget(self.ASV_gps_altitude_label)
        layout.addWidget(self.ASV_gps_lat_label)
        layout.addWidget(self.ASV_gps_lon_label)
        layout.addWidget(self.ASV_gps_easting_label)
        layout.addWidget(self.ASV_gps_northing_label)
        layout.addWidget(self.ASV_gps_zone_label)
        layout.addSpacing(10)

        # Attitude data
        self.ASV_attitude_label = QLabel("Attitude Data")
        self.ASV_attitude_roll_label = QLabel("Roll: --")
        self.ASV_attitude_pitch_label = QLabel("Pitch: --")
        self.ASV_attitude_yaw_label = QLabel("Yaw: --")
        self.ASV_attitude_label.setStyleSheet("font-weight: bold; font-size: 12px;")
        layout.addWidget(self.ASV_attitude_label)
        layout.addWidget(self.ASV_attitude_roll_label)
        layout.addWidget(self.ASV_attitude_pitch_label)
        layout.addWidget(self.ASV_attitude_yaw_label)
        layout.addSpacing(10)

        # Sonar data
        self.ASV_sonar_label = QLabel("Sonar Data")
        self.ASV_sonar_ping_label = QLabel("Ping Sonar: -- m")
        self.ASV_sonar_label.setStyleSheet("font-weight: bold; font-size: 12px;")
        layout.addWidget(self.ASV_sonar_label)
        layout.addWidget(self.ASV_sonar_ping_label)
        layout.addSpacing(10)

        # Water quality data
        self.ASV_water_quality_label = QLabel("Water Quality Data")
        self.ASV_water_quality_temp_label = QLabel("Temperature: -- °C")
        self.ASV_water_quality_ph_label = QLabel("PH: --")
        self.ASV_water_quality_turbidity_label = QLabel("Turbidity: -- NTU")
        self.ASV_water_quality_dissolved_oxygen_label = QLabel("Dissolved Oxygen: -- mg/L")
        self.ASV_water_quality_conductivity_label = QLabel("Conductivity: -- µS/cm")
        self.ASV_water_quality_salinity_label = QLabel("Salinity: -- ppt")
        self.ASV_water_quality_label.setStyleSheet("font-weight: bold; font-size: 12px;")
        layout.addWidget(self.ASV_water_quality_label)
        layout.addWidget(self.ASV_water_quality_temp_label)
        layout.addWidget(self.ASV_water_quality_ph_label)
        layout.addWidget(self.ASV_water_quality_turbidity_label)
        layout.addWidget(self.ASV_water_quality_dissolved_oxygen_label)
        layout.addWidget(self.ASV_water_quality_conductivity_label)
        layout.addWidget(self.ASV_water_quality_salinity_label)
        layout.addSpacing(10)

        # System time
        self.ASV_system_time_label = QLabel("System Time")
        self.ASV_system_time_unix_label = QLabel("UNIX Time: --")
        self.ASV_system_time_boot_label = QLabel("Boot Time: --")
        self.ASV_system_time_machine_label = QLabel("Machine Time: --")
        self.ASV_system_time_label.setStyleSheet("font-weight: bold; font-size: 12px;")
        layout.addWidget(self.ASV_system_time_label)
        layout.addWidget(self.ASV_system_time_unix_label)
        layout.addWidget(self.ASV_system_time_boot_label)
        layout.addWidget(self.ASV_system_time_machine_label)
        layout.addSpacing(10)
        # Battery status       
        self.ASV_battery_status_label = QLabel("Battery Status")
        self.ASV_battery_voltage_label = QLabel("Voltage: -- V")
        self.ASV_battery_current_label = QLabel("Current: -- A")
        self.ASV_battery_remaining_label = QLabel("Remaining: -- %")
        self.ASV_battery_temperature_label = QLabel("Temperature: -- °C")
        self.ASV_battery_status_label.setStyleSheet("font-weight: bold; font-size: 12px;")
        layout.addWidget(self.ASV_battery_status_label)
        layout.addWidget(self.ASV_battery_voltage_label)
        layout.addWidget(self.ASV_battery_current_label)
        layout.addWidget(self.ASV_battery_remaining_label)
        layout.addWidget(self.ASV_battery_temperature_label)
        layout.addSpacing(10)

        # Depth label
        self.depth_label = QLabel("NMEA Depth: -- m")
        self.depth_label.setStyleSheet("font-weight: bold; font-size: 12px;")
        layout.addWidget(self.depth_label)

        #warp the panel inside a QScrollArea
        scroll_area = QScrollArea()
        scroll_area.setWidgetResizable(True)  # Important to resize scroll area with window
        scroll_area.setWidget(rightpanel)


        layout.addStretch()
        rightpanel.setLayout(layout)
        dock.setWidget(scroll_area)
        #dock.setFeatures(QDockWidget.NoDockWidgetFeatures)
        self.addDockWidget(Qt.RightDockWidgetArea, dock)



    def init_bottom_panel(self):
        dock = QDockWidget("Elevation / Log", self)

        panel = QWidget()
        layout = QVBoxLayout()

        self.route_summary_label = QLabel("Data Log")
        self.log_text_edit = QTextEdit()
        self.log_text_edit.setReadOnly(True)
        self.log_text_edit.setText("Waiting For Data")

        layout.addWidget(self.route_summary_label)
        layout.addWidget(self.log_text_edit)

        panel.setLayout(layout)

        # Wrap your panel inside a QScrollArea
        scroll_area = QScrollArea()
        scroll_area.setWidgetResizable(True)  # Important to resize scroll area with window
        scroll_area.setWidget(panel)

        dock.setWidget(scroll_area)
        #dock.setFeatures(QDockWidget.NoDockWidgetFeatures)
        self.addDockWidget(Qt.BottomDockWidgetArea, dock)
        

    # Waypoint editor
    def insert_servo_actions(self, lines, servo_number=5, high_pwm=1900, low_pwm=1100, delay_seconds=30):
        output_lines = []
        seq = 0  # Recalculate sequence numbers

        header = lines[0].strip()
        output_lines.append(header)

        for i in range(1, len(lines)):
            line = lines[i].strip()
            if not line:
                continue

            parts = line.split('\t')
            if len(parts) != 12:
                continue

            cmd = int(parts[3])
            output_lines.append('\t'.join([str(seq)] + parts[1:]))
            seq += 1

            if cmd == WAYPOINT_CMD:
                # Insert DO_SET_SERVO (HIGH)
                output_lines.append('\t'.join([
                    str(seq), parts[1], parts[2], str(DO_SET_SERVO),
                    str(servo_number), str(high_pwm), "0", "0", "0", "0", "0", parts[11]
                ]))
                seq += 1

                # Insert DO_DELAY
                output_lines.append('\t'.join([
                    str(seq), parts[1], parts[2], str(DO_DELAY),
                    str(delay_seconds), "0", "0", "0", "0", "0", "0", parts[11]
                ]))
                seq += 1

                # Insert DO_SET_SERVO (LOW)
                output_lines.append('\t'.join([
                    str(seq), parts[1], parts[2], str(DO_SET_SERVO),
                    str(servo_number), str(low_pwm), "0", "0", "0", "0", "0", parts[11]
                ]))
                seq += 1

        return output_lines


    def open_and_process_file(self):
        from PyQt5.QtWidgets import QFileDialog
        input_path, _ = QFileDialog.getOpenFileName(self, "Select Waypoint File", "", "Waypoint Files (*.waypoints *.txt)")
        if not input_path:
            return
        with open(input_path, 'r') as f:
            lines = f.readlines()
        modified_lines = self.insert_servo_actions(lines)
        output_path, _ = QFileDialog.getSaveFileName(self, "Save Modified File", "", "Waypoint Files (*.waypoints)")
        if not output_path:
            return
        with open(output_path, 'w') as f:
            for line in modified_lines:
                f.write(line + '\n')
        QMessageBox.information(self, "Done", "Modified waypoint file saved.")

    def refresh_ports(self):
        self.port_dropdown.clear()
        ports = serial.tools.list_ports.comports()
        for port in ports:
            self.port_dropdown.addItem(port.device)

    def connect_sonar(self):
        selected_port = self.port_dropdown.currentText()
        if not selected_port:
            QMessageBox.warning(self, "No Port", "No serial port selected.")
            return
        try:
            if self.sonar_serial and self.sonar_serial.is_open:
                self.sonar_serial.close()
            self.sonar_serial = serial.Serial(selected_port, baudrate=115200, timeout=1)
            self.sonar_serial.write(b"\r\n")
            response = self.sonar_serial.readline().decode(errors='ignore').strip()
            if "ISA500" in response or len(response) > 0:
                QMessageBox.information(self, "Connection", f"Connected to Sonar on {selected_port}")
            else:
                QMessageBox.warning(self, "Connection", f"Port opened but no ISA500 response")
        except Exception as e:
            QMessageBox.critical(self, "Connection Error", str(e))

    def read_serial_data(self):
        """Read and parse serial data from sonar."""
        if self.sonar_serial and self.sonar_serial.is_open:
            try:
                if self.sonar_serial.in_waiting:
                    line = self.sonar_serial.readline().decode(errors='ignore').strip()
                    if line:
                        print(f"Serial Data: {line}")
                        if line.startswith("$SDDPT"):
                            parts = line.split(",")
                            if len(parts) > 1:
                                try:
                                    depth = float(parts[1])
                                    self.soft_vars.depth = depth
                                    self.depth_label.setText(f"Depth: {depth:.2f} m")  # <-- Add this line
                                except ValueError:
                                    pass
            except Exception as e:
                print(f"Serial read error: {e}")
    


    def open_survey_folder(self):
        folder_path = os.path.abspath(self.survey_dir)
        if os.path.exists(folder_path):
            if sys.platform == "win32":
                os.startfile(folder_path)
            elif sys.platform == "darwin":
                subprocess.Popen(["open", folder_path])
            else:
                subprocess.Popen(["xdg-open", folder_path])


    def lat_lon_to_utm(self, latitude, longitude):
        zone_number, hemisphere = self.get_utm_zone(latitude, longitude)
        utm_crs_code = f"EPSG:326{zone_number}" if hemisphere == 'N' else f"EPSG:327{zone_number}"
    
        wgs84 = pyproj.CRS("EPSG:4326")
        utm_crs = pyproj.CRS(utm_crs_code)
        transformer = pyproj.Transformer.from_crs(wgs84, utm_crs, always_xy=True)
    
        easting, northing = transformer.transform(longitude, latitude)
    
        return easting, northing, zone_number, hemisphere

    # Function to calculate UTM zone
    def get_utm_zone(self, latitude, longitude):
        zone_number = math.floor((longitude + 180) / 6) + 1
        hemisphere = 'N' if latitude >= 0 else 'S'
        return zone_number, hemisphere

    
    def start_mavlink_thread(self, udp_ip, udp_port):
        """Start a thread to receive and parse MAVLink messages."""
        self.mavlink_running = True
        self.mavlink_thread = threading.Thread(
            target=self.mavlink_receiver, args=(udp_ip, udp_port), daemon=True
        )
        self.mavlink_thread.start()
        print(f"MAVLink thread started on {udp_ip}:{udp_port}")

    def stop_mavlink_thread(self):
        self.mavlink_running = False

    def mavlink_receiver(self, udp_ip, udp_port):
        mav = mavutil.mavlink_connection(f'udp:{udp_ip}:{udp_port}', baud=115200)
        while self.mavlink_running:
            msg = mav.recv_match(blocking=True, timeout=1)
            #print(f"Received MAVLink message: {msg}")
            if msg is None:
                continue
            msg_type = msg.get_type()
            # Parse only the messages you want
            if msg_type == "SYS_STATUS":
                #print("SYS_STATUS:", msg.to_dict())
                pass
            elif msg_type == "GPS_RAW_INT":
                gps_data = msg.to_dict()
                time_usec = gps_data.get("time_usec", 0)
                # Check if time_usec is a plausible UNIX timestamp (after year 2000)
                if time_usec > 946684800_000_000:  # 2000-01-01 in microseconds
                    dt = datetime.datetime.utcfromtimestamp(time_usec / 1e6)
                    self.soft_vars.ASV_gps["date_time"] = dt.strftime("%Y-%m-%d,%H:%M:%S")
                else:
                    self.soft_vars.ASV_gps["date_time"] = datetime.datetime.now().strftime("%Y-%m-%d,%H:%M:%S")
                    #print(f"Invalid time_usec: {time_usec}, using current time instead.")


                self.soft_vars.ASV_gps["latitude"] = gps_data.get("lat", 0) / 1e7
                self.soft_vars.ASV_gps["longitude"] = gps_data.get("lon", 0) / 1e7
                self.soft_vars.ASV_gps["easting"], self.soft_vars.ASV_gps["northing"], self.soft_vars.ASV_gps["zone"], _ = self.lat_lon_to_utm(
                    self.soft_vars.ASV_gps["latitude"], self.soft_vars.ASV_gps["longitude"]
                )

            elif msg_type == "ATTITUDE":
                #print("ATTITUDE:", msg.to_dict())
                attitude = msg.to_dict()
                self.soft_vars.ASV_attitude["roll"] = math.degrees(attitude.get("roll", 0.0))
                self.soft_vars.ASV_attitude["pitch"] = math.degrees(attitude.get("pitch", 0.0))
                self.soft_vars.ASV_attitude["yaw"] = math.degrees(attitude.get("yaw", 0.0))
                self.soft_vars.ASV_attitude["yaw"] = (self.soft_vars.ASV_attitude["yaw"] + 360) % 360  # Normalize to 0-360 degrees
                
            elif msg_type == "GLOBAL_POSITION_INT":
                global_pos = msg.to_dict()
                # Example fields: time_boot_ms, lat, lon, alt, relative_alt, vx, vy, vz, hdg
                self.soft_vars.ASV_gps["altitude"] = global_pos.get("alt", 0.0) / 1000.0  # Convert mm to m
                self.soft_vars.ASV_gps["latitude"] = global_pos.get("lat", 0.0) / 1e7
                self.soft_vars.ASV_gps["longitude"] = global_pos.get("lon", 0.0) / 1e7
                self.soft_vars.ASV_gps["easting"], self.soft_vars.ASV_gps["northing"], self.soft_vars.ASV_gps["zone"], _ = self.lat_lon_to_utm(
                self.soft_vars.ASV_gps["latitude"], self.soft_vars.ASV_gps["longitude"]
                )
                self.soft_vars.ASV_gps["hdop"] = global_pos.get("hdg", 0.0) / 100.0  # Convert to degrees
                self.soft_vars.ASV_gps["satellites_visible"] = global_pos.get("satellites_visible", 0)
                self.soft_vars.ASV_gps["altitude"] = global_pos.get("alt", 0.0) / 1000.0  # Convert mm to m
                self.soft_vars.ASV_gps["vdop"] = global_pos.get("vdop", 0.0) / 100.0  # Convert to degrees


            elif msg_type == "DISTANCE_SENSOR":
                print("DISTANCE_SENSOR:", msg.to_dict())
                distance_sensor = msg.to_dict()
                self.soft_vars.ASV_pingsonar = distance_sensor.get("current_distance", 0.0) / 100.0  # Convert cm to m
                pass
            elif msg_type == "RANGEFINDER":
                print("RANGEFINDER:", msg.to_dict())
                rangefinder = msg.to_dict()
                self.soft_vars.ASV_pingsonar = rangefinder.get("distance", 0.0) / 100.0  # Convert cm to m
                pass

            elif msg_type == "WATER_DEPTH":
                print("WATER_DEPTH:", msg.to_dict())
                water_depth = msg.to_dict()
                self.soft_vars.depth = water_depth.get("depth", 0.0) / 100.0  # Convert cm to m
                pass

    def start_nmea_thread(self, udp_ip=None, udp_port=None, serial_port=None, baudrate=None):
        print
        self.nmea_running = True
        if udp_ip and udp_port:
            self.nmea_thread = threading.Thread(
                target=self.nmea_receiver_udp, args=(udp_ip, udp_port), daemon=True
            )
            print(f"Starting NMEA thread for UDP {udp_ip}:{udp_port}")
        elif serial_port and baudrate:
            self.nmea_thread = threading.Thread(
                target=self.nmea_receiver_serial, args=(serial_port, baudrate), daemon=True
            )
            print(f"Starting NMEA thread for Serial {serial_port} at {baudrate} baud")
        self.nmea_thread.start()

    def nmea_receiver_serial(self, serial_port, baudrate):

        #code to decode all the nmea sentences from the serial port

        try:
            with serial.Serial(serial_port, baudrate, timeout=1) as ser:
                while self.nmea_running:
                    line = ser.readline().decode(errors='ignore').strip()
                    if line.startswith("$"):
                        self.parse_nmea_sentence(line)
        except serial.SerialException as e:
            print(f"Serial error: {e}")

    def nmea_receiver_udp(self, udp_ip, udp_port):
        """Receive NMEA sentences over UDP."""
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.bind((udp_ip, udp_port))
            while self.nmea_running:
                try:
                    data, _ = sock.recvfrom(1024)
                    line = data.decode(errors='ignore').strip()
                    if line.startswith("$"):
                        self.parse_nmea_sentence(line)
                except socket.error as e:
                    print(f"Socket error: {e}")

    
    def parse_nmea_sentence(self, sentence):
        """Parse NMEA sentence and update software variables."""
        parts = sentence.split(',')
        if not parts:
            return

        # Handle SDDPT (Depth)
        if parts[0] == "$SDDPT" and len(parts) >= 3:
            try:
                depth = float(parts[1])
                self.soft_vars.nmea_data["SDDPT"]["depth"] = depth
                self.soft_vars.nmea_data["SDDPT"]["units"] = parts[2] if len(parts) > 2 else "m"
            except ValueError:
                pass
        elif parts[0] == "$SDDBT" and len(parts) >= 3:
            try:
                depth = float(parts[1])
                self.soft_vars.nmea_data["SDDBT"]["depth"] = depth
                self.soft_vars.nmea_data["SDDBT"]["units"] = parts[2] if len(parts) > 2 else "m"
            except ValueError:
                pass
        else:
            # Handle other NMEA sentences as needed
            print(f"Unhandled NMEA sentence: {sentence}")




    def closeEvent(self, event):
        """Ensure serial port and MAVLink thread are closed on exit."""
        self.stop_mavlink_thread()
        event.accept()


    def init_menu_bar(self):
        menu_bar = QMenuBar(self)
        self.setMenuBar(menu_bar)

        # File menu
        file_menu = menu_bar.addMenu("File")

        new_action = QAction("New Project", self)
        new_action.triggered.connect(self.file_new_function)
        file_menu.addAction(new_action)
        open_action = QAction("Open Project", self)
        open_action.triggered.connect(self.file_open_function)
        file_menu.addAction(open_action)
        save_action = QAction("Save Project", self)
        save_action.triggered.connect(self.file_save_function)
        file_menu.addAction(save_action)
        save_as_action = QAction("Save Project As", self)
        save_as_action.triggered.connect(self.file_save_as_function)
        file_menu.addAction(save_as_action)
        export_action = QAction("Export Project", self)
        export_action.triggered.connect(self.file_export_function)
        file_menu.addAction(export_action)
        import_action = QAction("Import Project", self)
        import_action.triggered.connect(self.file_import_function)
        file_menu.addAction(import_action)

                
        exit_action = QAction("Close", self)
        exit_action.triggered.connect(self.close)
        file_menu.addAction(exit_action)

        # Connection menu
        connection_menu = menu_bar.addMenu("Connection")
        
        connect_mav = QAction("Connect Mavlink", self)
        connect_mav.triggered.connect(self.conn_connectMavlink_function)
        connection_menu.addAction(connect_mav)
        connect_nmea = QAction("Connect NMEA", self)
        connect_nmea.triggered.connect(self.conn_connectNMEA_function)
        connection_menu.addAction(connect_nmea)


        # Settings menu
        settings_menu = menu_bar.addMenu("Settings")
        # Add actions as needed

        # Window menu
        window_menu = menu_bar.addMenu("Window")
        # Add actions as needed

        # Help menu (optional)
        help_menu = menu_bar.addMenu("Help")
        meme_action = QAction("Help", self)
        meme_action.triggered.connect(lambda: QMessageBox.information(self, "Help", "Ask god for help. 👍 "))
        help_menu.addAction(meme_action)
        # Add actions as needed

    def update_ui(self):
        self.ASV_gps_datetime_label.setText(f"Date Time: {self.soft_vars.ASV_gps['date_time']}")
        self.ASV_gps_hdop_label.setText(f"HDOP: {self.soft_vars.ASV_gps['hdop']:.2f}")
        self.ASV_gps_vdop_label.setText(f"VDOP: {self.soft_vars.ASV_gps['vdop']:.2f}")
        self.ASV_gps_satellites_label.setText(f"Satellites: {self.soft_vars.ASV_gps['satellites_visible']}")
        self.ASV_gps_altitude_label.setText(f"Altitude: {self.soft_vars.ASV_gps['altitude']:.2f} m")
        self.ASV_gps_lat_label.setText(f"Latitude: {self.soft_vars.ASV_gps['latitude']:.6f}")
        self.ASV_gps_lon_label.setText(f"Longitude: {self.soft_vars.ASV_gps['longitude']:.6f}")
        self.ASV_gps_easting_label.setText(f"Easting: {self.soft_vars.ASV_gps['easting']:.2f} m")
        self.ASV_gps_northing_label.setText(f"Northing: {self.soft_vars.ASV_gps['northing']:.2f} m")
        self.ASV_gps_zone_label.setText(f"Zone: {self.soft_vars.ASV_gps['zone']}")
        self.ASV_attitude_roll_label.setText(f"Roll: {self.soft_vars.ASV_attitude['roll']:.2f} deg")
        self.ASV_attitude_pitch_label.setText(f"Pitch: {self.soft_vars.ASV_attitude['pitch']:.2f} deg")
        self.ASV_attitude_yaw_label.setText(f"Yaw: {self.soft_vars.ASV_attitude['yaw']:.2f} rad")
        self.ASV_sonar_ping_label.setText(f"Ping Sonar: {self.soft_vars.ASV_pingsonar:.2f} m")
        self.ASV_water_quality_temp_label.setText(f"Temperature: {self.soft_vars.WaterQuality['temperature']:.2f} °C")
        self.ASV_water_quality_ph_label.setText(f"PH: {self.soft_vars.WaterQuality['ph']:.2f}")
        self.ASV_water_quality_turbidity_label.setText(f"Turbidity: {self.soft_vars.WaterQuality['turbidity']:.2f} NTU")
        self.ASV_water_quality_dissolved_oxygen_label.setText(f"Dissolved Oxygen: {self.soft_vars.WaterQuality['dissolved_oxygen']:.2f} mg/L")
        self.ASV_water_quality_conductivity_label.setText(f"Conductivity: {self.soft_vars.WaterQuality['conductivity']:.2f} µS/cm")
        self.ASV_water_quality_salinity_label.setText(f"Salinity: {self.soft_vars.WaterQuality['salinity']:.2f} ppt")
        self.ASV_system_time_unix_label.setText(f"UNIX Time: {self.soft_vars.ASV_system_time['time_unix_usec']}")
        self.ASV_system_time_boot_label.setText(f"Boot Time: {self.soft_vars.ASV_system_time['time_boot_ms']} ms")
        self.ASV_system_time_machine_label.setText(f"Machine Time: {self.soft_vars.ASV_system_time['time_boot_ms'] / 1000:.2f} s")
        self.ASV_battery_voltage_label.setText(f"Voltage: {self.soft_vars.ASV_battery_status['voltage']:.2f} V")
        self.ASV_battery_current_label.setText(f"Current: {self.soft_vars.ASV_battery_status['current']:.2f} A")
        self.ASV_battery_remaining_label.setText(f"Remaining: {self.soft_vars.ASV_battery_status['remaining']:.2f} %")
        self.ASV_battery_temperature_label.setText(f"Temperature: {self.soft_vars.ASV_battery_status['temperature']:.2f} °C")
        self.depth_label.setText(f"NMEA Depth: {self.soft_vars.nmea_data['SDDPT']['depth']:.2f} m")
        
    def update_MAP(self):
        #show a boat icon on the map at the current GPS position
        if self.soft_vars.ASV_gps["latitude"] != 0.0 and self.soft_vars.ASV_gps["longitude"] != 0.0:
            lat = self.soft_vars.ASV_gps["latitude"]
            lon = self.soft_vars.ASV_gps["longitude"]
            easting = self.soft_vars.ASV_gps["easting"]
            northing = self.soft_vars.ASV_gps["northing"]
            zone = self.soft_vars.ASV_gps["zone"]
            head = self.soft_vars.ASV_attitude["yaw"]

            # Update the map with the new position
            script = f"""
                updateMap({lat}, {lon}, {easting}, {northing}, '{zone}',{head});
            """
            self.browser.page().runJavaScript(script)
            self.update_ui()
        else:
            #print("GPS data not available yet. Waiting for valid GPS position...")
            pass

    def record_all_points(self):
        # Prompt user for survey location name
        location_name, ok = QInputDialog.getText(self, "New Survey", "Enter survey location name:")
        if not ok or not location_name.strip():
            QMessageBox.warning(self, "Input Error", "Survey location name cannot be empty.")
            return
        
        # Prompt user for record data rate (Hz)
        data_rate, ok = QInputDialog.getInt(
            self, "Record Data Rate", "Enter record data rate (Hz):", value=1, min=1, max=10, step=1
        )
        if not ok:
            return
        
        

        # Choose directory to save logs
        base_dir = QFileDialog.getExistingDirectory(self, "Select Directory to Save Survey Logs")
        if not base_dir:
            return

        self.projectFolder.setText(f"Project Name: {location_name}")
        self.open_folder_btn.setDisabled(False)

        # Create folder for the new survey
        survey_folder = os.path.join(base_dir, location_name.strip())
        os.makedirs(survey_folder, exist_ok=True)

        # Example: Save a log file in the survey folder
        log_filename = f"log_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.txt"
        log_path = os.path.join(survey_folder, log_filename)
        with open(log_path, "w") as log_file:
            log_file.write("Survey log started...\n")
            # Write actual log data here

        self.soft_vars.soft_logrecord_path = log_path
        self.soft_vars.soft_logrecord_rate = data_rate
        self.soft_vars.soft_logrecord_flag = True  # Set logging flag to True

        self.recordTimer = QTimer(self)
        self.recordTimer.timeout.connect(self.runRecord)
        timer_record = int(1000 / self.soft_vars.soft_logrecord_rate)  # Convert Hz to milliseconds and ensure int
        self.recordTimer.start(timer_record)


        QMessageBox.information(self, "Survey Log", f"Recording started.")

    def recordLog(self, log_path, data):
        """Record data to a log file."""
        with open(log_path, "a") as log_file:
            log_file.write(data + "\n")
        self.log_text_edit.append(data)


        

    def runRecord(self):
        """Run the record function to log data."""
        if self.soft_vars.soft_logrecord_flag:
            data = f"Timestamp: {self.soft_vars.ASV_gps['date_time']}, "
            data += f"GPS: {self.soft_vars.ASV_gps['latitude']}, {self.soft_vars.ASV_gps['longitude']}, "
            data += f"Ping Sonar: {self.soft_vars.ASV_pingsonar}, "
            data += f"Attitude: {self.soft_vars.ASV_attitude['roll']}, {self.soft_vars.ASV_attitude['pitch']}, {self.soft_vars.ASV_attitude['yaw']}"

        self.recordLog(self.soft_vars.soft_logrecord_path, data)

        # add a marker to the map for the current position
        if self.soft_vars.ASV_gps["latitude"] != 0.0 and self.soft_vars.ASV_gps["longitude"] != 0.0:
            lat = self.soft_vars.ASV_gps["latitude"]
            lon = self.soft_vars.ASV_gps["longitude"]
            depth = self.soft_vars.ASV_pingsonar or self.soft_vars.nmea_data["SDDPT"]["depth"]
            timestamp = self.soft_vars.ASV_gps['date_time']

            # Escape the timestamp correctly
            script = f"""
                addRecordMarker({lat}, {lon}, {depth}, "{timestamp}");
            """
            self.browser.page().runJavaScript(script)

        else:
            print("GPS data not available yet. Waiting for valid GPS position...")

    def StropRecording(self):
        """Stop the recording and reset the flag."""
        if self.soft_vars.soft_logrecord_flag:
            self.soft_vars.soft_logrecord_flag = False
            self.recordTimer.stop()
            QMessageBox.information(self, "Recording Stopped", "Data recording has been stopped.")
        else:
            QMessageBox.warning(self, "No Recording", "No data is currently being recorded.")


    def file_new_function(self):
        """Function to handle 'New Project' action."""
        print("New Project action triggered")

    def file_open_function(self):
        """Function to handle 'Open Project' action."""
        print("Open Project action triggered")

    def file_save_function(self):
        """Function to handle 'Save Project' action."""
        print("Save Project action triggered")

    def file_save_as_function(self):
        """Function to handle 'Save Project As' action."""
        print("Save Project As action triggered")

    def file_export_function(self):
        """Function to handle 'Export Project' action."""
        print("Export Project action triggered")

    def file_import_function(self):
        """Function to handle 'Import Project' action."""
        print("Import Project action triggered")    

    def conn_connectMavlink_function(self):
        from PyQt5.QtWidgets import QDialog, QVBoxLayout, QHBoxLayout, QLabel, QLineEdit, QPushButton, QMessageBox

        
        class ConnectDialog(QDialog):
            def __init__(self, parent=None):
                super().__init__(parent)
                self.setWindowTitle("Connect to MAVLink")
                self.setModal(True)
                layout = QVBoxLayout(self)

                # IP input
                ip_layout = QHBoxLayout()
                ip_layout.addWidget(QLabel("IP:"))
                self.ip_edit = QLineEdit("127.0.0.1")
                ip_layout.addWidget(self.ip_edit)
                layout.addLayout(ip_layout)

                # Port input
                port_layout = QHBoxLayout()
                port_layout.addWidget(QLabel("Port:"))
                self.port_edit = QLineEdit("14555")
                port_layout.addWidget(self.port_edit)
                layout.addLayout(port_layout)

                # Reconnect button
                self.reconnect_btn = QPushButton("Reconnect")
                self.reconnect_btn.clicked.connect(self.accept)
                layout.addWidget(self.reconnect_btn)

        # Show dialog
        dialog = ConnectDialog(self)
        if dialog.exec_() == QDialog.Accepted:
            ip = dialog.ip_edit.text().strip()
            port = dialog.port_edit.text().strip()
            if not ip or not port.isdigit():
                QMessageBox.warning(self, "Input Error", "Please enter a valid IP and port.")
                return
            port = int(port)
            # Restart MAVLink thread with new IP and port
            if hasattr(self, 'mavlink_thread') and self.mavlink_thread.is_alive():
                self.stop_mavlink_thread()
                print("Stopping existing MAVLink thread...")
            self.start_mavlink_thread(udp_ip=ip, udp_port=port)
    
    def conn_connectNMEA_function(self):
        from PyQt5.QtWidgets import QDialog, QVBoxLayout, QHBoxLayout, QLabel, QLineEdit, QPushButton, QComboBox, QMessageBox

        class ConnectDialog(QDialog):
            def __init__(self, parent=None):
                super().__init__(parent)
                self.setWindowTitle("Connect to NMEA")
                self.setModal(True)
                layout = QVBoxLayout(self)

                # Connection type selection
                type_layout = QHBoxLayout()
                type_layout.addWidget(QLabel("Connection Type:"))
                self.type_combo = QComboBox()
                self.type_combo.addItems(["UDP", "Serial"])
                type_layout.addWidget(self.type_combo)
                layout.addLayout(type_layout)

                # UDP inputs
                self.udp_widget = QWidget()
                udp_layout = QHBoxLayout(self.udp_widget)
                udp_layout.addWidget(QLabel("IP:"))
                self.ip_edit = QLineEdit("127.0.0.1")
                udp_layout.addWidget(self.ip_edit)
                udp_layout.addWidget(QLabel("Port:"))
                self.port_edit = QLineEdit("10110")
                udp_layout.addWidget(self.port_edit)
                layout.addWidget(self.udp_widget)

                # Serial inputs
                self.serial_widget = QWidget()
                serial_layout = QHBoxLayout(self.serial_widget)
                serial_layout.addWidget(QLabel("Port:"))
                self.serial_port_edit = QLineEdit("COM4")
                serial_layout.addWidget(self.serial_port_edit)
                serial_layout.addWidget(QLabel("Baudrate:"))
                self.baudrate_edit = QLineEdit("4800")
                serial_layout.addWidget(self.baudrate_edit)
                layout.addWidget(self.serial_widget)
                self.serial_widget.hide()

                # Change input fields based on connection type
                self.type_combo.currentTextChanged.connect(self.toggle_conn_type)

                # Connect button
                self.connect_btn = QPushButton("Connect")
                self.connect_btn.clicked.connect(self.accept)
                layout.addWidget(self.connect_btn)

            def toggle_conn_type(self, text):
                if text == "UDP":
                    self.udp_widget.show()
                    self.serial_widget.hide()
                else:
                    self.udp_widget.hide()
                    self.serial_widget.show()

        # Show dialog
        dialog = ConnectDialog(self)
        if dialog.exec_() == QDialog.Accepted:
            conn_type = dialog.type_combo.currentText()
            # stop any existing NMEA thread before starting a new one
            if hasattr(self, 'nmea_thread') and self.nmea_thread.is_alive():
                self.nmea_running = False
                self.nmea_thread.join()
                
            if conn_type == "UDP":
                ip = dialog.ip_edit.text().strip()
                port = dialog.port_edit.text().strip()
                if not ip or not port.isdigit():
                    QMessageBox.warning(self, "Input Error", "Please enter a valid IP and port.")
                    return
                port = int(port)
                # Start your NMEA UDP reading thread here
                self.start_nmea_thread(udp_ip=ip, udp_port=port)
            else:
                serial_port = dialog.serial_port_edit.text().strip()
                baudrate = dialog.baudrate_edit.text().strip()
                if not serial_port or not baudrate.isdigit():
                    QMessageBox.warning(self, "Input Error", "Please enter a valid serial port and baudrate.")
                    return
                baudrate = int(baudrate)
                # Start your NMEA serial reading thread here
                self.start_nmea_thread(serial_port=serial_port, baudrate=baudrate)

    

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MapWindow()
    window.show()
    sys.exit(app.exec_())
