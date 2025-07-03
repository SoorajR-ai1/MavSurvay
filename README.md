# MavSurvay - ASV Telemetry and Survey Logger

**MavSurvay** is a comprehensive GUI application built with PyQt5 for real-time telemetry monitoring, data logging, and mission management of Autonomous Surface Vehicles (ASVs). It integrates MAVLink telemetry, NMEA depth sonar input, water quality sensor data, and survey waypoint control with a live map interface.

---

## 🚀 Features

- **Live MAVLink Telemetry**
  - GPS position (lat/lon, UTM, altitude, satellites)
  - Attitude (roll, pitch, yaw)
  - Battery voltage, current, remaining
  - System uptime

- **NMEA Depth Sonar Integration**
  - Compatible with ISA500 and NMEA sentences (`$SDDPT`, `$SDDBT`)
  - Serial and UDP support

- **Water Quality Monitoring**
  - Displays live values of temperature, pH, turbidity, dissolved oxygen, conductivity, and salinity

- **Interactive GUI with PyQt5**
  - Left Dock: Mission config, waypoint editor, start/stop logging
  - Right Dock: Telemetry data panel
  - Bottom Dock: Data log viewer
  - Center View: Live map interface (HTML via QWebEngine)

- **Survey Logging**
  - User-configurable logging interval (Hz)
  - Records to timestamped log files
  - Automatic marker placement on the map

- **Waypoint Processor**
  - Parses ArduPilot `.waypoints` file
  - Inserts DO_SET_SERVO and DO_DELAY actions after each waypoint

- **Connection Interface**
  - MAVLink over UDP
  - NMEA via Serial or UDP

