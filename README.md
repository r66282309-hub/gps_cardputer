# M5Cardputer GPS Tracker

A standalone GPS tracking firmware for the **M5Cardputer**, based on [AtomDream](https://github.com/AtomDreams/CardputerSMAGPS)'s original work and extended with support for:

- Waypoint saving
- Screen toggling
- Basic (unstable) map view

---

## 📦 Features

- Real-time display of:
  - Latitude & Longitude
  - Altitude
  - Speed (km/h)
  - UTC Date & Time
  - Satellite count
  - HDOP & GPS Fix Quality
  - Course over ground
- Save GPX track to SD card
- Save waypoints to `waypoints.txt`
- Scrollable map view (EXPERIMENTAL — may crash!)
- Toggle screen on/off to save power
- Battery level indicator
- Minimal, readable UI on 240x135 TFT screen

---

## 🔘 Controls

| Key | Action |
|-----|--------|
| `R` | Start/stop GPX track recording |
| `S` | Save current location as waypoint |
| `D` | Turn display on/off |
| `M` | Open map view *(unstable, may crash)* |
| `N` | Open waypoint list *(unstable, may crash)* |
| `Enter` | Return to main data screen (unstable, may crash)|
| Arrow keys | Move map view (in MAP mode) |

Waypoints are stored as:  
/waypoints.txt → NAME,LAT,LON

GPX tracks are saved as:  
/track_YYYY-MM-DD_[timestamp].gpx

## 💾 Requirements

- M5Cardputer
- GPS module (e.g., GPS V1.1 AT6668 + MAX2659), UART connected to GPIO 1 (RX) and 2 (TX)
- microSD card (FAT32)

✅ No need to flash via USB if using launcher.

(/images/gps_2.jpg)

