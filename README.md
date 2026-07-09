# SmartHome

A modular smart home system built around an ESP8266 multi-purpose controller with support for appliance switching, addressable LED lighting, PC Wake on LAN, and optional AI-based room occupancy detection. Designed for seamless integration with Home Assistant via MQTT auto-discovery.

---

## Repository Structure

| Directory | Description |
|---|---|
| `sketch_jul8a/` | ESP8266 firmware — relay, LED strip, WOL, web portal, MQTT |
| `HumanDetection/` | Python occupancy detector using YOLOv8 and a webcam |

---

## System Architecture

```
                    ┌──────────────────────────────┐
                    │        Home Assistant         │
                    │     (Central Control Hub)     │
                    └──────┬───────────────┬────────┘
                           │ MQTT          │ MQTT
             ┌─────────────▼────────┐  ┌──▼─────────────────────┐
             │  ESP8266 Controller  │  │  Occupancy Detector     │
             │                      │  │  (optional)             │
             │  · Relay switch      │  │  · YOLOv8 on webcam     │
             │  · LED strip         │  │  · Publishes room state  │
             │  · Wake on LAN       │  └─────────────────────────┘
             └─────────┬────────────┘
                       │ UDP Broadcast
             ┌─────────▼────────────┐
             │      Target PC       │
             └──────────────────────┘
```

---

## ESP8266 Controller

### Relay Control

Controls any mains-powered appliance (fan, light, socket, etc.) through an Active-Low relay connected to GPIO D2. Relay state is persisted across MQTT and the web dashboard, and published to Home Assistant on connect.

---

### Addressable LED Strip

Full-featured LED strip driver using FastLED, supporting WS2811 and WS2812B strips up to 300 pixels on GPIO D7.

**Effects (9 total)**

| Effect | Description |
|---|---|
| Solid | Static color fill |
| Rainbow | Rotating hue cycle across the strip |
| Breathing | Sinusoidal brightness pulse in the selected color |
| Color Wipe | Sequential fill and clear from one end |
| Theater Chase | Classic 1-in-3 chasing pattern |
| Fire | Randomized heat simulation |
| Police | Alternating red and blue flash split across the strip |
| Ocean | Scrolling ocean color palette |
| Christmas | Alternating red and green |

**Controls**

- RGB color picker with per-channel (0–255) control
- Brightness slider (0–255) with smooth hardware transition
- Effect speed slider (0–255)
- All runtime state (on/off, color, effect, speed) is auto-saved to EEPROM 5 seconds after the last change
- Configurable startup defaults saved separately in settings

---

### PC Wake on LAN

Allows waking a sleeping or powered-off PC on the same local network by broadcasting a standard UDP Magic Packet.

- Store the target PC's MAC address in the Settings page (EEPROM-persisted)
- Accepts standard colon-separated or dash-separated MAC notation (e.g., `AA:BB:CC:DD:EE:FF`)
- Broadcasts to `255.255.255.255` on UDP port 9
- Accessible from the web dashboard with live button feedback (`Sending...` / `Sent!` / `Failed`)
- Exposes a **Wake PC** button entity in Home Assistant via MQTT discovery, controllable from automations and dashboards

---

### WiFi Disconnection Alert

Provides a visual indicator when the device loses its network connection after a successful boot.

- Monitors WiFi status continuously in the main loop
- On disconnect (post-boot only, excluding initial connection attempts): turns the LED strip to **Solid Red at 50% brightness**
- Automatically turns off if WiFi reconnects at any point during the 5-minute window
- Automatically turns off after **5 minutes** regardless of reconnection status

---

### Web Portal

The device hosts a full web interface accessible at its local IP or `http://fancontroller.local` (mDNS).

**Dashboard**
- Toggle relay on/off
- Full LED strip controls (state, brightness, effect, speed, color)
- PC Wake on LAN card showing configured MAC and wake button
- Live diagnostics: WiFi RSSI, free heap, local IP, MQTT connection, uptime, firmware version
- Real-time updates via Server-Sent Events (SSE) with 1-second polling fallback

**Settings**
- WiFi SSID and password
- MQTT broker address, port, username, and password
- MQTT topic prefix
- OTA update password
- Device name and room name (used in Home Assistant)
- LED strip defaults (enabled, count, brightness, effect, speed, color)
- PC MAC Address for Wake on LAN

**AP Setup Mode**

If no valid configuration is stored in EEPROM, the device starts as a Wi-Fi Access Point (`ESP8266-XXXX`, password `12345678`). Connecting to the AP and navigating to `http://192.168.4.1` opens a first-time configuration form.

---

### Home Assistant MQTT Auto-Discovery

On every MQTT connection, the device publishes discovery payloads for the following entities, all grouped under a single HA device:

| Entity Type | Name | Notes |
|---|---|---|
| Switch | Fan | ON/OFF relay control |
| Light | LED | RGB, brightness, effect, speed via JSON schema |
| Button | Wake PC | Triggers UDP Magic Packet broadcast |
| Sensor | WiFi Signal | dBm, `signal_strength` device class |
| Sensor | IP Address | Current local IP |
| Sensor | Free Heap | Available RAM in bytes |
| Sensor | Firmware | Firmware version string |
| Sensor | LED Effect | Current active effect name |
| Sensor | LED Count | Configured pixel count |

Availability is tracked via a Last Will and Testament (LWT) topic, so Home Assistant will mark entities as unavailable if the device goes offline.

---

### OTA Updates & mDNS

- ArduinoOTA support for wireless firmware flashing from the Arduino IDE
- OTA password configurable in settings
- mDNS hostname: `fancontroller.local`

---

### Hardware Factory Reset

Hold the BOOT/FLASH button (GPIO0) for 10 seconds to wipe all EEPROM configuration and restart into AP setup mode.

---

## Optional: Room Occupancy Detector

A Python module that uses a webcam and YOLOv8 to detect whether a person is present in a room, publishing the state over MQTT so Home Assistant (or any subscriber) can act on it.

See [HumanDetection/README.md](HumanDetection/README.md) for full documentation, configuration, and installation instructions.

**Key features:**
- YOLOv8 real-time person detection (GPU-accelerated when available)
- Configurable detection zone using normalized coordinates — occupancy only triggers when a person's center is within the defined region
- Debounced state machine: 5-second ON delay, 120-second OFF delay to prevent false triggers
- Publishes to a dedicated occupancy topic decoupled from the relay topic, so Home Assistant automations control the logic
- Automatic camera disconnect recovery
- Annotated HUD display with live metrics (FPS, occupancy timers, MQTT status)
- Saves annotated screenshots on first person detection

---

## Configuration Reference

### EEPROM Layout (Config Struct)

| Field | Type | Description |
|---|---|---|
| `magic` | `uint8_t` | Validity marker (`0xA5`) |
| `configVersion` | `uint16_t` | Schema version (current: 3) |
| `wifiSSID` | `char[33]` | WiFi network name |
| `wifiPass` | `char[65]` | WiFi password |
| `mqttServer` | `char[65]` | MQTT broker host |
| `mqttPort` | `uint16_t` | MQTT broker port (default: 1883) |
| `mqttUser` | `char[33]` | MQTT username |
| `mqttPass` | `char[65]` | MQTT password |
| `mqttPrefix` | `char[33]` | MQTT topic prefix |
| `otaPass` | `char[33]` | OTA update password |
| `deviceName` | `char[33]` | Device label (shown in HA) |
| `roomName` | `char[33]` | Room label (used in HA area) |
| `ledEnabled` | `bool` | Enable LED subsystem |
| `ledCount` | `uint16_t` | Number of LEDs on the strip |
| `ledDefaultBrightness` | `uint8_t` | Boot brightness (0–255) |
| `ledDefaultEffect` | `uint8_t` | Boot effect index |
| `ledDefaultR/G/B` | `uint8_t` | Boot color |
| `ledDefaultSpeed` | `uint8_t` | Boot effect speed |
| `ledState` | `bool` | Last runtime on/off state |
| `ledBrightness` | `uint8_t` | Last runtime brightness |
| `ledEffect` | `uint8_t` | Last runtime effect |
| `ledR/G/B` | `uint8_t` | Last runtime color |
| `ledSpeed` | `uint8_t` | Last runtime speed |
| `pcMac` | `char[18]` | Target PC MAC for Wake on LAN |

### MQTT Topics

All topics use the configured `mqttPrefix` (default: `home/fancontroller`).

| Topic | Direction | Description |
|---|---|---|
| `<prefix>/relay/set` | Subscribe | `ON` or `OFF` — controls the relay |
| `<prefix>/relay/state` | Publish | Current relay state |
| `<prefix>/light/set` | Subscribe | JSON payload — controls LED strip |
| `<prefix>/light/state` | Publish | Current LED state as JSON |
| `<prefix>/pc/wake` | Subscribe | `WAKE` — triggers Magic Packet |
| `<prefix>/rssi` | Publish | WiFi signal strength (dBm) |
| `<prefix>/ip` | Publish | Local IP address |
| `<prefix>/heap` | Publish | Free heap memory (bytes) |
| `<prefix>/firmware` | Publish | Firmware version string |
| `<prefix>/led_effect` | Publish | Current effect name |
| `<prefix>/led_count` | Publish | Configured pixel count |
| `<prefix>/availability` | Publish | `Online` / `Offline` (LWT) |

### Hardware Pin Mapping

| Pin | GPIO | Function |
|---|---|---|
| D2 | GPIO4 | Relay (Active Low) |
| D7 | GPIO13 | LED strip data |
| D3 | GPIO0 | BOOT button (factory reset on 10s hold) |
