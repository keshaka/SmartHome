# Human Presence Occupancy Detector

A modular Python application that utilizes a webcam and YOLOv8 object detection to track room occupancy. It publishes presence events to an MQTT broker, enabling touchless automation of appliances, lighting, and general smart home routines.

This application is optimized for Windows and features frame-rate limiting to control CPU utilization, camera disconnect recovery, custom zone filtering, and background MQTT client management.

---

## System Architecture

```
                      ┌──────────────────────────────┐
                      │          USB Webcam          │
                      └──────────────┬───────────────┘
                                     │ Video Feed
                      ┌──────────────▼───────────────┐
                      │          Local PC            │
                      │   (YOLOv8 / OpenCV / Python) │
                      └──────────────┬───────────────┘
                                     │ Publish: home/fancontroller/occupancy/state = ON/OFF
                      ┌──────────────▼───────────────┐
                      │      MQTT Message Broker     │
                      └──────────────┬───────────────┘
                                     │
                      ┌──────────────▼───────────────┐
                      │        Home Assistant        │
                      │    (or Automation Engine)    │
                      └──────────────┬───────────────┘
                                     │ Command: home/fancontroller/relay/set = ON/OFF
                      ┌──────────────▼───────────────┐
                      │      ESP8266 Controller      │
                      └──────────────────────────────┘
```

### Why Decouple Occupancy and Appliance States?
Instead of sending direct commands to target appliance topics (like a relay switch), the occupancy detector publishes presence data to `home/fancontroller/occupancy/state`. This maintains separation of concerns, allowing automation engines like Home Assistant to route the occupancy state to multiple targets (e.g., turning on lighting, setting climate control, triggering security rules, or collecting analytics) without modification to the tracking script.

---

## Features

- **YOLOv8 Inference**: Real-time identification of human presence, ignoring pets, vacuums, or other moving objects.
- **Configurable Detection Zone**: Define coordinates for a specific active region. Bounding boxes are tracked for everyone in the frame, but presence is only triggered when a person enters the active zone.
- **Debounced Occupancy State Machine**:
  - **Turn ON delay**: A person must stay inside the active zone continuously for 5 seconds before occupancy state transitions to `ON`.
  - **Turn OFF delay**: The zone must remain completely empty for 120 seconds before transitioning to `OFF`, preventing false shutdowns when someone temporarily steps away.
- **Resilient MQTT Handler**: Multi-threaded publisher wrapper with automatic background reconnection.
- **Camera Disconnect Recovery**: Gracefully checks for camera reconnection every 5 seconds if unplugged, displaying warning notifications on the screen.
- **Telemetry HUD**: Displays real-time metrics including loop frame rate (FPS), occupancy timers, connection states, and MQTT transmission statuses.
- **First Appearance Snapshot**: Saves an annotated JPEG to the `captures/` directory automatically upon initial person entry into the zone.

---

## Installation

### Prerequisites
- **Python 3.11+** installed and added to the system `PATH`.
- A USB Webcam connected to your PC.
- An internet connection on first run to download the YOLOv8 model weights.

### 1. File Structure
Ensure the module directory contains:
```
HumanDetection/
├── config.py           # Configuration parameters
├── mqtt_client.py     # Background MQTT handler
├── detector.py         # YOLOv8 object detector wrapper
├── main.py             # Main entry point script
├── requirements.txt    # Project dependencies
└── README.md           # Documentation
```

### 2. Virtual Environment and Dependency Setup
Open PowerShell or Command Prompt in the `HumanDetection` directory:

```powershell
# Create a virtual environment
python -m venv venv

# Activate the virtual environment
.\venv\Scripts\activate

# Install the required packages
pip install -r requirements.txt

# To run YOLOv8 on an NVIDIA GPU (CUDA), override the standard PyTorch library:
pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu121 --force-reinstall
```

---

## Configuration

Open [config.py](file:///g:/SmartHome/HumanDetection/config.py) to customize parameters:

| Config Key | Description | Default Value |
|:---|:---|:---|
| `MQTT_BROKER` | Host address of Mosquitto broker | `"45.32.110.119"` |
| `MQTT_PORT` | Port of the broker | `1883` |
| `MQTT_USERNAME` | Username for MQTT authentication | `"esp8266"` |
| `MQTT_PASSWORD` | Password for MQTT authentication | `"********"` |
| `MQTT_TOPIC` | State topic for occupancy updates | `"home/fancontroller/occupancy/state"` |
| `CONFIDENCE_THRESHOLD` | Confidence threshold for YOLO detection | `0.6` |
| `DETECTION_ZONE` | Active viewport boundary `[x_min, y_min, x_max, y_max]` (0.0 to 1.0) | `[0.1, 0.1, 0.9, 0.9]` |
| `DETECTION_ON_DELAY` | Seconds of continuous presence required to trigger ON state | `5.0` |
| `DETECTION_OFF_DELAY` | Seconds of continuous absence required to trigger OFF state | `120.0` |
| `CAMERA_INDEX` | Camera hardware index | `0` |
| `FPS_LIMIT` | Hard cap on loop speed (reduces CPU usage) | `20` |

---

## Running the Application

Make sure your virtual environment is active, then run:

```powershell
python main.py
```

### Keyboard Controls
With the camera stream window focused:
- **`ESC`**: Disconnect MQTT, release camera feed, and quit.
- **`R`**: Reset active presence and absence state machine timers.
- **`M`**: Toggle MQTT transmission on/off (useful for testing zone configurations without triggering actual automation rules).

---

## Home Assistant Automation Example

To bridge the occupancy state to the ESP8266 appliance controller, create a simple automation in Home Assistant (`automations.yaml`):

```yaml
- id: 'smart_home_presence_relay_control'
  alias: 'Presence Appliance Automation'
  trigger:
    - platform: mqtt
      topic: 'home/fancontroller/occupancy/state'
  action:
    - service: mqtt.publish
      data:
        topic: 'home/fancontroller/relay/set'
        payload: "{{ trigger.payload }}"
```

---

## Troubleshooting

### Camera Not Found / Black Screen
- Verify that your camera index is correct in `config.py` (e.g. change index to `1` or `2` if multiple video input devices are attached).
- Ensure no other applications (e.g., Teams, Zoom, or web browsers) are currently accessing the webcam.

### High CPU Usage
- Reduce `FPS_LIMIT` to `10` or `15` in `config.py`. Because room occupancy changes slowly, lower update rates are typically sufficient for automations while significantly lowering CPU overhead.

### MQTT Connection Fails
- Verify network connectivity to the broker.
- Check that port `1883` is allowed through local firewall policies.
- Confirm broker authentication details inside your environment or `config.py`.
