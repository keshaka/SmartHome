# Human Detection Fan Automation (YOLOv8 + MQTT)

A modular, production-ready Python application that uses a webcam and YOLOv8 object detection to track room occupancy and control smart appliances (such as a ceiling fan) via MQTT.

This application is optimized for **Windows** and features low CPU utilization (configurable frame rate), camera disconnect recovery, a detection zone overlay, and startup safety guards.

---

## System Architecture

```
                       +-----------------------------+
                       |         USB Webcam          |
                       +--------------+--------------+
                                      |
                                      v (Video Feed)
                       +--------------+--------------+
                       |          Windows PC         |
                       |    (YOLOv8 / OpenCV / Python)|
                       +--------------+--------------+
                                      |
                                      v (Publish: home/fancontroller/occupancy/state = ON/OFF)
                       +--------------+--------------+
                       |    Mosquitto MQTT Broker    |
                       |        (Ubuntu VPS)         |
                       +--------------+--------------+
                                      |
                                      v (Trigger Action)
                       +--------------+--------------+
                       |        Home Assistant       |
                       |   (Automation Rules Engine) |
                       +--------------+--------------+
                                      |
                                      v (Command: home/fancontroller/relay/set = ON/OFF)
                       +--------------+--------------+
                       |      ESP8266 Smart Fan      |
                       |      (Firmware & Relay)     |
                       +-----------------------------+
```

### Why Decouple Occupancy and Relay?
Instead of sending MQTT commands directly to the relay topic, the Python app publishes room occupancy (`ON` or `OFF`) to a dedicated occupancy topic: `home/fancontroller/occupancy/state`. 
This allows you to add future automations (e.g., turning on smart lights, playing ambient music, recording room occupancy analytics, sending Telegram alerts) in Home Assistant without altering this Python program.

---

## Features

- **YOLOv8 Inference**: Detects only `person` objects, filtering out all other objects.
- **Configurable Detection Zone**: Define a region of interest (in normalized coordinates). Bounding boxes are drawn for everyone, but occupancy is only triggered when a person's center point is inside the active zone.
- **Occupancy Timing Rules**:
  - **Occupancy ON**: A person must be inside the zone continuously for **5 seconds** before state changes.
  - **Occupancy OFF**: The zone must be empty continuously for **120 seconds** before state changes.
- **Resilient MQTT Handler**: Automatic background reconnection with publish confirmation latency tracking.
- **Robust Camera Recovery**: Retries connection every 5 seconds if the camera disconnects, keeping the HUD window open and responsive with a warning indicator.
- **Advanced HUD Dashboard**: Displays real-time metrics, connection statuses, people counts, timers, average FPS, and network latency.
- **Startup Safety**: The program initializes in the `OFF` state but does not publish to MQTT immediately, avoiding turning off a running fan on start.
- **Auto Screenshot Capture**: Automatically takes an annotated snapshot and saves it in the `captures/` folder when a person first appears inside the zone.

---

## Installation

### Prerequisites
- **Python 3.11+** installed and added to your system `PATH`.
- A USB Webcam connected to your Windows PC.
- Active internet connection (on first run) to automatically download the YOLOv8 model.

### 1. Clone or Copy the Files
Ensure you have the following file structure:
```
HumanDetection/
│
├── config.py           # Configuration parameters
├── mqtt_client.py     # Resilient MQTT client wrapper
├── detector.py         # YOLOv8 object detector wrapper
├── main.py             # Main application entry point
├── requirements.txt    # Python dependencies
└── README.md           # Documentation
```

### 2. Set Up a Virtual Environment & Install Dependencies
Open **PowerShell** or **Command Prompt** in the `HumanDetection` directory:

```powershell
# Create a virtual environment
python -m venv venv

# Activate the virtual environment
# On Windows:
.\venv\Scripts\activate

# Install the required packages
pip install -r requirements.txt

# IMPORTANT: Enabling GPU (CUDA) support on Windows
# Standard pip installs download the CPU-only version of PyTorch by default.
# To run YOLOv8 on your NVIDIA GPU, you must override it with the CUDA-compiled PyTorch wheel.
# Choose the command that matches your driver (CUDA 12.1 is recommended for newer cards):

# For CUDA 12.1 support:
pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu121 --force-reinstall

# OR For CUDA 11.8 support:
# pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118 --force-reinstall
```

---

## Configuration

Open [config.py](file:///c:/Users/kesha/Downloads/New%20folder/SmartHome/HumanDetection/config.py) to customize parameters:

| Config Key | Description | Default Value |
| :--- | :--- | :--- |
| `MQTT_BROKER` | IP/domain of Mosquitto broker | `"45.32.110.119"` |
| `MQTT_PORT` | Port of the broker | `1883` |
| `MQTT_USERNAME` | Username for MQTT authentication | `"esp8266"` |
| `MQTT_PASSWORD` | Password for MQTT authentication | `"********"` |
| `MQTT_TOPIC` | State topic for occupancy updates | `"home/fancontroller/occupancy/state"` |
| `CONFIDENCE_THRESHOLD` | Confidence limit for YOLO detections | `0.6` |
| `DETECTION_ZONE` | Bounding box boundary `[x_min, y_min, x_max, y_max]` (0.0 to 1.0) | `[0.1, 0.1, 0.9, 0.9]` (10% border margins) |
| `DETECTION_ON_DELAY` | Seconds of continuous presence to trigger ON | `5.0` |
| `DETECTION_OFF_DELAY` | Seconds of continuous absence to trigger OFF | `120.0` |
| `CAMERA_INDEX` | Camera source index (0 for default webcam) | `0` |
| `FPS_LIMIT` | Hard cap on loop speed (reduces CPU usage) | `20` |

---

## Running the Application

Make sure your virtual environment is active, then run:

```powershell
python main.py
```

Upon launch:
1. The app initializes MQTT and prints connect status.
2. The YOLOv8 model (`yolov8n.pt`) is downloaded (if not present) and loaded.
3. The camera feed window is displayed with a telemetry HUD on the left.

### Keyboard Controls
Inside the video feed window:
- **`ESC`**: Safely disconnect MQTT, close the camera, and quit.
- **`R`**: Reset active ON/OFF timers.
- **`M`**: Toggle MQTT publish transmission (simulation mode / live mode).

---

## Home Assistant Integration Example

Once the Python application starts sending occupancy states, you can create a simple Automation in Home Assistant (`automations.yaml`) to bridge the occupancy topic to your fan relay:

```yaml
- id: 'smart_fan_occupancy_control'
  alias: 'Smart Fan Occupancy Automation'
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
- Double check that your camera index is correct in `config.py` (e.g. `1` or `2` if you have multiple webcams).
- Verify that other apps (like Skype or Zoom) are not using the webcam.

### High CPU Usage
- The app uses an FPS frame-rate limiter (`FPS_LIMIT = 20` by default). You can reduce this to `10` or `15` in `config.py` to save CPU resources. YOLOv8 inference is computationally heavy, and running at lower framerates is typically sufficient for occupancy automation.

### MQTT Connection Fails
- Verify that your PC can ping the broker IP (`45.32.110.119`).
- Ensure the port `1883` is not blocked by Windows Firewall or local router.
- Confirm your MQTT username and password in `config.py`.
- The application will automatically attempt background reconnection; look for the `MQTT: Connected` status on the HUD.
