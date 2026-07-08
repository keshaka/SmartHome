import os
from dotenv import load_dotenv

# Load environment variables from the .env file in the current directory
load_dotenv()

# MQTT Broker Configuration
MQTT_BROKER = os.getenv("MQTT_BROKER", "127.0.0.1")
MQTT_PORT = int(os.getenv("MQTT_PORT", 1883))
MQTT_USERNAME = os.getenv("MQTT_USERNAME", "your_mqtt_username")
MQTT_PASSWORD = os.getenv("MQTT_PASSWORD", "your_mqtt_password")
MQTT_TOPIC = os.getenv("MQTT_TOPIC", "home/fancontroller/occupancy/state")
MQTT_CLIENT_ID = os.getenv("MQTT_CLIENT_ID", "YOLO_Occupancy_Detector")

# Detection Settings
MODEL_NAME = os.getenv("MODEL_NAME", "yolov8n.pt")
CONFIDENCE_THRESHOLD = float(os.getenv("CONFIDENCE_THRESHOLD", 0.6))
TARGET_CLASS_ID = int(os.getenv("TARGET_CLASS_ID", 0))  # YOLO class 0 is 'person'
DEVICE = os.getenv("DEVICE", "cuda")                  # "cuda" / "0" for GPU, "cpu" for CPU

# Detection Zone Configuration
# Parses a comma-separated coordinate string e.g., "0.1,0.1,0.9,0.9" to a float list
zone_raw = os.getenv("DETECTION_ZONE", "0.1,0.1,0.9,0.9")
try:
    DETECTION_ZONE = [float(val.strip()) for val in zone_raw.split(",")]
except Exception:
    DETECTION_ZONE = [0.1, 0.1, 0.9, 0.9]

# Timing Rules (in seconds)
DETECTION_ON_DELAY = float(os.getenv("DETECTION_ON_DELAY", 5.0))
DETECTION_OFF_DELAY = float(os.getenv("DETECTION_OFF_DELAY", 120.0))

# Camera Settings
CAMERA_INDEX = int(os.getenv("CAMERA_INDEX", 0))
CAMERA_RECONNECT_INTERVAL = float(os.getenv("CAMERA_RECONNECT_INTERVAL", 5.0))
FPS_LIMIT = int(os.getenv("FPS_LIMIT", 20))

# UI Settings
WINDOW_WIDTH = int(os.getenv("WINDOW_WIDTH", 960))
WINDOW_HEIGHT = int(os.getenv("WINDOW_HEIGHT", 720))
WINDOW_NAME = os.getenv("WINDOW_NAME", "Smart Fan Human Detection Automation")

# Paths & Features
SAVE_SCREENSHOTS = os.getenv("SAVE_SCREENSHOTS", "False").lower() in ("true", "1", "yes")
CAPTURE_DIR = os.getenv("CAPTURE_DIR", "captures")
