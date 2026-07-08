import time
import logging
import paho.mqtt.client as mqtt
import config

class FanMQTTClient:
    def __init__(self):
        # Configure logging
        self.logger = logging.getLogger("MQTTClient")
        
        # Initialize paho client
        self.client = mqtt.Client(client_id=config.MQTT_CLIENT_ID)
        self.client.username_pw_set(config.MQTT_USERNAME, config.MQTT_PASSWORD)
        
        # Register callback handlers
        self.client.on_connect = self._on_connect
        self.client.on_disconnect = self._on_disconnect
        self.client.on_publish = self._on_publish
        
        # Internal state
        self.is_connected = False
        self.is_enabled = True  # Can be toggled with key 'M'
        self.latency_ms = None
        self._sent_messages = {}  # mid -> timestamp

    def _on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.is_connected = True
            self.logger.info("MQTT Connected successfully")
            print("[STATUS] MQTT Connected")
        else:
            self.is_connected = False
            self.logger.error(f"MQTT Connection failed with code {rc}")
            print(f"[STATUS] MQTT Connection failed (rc={rc})")

    def _on_disconnect(self, client, userdata, rc):
        self.is_connected = False
        self.logger.warning(f"MQTT Disconnected (rc={rc})")
        print("[STATUS] MQTT Reconnecting...")

    def _on_publish(self, client, userdata, mid):
        # Calculate publish acknowledgment latency
        if mid in self._sent_messages:
            latency = (time.time() - self._sent_messages[mid]) * 1000.0
            self.latency_ms = round(latency, 1)
            self.logger.debug(f"Message {mid} published. Latency: {self.latency_ms} ms")
            # Clean up message entry
            del self._sent_messages[mid]

    def connect(self):
        """
        Connects asynchronously to the MQTT broker and starts the background loop.
        """
        try:
            self.logger.info(f"Connecting to MQTT broker at {config.MQTT_BROKER}:{config.MQTT_PORT}...")
            # connect_async handles background reconnection retries automatically
            self.client.connect_async(config.MQTT_BROKER, config.MQTT_PORT, keepalive=60)
            self.client.loop_start()
        except Exception as e:
            self.logger.error(f"Failed to initiate MQTT connection: {e}")
            print(f"[ERROR] MQTT Connection initialization failed: {e}")

    def publish_state(self, state: str):
        """
        Publishes the occupancy state (ON or OFF) to the command topic.
        Returns True if publish request was successfully queued, False otherwise.
        """
        if not self.is_enabled:
            self.logger.info(f"MQTT publish skipped (MQTT disabled in UI): {config.MQTT_TOPIC} -> {state}")
            print(f"[MQTT] (Simulated) State change: {state}")
            return False

        if not self.is_connected:
            self.logger.warning("MQTT not connected. Cannot publish state.")
            print(f"[MQTT] Error: Cannot publish state '{state}'. Broker disconnected.")
            return False

        try:
            # QoS 1 ensures delivery and gives us a reliable on_publish callback
            result = self.client.publish(config.MQTT_TOPIC, state, qos=1)
            mid = result.mid
            self._sent_messages[mid] = time.time()
            self.logger.info(f"MQTT Published: {config.MQTT_TOPIC} -> {state} (mid={mid})")
            print(f"[MQTT] Occupancy state published: {state}")
            
            # Periodically prune old message timestamps to prevent memory leaks if broker fails to ack
            if len(self._sent_messages) > 100:
                current_time = time.time()
                # Remove keys older than 10 seconds
                self._sent_messages = {
                    m: t for m, t in self._sent_messages.items() if current_time - t < 10.0
                }
            return True
        except Exception as e:
            self.logger.error(f"Failed to publish MQTT message: {e}")
            print(f"[MQTT] Error publishing state: {e}")
            return False

    def toggle_enabled(self):
        """
        Toggles whether MQTT commands are physically published.
        """
        self.is_enabled = not self.is_enabled
        status_str = "Enabled" if self.is_enabled else "Disabled"
        self.logger.info(f"MQTT Transmission toggled: {status_str}")
        print(f"[STATUS] MQTT Transmission: {status_str}")
        return self.is_enabled

    def disconnect(self):
        """
        Stops background loop and disconnects client.
        """
        self.client.loop_stop()
        self.client.disconnect()
        self.logger.info("MQTT Client disconnected and loop stopped.")
