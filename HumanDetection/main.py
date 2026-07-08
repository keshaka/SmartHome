import os
import cv2
import numpy as np
import time
import logging
from datetime import datetime
import collections

# Import project modules
import config
from mqtt_client import FanMQTTClient
from detector import YOLODetector

def setup_logging():
    """
    Configures application-wide logging.
    """
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
        handlers=[
            logging.FileHandler("app.log", encoding="utf-8"),
            logging.StreamHandler()
        ]
    )

def main():
    setup_logging()
    logger = logging.getLogger("MainApp")
    logger.info("Starting Human Detection Fan Automation Application...")
    print("====================================================")
    print("       SMART HOME OCCUPANCY DETECTION SYSTEM        ")
    print("====================================================")

    # Ensure captures directory exists
    os.makedirs(config.CAPTURE_DIR, exist_ok=True)

    # Initialize MQTT client
    mqtt_client = FanMQTTClient()
    mqtt_client.connect()

    # Initialize YOLOv8 Detector
    print("[STATUS] YOLO: Loading...")
    try:
        detector = YOLODetector()
    except Exception as e:
        logger.critical(f"Failed to initialize YOLO detector: {e}")
        return

    # Initialize Occupancy State Machine Variables
    # Startup Behavior: Start in Fan OFF state, and don't publish OFF immediately.
    current_state = "OFF"
    
    person_detected_start_time = None
    no_person_start_time = None
    was_person_detected = False  # Track for first-appearance screenshots

    # Camera state and connection variables
    cap = None
    camera_ok = False
    last_camera_retry_time = 0.0

    # FPS calculations & limiting variables
    frame_times = collections.deque(maxlen=50)
    last_frame_time = time.time()
    avg_fps = 0.0

    # Create UI window (resizable for desktop experience)
    cv2.namedWindow(config.WINDOW_NAME, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(config.WINDOW_NAME, config.WINDOW_WIDTH, config.WINDOW_HEIGHT)

    logger.info("Application initialized. Entering main loop.")
    print("[INFO] Application running. Press ESC to quit, R to reset timers, M to toggle MQTT.")

    try:
        while True:
            loop_start_time = time.time()

            # ----------------------------------------------------
            # 1. Camera Connection and Capture Handling
            # ----------------------------------------------------
            if not camera_ok:
                # Try to connect/reconnect camera
                current_time = time.time()
                if cap is None and (current_time - last_camera_retry_time >= config.CAMERA_RECONNECT_INTERVAL):
                    last_camera_retry_time = current_time
                    logger.info(f"Attempting to open camera (index {config.CAMERA_INDEX})...")
                    cap = cv2.VideoCapture(config.CAMERA_INDEX)
                    if cap.isOpened():
                        # Set custom buffer sizes if supported to minimize latency
                        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                        camera_ok = True
                        logger.info("Camera connected successfully.")
                        print("[STATUS] Camera: OK")
                    else:
                        cap = None
                        logger.warning("Camera connection failed. Will retry.")
                        print("[STATUS] Camera: Disconnected (Retrying...)")

            # Read frame if camera is connected
            frame = None
            if camera_ok and cap is not None:
                ret, frame = cap.read()
                if not ret:
                    logger.warning("Failed to grab frame. Camera disconnected.")
                    print("[STATUS] Camera: Disconnected")
                    camera_ok = False
                    cap.release()
                    cap = None
                    last_camera_retry_time = time.time()

            # Fallback to black screen if camera is disconnected
            if frame is None:
                # Create a black frame for the UI
                frame = np.zeros((config.WINDOW_HEIGHT, config.WINDOW_WIDTH, 3), dtype=np.uint8)
                
                # Draw "CAMERA DISCONNECTED" watermark in the center
                cv2.putText(
                    frame, 
                    "CAMERA DISCONNECTED", 
                    (int(config.WINDOW_WIDTH / 2) - 220, int(config.WINDOW_HEIGHT / 2)), 
                    cv2.FONT_HERSHEY_SIMPLEX, 
                    1.0, 
                    (0, 0, 255), 
                    2, 
                    cv2.LINE_AA
                )
                
                # Show countdown to retry
                time_to_retry = max(0.0, config.CAMERA_RECONNECT_INTERVAL - (time.time() - last_camera_retry_time))
                cv2.putText(
                    frame, 
                    f"Retrying in {time_to_retry:.1f}s...", 
                    (int(config.WINDOW_WIDTH / 2) - 100, int(config.WINDOW_HEIGHT / 2) + 40), 
                    cv2.FONT_HERSHEY_SIMPLEX, 
                    0.6, 
                    (0, 255, 255), 
                    1, 
                    cv2.LINE_AA
                )
                
                active_people_count = 0
                people_inside_zone = False
                annotated_frame = frame.copy()
            else:
                # Resize frame to matching UI dimensions if necessary
                if frame.shape[1] != config.WINDOW_WIDTH or frame.shape[0] != config.WINDOW_HEIGHT:
                    frame = cv2.resize(frame, (config.WINDOW_WIDTH, config.WINDOW_HEIGHT))
                
                # ----------------------------------------------------
                # 2. YOLO Object Detection & Zone Filtering
                # ----------------------------------------------------
                annotated_frame, active_people_count, people_inside_zone = detector.detect(frame)

            # ----------------------------------------------------
            # 3. Screenshot Capture on First Appearance
            # ----------------------------------------------------
            if people_inside_zone:
                if not was_person_detected:
                    if config.SAVE_SCREENSHOTS:
                        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                        screenshot_name = f"capture_{timestamp}.jpg"
                        screenshot_path = os.path.join(config.CAPTURE_DIR, screenshot_name)
                        cv2.imwrite(screenshot_path, annotated_frame)
                        logger.info(f"Person first appeared inside zone. Screenshot saved: {screenshot_path}")
                        print(f"[INFO] Person appeared! Saved screenshot to {screenshot_path}")
                    was_person_detected = True
            else:
                was_person_detected = False

            # ----------------------------------------------------
            # 4. Occupancy State Machine (Timing Rules)
            # ----------------------------------------------------
            current_time = time.time()
            on_timer_val = 0.0
            off_timer_val = 0.0

            if current_state == "OFF":
                no_person_start_time = None  # Reset OFF timer since we are already OFF
                
                if people_inside_zone:
                    if person_detected_start_time is None:
                        person_detected_start_time = current_time
                        logger.info("Person detected inside zone. Starting ON timer...")
                        print("[INFO] Person detected. Timing ON...")
                    
                    on_timer_val = current_time - person_detected_start_time
                    
                    # Check if continuous detection exceeds threshold
                    if on_timer_val >= config.DETECTION_ON_DELAY:
                        current_state = "ON"
                        logger.info(f"ON timer threshold reached ({config.DETECTION_ON_DELAY}s). Transitioning state to ON.")
                        mqtt_client.publish_state("ON")
                        person_detected_start_time = None
                else:
                    person_detected_start_time = None

            elif current_state == "ON":
                person_detected_start_time = None  # Reset ON timer since we are already ON
                
                if not people_inside_zone:
                    if no_person_start_time is None:
                        no_person_start_time = current_time
                        logger.info("No person detected inside zone. Starting OFF timer...")
                        print("[INFO] No person detected. Timing OFF...")
                    
                    off_timer_val = current_time - no_person_start_time
                    
                    # Check if continuous absence exceeds threshold
                    if off_timer_val >= config.DETECTION_OFF_DELAY:
                        current_state = "OFF"
                        logger.info(f"OFF timer threshold reached ({config.DETECTION_OFF_DELAY}s). Transitioning state to OFF.")
                        mqtt_client.publish_state("OFF")
                        no_person_start_time = None
                else:
                    no_person_start_time = None

            # ----------------------------------------------------
            # 5. Render HUD Panel Overlay
            # ----------------------------------------------------
            # Render HUD panel on the left side (semi-transparent panel overlay)
            panel_width = 300
            overlay = annotated_frame.copy()
            cv2.rectangle(overlay, (0, 0), (panel_width, config.WINDOW_HEIGHT), (25, 25, 25), -1)
            cv2.addWeighted(overlay, 0.75, annotated_frame, 0.25, 0, annotated_frame)

            # Draw HUD Text Content
            y_offset = 35
            
            # Header
            cv2.putText(annotated_frame, "SMART AUTOMATION HUD", (15, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2, cv2.LINE_AA)
            y_offset += 15
            cv2.line(annotated_frame, (15, y_offset), (panel_width - 15, y_offset), (100, 100, 100), 1)
            y_offset += 25

            # Section: SYSTEM STATUS
            cv2.putText(annotated_frame, "SYSTEM STATUS", (15, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1, cv2.LINE_AA)
            y_offset += 25
            
            # Camera status
            cam_txt = "OK" if camera_ok else "Disconnected"
            cam_col = (0, 255, 0) if camera_ok else (0, 0, 255)
            cv2.putText(annotated_frame, "Camera: ", (15, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
            cv2.putText(annotated_frame, cam_txt, (110, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.5, cam_col, 1, cv2.LINE_AA)
            y_offset += 20

            # YOLO status
            cv2.putText(annotated_frame, "YOLO: ", (15, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
            cv2.putText(annotated_frame, "Loaded", (110, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
            y_offset += 20

            # MQTT status
            if not mqtt_client.is_enabled:
                mqtt_txt = "Disabled"
                mqtt_col = (128, 128, 128)
            elif mqtt_client.is_connected:
                mqtt_txt = "Connected"
                mqtt_col = (0, 255, 0)
            else:
                mqtt_txt = "Disconnected"
                mqtt_col = (0, 0, 255)
            cv2.putText(annotated_frame, "MQTT: ", (15, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
            cv2.putText(annotated_frame, mqtt_txt, (110, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.5, mqtt_col, 1, cv2.LINE_AA)
            
            y_offset += 15
            cv2.line(annotated_frame, (15, y_offset), (panel_width - 15, y_offset), (100, 100, 100), 1)
            y_offset += 25

            # Section: OCCUPANCY INFO
            cv2.putText(annotated_frame, "OCCUPANCY INFO", (15, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1, cv2.LINE_AA)
            y_offset += 25

            # Person in zone
            person_txt = f"YES ({active_people_count})" if active_people_count > 0 else "NO"
            person_col = (0, 255, 0) if active_people_count > 0 else (128, 128, 128)
            cv2.putText(annotated_frame, "In Zone: ", (15, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
            cv2.putText(annotated_frame, person_txt, (110, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.5, person_col, 1, cv2.LINE_AA)
            y_offset += 20

            # Occupancy/Fan State
            state_col = (0, 255, 0) if current_state == "ON" else (0, 0, 255)
            cv2.putText(annotated_frame, "Fan State: ", (15, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
            cv2.putText(annotated_frame, current_state, (110, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.5, state_col, 1, cv2.LINE_AA)
            
            y_offset += 15
            cv2.line(annotated_frame, (15, y_offset), (panel_width - 15, y_offset), (100, 100, 100), 1)
            y_offset += 25

            # Section: TIMERS
            cv2.putText(annotated_frame, "ACTIVE TIMERS", (15, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1, cv2.LINE_AA)
            y_offset += 25

            # ON Timer
            on_timer_txt = f"{on_timer_val:.1f}s / {config.DETECTION_ON_DELAY}s" if person_detected_start_time is not None else "N/A"
            on_timer_col = (0, 255, 255) if person_detected_start_time is not None else (128, 128, 128)
            cv2.putText(annotated_frame, "ON Timer: ", (15, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
            cv2.putText(annotated_frame, on_timer_txt, (110, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.5, on_timer_col, 1, cv2.LINE_AA)
            y_offset += 20

            # OFF Timer
            off_timer_txt = f"{off_timer_val:.1f}s / {config.DETECTION_OFF_DELAY}s" if no_person_start_time is not None else "N/A"
            off_timer_col = (0, 255, 255) if no_person_start_time is not None else (128, 128, 128)
            cv2.putText(annotated_frame, "OFF Timer: ", (15, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
            cv2.putText(annotated_frame, off_timer_txt, (110, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.5, off_timer_col, 1, cv2.LINE_AA)
            
            y_offset += 15
            cv2.line(annotated_frame, (15, y_offset), (panel_width - 15, y_offset), (100, 100, 100), 1)
            y_offset += 25

            # Section: METRICS
            cv2.putText(annotated_frame, "PERFORMANCE METRICS", (15, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1, cv2.LINE_AA)
            y_offset += 25

            # FPS metric
            cv2.putText(annotated_frame, f"FPS: {avg_fps:.1f} (Limit: {config.FPS_LIMIT})", (15, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
            y_offset += 20

            # Latency metric
            lat_txt = f"{mqtt_client.latency_ms} ms" if mqtt_client.latency_ms is not None else "N/A"
            cv2.putText(annotated_frame, f"MQTT Latency: {lat_txt}", (15, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
            
            y_offset += 15
            cv2.line(annotated_frame, (15, y_offset), (panel_width - 15, y_offset), (100, 100, 100), 1)
            y_offset += 25

            # Section: CONTROLS
            cv2.putText(annotated_frame, "KEYBOARD SHORTCUTS", (15, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1, cv2.LINE_AA)
            y_offset += 25
            cv2.putText(annotated_frame, "[ESC] Quit Application", (15, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (170, 170, 170), 1, cv2.LINE_AA)
            y_offset += 20
            cv2.putText(annotated_frame, "[R]   Reset Timers", (15, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (170, 170, 170), 1, cv2.LINE_AA)
            y_offset += 20
            cv2.putText(annotated_frame, "[M]   Toggle MQTT Send", (15, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (170, 170, 170), 1, cv2.LINE_AA)

            # Display the final frame
            cv2.imshow(config.WINDOW_NAME, annotated_frame)

            # ----------------------------------------------------
            # 6. Keyboard Input Processing
            # ----------------------------------------------------
            key = cv2.waitKey(1) & 0xFF
            if key == 27:  # ESC Key
                logger.info("ESC key pressed. Initiating exit...")
                break
            elif key in (ord('r'), ord('R')):
                logger.info("Timer reset command received.")
                person_detected_start_time = None
                no_person_start_time = None
                print("[INFO] Timers reset successfully.")
            elif key in (ord('m'), ord('M')):
                mqtt_client.toggle_enabled()

            # ----------------------------------------------------
            # 7. FPS Framerate Limiter
            # ----------------------------------------------------
            current_time = time.time()
            elapsed_time = current_time - loop_start_time
            target_time = 1.0 / config.FPS_LIMIT
            
            if elapsed_time < target_time:
                sleep_duration = target_time - elapsed_time
                time.sleep(sleep_duration)
            
            # FPS tracking calculations
            now = time.time()
            frame_times.append(now - last_frame_time)
            last_frame_time = now
            if len(frame_times) > 0:
                avg_fps = 1.0 / (sum(frame_times) / len(frame_times))

    except KeyboardInterrupt:
        logger.info("Keyboard interrupt received.")
    except Exception as e:
        logger.error(f"Unexpected application crash: {e}", exc_info=True)
    finally:
        # Clean up resources
        logger.info("Cleaning up application resources...")
        if cap is not None:
            cap.release()
        mqtt_client.disconnect()
        cv2.destroyAllWindows()
        logger.info("Shutdown sequence complete. Exiting.")
        print("====================================================")
        print("            APPLICATION SHUTDOWN COMPLETE           ")
        print("====================================================")

if __name__ == "__main__":
    main()
