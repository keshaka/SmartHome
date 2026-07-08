import logging
import cv2
import numpy as np
from ultralytics import YOLO
import config

class YOLODetector:
    def __init__(self):
        self.logger = logging.getLogger("YOLODetector")
        self.logger.info("Initializing YOLOv8 model...")
        
        # Determine execution device (GPU/CPU fallback verification)
        self.device = config.DEVICE
        try:
            import torch
            if self.device in ("cuda", "0", 0) and not torch.cuda.is_available():
                self.logger.warning("CUDA is requested in config but not available. Falling back to CPU.")
                print("[WARNING] GPU/CUDA requested but not available. Falling back to CPU.")
                self.device = "cpu"
            else:
                self.logger.info(f"Using device: {self.device}")
                print(f"[STATUS] YOLO: Using device '{self.device}'")
        except ImportError:
            self.logger.warning("Could not import torch to verify CUDA. Letting YOLO handle fallback.")

        # Load YOLO model. Ultralytics automatically downloads it to current directory or ~/.ultralytics if missing.
        try:
            self.model = YOLO(config.MODEL_NAME)
            # Transfer model to designated device
            self.model.to(self.device)
            self.logger.info(f"YOLOv8 model '{config.MODEL_NAME}' loaded successfully.")
            print(f"[STATUS] YOLO: Loaded ({config.MODEL_NAME})")
        except Exception as e:
            self.logger.error(f"Failed to load YOLO model: {e}")
            print(f"[ERROR] YOLO: Failed to load model: {e}")
            raise e

    def detect(self, frame):
        """
        Performs object detection on the frame.
        Filters for 'person' class inside the configured detection zone.
        
        Returns:
            processed_frame: Frame with drawn bounding boxes, labels, and zone overlay.
            active_people_count: Number of persons detected INSIDE the detection zone.
            people_inside_zone: Boolean indicating if there is at least one person in the zone.
        """
        # Make a copy of the frame to draw on
        annotated_frame = frame.copy()
        h, w = frame.shape[:2]

        # Calculate absolute pixel coordinates for the detection zone
        zx_min, zy_min, zx_max, zy_max = config.DETECTION_ZONE
        zx1, zy1 = int(zx_min * w), int(zy_min * h)
        zx2, zy2 = int(zx_max * w), int(zy_max * h)

        # Draw the detection zone boundary (Orange/Cyan style)
        # We will draw a stylish rectangle with corners
        zone_color = (255, 165, 0)  # BGR Orange
        cv2.rectangle(annotated_frame, (zx1, zy1), (zx2, zy2), zone_color, 2)
        cv2.putText(
            annotated_frame, 
            "DETECTION ZONE", 
            (zx1 + 10, zy1 + 25), 
            cv2.FONT_HERSHEY_SIMPLEX, 
            0.6, 
            zone_color, 
            2, 
            cv2.LINE_AA
        )

        active_people_count = 0
        people_inside_zone = False

        # Run inference (verbose=False keeps our stdout clean)
        try:
            results = self.model.predict(
                frame, 
                conf=config.CONFIDENCE_THRESHOLD, 
                classes=[config.TARGET_CLASS_ID], 
                device=self.device,
                verbose=False
            )
        except Exception as e:
            self.logger.error(f"Inference error: {e}")
            return annotated_frame, 0, False

        # If results exist, extract boxes
        if results and len(results) > 0:
            boxes = results[0].boxes
            for box in boxes:
                # Confidence score
                conf = float(box.conf[0])
                # Class id
                cls = int(box.cls[0])
                
                if cls != config.TARGET_CLASS_ID:
                    continue

                # Bounding box coordinates
                xyxy = box.xyxy[0].cpu().numpy()
                x1, y1, x2, y2 = map(int, xyxy)
                
                # Calculate bounding box center
                cx = int((x1 + x2) / 2)
                cy = int((y1 + y2) / 2)

                # Check if center lies inside the detection zone
                is_inside = (zx1 <= cx <= zx2) and (zy1 <= cy <= zy2)

                if is_inside:
                    active_people_count += 1
                    people_inside_zone = True

                    # Determine color based on confidence score (Green, Yellow, Red)
                    if conf >= 0.90:
                        box_color = (0, 255, 0)      # Green
                    elif conf >= 0.70:
                        box_color = (0, 255, 255)    # Yellow
                    else:
                        box_color = (0, 0, 255)      # Red
                    
                    label = f"Person: {conf:.1%}"
                    # Draw center point as green dot
                    cv2.circle(annotated_frame, (cx, cy), 5, (0, 255, 0), -1)
                else:
                    # Gray color for ignored person outside the zone
                    box_color = (128, 128, 128)      # Gray
                    label = f"Person (Outside): {conf:.1%}"
                    # Draw center point as gray dot
                    cv2.circle(annotated_frame, (cx, cy), 5, (128, 128, 128), -1)

                # Draw bounding box
                cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), box_color, 2)
                
                # Draw styled text background
                (text_w, text_h), baseline = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
                cv2.rectangle(
                    annotated_frame, 
                    (x1, y1 - text_h - 10), 
                    (x1 + text_w, y1), 
                    box_color, 
                    -1
                )
                
                # Draw text label
                cv2.putText(
                    annotated_frame, 
                    label, 
                    (x1, y1 - 5), 
                    cv2.FONT_HERSHEY_SIMPLEX, 
                    0.5, 
                    (0, 0, 0) if box_color != (128, 128, 128) else (255, 255, 255), 
                    1, 
                    cv2.LINE_AA
                )

        return annotated_frame, active_people_count, people_inside_zone
