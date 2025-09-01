import time
import cv2
import numpy as np
from picamera2 import Picamera2
from ultralytics import YOLO

# Configuration
MODEL_PATH = "yolo11n.pt"  # YOLOv11 Nano model - will auto-download
CONFIDENCE_THRESHOLD = 0.5
RESOLUTION = (640, 480)

# Set bounding box colors
bbox_colors = [(164,120,87), (68,148,228), (93,97,209), (178,182,133), (88,159,106), 
              (96,202,231), (159,124,168), (169,162,241), (98,118,150), (172,176,184)]

# Initialize camera
cap = Picamera2()
cap.configure(cap.create_video_configuration(main={"format": 'RGB888', "size": RESOLUTION}))
cap.start()

# Load YOLO model (will auto-download on first use)
print("Loading YOLO model...")
model = YOLO(MODEL_PATH, task='detect')
labels = model.names

# Initialize FPS tracking
frame_rate_buffer = []
fps_avg_len = 30
avg_frame_rate = 0

print("Starting YOLO object detection. Press 'q' to quit, 's' to pause, 'p' to save screenshot.")

while True:
    t_start = time.perf_counter()
    
    # Capture frame from Picamera
    frame = cap.capture_array()
    if frame is None:
        print('Unable to read frames from the Picamera.')
        break
    
    # Run YOLO inference
    results = model(frame, verbose=False)
    detections = results[0].boxes
    
    object_count = 0
    
    # Process each detection
    for i in range(len(detections)):
        # Get bounding box coordinates
        xyxy_tensor = detections[i].xyxy.cpu()
        xyxy = xyxy_tensor.numpy().squeeze()
        xmin, ymin, xmax, ymax = xyxy.astype(int)
        
        # Get class and confidence
        classidx = int(detections[i].cls.item())
        classname = labels[classidx]
        conf = detections[i].conf.item()
        
        # Draw detection if confidence is high enough
        if conf > CONFIDENCE_THRESHOLD:
            color = bbox_colors[classidx % 10]
            cv2.rectangle(frame, (xmin, ymin), (xmax, ymax), color, 2)
            
            label = f'{classname}: {int(conf*100)}%'
            labelSize, baseLine = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            label_ymin = max(ymin, labelSize[1] + 10)
            cv2.rectangle(frame, (xmin, label_ymin-labelSize[1]-10), 
                         (xmin+labelSize[0], label_ymin+baseLine-10), color, cv2.FILLED)
            cv2.putText(frame, label, (xmin, label_ymin-7), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
            
            object_count += 1
    
    # Calculate and display FPS
    t_stop = time.perf_counter()
    frame_rate_calc = float(1/(t_stop - t_start))
    
    if len(frame_rate_buffer) >= fps_avg_len:
        frame_rate_buffer.pop(0)
    frame_rate_buffer.append(frame_rate_calc)
    avg_frame_rate = np.mean(frame_rate_buffer)
    
    # Display info on frame
    cv2.putText(frame, f'FPS: {avg_frame_rate:.2f}', (10, 20), 
               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
    cv2.putText(frame, f'Objects: {object_count}', (10, 50), 
               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
    
    # Show frame
    cv2.imshow('YOLO Object Detection', frame)
    
    # Handle key presses
    key = cv2.waitKey(5) & 0xFF
    if key == ord('q') or key == ord('Q'):
        break
    elif key == ord('s') or key == ord('S'):
        cv2.waitKey()
    elif key == ord('p') or key == ord('P'):
        cv2.imwrite('detection_capture.png', frame)
        print("Screenshot saved as 'detection_capture.png'")

# Cleanup
print(f'Average FPS: {avg_frame_rate:.2f}')
cap.stop()
cv2.destroyAllWindows()