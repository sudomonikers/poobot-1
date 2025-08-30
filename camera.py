#!/usr/bin/env python3
"""
Camera Controller Module
Handles camera operations and AI model integration for poop detection
"""

import time
import threading
import queue
import cv2
import numpy as np
import logging

logger = logging.getLogger(__name__)

class CameraController:
    """Handles camera operations and AI model integration"""
    
    def __init__(self):
        self.camera = cv2.VideoCapture(0)  # Pi Camera
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        
        # Image storage
        self.image_queue = queue.Queue(maxsize=100)
        self.latest_image = None
        self.capture_thread = None
        self.is_capturing = False
        
        # AI model placeholder (you'll replace this with your trained model)
        self.poop_detection_model = None
        
    def load_ai_model(self, model_path):
        """Load your trained poop detection model"""
        try:
            # Placeholder - replace with your actual model loading
            # Example for TensorFlow Lite:
            # import tflite_runtime.interpreter as tflite
            # self.poop_detection_model = tflite.Interpreter(model_path)
            # self.poop_detection_model.allocate_tensors()
            
            logger.info(f"AI model loaded from {model_path}")
            return True
        except Exception as e:
            logger.error(f"Failed to load AI model: {e}")
            return False
    
    def start_continuous_capture(self):
        """Start continuous image capture in background thread"""
        if self.is_capturing:
            return
        
        self.is_capturing = True
        self.capture_thread = threading.Thread(target=self._capture_loop)
        self.capture_thread.daemon = True
        self.capture_thread.start()
        logger.info("Started continuous camera capture")
    
    def stop_continuous_capture(self):
        """Stop continuous image capture"""
        self.is_capturing = False
        if self.capture_thread:
            self.capture_thread.join()
        logger.info("Stopped camera capture")
    
    def _capture_loop(self):
        """Background thread for continuous image capture"""
        while self.is_capturing:
            ret, frame = self.camera.read()
            if ret:
                self.latest_image = frame.copy()
                
                # Add to queue (remove oldest if full)
                try:
                    self.image_queue.put_nowait(frame)
                except queue.Full:
                    try:
                        self.image_queue.get_nowait()  # Remove oldest
                        self.image_queue.put_nowait(frame)
                    except queue.Empty:
                        pass
            
            time.sleep(0.1)  # 10 FPS capture rate
    
    def take_photo(self, save_path=None):
        """Take a single photo and optionally save it"""
        ret, frame = self.camera.read()
        if not ret:
            logger.error("Failed to capture image")
            return None
        
        if save_path:
            cv2.imwrite(save_path, frame)
            logger.info(f"Photo saved to {save_path}")
        
        return frame
    
    def analyze_image_for_poop(self, image=None):
        """Analyze image for poop detection using AI model"""
        if image is None:
            image = self.latest_image
        
        if image is None:
            return False, 0.0
        
        # Placeholder for AI model inference
        # Replace this with your actual model inference
        if self.poop_detection_model is None:
            # Dummy detection for testing
            return False, 0.0
        
        try:
            # Example TensorFlow Lite inference:
            # input_details = self.poop_detection_model.get_input_details()
            # output_details = self.poop_detection_model.get_output_details()
            
            # # Preprocess image
            # processed_image = cv2.resize(image, (224, 224))
            # processed_image = np.expand_dims(processed_image, axis=0)
            # processed_image = processed_image.astype(np.float32) / 255.0
            
            # # Run inference
            # self.poop_detection_model.set_tensor(input_details[0]['index'], processed_image)
            # self.poop_detection_model.invoke()
            # predictions = self.poop_detection_model.get_tensor(output_details[0]['index'])
            
            # has_poop = predictions[0][0] > 0.5  # Confidence threshold
            # confidence = float(predictions[0][0])
            
            # return has_poop, confidence
            
            return False, 0.0
            
        except Exception as e:
            logger.error(f"Error in poop detection: {e}")
            return False, 0.0
    
    def cleanup(self):
        """Clean up camera resources"""
        self.stop_continuous_capture()
        if self.camera:
            self.camera.release()
        logger.info("Camera resources cleaned up")