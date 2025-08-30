#!/usr/bin/env python3
"""
Dog Poop Picking Robot - Complete System
Comprehensive Raspberry Pi code with motor control, camera, mapping, sensors, and AI integration
"""


import time
import threading
import numpy as np
import cv2
import json
import math
import queue
from datetime import datetime
from enum import Enum
from dataclasses import dataclass
from typing import List, Tuple, Optional
import logging



#import RPi.GPIO as GPIO #import this in the actual raspberry pi
##for development on mac, use this class
class MockGPIO:
    """Mock GPIO class for development on non-Raspberry Pi systems"""
    
    BCM = "BCM"
    OUT = "OUT"
    IN = "IN"
    HIGH = 1
    LOW = 0
    
    _pins = {}
    _pwm_instances = {}
    
    @classmethod
    def setmode(cls, mode):
        logger.debug(f"Mock GPIO: Set mode to {mode}")
    
    @classmethod
    def setwarnings(cls, warnings):
        logger.debug(f"Mock GPIO: Set warnings to {warnings}")
    
    @classmethod
    def setup(cls, pin, mode):
        cls._pins[pin] = {'mode': mode, 'value': 0}
        logger.debug(f"Mock GPIO: Setup pin {pin} as {mode}")
    
    @classmethod
    def output(cls, pin, value):
        if pin in cls._pins:
            cls._pins[pin]['value'] = value
            logger.debug(f"Mock GPIO: Pin {pin} set to {value}")
    
    @classmethod
    def input(cls, pin):
        if pin in cls._pins:
            # Simulate sensor readings
            return random.choice([0, 1])
        return 0
    
    @classmethod
    def PWM(cls, pin, frequency):
        pwm = MockPWM(pin, frequency)
        cls._pwm_instances[pin] = pwm
        return pwm
    
    @classmethod
    def cleanup(cls):
        logger.info("Mock GPIO: Cleanup called")
        cls._pins.clear()
        cls._pwm_instances.clear()

class MockPWM:
    """Mock PWM class for development"""
    
    def __init__(self, pin, frequency):
        self.pin = pin
        self.frequency = frequency
        self.duty_cycle = 0
        self.running = False
        logger.debug(f"Mock PWM: Created for pin {pin} at {frequency}Hz")
    
    def start(self, duty_cycle):
        self.duty_cycle = duty_cycle
        self.running = True
        logger.debug(f"Mock PWM: Started pin {self.pin} at {duty_cycle}% duty cycle")
    
    def ChangeDutyCycle(self, duty_cycle):
        self.duty_cycle = duty_cycle
        logger.debug(f"Mock PWM: Pin {self.pin} duty cycle changed to {duty_cycle}%")
    
    def stop(self):
        self.running = False
        logger.debug(f"Mock PWM: Stopped pin {self.pin}")

# Use mock GPIO for development
GPIO = MockGPIO()




# Setup logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

@dataclass
class Position:
    x: float
    y: float
    heading: float  # Angle in degrees

@dataclass
class GridCell:
    x: int
    y: int
    is_obstacle: bool = False
    is_visited: bool = False
    has_poop: bool = False
    confidence: float = 0.0

class RobotState(Enum):
    IDLE = "idle"
    MAPPING = "mapping"
    PATROLLING = "patrolling"
    INVESTIGATING = "investigating"
    COLLECTING = "collecting"
    RETURNING = "returning"

class Direction(Enum):
    FORWARD = "forward"
    BACKWARD = "backward"
    LEFT = "left"
    RIGHT = "right"
    STOP = "stop"

class SensorController:
    """Handles ultrasonic sensors for obstacle detection"""
    
    def __init__(self):
        # Ultrasonic sensor pins (HC-SR04)
        self.sensors = {
            'front': {'trig': 24, 'echo': 23},
            'left': {'trig': 26, 'echo': 25},
            'right': {'trig': 6, 'echo': 5}
        }
        
        # Setup GPIO for sensors
        for sensor in self.sensors.values():
            GPIO.setup(sensor['trig'], GPIO.OUT)
            GPIO.setup(sensor['echo'], GPIO.IN)
            GPIO.output(sensor['trig'], False)
        
        time.sleep(2)  # Sensor settle time
    
    def get_distance(self, sensor_name):
        """Get distance from specific sensor in cm"""
        if sensor_name not in self.sensors:
            return float('inf')
        
        sensor = self.sensors[sensor_name]
        
        # Send trigger pulse
        GPIO.output(sensor['trig'], True)
        time.sleep(0.00001)
        GPIO.output(sensor['trig'], False)
        
        # Measure echo time
        start_time = time.time()
        while GPIO.input(sensor['echo']) == 0:
            pulse_start = time.time()
            if pulse_start - start_time > 0.1:  # Timeout
                return float('inf')
        
        while GPIO.input(sensor['echo']) == 1:
            pulse_end = time.time()
            if pulse_end - pulse_start > 0.1:  # Timeout
                return float('inf')
        
        # Calculate distance
        pulse_duration = pulse_end - pulse_start
        distance = pulse_duration * 17150  # Speed of sound calculation
        
        return round(distance, 2)
    
    def get_all_distances(self):
        """Get distances from all sensors"""
        distances = {}
        for name in self.sensors.keys():
            distances[name] = self.get_distance(name)
            time.sleep(0.1)  # Small delay between readings
        return distances
    
    def has_obstacle(self, min_distance=20):
        """Check if there's an obstacle in any direction"""
        distances = self.get_all_distances()
        return any(dist < min_distance for dist in distances.values() if dist != float('inf'))

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

class EnvironmentMapper:
    """Handles environment mapping and navigation"""
    
    def __init__(self, grid_size=200, cell_size_cm=10):
        self.grid_size = grid_size  # Grid dimensions
        self.cell_size_cm = cell_size_cm  # Each cell represents 10cm x 10cm
        
        # Initialize grid
        self.grid = [[GridCell(x, y) for y in range(grid_size)] for x in range(grid_size)]
        
        # Robot position tracking
        self.robot_position = Position(grid_size//2, grid_size//2, 0)  # Start at center
        self.start_position = Position(grid_size//2, grid_size//2, 0)
        
        # Path planning
        self.current_path = []
        self.visited_cells = set()
        
        # Mapping progress
        self.total_cells = grid_size * grid_size
        self.mapped_cells = 0
    
    def update_robot_position(self, dx_cm, dy_cm, heading_change=0):
        """Update robot position based on movement"""
        # Convert cm to grid cells
        dx_cells = dx_cm / self.cell_size_cm
        dy_cells = dy_cm / self.cell_size_cm
        
        # Update position
        self.robot_position.x += dx_cells
        self.robot_position.y += dy_cells
        self.robot_position.heading = (self.robot_position.heading + heading_change) % 360
        
        # Mark current cell as visited
        self.mark_cell_visited(int(self.robot_position.x), int(self.robot_position.y))
    
    def mark_cell_visited(self, x, y):
        """Mark a cell as visited"""
        if 0 <= x < self.grid_size and 0 <= y < self.grid_size:
            if not self.grid[x][y].is_visited:
                self.grid[x][y].is_visited = True
                self.visited_cells.add((x, y))
                self.mapped_cells += 1
    
    def mark_obstacle(self, x, y):
        """Mark a cell as containing an obstacle"""
        if 0 <= x < self.grid_size and 0 <= y < self.grid_size:
            self.grid[x][y].is_obstacle = True
    
    def mark_poop_location(self, x, y, confidence):
        """Mark a cell as containing poop"""
        if 0 <= x < self.grid_size and 0 <= y < self.grid_size:
            self.grid[x][y].has_poop = True
            self.grid[x][y].confidence = confidence
            logger.info(f"Poop detected at grid position ({x}, {y}) with confidence {confidence}")
    
    def get_next_exploration_target(self):
        """Get next cell to explore using systematic pattern"""
        current_x = int(self.robot_position.x)
        current_y = int(self.robot_position.y)
        
        # Simple lawn mower pattern
        search_radius = 1
        while search_radius < self.grid_size // 2:
            for dx in range(-search_radius, search_radius + 1):
                for dy in range(-search_radius, search_radius + 1):
                    target_x = current_x + dx
                    target_y = current_y + dy
                    
                    if (0 <= target_x < self.grid_size and 
                        0 <= target_y < self.grid_size and
                        not self.grid[target_x][target_y].is_visited and
                        not self.grid[target_x][target_y].is_obstacle):
                        return (target_x, target_y)
            
            search_radius += 1
        
        return None  # No more cells to explore
    
    def plan_path_to_target(self, target_x, target_y):
        """Simple A* pathfinding to target"""
        # Simplified pathfinding - just return direct path for now
        current_x = int(self.robot_position.x)
        current_y = int(self.robot_position.y)
        
        path = []
        
        # Simple direct path (can be enhanced with proper A*)
        dx = 1 if target_x > current_x else (-1 if target_x < current_x else 0)
        dy = 1 if target_y > current_y else (-1 if target_y < current_y else 0)
        
        x, y = current_x, current_y
        while x != target_x or y != target_y:
            if x != target_x:
                x += dx
            if y != target_y:
                y += dy
            
            if 0 <= x < self.grid_size and 0 <= y < self.grid_size:
                if not self.grid[x][y].is_obstacle:
                    path.append((x, y))
                else:
                    break  # Hit obstacle, need better pathfinding
        
        self.current_path = path
        return path
    
    def get_poop_locations(self):
        """Get all detected poop locations"""
        poop_locations = []
        for x in range(self.grid_size):
            for y in range(self.grid_size):
                if self.grid[x][y].has_poop:
                    poop_locations.append((x, y, self.grid[x][y].confidence))
        return poop_locations
    
    def save_map(self, filename):
        """Save map to file"""
        map_data = {
            'grid_size': self.grid_size,
            'cell_size_cm': self.cell_size_cm,
            'robot_position': {
                'x': self.robot_position.x,
                'y': self.robot_position.y,
                'heading': self.robot_position.heading
            },
            'visited_cells': list(self.visited_cells),
            'obstacles': [],
            'poop_locations': []
        }
        
        # Collect obstacles and poop locations
        for x in range(self.grid_size):
            for y in range(self.grid_size):
                cell = self.grid[x][y]
                if cell.is_obstacle:
                    map_data['obstacles'].append([x, y])
                if cell.has_poop:
                    map_data['poop_locations'].append([x, y, cell.confidence])
        
        with open(filename, 'w') as f:
            json.dump(map_data, f, indent=2)
        
        logger.info(f"Map saved to {filename}")

class RobotMotorController:
    """Enhanced motor controller with position tracking"""
    
    def __init__(self, mapper=None):
        # Motor GPIO pins (L298N motor driver)
        self.motor_pins = {
            'left_motor': {'in1': 18, 'in2': 19, 'enable': 12},
            'right_motor': {'in1': 20, 'in2': 21, 'enable': 13}
        }
        
        # Setup GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        
        for motor in self.motor_pins.values():
            for pin in motor.values():
                GPIO.setup(pin, GPIO.OUT)
        
        # PWM setup
        self.left_pwm = GPIO.PWM(self.motor_pins['left_motor']['enable'], 1000)
        self.right_pwm = GPIO.PWM(self.motor_pins['right_motor']['enable'], 1000)
        self.left_pwm.start(0)
        self.right_pwm.start(0)
        
        self.current_speed = 50
        self.is_moving = False
        self.mapper = mapper
        
        # Movement calibration (adjust based on your robot)
        self.cm_per_second_at_50_speed = 20  # Calibrate this
        self.degrees_per_second_turn = 90   # Calibrate this
    
    def set_motor_direction(self, motor, direction):
        """Set individual motor direction"""
        pins = self.motor_pins[motor]
        
        if direction == "forward":
            GPIO.output(pins['in1'], GPIO.HIGH)
            GPIO.output(pins['in2'], GPIO.LOW)
        elif direction == "backward":
            GPIO.output(pins['in1'], GPIO.LOW)
            GPIO.output(pins['in2'], GPIO.HIGH)
        elif direction == "stop":
            GPIO.output(pins['in1'], GPIO.LOW)
            GPIO.output(pins['in2'], GPIO.LOW)
    
    def set_motor_speed(self, motor, speed):
        """Set individual motor speed (0-100%)"""
        speed = max(0, min(100, speed))
        
        if motor == 'left_motor':
            self.left_pwm.ChangeDutyCycle(speed)
        elif motor == 'right_motor':
            self.right_pwm.ChangeDutyCycle(speed)
    
    def move_forward_distance(self, distance_cm, speed=None):
        """Move forward for specific distance with position tracking"""
        speed = speed or self.current_speed
        
        # Calculate time needed
        speed_factor = speed / 50.0
        time_needed = distance_cm / (self.cm_per_second_at_50_speed * speed_factor)
        
        # Move
        self.set_motor_direction('left_motor', 'forward')
        self.set_motor_direction('right_motor', 'forward')
        self.set_motor_speed('left_motor', speed)
        self.set_motor_speed('right_motor', speed)
        
        self.is_moving = True
        time.sleep(time_needed)
        self.stop()
        
        # Update mapper if available
        if self.mapper:
            # Convert heading to movement vector
            heading_rad = math.radians(self.mapper.robot_position.heading)
            dx = distance_cm * math.cos(heading_rad)
            dy = distance_cm * math.sin(heading_rad)
            self.mapper.update_robot_position(dx, dy, 0)
        
        logger.info(f"Moved forward {distance_cm}cm")
    
    def turn_degrees(self, degrees, speed=None):
        """Turn for specific degrees with position tracking"""
        speed = speed or 40  # Slower for precision
        
        # Calculate turn time
        time_needed = abs(degrees) / self.degrees_per_second_turn
        
        if degrees > 0:  # Turn right
            self.set_motor_direction('left_motor', 'forward')
            self.set_motor_direction('right_motor', 'backward')
        else:  # Turn left
            self.set_motor_direction('left_motor', 'backward')
            self.set_motor_direction('right_motor', 'forward')
        
        self.set_motor_speed('left_motor', speed)
        self.set_motor_speed('right_motor', speed)
        
        self.is_moving = True
        time.sleep(time_needed)
        self.stop()
        
        # Update mapper if available
        if self.mapper:
            self.mapper.update_robot_position(0, 0, degrees)
        
        logger.info(f"Turned {degrees} degrees")
    
    def navigate_to_grid_position(self, target_x, target_y):
        """Navigate to specific grid position"""
        if not self.mapper:
            logger.error("No mapper available for navigation")
            return False
        
        # Plan path
        path = self.mapper.plan_path_to_target(target_x, target_y)
        if not path:
            logger.warning("No path found to target")
            return False
        
        # Execute path
        for waypoint_x, waypoint_y in path:
            current_x = int(self.mapper.robot_position.x)
            current_y = int(self.mapper.robot_position.y)
            
            # Calculate movement needed
            dx = waypoint_x - current_x
            dy = waypoint_y - current_y
            
            # Calculate distance and angle
            distance_cells = math.sqrt(dx*dx + dy*dy)
            distance_cm = distance_cells * self.mapper.cell_size_cm
            
            if distance_cm > 0:
                target_angle = math.degrees(math.atan2(dy, dx))
                current_angle = self.mapper.robot_position.heading
                
                # Calculate turn needed
                angle_diff = (target_angle - current_angle + 180) % 360 - 180
                
                # Turn to face target
                if abs(angle_diff) > 5:  # 5-degree tolerance
                    self.turn_degrees(angle_diff)
                
                # Move forward
                self.move_forward_distance(distance_cm)
        
        logger.info(f"Navigated to grid position ({target_x}, {target_y})")
        return True
    
    def stop(self):
        """Stop all motors"""
        self.set_motor_direction('left_motor', 'stop')
        self.set_motor_direction('right_motor', 'stop')
        self.set_motor_speed('left_motor', 0)
        self.set_motor_speed('right_motor', 0)
        self.is_moving = False
    
    def cleanup(self):
        """Clean up GPIO resources"""
        self.stop()
        self.left_pwm.stop()
        self.right_pwm.stop()
        GPIO.cleanup()

class ClawController:
    """Controls the poop collection claw mechanism"""
    
    def __init__(self):
        # Servo pins for claw control
        self.claw_servo_pin = 17
        self.lift_servo_pin = 27
        
        GPIO.setup(self.claw_servo_pin, GPIO.OUT)
        GPIO.setup(self.lift_servo_pin, GPIO.OUT)
        
        # PWM for servos (50Hz for standard servos)
        self.claw_pwm = GPIO.PWM(self.claw_servo_pin, 50)
        self.lift_pwm = GPIO.PWM(self.lift_servo_pin, 50)
        
        self.claw_pwm.start(0)
        self.lift_pwm.start(0)
        
        # Servo positions (duty cycle values)
        self.claw_open_pos = 2.5   # Adjust based on your servo
        self.claw_closed_pos = 12.5
        self.lift_down_pos = 2.5
        self.lift_up_pos = 12.5
        
        # Initialize to starting position
        self.open_claw()
        self.lift_up()
    
    def set_servo_position(self, pwm, position):
        """Set servo to specific position"""
        pwm.ChangeDutyCycle(position)
        time.sleep(0.5)  # Allow time for servo to move
        pwm.ChangeDutyCycle(0)  # Stop sending signal
    
    def open_claw(self):
        """Open the claw"""
        self.set_servo_position(self.claw_pwm, self.claw_open_pos)
        logger.info("Claw opened")
    
    def close_claw(self):
        """Close the claw"""
        self.set_servo_position(self.claw_pwm, self.claw_closed_pos)
        logger.info("Claw closed")
    
    def lift_up(self):
        """Lift the claw up"""
        self.set_servo_position(self.lift_pwm, self.lift_up_pos)
        logger.info("Claw lifted up")
    
    def lift_down(self):
        """Lower the claw down"""
        self.set_servo_position(self.lift_pwm, self.lift_down_pos)
        logger.info("Claw lowered down")
    
    def collect_poop_sequence(self):
        """Execute full poop collection sequence"""
        logger.info("Starting poop collection sequence")
        
        # Lower claw
        self.lift_down()
        time.sleep(1)
        
        # Close claw to grab
        self.close_claw()
        time.sleep(1)
        
        # Lift up
        self.lift_up()
        time.sleep(1)
        
        logger.info("Poop collection sequence completed")
        return True
    
    def dispose_poop_sequence(self):
        """Execute poop disposal sequence"""
        logger.info("Starting poop disposal sequence")
        
        # Lower claw over disposal area
        self.lift_down()
        time.sleep(1)
        
        # Open claw to release
        self.open_claw()
        time.sleep(1)
        
        # Lift back up
        self.lift_up()
        time.sleep(1)
        
        logger.info("Poop disposal sequence completed")
    
    def cleanup(self):
        """Clean up servo resources"""
        self.claw_pwm.stop()
        self.lift_pwm.stop()

class PoopRobot:
    """Main robot controller integrating all systems"""
    
    def __init__(self):
        logger.info("Initializing Poop Robot...")
        
        # Initialize all subsystems
        self.mapper = EnvironmentMapper()
        self.motor_controller = RobotMotorController(self.mapper)
        self.sensor_controller = SensorController()
        self.camera_controller = CameraController()
        self.claw_controller = ClawController()
        
        # Robot state
        self.state = RobotState.IDLE
        self.is_running = False
        self.patrol_mode = False
        
        # Collection statistics
        self.poop_collected = 0
        self.mission_start_time = None
        
        logger.info("Poop Robot initialized successfully!")
    
    def start_mission(self, mode="explore"):
        """Start robot mission"""
        logger.info(f"Starting mission in {mode} mode")
        self.is_running = True
        self.mission_start_time = datetime.now()
        
        # Start camera capture
        self.camera_controller.start_continuous_capture()
        
        if mode == "explore":
            self.explore_and_map()
        elif mode == "patrol":
            self.patrol_mode = True
            self.patrol_yard()
        elif mode == "collect":
            self.collect_all_poop()
    
    def explore_and_map(self):
        """Explore the yard and build environment map"""
        self.state = RobotState.MAPPING
        logger.info("Starting yard exploration and mapping")
        
        while self.is_running:
            try:
                # Get sensor readings
                distances = self.sensor_controller.get_all_distances()
                
                # Check for obstacles
                if distances['front'] < 20:  # 20cm obstacle threshold
                    logger.info("Obstacle detected ahead")
                    # Mark obstacle in map
                    current_x = int(self.mapper.robot_position.x)
                    current_y = int(self.mapper.robot_position.y)
                    
                    # Calculate obstacle position
                    heading_rad = math.radians(self.mapper.robot_position.heading)
                    obs_x = current_x + int(2 * math.cos(heading_rad))
                    obs_y = current_y + int(2 * math.sin(heading_rad))
                    self.mapper.mark_obstacle(obs_x, obs_y)
                    
                    # Turn to avoid obstacle
                    self.motor_controller.turn_degrees(90)
                    continue
                
                # Take and analyze photo
                has_poop, confidence = self.camera_controller.analyze_image_for_poop()
                if has_poop and confidence > 0.7:
                    logger.info(f"Poop detected with confidence {confidence}")
                    current_x = int(self.mapper.robot_position.x)
                    current_y = int(self.mapper.robot_position.y)
                    self.mapper.mark_poop_location(current_x, current_y, confidence)
                
                # Find next exploration target
                target = self.mapper.get_next_exploration_target()
                if target is None:
                    logger.info("Exploration complete!")
                    break
                
                # Navigate to target
                target_x, target_y = target
                if self.motor_controller.navigate_to_grid_position(target_x, target_y):
                    logger.info(f"Reached exploration target ({target_x}, {target_y})")
                else:
                    logger.warning("Failed to reach exploration target")
                    # Try alternative movement
                    self.motor_controller.move_forward_distance(30)
                
                time.sleep(0.5)  # Brief pause
                
            except KeyboardInterrupt:
                logger.info("Exploration interrupted by user")
                break
            except Exception as e:
                logger.error(f"Error during exploration: {e}")
                time.sleep(1)
        
        # Save map
        self.mapper.save_map(f"yard_map_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json")
        self.state = RobotState.IDLE
    
    def patrol_yard(self):
        """Patrol the yard looking for new poop"""
        self.state = RobotState.PATROLLING
        logger.info("Starting yard patrol")
        
        patrol_pattern = [
            (50, 50), (150, 50), (150, 150), (50, 150),  # Rectangle pattern
            (100, 100)  # Return to center
        ]
        
        while self.is_running and self.patrol_mode:
            try:
                for target_x, target_y in patrol_pattern:
                    if not self.is_running:
                        break
                    
                    # Navigate to patrol point
                    if self.motor_controller.navigate_to_grid_position(target_x, target_y):
                        logger.info(f"Reached patrol point ({target_x}, {target_y})")
                        
                        # Look around for poop
                        for angle in [0, 90, 180, 270]:
                            self.motor_controller.turn_degrees(90)
                            time.sleep(1)
                            
                            # Check for poop
                            has_poop, confidence = self.camera_controller.analyze_image_for_poop()
                            if has_poop and confidence > 0.7:
                                logger.info("New poop detected during patrol!")
                                current_x = int(self.mapper.robot_position.x)
                                current_y = int(self.mapper.robot_position.y)
                                self.mapper.mark_poop_location(current_x, current_y, confidence)
                                
                                # Collect it immediately
                                self.collect_poop_at_current_location()
                    
                    time.sleep(2)  # Pause between patrol points
                
                time.sleep(10)  # Wait before next patrol cycle
                
            except KeyboardInterrupt:
                logger.info("Patrol interrupted by user")
                break
            except Exception as e:
                logger.error(f"Error during patrol: {e}")
                time.sleep(5)
        
        self.state = RobotState.IDLE
    
    def collect_all_poop(self):
        """Collect all known poop locations"""
        self.state = RobotState.COLLECTING
        poop_locations = self.mapper.get_poop_locations()
        
        if not poop_locations:
            logger.info("No poop locations found to collect")
            return
        
        logger.info(f"Starting collection of {len(poop_locations)} poop locations")
        
        for x, y, confidence in poop_locations:
            if not self.is_running:
                break
            
            try:
                logger.info(f"Navigating to poop location ({x}, {y})")
                
                # Navigate to poop location
                if self.motor_controller.navigate_to_grid_position(x, y):
                    # Verify poop is still there
                    has_poop, current_confidence = self.camera_controller.analyze_image_for_poop()
                    
                    if has_poop and current_confidence > 0.5:
                        # Collect the poop
                        if self.collect_poop_at_current_location():
                            self.poop_collected += 1
                            # Remove from map
                            self.mapper.grid[x][y].has_poop = False
                            logger.info(f"Successfully collected poop at ({x}, {y})")
                    else:
                        logger.info(f"Poop no longer detected at ({x}, {y}) - may have been moved")
                
                time.sleep(1)
                
            except Exception as e:
                logger.error(f"Error collecting poop at ({x}, {y}): {e}")
        
        # Return to start position
        self.return_to_start()
        self.state = RobotState.IDLE
        logger.info(f"Collection mission completed. Total poop collected: {self.poop_collected}")
    
    def collect_poop_at_current_location(self):
        """Collect poop at current robot location"""
        logger.info("Starting poop collection at current location")
        
        try:
            # Fine positioning - move slowly to center on poop
            self.fine_position_on_target()
            
            # Execute collection sequence
            success = self.claw_controller.collect_poop_sequence()
            
            if success:
                logger.info("Poop collection successful")
                return True
            else:
                logger.warning("Poop collection may have failed")
                return False
                
        except Exception as e:
            logger.error(f"Error during poop collection: {e}")
            return False
    
    def fine_position_on_target(self):
        """Fine-tune robot position to center on poop target"""
        logger.info("Fine positioning on target")
        
        # Take multiple images and adjust position
        for attempt in range(3):
            image = self.camera_controller.take_photo()
            if image is not None:
                # Here you could add computer vision to find poop center
                # and adjust robot position accordingly
                
                # For now, just small forward movement
                self.motor_controller.move_forward_distance(5, 30)  # 5cm at slow speed
                time.sleep(0.5)
    
    def return_to_start(self):
        """Return robot to starting position"""
        self.state = RobotState.RETURNING
        logger.info("Returning to start position")
        
        start_x = int(self.mapper.start_position.x)
        start_y = int(self.mapper.start_position.y)
        
        if self.motor_controller.navigate_to_grid_position(start_x, start_y):
            logger.info("Successfully returned to start position")
        else:
            logger.warning("Failed to return to exact start position")
        
        # Face original direction
        target_heading = self.mapper.start_position.heading
        current_heading = self.mapper.robot_position.heading
        angle_diff = (target_heading - current_heading + 180) % 360 - 180
        
        if abs(angle_diff) > 5:
            self.motor_controller.turn_degrees(angle_diff)
        
        self.state = RobotState.IDLE
    
    def emergency_stop(self):
        """Emergency stop all robot functions"""
        logger.warning("EMERGENCY STOP ACTIVATED")
        self.is_running = False
        self.patrol_mode = False
        self.motor_controller.stop()
        self.state = RobotState.IDLE
    
    def get_status(self):
        """Get current robot status"""
        status = {
            'state': self.state.value,
            'is_running': self.is_running,
            'position': {
                'x': self.mapper.robot_position.x,
                'y': self.mapper.robot_position.y,
                'heading': self.mapper.robot_position.heading
            },
            'poop_collected': self.poop_collected,
            'mapped_cells': self.mapper.mapped_cells,
            'total_cells': self.mapper.total_cells,
            'mapping_progress': (self.mapper.mapped_cells / self.mapper.total_cells) * 100
        }
        
        if self.mission_start_time:
            mission_duration = datetime.now() - self.mission_start_time
            status['mission_duration'] = str(mission_duration)
        
        return status
    
    def load_ai_model(self, model_path):
        """Load the poop detection AI model"""
        return self.camera_controller.load_ai_model(model_path)
    
    def save_mission_report(self):
        """Save detailed mission report"""
        report = {
            'mission_date': datetime.now().isoformat(),
            'mission_duration': str(datetime.now() - self.mission_start_time) if self.mission_start_time else None,
            'poop_collected': self.poop_collected,
            'mapping_progress': (self.mapper.mapped_cells / self.mapper.total_cells) * 100,
            'poop_locations': self.mapper.get_poop_locations(),
            'final_position': {
                'x': self.mapper.robot_position.x,
                'y': self.mapper.robot_position.y,
                'heading': self.mapper.robot_position.heading
            }
        }
        
        filename = f"mission_report_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
        with open(filename, 'w') as f:
            json.dump(report, f, indent=2)
        
        logger.info(f"Mission report saved to {filename}")
    
    def cleanup(self):
        """Clean up all robot resources"""
        logger.info("Shutting down robot systems...")
        
        self.emergency_stop()
        
        # Save final mission report
        if self.mission_start_time:
            self.save_mission_report()
        
        # Cleanup all subsystems
        self.camera_controller.cleanup()
        self.motor_controller.cleanup()
        self.claw_controller.cleanup()
        
        logger.info("Robot shutdown complete")

def main():
    """Main function demonstrating robot usage"""
    robot = PoopRobot()
    
    try:
        print("🤖 Dog Poop Picking Robot - Ready to Clean!")
        print("\nAvailable commands:")
        print("1. explore - Map the yard and find poop")
        print("2. patrol - Continuously patrol for new poop")
        print("3. collect - Collect all known poop locations")
        print("4. status - Show current robot status")
        print("5. stop - Emergency stop")
        print("6. quit - Shutdown robot")
        
        # Load AI model (replace with your model path)
        model_path = "poop_detection_model.tflite"  # Your trained model
        if robot.load_ai_model(model_path):
            print(f"✅ AI model loaded: {model_path}")
        else:
            print("⚠️ AI model not found - running in test mode")
        
        while True:
            command = input("\nEnter command: ").strip().lower()
            
            if command == "explore" or command == "1":
                print("🗺️ Starting exploration mission...")
                robot.start_mission("explore")
                
            elif command == "patrol" or command == "2":
                print("👁️ Starting patrol mission...")
                robot.start_mission("patrol")
                
            elif command == "collect" or command == "3":
                print("🦾 Starting collection mission...")
                robot.start_mission("collect")
                
            elif command == "status" or command == "4":
                status = robot.get_status()
                print(f"\n📊 Robot Status:")
                print(f"   State: {status['state']}")
                print(f"   Position: ({status['position']['x']:.1f}, {status['position']['y']:.1f})")
                print(f"   Heading: {status['position']['heading']:.1f}°")
                print(f"   Poop Collected: {status['poop_collected']}")
                print(f"   Mapping Progress: {status['mapping_progress']:.1f}%")
                if 'mission_duration' in status:
                    print(f"   Mission Duration: {status['mission_duration']}")
                
            elif command == "stop" or command == "5":
                print("🛑 Emergency stop activated!")
                robot.emergency_stop()
                
            elif command == "quit" or command == "6":
                print("👋 Shutting down robot...")
                break
                
            else:
                print("❓ Unknown command. Try: explore, patrol, collect, status, stop, or quit")
    
    except KeyboardInterrupt:
        print("\n🛑 Program interrupted by user")
    
    except Exception as e:
        print(f"❌ Error: {e}")
        logger.error(f"Main program error: {e}")
    
    finally:
        robot.cleanup()
        print("🔌 Robot systems shut down. Goodbye!")

if __name__ == "__main__":
    main()