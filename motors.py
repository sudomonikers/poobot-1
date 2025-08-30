#!/usr/bin/env python3
"""
Motor Controller Module
Enhanced motor controller with position tracking for differential drive robot
"""

import time
import math
import logging
from gpio_mock import GPIO

logger = logging.getLogger(__name__)

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