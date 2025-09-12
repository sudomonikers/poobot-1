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
    """Enhanced motor controller with position tracking for 4-wheel drive"""
    
    def __init__(self, mapper=None):
        # Motor GPIO pins for direct 4-wheel drive control
        self.motor_pins = {
            'front_left': {'positive': 1, 'negative': 2},
            'front_right': {'positive': 5, 'negative': 6},
            'rear_left': {'positive': 9, 'negative': 10},
            'rear_right': {'positive': 13, 'negative': 14}
        }
        
        # Setup GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        
        for motor in self.motor_pins.values():
            for pin in motor.values():
                GPIO.setup(pin, GPIO.OUT)
        
        # PWM setup for speed control on positive pins
        self.front_left_pwm = GPIO.PWM(self.motor_pins['front_left']['positive'], 1000)
        self.front_right_pwm = GPIO.PWM(self.motor_pins['front_right']['positive'], 1000)
        self.rear_left_pwm = GPIO.PWM(self.motor_pins['rear_left']['positive'], 1000)
        self.rear_right_pwm = GPIO.PWM(self.motor_pins['rear_right']['positive'], 1000)
        
        self.front_left_pwm.start(0)
        self.front_right_pwm.start(0)
        self.rear_left_pwm.start(0)
        self.rear_right_pwm.start(0)
        
        self.current_speed = 50
        self.is_moving = False
        self.mapper = mapper
        
        # Movement calibration (adjust based on your robot)
        self.cm_per_second_at_50_speed = 20  # Calibrate this
        self.degrees_per_second_turn = 90   # Calibrate this
    
    def set_motor_direction(self, motor, direction):
        """Set individual motor direction for direct GPIO control"""
        pins = self.motor_pins[motor]
        
        if direction == "forward":
            # Positive pin gets PWM, negative pin is LOW
            GPIO.output(pins['negative'], GPIO.LOW)
        elif direction == "backward":
            # Positive pin gets PWM, negative pin is HIGH (reversed polarity)
            GPIO.output(pins['negative'], GPIO.HIGH)
        elif direction == "stop":
            # Both pins LOW, motor stops
            GPIO.output(pins['negative'], GPIO.LOW)
            # PWM will be set to 0 in set_motor_speed
    
    def set_motor_speed(self, motor, speed):
        """Set individual motor speed (0-100%)"""
        speed = max(0, min(100, speed))
        
        if motor == 'front_left':
            self.front_left_pwm.ChangeDutyCycle(speed)
        elif motor == 'front_right':
            self.front_right_pwm.ChangeDutyCycle(speed)
        elif motor == 'rear_left':
            self.rear_left_pwm.ChangeDutyCycle(speed)
        elif motor == 'rear_right':
            self.rear_right_pwm.ChangeDutyCycle(speed)
        # Backward compatibility
        elif motor == 'left_motor':
            self.front_left_pwm.ChangeDutyCycle(speed)
            self.rear_left_pwm.ChangeDutyCycle(speed)
        elif motor == 'right_motor':
            self.front_right_pwm.ChangeDutyCycle(speed)
            self.rear_right_pwm.ChangeDutyCycle(speed)
    
    def set_left_motors(self, direction, speed=None):
        """Control both left motors"""
        speed = speed or self.current_speed
        self.set_motor_direction('front_left', direction)
        self.set_motor_direction('rear_left', direction)
        self.set_motor_speed('front_left', speed)
        self.set_motor_speed('rear_left', speed)
    
    def set_right_motors(self, direction, speed=None):
        """Control both right motors"""
        speed = speed or self.current_speed
        self.set_motor_direction('front_right', direction)
        self.set_motor_direction('rear_right', direction)
        self.set_motor_speed('front_right', speed)
        self.set_motor_speed('rear_right', speed)
    
    def set_all_motors(self, direction, speed=None):
        """Control all 4 motors"""
        speed = speed or self.current_speed
        for motor in self.motor_pins.keys():
            self.set_motor_direction(motor, direction)
            self.set_motor_speed(motor, speed)
    
    def move_forward_distance(self, distance_cm, speed=None):
        """Move forward for specific distance with position tracking"""
        speed = speed or self.current_speed
        
        # Calculate time needed
        speed_factor = speed / 50.0
        time_needed = distance_cm / (self.cm_per_second_at_50_speed * speed_factor)
        
        # Move all motors forward
        self.set_all_motors('forward', speed)
        
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
            self.set_left_motors('forward', speed)
            self.set_right_motors('backward', speed)
        else:  # Turn left
            self.set_left_motors('backward', speed)
            self.set_right_motors('forward', speed)
        
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
        self.set_all_motors('stop', 0)
        self.is_moving = False
    
    def cleanup(self):
        """Clean up GPIO resources"""
        self.stop()
        self.front_left_pwm.stop()
        self.front_right_pwm.stop()
        self.rear_left_pwm.stop()
        self.rear_right_pwm.stop()
        GPIO.cleanup()


if __name__ == "__main__":
    print("Testing 4-wheel drive motor controller...")
    
    # Initialize motor controller without mapper for testing
    motor_controller = RobotMotorController()
    
    try:
        print("Moving forward for 1 second...")
        motor_controller.set_all_motors('forward', 50)  # 50% speed
        time.sleep(1)
        motor_controller.stop()
        print("Test complete!")
        
    except KeyboardInterrupt:
        print("Test interrupted by user")
    except Exception as e:
        print(f"Error during test: {e}")
    finally:
        motor_controller.cleanup()
        print("GPIO cleanup complete")