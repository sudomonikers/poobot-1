#!/usr/bin/env python3
"""
Claw Controller Module
Controls the poop collection claw mechanism with servo motors
"""

import time
import logging
from gpio_mock import GPIO

logger = logging.getLogger(__name__)

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