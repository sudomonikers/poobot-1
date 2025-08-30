#!/usr/bin/env python3
"""
GPIO Mock Classes for Development
Mock implementations of Raspberry Pi GPIO for development on non-Pi systems
"""

import time
import random
import logging

logger = logging.getLogger(__name__)

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

# Create the GPIO instance to be imported
try:
    import RPi.GPIO as GPIO
    logger.info("Using real RPi.GPIO")
except ImportError:
    GPIO = MockGPIO()
    logger.info("Using MockGPIO for development")