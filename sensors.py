#!/usr/bin/env python3
"""
Sensor Controller Module
Handles ultrasonic sensors for obstacle detection and perimeter wire detection
"""

import time
import logging
from gpio_mock import GPIO

logger = logging.getLogger(__name__)

class SensorController:
    """Handles ultrasonic sensors for obstacle detection and perimeter wire detection"""
    
    def __init__(self):
        # Ultrasonic sensor pins (HC-SR04)
        self.sensors = {
            'front': {'trig': 24, 'echo': 23},
            'left': {'trig': 26, 'echo': 25},
            'right': {'trig': 6, 'echo': 5}
        }
        
        # Perimeter wire detection pins
        self.perimeter_pins = {
            'front_left': 7,
            'front_right': 8,
            'rear_left': 9,
            'rear_right': 10
        }
        
        # Setup GPIO for ultrasonic sensors
        for sensor in self.sensors.values():
            GPIO.setup(sensor['trig'], GPIO.OUT)
            GPIO.setup(sensor['echo'], GPIO.IN)
            GPIO.output(sensor['trig'], False)
        
        # Setup GPIO for perimeter wire sensors (analog inputs for wire detection)
        for pin in self.perimeter_pins.values():
            GPIO.setup(pin, GPIO.IN)
        
        # Perimeter detection calibration
        self.wire_threshold = 512  # Adjust based on your perimeter wire system
        self.wire_samples = 5      # Number of samples for stable reading
        
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
    
    def get_wire_signal_strength(self, sensor_name):
        """Get perimeter wire signal strength from specific sensor"""
        if sensor_name not in self.perimeter_pins:
            return 0
        
        pin = self.perimeter_pins[sensor_name]
        
        # Take multiple samples for stable reading
        samples = []
        for _ in range(self.wire_samples):
            # Read digital input (1 if wire signal detected, 0 if not)
            # In a real implementation, you might use an ADC for analog readings
            sample = GPIO.input(pin)
            samples.append(sample * 1023)  # Convert to 0-1023 scale for consistency
            time.sleep(0.01)  # Small delay between samples
        
        # Return average signal strength
        return sum(samples) / len(samples)
    
    def is_at_perimeter_wire(self, sensor_name=None):
        """Check if robot is at or near perimeter wire"""
        if sensor_name:
            # Check specific sensor
            signal = self.get_wire_signal_strength(sensor_name)
            return signal > self.wire_threshold
        else:
            # Check all perimeter sensors
            for name in self.perimeter_pins.keys():
                signal = self.get_wire_signal_strength(name)
                if signal > self.wire_threshold:
                    return True
            return False
    
    def get_all_wire_signals(self):
        """Get signal strength from all perimeter wire sensors"""
        signals = {}
        for name in self.perimeter_pins.keys():
            signals[name] = self.get_wire_signal_strength(name)
        return signals
    
    def get_wire_direction(self):
        """Determine which direction the perimeter wire is detected"""
        signals = self.get_all_wire_signals()
        detected_sensors = []
        
        for name, signal in signals.items():
            if signal > self.wire_threshold:
                detected_sensors.append(name)
        
        if not detected_sensors:
            return None
        
        # Determine general direction based on detected sensors
        if any('front' in sensor for sensor in detected_sensors):
            if any('left' in sensor for sensor in detected_sensors):
                return 'front_left'
            elif any('right' in sensor for sensor in detected_sensors):
                return 'front_right'
            else:
                return 'front'
        elif any('rear' in sensor for sensor in detected_sensors):
            if any('left' in sensor for sensor in detected_sensors):
                return 'rear_left'
            elif any('right' in sensor for sensor in detected_sensors):
                return 'rear_right'
            else:
                return 'rear'
        elif any('left' in sensor for sensor in detected_sensors):
            return 'left'
        elif any('right' in sensor for sensor in detected_sensors):
            return 'right'
        
        return 'multiple'  # Wire detected in multiple directions
    
    def calibrate_wire_threshold(self):
        """Calibrate the wire detection threshold by sampling current environment"""
        logger.info("Starting perimeter wire threshold calibration...")
        
        # Sample all sensors multiple times
        all_readings = []
        for _ in range(20):  # Take 20 samples
            for name in self.perimeter_pins.keys():
                reading = self.get_wire_signal_strength(name)
                all_readings.append(reading)
            time.sleep(0.1)
        
        if all_readings:
            max_reading = max(all_readings)
            min_reading = min(all_readings)
            avg_reading = sum(all_readings) / len(all_readings)
            
            # Set threshold to halfway between average and max
            # This assumes calibration is done away from the wire
            self.wire_threshold = (avg_reading + max_reading) / 2
            
            logger.info(f"Wire threshold calibrated to: {self.wire_threshold}")
            logger.info(f"Min: {min_reading}, Max: {max_reading}, Avg: {avg_reading}")
            
            return True
        
        logger.error("Failed to calibrate wire threshold - no readings obtained")
        return False