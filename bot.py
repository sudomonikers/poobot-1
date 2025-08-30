#!/usr/bin/env python3
"""
Dog Poop Picking Robot - Main Controller
Comprehensive Raspberry Pi code with motor control, camera, mapping, sensors, and AI integration
"""

import time
import math
import json
from datetime import datetime
import logging

# Import our custom modules
from mapper import EnvironmentMapper, RobotState
from motors import RobotMotorController
from sensors import SensorController
from camera import CameraController
from claw import ClawController

# Setup logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

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
                
                # Check for perimeter wire first (yard boundary)
                if self.sensor_controller.is_at_perimeter_wire():
                    wire_direction = self.sensor_controller.get_wire_direction()
                    logger.info(f"Perimeter wire detected: {wire_direction}")
                    
                    # Mark current position as yard boundary in map
                    current_x = int(self.mapper.robot_position.x)
                    current_y = int(self.mapper.robot_position.y)
                    self.mapper.mark_obstacle(current_x, current_y)  # Treat boundary as obstacle
                    
                    # Turn away from perimeter wire
                    if wire_direction in ['front', 'front_left', 'front_right']:
                        self.motor_controller.turn_degrees(180)  # Turn around
                    elif wire_direction in ['left', 'rear_left']:
                        self.motor_controller.turn_degrees(90)   # Turn right
                    elif wire_direction in ['right', 'rear_right']:
                        self.motor_controller.turn_degrees(-90)  # Turn left
                    else:
                        self.motor_controller.turn_degrees(135)  # Turn away from multiple directions
                    continue
                
                # Check for physical obstacles
                if distances['front'] < 20:  # 20cm obstacle threshold
                    logger.info("Physical obstacle detected ahead")
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
        print("5. calibrate - Calibrate perimeter wire detection")
        print("6. stop - Emergency stop")
        print("7. quit - Shutdown robot")
        
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
                
                # Show perimeter wire status
                wire_signals = robot.sensor_controller.get_all_wire_signals()
                print(f"\n🔌 Perimeter Wire Signals:")
                for sensor, signal in wire_signals.items():
                    status_icon = "🟢" if signal > robot.sensor_controller.wire_threshold else "🔴"
                    print(f"   {sensor}: {signal:.1f} {status_icon}")
                print(f"   Threshold: {robot.sensor_controller.wire_threshold:.1f}")
                
            elif command == "calibrate" or command == "5":
                print("🎯 Starting perimeter wire calibration...")
                print("Make sure robot is positioned AWAY from the perimeter wire!")
                input("Press Enter when ready to calibrate...")
                
                if robot.sensor_controller.calibrate_wire_threshold():
                    print(f"✅ Calibration complete! Threshold set to: {robot.sensor_controller.wire_threshold:.1f}")
                else:
                    print("❌ Calibration failed!")
                
            elif command == "stop" or command == "6":
                print("🛑 Emergency stop activated!")
                robot.emergency_stop()
                
            elif command == "quit" or command == "7":
                print("👋 Shutting down robot...")
                break
                
            else:
                print("❓ Unknown command. Try: explore, patrol, collect, status, calibrate, stop, or quit")
    
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