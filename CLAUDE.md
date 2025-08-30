# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a Raspberry Pi-based autonomous dog poop detection and collection robot. The system integrates computer vision, motor control, environmental mapping, and robotic manipulation to autonomously patrol a yard, detect dog poop using AI, and collect it using a claw mechanism.

## File Structure

The codebase is modularized into separate components:

- `bot.py` - Main robot controller and user interface
- `sensors.py` - Ultrasonic sensors and perimeter wire detection
- `motors.py` - Differential drive motor controller with position tracking  
- `camera.py` - Camera operations and AI model integration
- `mapper.py` - Grid-based environment mapping and navigation
- `claw.py` - Servo-controlled collection mechanism
- `gpio_mock.py` - Mock GPIO classes for development on non-Pi systems

## Development Commands

### Running the Robot
```bash
python3 bot.py
```

### Installing Dependencies
```bash
pip3 install -r requirements.txt
```

### Testing on Non-Pi Systems
The `gpio_mock.py` module provides MockGPIO and MockPWM classes for development on Mac/PC systems. The module automatically detects whether RPi.GPIO is available and uses the appropriate implementation.

## Architecture

### Core Systems Integration
The main `PoopRobot` class in `bot.py` coordinates five major subsystems:

1. **EnvironmentMapper** (`mapper.py`) - Grid-based mapping system (200x200 cells, 10cm per cell)
2. **RobotMotorController** (`motors.py`) - Differential drive control with position tracking
3. **SensorController** (`sensors.py`) - Ultrasonic sensors (HC-SR04) for obstacle detection and perimeter wire boundary detection
4. **CameraController** (`camera.py`) - Computer vision and AI model integration
5. **ClawController** (`claw.py`) - Servo-based collection mechanism

### State Machine
Robot operates in distinct states: IDLE, MAPPING, PATROLLING, INVESTIGATING, COLLECTING, RETURNING

### Mission Modes
- **explore** - Systematic yard mapping with poop detection and boundary detection
- **patrol** - Continuous monitoring using predefined patrol patterns
- **collect** - Navigate to all known poop locations for collection
- **calibrate** - Calibrate perimeter wire detection threshold

## Key Components

### Hardware Interface Layer
- GPIO pin assignments for motors, sensors, servos, camera, and perimeter wire detection
- PWM control for motor speeds and servo positioning
- Perimeter wire sensors on pins 7, 8, 9, 10 for boundary detection
- Mock implementations for development without hardware

### Navigation System
- Grid-based coordinate system with position tracking
- Simple pathfinding (A* placeholder for enhancement)
- Movement calibration constants for distance/angle accuracy
- Perimeter wire boundary detection with directional awareness and automatic avoidance

### Computer Vision Pipeline
- Continuous camera capture with background threading
- AI model integration (TensorFlow Lite placeholder)
- Confidence-based poop detection and mapping

### Data Persistence
- JSON-based map saving with obstacles, visited cells, and poop locations
- Mission reports with collection statistics and duration tracking

## Development Considerations

### Hardware Calibration Required
Key calibration constants in RobotMotorController:
- `cm_per_second_at_50_speed = 20` - Movement speed calibration
- `degrees_per_second_turn = 90` - Turn rate calibration

### AI Model Integration
The camera controller expects a TensorFlow Lite model at runtime. The placeholder code shows the expected interface for model loading and inference.

### Threading Model
Camera capture runs in a separate daemon thread for continuous image acquisition while main robot operations execute.

### Safety Systems
Emergency stop functionality immediately halts all motors and operations. GPIO cleanup ensures proper hardware state on shutdown.