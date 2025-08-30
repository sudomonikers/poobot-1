#!/usr/bin/env python3
"""
Environment Mapper Module
Handles environment mapping and navigation for the robot
"""

import json
import logging
from dataclasses import dataclass
from enum import Enum

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