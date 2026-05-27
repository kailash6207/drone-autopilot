import pygame
import math
from config import DRONE_SPEED

class Drone:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.max_speed = DRONE_SPEED + 1.0
        self.acceleration = 0.15
        
        # High-traction drag coefficient to stop cornering drift entirely
        self.friction = 0.80  
        
        self.angle = 0.0
        self.target_angle = 0.0
        self.rotation_speed = 0.15  
        self.radius = 12
        
        # Telemetry & Auto-Docking Properties
        self.battery = 100.0
        self.home_x = 0
        self.home_y = 0
        self.is_docking = False

        # Hardware Upgrade Module Flags
        self.turbo_boost = False
        self.high_res_radar = False

    def update_physics(self, target_x=None, target_y=None):
        """
        Calculates inertial velocity vectors and handles rotation orientation.
        """
        # Configure hardware upgrade modifiers dynamically
        speed_modifier = 3.0 if self.turbo_boost else 1.0
        battery_drain = 0.04 if self.turbo_boost else 0.015
        
        self.max_speed = (DRONE_SPEED + 1.0) * speed_modifier
        self.acceleration = 0.5 * speed_modifier

        # Drain battery slowly when active/moving
        if self.x != self.home_x or self.vx != 0:
            self.battery = max(0.0, self.battery - battery_drain)

        if target_x is not None and target_y is not None:
            # Determine vector heading to next node coordinate
            dx = target_x - self.x
            dy = target_y - self.y
            dist = math.sqrt(dx**2 + dy**2)
            
            if dist > 1:
                # Calculate movement angle heading
                move_angle = math.atan2(dy, dx)
                self.target_angle = math.degrees(move_angle)
                
                # Apply physics engine force acceleration
                self.vx += math.cos(move_angle) * self.acceleration
                self.vy += math.sin(move_angle) * self.acceleration
        
        # Clamp velocities to maximum performance caps
        speed = math.sqrt(self.vx**2 + self.vy**2)
        if speed > self.max_speed:
            self.vx = (self.vx / speed) * self.max_speed
            self.vy = (self.vy / speed) * self.max_speed
            
        # Apply environmental drag friction
        self.vx *= self.friction
        self.vy *= self.friction
        
        # Apply velocities to raw positions
        self.x += self.vx
        self.y += self.vy
        
        # Smooth rotational interpolation heading updates
        diff = (self.target_angle - self.angle + 180) % 360 - 180
        self.angle += diff * self.rotation_speed

    def draw(self, surface):
        """
        Renders a rotated quad-copter asset design onto the map surface.
        """
        pos_x, pos_y = int(self.x), int(self.y)
        rad = math.radians(self.angle)
        
        # Core Hub Chassis
        pygame.draw.circle(surface, (40, 50, 60), (pos_x, pos_y), self.radius)
        pygame.draw.circle(surface, (0, 150, 255), (pos_x, pos_y), self.radius - 4)
        
        # Rotated Propeller Strut Vectors (X-Chassis Layout)
        offsets = [(-12, -12), (12, -12), (-12, 12), (12, 12)]
        cos_r = math.cos(rad)
        sin_r = math.sin(rad)
        
        for ox, oy in offsets:
            rx = pos_x + int(ox * cos_r - oy * sin_r)
            ry = pos_y + int(ox * sin_r + oy * cos_r)
            
            pygame.draw.line(surface, (80, 90, 100), (pos_x, pos_y), (rx, ry), 3)
            pygame.draw.circle(surface, (200, 200, 200), (rx, ry), 6, 1)  # Rotor Guard Ring
            
        # Directional Nose Cone Point (Forward Orientation Indicator)
        nx = pos_x + int((self.radius + 4) * cos_r)
        ny = pos_y + int((self.radius + 4) * sin_r)
        pygame.draw.circle(surface, (255, 255, 255), (nx, ny), 4)