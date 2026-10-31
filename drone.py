import pygame
import math
from collections import deque
from config import DRONE_SPEED, MAP_WIDTH, WINDOW_HEIGHT, BLUE, CYAN, RED, GREEN, WHITE

class Drone:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.max_speed = DRONE_SPEED + 1.0
        
        self.angle = 0.0
        self.target_angle = 0.0
        self.rotation_speed = 0.14  
        self.radius = 12
        
        # Spinning rotor animation counter
        self.rotor_spin = 0.0
        
        # Motion trail history
        self.trail = deque(maxlen=28)
        
        # Telemetry & Auto-Docking Properties
        self.battery = 100.0
        self.home_x = 0.0
        self.home_y = 0.0
        self.is_docking = False
        self.geofence_breached = False

        # Hardware Upgrade Module Flags
        self.turbo_boost = False
        self.high_res_radar = False

    def update_physics(self, target_x=None, target_y=None, dist_map=None, grid_size=4, is_final=False, speed_limit=None):
        """
        Calculates inertial velocity vectors, rotation orientation, 
        smooth aerodynamic thrust, proportional arrival braking, and flight trail.
        """
        speed_modifier = 1.9 if self.turbo_boost else 1.0
        battery_drain = 0.035 if self.turbo_boost else 0.015
        
        base_max_speed = (DRONE_SPEED + 1.0) * speed_modifier
        self.max_speed = min(speed_limit, base_max_speed) if speed_limit is not None else base_max_speed

        # Drain battery slowly when active
        if self.battery > 0:
            if math.hypot(self.x - self.home_x, self.y - self.home_y) > 8 or abs(self.vx) > 0.1 or abs(self.vy) > 0.1:
                self.battery = max(0.0, self.battery - battery_drain)

        if target_x is not None and target_y is not None:
            dx = target_x - self.x
            dy = target_y - self.y
            dist = math.hypot(dx, dy)
            
            if dist > 0.5:
                move_angle = math.atan2(dy, dx)
                self.target_angle = math.degrees(move_angle)
                
                # Proportional smooth approach speed targeting
                if is_final and dist < 85:
                    desired_speed = max(0.4, (dist / 85.0) * self.max_speed * 0.65)
                    lerp_rate = 0.14
                elif speed_limit is not None:
                    desired_speed = self.max_speed
                    lerp_rate = 0.14
                else:
                    desired_speed = self.max_speed
                    lerp_rate = 0.10

                desired_vx = math.cos(move_angle) * desired_speed
                desired_vy = math.sin(move_angle) * desired_speed

                # Smooth aerodynamic thrust interpolation (eliminates high-torque jerks)
                self.vx += (desired_vx - self.vx) * lerp_rate
                self.vy += (desired_vy - self.vy) * lerp_rate
            else:
                self.vx *= 0.55
                self.vy *= 0.55
        else:
            self.vx *= 0.65
            self.vy *= 0.65

        # Active Obstacle & Corner Repulsion Cushion
        if dist_map is not None:
            map_h = len(dist_map)
            map_w = len(dist_map[0]) if map_h > 0 else 0
            gx = int(self.x // grid_size)
            gy = int(self.y // grid_size)
            if 0 <= gx < map_w and 0 <= gy < map_h:
                d_c = dist_map[gy][gx]
                if d_c < 5:
                    push_x, push_y = 0.0, 0.0
                    for ox, oy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                        nx, ny = gx + ox, gy + oy
                        if 0 <= nx < map_w and 0 <= ny < map_h:
                            diff = dist_map[ny][nx] - d_c
                            push_x += ox * diff
                            push_y += oy * diff
                    mag = math.hypot(push_x, push_y)
                    if mag > 0:
                        push_power = (5.0 - d_c) * 0.35
                        self.vx += (push_x / mag) * push_power
                        self.vy += (push_y / mag) * push_power
                if d_c <= 2:
                    self.vx *= 0.3
                    self.vy *= 0.3

        # Clamp speed
        speed = math.hypot(self.vx, self.vy)
        if speed > self.max_speed:
            self.vx = (self.vx / speed) * self.max_speed
            self.vy = (self.vy / speed) * self.max_speed

        # Apply raw position update
        self.x += self.vx
        self.y += self.vy

        # Safety boundary clamp
        self.x = max(self.radius + 2, min(MAP_WIDTH - self.radius - 2, self.x))
        self.y = max(self.radius + 2, min(WINDOW_HEIGHT - self.radius - 2, self.y))
        
        # Smooth rotational heading updates (soft natural bank)
        diff = (self.target_angle - self.angle + 180) % 360 - 180
        self.angle += diff * self.rotation_speed

        # Animate rotors
        spin_rate = 25.0 + speed * 10.0
        self.rotor_spin = (self.rotor_spin + spin_rate) % 360

        # Record position into trail when moving
        if speed > 0.2:
            self.trail.append((int(self.x), int(self.y)))
        elif len(self.trail) > 0:
            self.trail.popleft()

    def draw(self, surface):
        """
        Renders flight trail, animated quad-copter chassis, and spinning rotor discs.
        """
        # 1. Render Fading Flight Trail
        if len(self.trail) > 1:
            points = list(self.trail)
            for i in range(len(points) - 1):
                progress = i / len(points)
                alpha = int(progress * 130)
                thickness = max(1, int(progress * 3))
                trail_color = (0, int(180 * progress + 50), 255)
                pygame.draw.line(surface, trail_color, points[i], points[i + 1], thickness)

        pos_x, pos_y = int(self.x), int(self.y)
        rad = math.radians(self.angle)
        cos_r = math.cos(rad)
        sin_r = math.sin(rad)

        # 2. Quadcopter Arm Offsets (X-Chassis Layout)
        arm_dist = 13
        offsets = [(-arm_dist, -arm_dist), (arm_dist, -arm_dist), 
                   (-arm_dist, arm_dist), (arm_dist, arm_dist)]
        
        # Rotor blade angle in radians
        spin_rad = math.radians(self.rotor_spin)
        blade_len = 5

        for ox, oy in offsets:
            rx = pos_x + int(ox * cos_r - oy * sin_r)
            ry = pos_y + int(ox * sin_r + oy * cos_r)
            
            # Structural carbon strut
            pygame.draw.line(surface, (70, 75, 85), (pos_x, pos_y), (rx, ry), 3)
            # Rotor guard ring
            pygame.draw.circle(surface, (180, 190, 205), (rx, ry), 7, 1)

            # Animated spinning rotor blades
            bx1 = rx + int(blade_len * math.cos(spin_rad))
            by1 = ry + int(blade_len * math.sin(spin_rad))
            bx2 = rx - int(blade_len * math.cos(spin_rad))
            by2 = ry - int(blade_len * math.sin(spin_rad))
            pygame.draw.line(surface, (230, 240, 255), (bx1, by1), (bx2, by2), 2)
            
            # Rotor hub motor
            pygame.draw.circle(surface, (40, 45, 55), (rx, ry), 2)

        # 3. Central Core Chassis
        pygame.draw.circle(surface, (30, 35, 45), (pos_x, pos_y), self.radius)
        # Inner canopy glow
        canopy_color = CYAN if self.turbo_boost else BLUE
        pygame.draw.circle(surface, canopy_color, (pos_x, pos_y), self.radius - 3)
        pygame.draw.circle(surface, (220, 240, 255), (pos_x, pos_y), self.radius - 7)

        # 4. Status Indicator LED (Rapid red strobe on geofence breach / low battery)
        led_color = GREEN
        if getattr(self, 'geofence_breached', False):
            led_color = RED if (pygame.time.get_ticks() // 150) % 2 == 0 else (120, 0, 0)
        elif self.battery <= 20.0:
            led_color = RED if (pygame.time.get_ticks() // 200) % 2 == 0 else (100, 0, 0)
        elif self.is_docking:
            led_color = CYAN
        pygame.draw.circle(surface, led_color, (pos_x, pos_y), 3)

        # 5. Forward Orientation Indicator (Nose Cone Point)
        nx = pos_x + int((self.radius + 4) * cos_r)
        ny = pos_y + int((self.radius + 4) * sin_r)
        pygame.draw.circle(surface, WHITE, (nx, ny), 3)
