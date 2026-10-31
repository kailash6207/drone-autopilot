import warnings
warnings.filterwarnings("ignore")

import pygame
import math
from collections import deque

from config import *
from drone import Drone
from image_processing import process_floorplan
from pathfinding import astar, invalidate_distance_cache, get_distance_map

pygame.init()
pygame.font.init()

# Setup Core Fonts
FONT_TITLE = pygame.font.SysFont("Consolas", 15, bold=True)
FONT_BOLD = pygame.font.SysFont("Consolas", 14, bold=True)
FONT_MAIN = pygame.font.SysFont("Consolas", 12)
FONT_SMALL = pygame.font.SysFont("Consolas", 11)
FONT_NUM = pygame.font.SysFont("Consolas", 10, bold=True)

screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
pygame.display.set_caption("AI Autonomous Drone - Command Console & Autopilot")
clock = pygame.time.Clock()

# Systems Setup
drone = Drone()
drone_placed = False 

binary_map = process_floorplan("images/house.png")
original_binary_map = [row.copy() for row in binary_map]
floorplan_image = pygame.image.load("images/house.png")
floorplan_image = pygame.transform.scale(floorplan_image, (MAP_WIDTH, WINDOW_HEIGHT))

path = []
target = None
obstacles = []

# Tracker to capture the closest the drone got to its current target
min_distance_recorded = float('inf')

# Timestamp tracker for the destination hover delay
destination_hold_time = 0  

# Destination Ring Indicator Animation Properties
ring_radius = 0
ring_alpha = 255

# Exact 7-Room Comprehensive Surveillance Circuit (Exactly One Waypoint per Physical Room)
WAYPOINTS = [
    (150, 125), # 1. Top-Left Study (Northwest Wing)
    (500, 210), # 2. Central Living Room (Upper Center Hall)
    (830, 125), # 3. Top-Right Bedroom (Northeast Wing)
    (830, 300), # 4. Kitchen & Dining (East Wing)
    (500, 460), # 5. Central Dining Hall (Lower Center Hall)
    (175, 520), # 6. Master Bedroom Suite (Southwest Wing)
    (175, 300), # 7. Mid-Left Family Lounge (West Central Wing)
]

WAYPOINT_NAMES = [
    "Top-Left Study",
    "Central Living Room",
    "Top-Right Bedroom",
    "Kitchen & Dining",
    "Central Dining Hall",
    "Master Bedroom",
    "Mid-Left Lounge",
]
patrol_queue = []
patrol_mode = False
sensor_collision = False

# Interactive UI Button Boundaries in Sidebar
btn_turbo_rect  = pygame.Rect(MAP_WIDTH + 18, 360, 244, 30)
btn_radar_rect  = pygame.Rect(MAP_WIDTH + 18, 396, 244, 30)
btn_patrol_rect = pygame.Rect(MAP_WIDTH + 18, 432, 244, 30)
btn_rth_rect    = pygame.Rect(MAP_WIDTH + 18, 468, 244, 30)
btn_clear_rect  = pygame.Rect(MAP_WIDTH + 18, 504, 244, 30)

def is_inside_house_blueprint(x, y):
    """
    Checks whether coordinates (x, y) are inside the architectural perimeter.
    Returns False when the drone leaves the house blueprint area.
    """
    if x < 48 or y < 62:
        return False
    if y <= 420:
        return x <= 952
    elif y <= 595:
        return x <= 706
    elif y <= 636:
        return x <= 310
    return False

def trigger_rth():
    """Commands the drone to abort and return to docking charging pad."""
    global target, path, min_distance_recorded, destination_hold_time, patrol_mode
    if drone_placed and (drone.home_x != 0 or drone.home_y != 0):
        drone.is_docking = True
        patrol_mode = False
        target = (drone.home_x, drone.home_y)
        min_distance_recorded = float('inf')
        destination_hold_time = 0
        start_node = (int(drone.x // GRID_SIZE), int(drone.y // GRID_SIZE))
        goal_node = (int(drone.home_x // GRID_SIZE), int(drone.home_y // GRID_SIZE))
        path = astar(binary_map, start_node, goal_node, GRID_SIZE)

def clear_all_obstacles():
    """Resets dynamic obstacles and recalculates active flight path."""
    global binary_map, obstacles, path
    binary_map = [row.copy() for row in original_binary_map]
    obstacles.clear()
    invalidate_distance_cache()
    if drone_placed and target:
        start_node = (int(drone.x // GRID_SIZE), int(drone.y // GRID_SIZE))
        goal_node = (int(target[0] // GRID_SIZE), int(target[1] // GRID_SIZE))
        path = astar(binary_map, start_node, goal_node, GRID_SIZE)

running = True

while running:
    # Clear screen frame backgrounds
    screen.fill(PANEL_BG)
    screen.blit(floorplan_image, (0, 0))

    distance_to_home = math.hypot(drone.x - drone.home_x, drone.y - drone.home_y) if drone_placed else 0.0

    # ==========================================
    # EVENTS MANAGEMENT & INPUT HANDLING
    # ==========================================
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

        # Keyboard Shortcuts
        if event.type == pygame.KEYDOWN and drone_placed:
            if event.key == pygame.K_p and (not drone.is_docking or drone.battery >= 35.0):
                patrol_mode = not patrol_mode
                if patrol_mode:
                    drone.is_docking = False  
                    patrol_queue = WAYPOINTS.copy()
                    target = None
                    path = []
                    min_distance_recorded = float('inf')
                    destination_hold_time = 0
                else:
                    patrol_queue, path = [], []
                    target = None
            elif event.key == pygame.K_h:
                trigger_rth()
            elif event.key == pygame.K_c:
                clear_all_obstacles()
            elif event.key == pygame.K_t:
                drone.turbo_boost = not drone.turbo_boost
            elif event.key == pygame.K_r:
                drone.high_res_radar = not drone.high_res_radar

        # Mouse Clicks
        if event.type == pygame.MOUSEBUTTONDOWN:
            mx, my = pygame.mouse.get_pos()
            
            # SIDEBAR DASHBOARD INTERACTIONS
            if mx >= MAP_WIDTH and drone_placed:
                if btn_turbo_rect.collidepoint(mx, my):
                    drone.turbo_boost = not drone.turbo_boost
                elif btn_radar_rect.collidepoint(mx, my):
                    drone.high_res_radar = not drone.high_res_radar
                elif btn_patrol_rect.collidepoint(mx, my):
                    if not drone.is_docking or drone.battery >= 35.0:
                        patrol_mode = not patrol_mode
                        if patrol_mode:
                            drone.is_docking = False
                            patrol_queue = WAYPOINTS.copy()
                            target = None
                            path = []
                            min_distance_recorded = float('inf')
                            destination_hold_time = 0
                        else:
                            patrol_queue, path = [], []
                            target = None
                elif btn_rth_rect.collidepoint(mx, my):
                    trigger_rth()
                elif btn_clear_rect.collidepoint(mx, my):
                    clear_all_obstacles()

            # MAP CANVAS INTERACTIONS
            elif 0 <= mx < MAP_WIDTH and 0 <= my < WINDOW_HEIGHT:
                # Spawn base station and drone on first left-click
                if not drone_placed:
                    if event.button == 1: 
                        if binary_map[my][mx] == 0:
                            drone.x, drone.y = float(mx), float(my)
                            drone.home_x, drone.home_y = float(mx), float(my)
                            drone_placed = True
                else:
                    # LEFT-CLICK: Spawn grid-locked obstacle block
                    if event.button == 1 and not drone.is_docking:
                        center_gx = mx // GRID_SIZE
                        center_gy = my // GRID_SIZE
                        cell_radius = 4 
                        
                        start_gx = max(0, center_gx - cell_radius)
                        end_gx = min(MAP_WIDTH // GRID_SIZE, center_gx + cell_radius)
                        start_gy = max(0, center_gy - cell_radius)
                        end_gy = min(WINDOW_HEIGHT // GRID_SIZE, center_gy + cell_radius)
                        
                        home_gx = int(drone.home_x // GRID_SIZE)
                        home_gy = int(drone.home_y // GRID_SIZE)
                        
                        if start_gx <= home_gx < end_gx and start_gy <= home_gy < end_gy:
                            print("Placement Rejected: Cannot block charging pad area!")
                        else:
                            for gy in range(start_gy, end_gy):
                                for gx in range(start_gx, end_gx):
                                    for px in range(gx * GRID_SIZE, (gx + 1) * GRID_SIZE):
                                        for py in range(gy * GRID_SIZE, (gy + 1) * GRID_SIZE):
                                            if 0 <= px < MAP_WIDTH and 0 <= py < WINDOW_HEIGHT:
                                                binary_map[py][px] = 1
                            
                            new_obstacle = pygame.Rect(
                                start_gx * GRID_SIZE, 
                                start_gy * GRID_SIZE, 
                                (end_gx - start_gx) * GRID_SIZE, 
                                (end_gy - start_gy) * GRID_SIZE
                            )
                            obstacles.append(new_obstacle)
                            invalidate_distance_cache()
                            
                            if target:
                                start_node = (int(drone.x // GRID_SIZE), int(drone.y // GRID_SIZE))
                                goal_node = (int(target[0] // GRID_SIZE), int(target[1] // GRID_SIZE))
                                path = astar(binary_map, start_node, goal_node, GRID_SIZE)

                    # RIGHT-CLICK: Set Navigation Target Destination
                    elif event.button == 3 and not patrol_mode:
                        if binary_map[my][mx] == 0:
                            if not drone.is_docking or drone.battery >= 35.0:
                                drone.is_docking = False  
                                target = (mx, my)
                                path = []
                                min_distance_recorded = float('inf')
                                destination_hold_time = 0
                                ring_radius = 0
                                ring_alpha = 255
                                start_node = (int(drone.x // GRID_SIZE), int(drone.y // GRID_SIZE))
                                goal_node = (int(mx // GRID_SIZE), int(my // GRID_SIZE))
                                path = astar(binary_map, start_node, goal_node, GRID_SIZE)

    # ==========================================
    # CORE ENGINE CALCULATIONS & AUTOPILOT
    # ==========================================
    if drone_placed:
        # Battery Die Safeguard
        if drone.battery <= 0.0:
            drone.x, drone.y = drone.home_x, drone.home_y
            drone.vx, drone.vy = 0.0, 0.0
            drone.is_docking = True
            patrol_mode = False
            target, path = None, []
            destination_hold_time = 0

        # Low Battery Automatic RTH Return
        if drone.battery <= 20.0 and drone.battery > 0 and not drone.is_docking and distance_to_home > 15:
            trigger_rth()

        # Proximity Charging Pad Connection
        if distance_to_home < 8:
            drone.battery = min(100.0, drone.battery + 0.35)
            if drone.battery >= 100.0 and drone.is_docking:
                drone.is_docking = False
                target = None
                destination_hold_time = 0

        # Autonomous Patrol Mode Loop Management (Initialize target if not set)
        if patrol_mode and target is None and len(patrol_queue) > 0:
            next_destination = patrol_queue.pop(0)
            target = next_destination
            min_distance_recorded = float('inf')
            destination_hold_time = 0
            ring_radius = 0
            ring_alpha = 255
            start_node = (int(drone.x // GRID_SIZE), int(drone.y // GRID_SIZE))
            goal_node = (int(target[0] // GRID_SIZE), int(target[1] // GRID_SIZE))
            path = astar(binary_map, start_node, goal_node, GRID_SIZE)

        # Follow Waypoints Along Path
        target_node_x, target_node_y = None, None
        speed_limit = None
        
        if len(path) > 0:
            next_node = path[0]
            target_node_x = (next_node[0] * GRID_SIZE) + (GRID_SIZE // 2)
            target_node_y = (next_node[1] * GRID_SIZE) + (GRID_SIZE // 2)

            # Lookahead corner braking & tight apex tracking
            current_speed = math.hypot(drone.vx, drone.vy)
            if len(path) > 1:
                future_node = path[1]
                fx = (future_node[0] * GRID_SIZE) + (GRID_SIZE // 2)
                fy = (future_node[1] * GRID_SIZE) + (GRID_SIZE // 2)
                
                angle_now = math.atan2(target_node_y - drone.y, target_node_x - drone.x)
                angle_future = math.atan2(fy - target_node_y, fx - target_node_x)
                angle_diff = abs((angle_future - angle_now + math.pi) % (2 * math.pi) - math.pi)
                
                if angle_diff > 0.35:
                    # Smooth natural corner transit
                    speed_limit = 4.8 if drone.turbo_boost else 3.6
                    arrival_radius = 8
                else:
                    arrival_radius = 12 + int(current_speed * 1.2)
            else:
                arrival_radius = 5

            if math.hypot(target_node_x - drone.x, target_node_y - drone.y) <= arrival_radius:
                path.pop(0)

        # Destination Arrival & Precision Hover Hold
        elif target is not None:
            dist_to_final_target = math.hypot(target[0] - drone.x, target[1] - drone.y)
            
            if dist_to_final_target < min_distance_recorded:
                min_distance_recorded = dist_to_final_target

            # In flight approach phase
            if dist_to_final_target > 4.0 and destination_hold_time == 0:
                target_node_x, target_node_y = target[0], target[1]
            else:
                # Reached destination: actively arrest velocity and lock position!
                target_node_x, target_node_y = target[0], target[1]
                drone.vx *= 0.50
                drone.vy *= 0.50
                if math.hypot(drone.vx, drone.vy) < 0.15:
                    drone.vx = 0.0
                    drone.vy = 0.0
                    # Position lock anchor
                    drone.x += (target[0] - drone.x) * 0.35
                    drone.y += (target[1] - drone.y) * 0.35

                if destination_hold_time == 0:
                    destination_hold_time = pygame.time.get_ticks()

                if patrol_mode:
                    # Conduct an 800ms room surveillance scan, then advance to next room
                    if pygame.time.get_ticks() - destination_hold_time >= 800:
                        destination_hold_time = 0
                        if len(patrol_queue) > 0:
                            next_destination = patrol_queue.pop(0)
                            target = next_destination
                            min_distance_recorded = float('inf')
                            ring_radius = 0
                            ring_alpha = 255
                            start_node = (int(drone.x // GRID_SIZE), int(drone.y // GRID_SIZE))
                            goal_node = (int(target[0] // GRID_SIZE), int(target[1] // GRID_SIZE))
                            path = astar(binary_map, start_node, goal_node, GRID_SIZE)
                        else:
                            # Completed patrolling every room in the house once: return to dock & stop
                            trigger_rth()
                elif distance_to_home > 12:
                    if pygame.time.get_ticks() - destination_hold_time >= 2000:
                        trigger_rth()
                else:
                    target = None
                    destination_hold_time = 0

        # Apply Physics Forces with Proportional Arrival Deceleration
        if drone.battery > 0.0:
            dist_map = get_distance_map(binary_map, GRID_SIZE)
            is_final_approach = (len(path) <= 1) and (target is not None)
            drone.update_physics(target_node_x, target_node_y, dist_map, GRID_SIZE, is_final=is_final_approach, speed_limit=speed_limit)

        # 360-Degree Lidar Radar Sweep
        max_scan_range = 220 if drone.high_res_radar else 125
        sensor_collision = False
        map_height = len(binary_map)
        map_width = len(binary_map[0])
        radar_sectors = [False] * 8  

        for angle in range(0, 360, 15):
            radians = math.radians(angle)
            sector_index = int((angle + 22.5) % 360) // 45
            cos_a = math.cos(radians)
            sin_a = math.sin(radians)

            hit = False
            for distance in range(16, max_scan_range, 7):
                sx = int(drone.x + cos_a * distance)
                sy = int(drone.y + sin_a * distance)

                if sx < 0 or sy < 0 or sx >= map_width or sy >= map_height:
                    hit = True
                    break
                if binary_map[sy][sx] > 0:
                    pygame.draw.circle(screen, RED, (sx, sy), 3)
                    sensor_collision = True
                    radar_sectors[sector_index] = True
                    hit = True
                    break
            
            if not hit:
                tip_x = int(drone.x + cos_a * max_scan_range)
                tip_y = int(drone.y + sin_a * max_scan_range)
                pygame.draw.circle(screen, (0, 120, 220), (tip_x, tip_y), 1)

        # ==========================================
        # VISUAL RENDER PIPELINE
        # ==========================================
        # Render Obstacles
        for obs in obstacles:
            pygame.draw.rect(screen, (70, 45, 10), obs)
            pygame.draw.rect(screen, AMBER, obs, 2)

        # Render Base Station Charging Pad
        pygame.draw.rect(screen, DARK_GRAY, (drone.home_x - 14, drone.home_y - 14, 28, 28))
        pygame.draw.rect(screen, CYAN if distance_to_home < 8 else AMBER, (drone.home_x - 14, drone.home_y - 14, 28, 28), 2)
        pygame.draw.circle(screen, CYAN if distance_to_home < 8 else AMBER, (int(drone.home_x), int(drone.home_y)), 5)

        # Render Planned Path Flight Route
        if len(path) > 0:
            waypoint_pts = [(int(drone.x), int(drone.y))]
            for node in path:
                wx = (node[0] * GRID_SIZE) + (GRID_SIZE // 2)
                wy = (node[1] * GRID_SIZE) + (GRID_SIZE // 2)
                waypoint_pts.append((wx, wy))
            
            if len(waypoint_pts) > 1:
                pygame.draw.lines(screen, CYAN, False, waypoint_pts, 2)
            
            for pt in waypoint_pts[1:]:
                pygame.draw.circle(screen, CYAN, pt, 5, 1)
                pygame.draw.circle(screen, WHITE, pt, 2)

        # Render Patrol Mode Room Waypoint Markers
        if patrol_mode:
            for idx, wpt in enumerate(WAYPOINTS):
                is_active_target = (target == wpt)
                node_col = GREEN if is_active_target else (0, 160, 220)
                pygame.draw.circle(screen, node_col, wpt, 6, 2 if not is_active_target else 3)
                pygame.draw.circle(screen, WHITE if is_active_target else (100, 200, 255), wpt, 2)
                badge_txt = FONT_NUM.render(str(idx + 1), True, (200, 240, 255) if not is_active_target else GREEN)
                screen.blit(badge_txt, (wpt[0] + 8, wpt[1] - 8))

        # Render Target Marker & Pulsing Ring
        if target:
            target_dot_col = GREEN if (patrol_mode and destination_hold_time > 0) else RED
            pygame.draw.circle(screen, target_dot_col, target, 6)
            if ring_alpha > 0:
                surf = pygame.Surface((100, 100), pygame.SRCALPHA)
                ring_col = (0, 255, 120, ring_alpha) if (patrol_mode and destination_hold_time > 0) else (255, 60, 60, ring_alpha)
                pygame.draw.circle(surf, ring_col, (50, 50), ring_radius, 2)
                screen.blit(surf, (target[0] - 50, target[1] - 50))
                ring_radius = (ring_radius + 2) % 40
                ring_alpha = max(0, ring_alpha - 5)
            else:
                ring_radius = 0
                ring_alpha = 255

        # Render 8-Sector Radar Shield Arc Around Drone
        radar_radius = 38
        pos_x, pos_y = int(drone.x), int(drone.y)
        for i in range(8):
            start_angle = math.radians(i * 45 - 22.5)
            stop_angle = math.radians((i + 1) * 45 - 22.5)
            ring_color = RED if radar_sectors[i] else GREEN
            arc_rect = pygame.Rect(pos_x - radar_radius, pos_y - radar_radius, radar_radius * 2, radar_radius * 2)
            pygame.draw.arc(screen, ring_color, arc_rect, -stop_angle, -start_angle, 2)

        # Render Animated Drone Asset
        drone.draw(screen)

        # Warning Signal for Perimeter Breach (Outside House Blueprint)
        if geofence_breached:
            flash_on = (pygame.time.get_ticks() // 280) % 2 == 0
            
            # 1. Flashing Red Warning Border around the viewport
            if flash_on:
                pygame.draw.rect(screen, RED, (0, 0, MAP_WIDTH, WINDOW_HEIGHT), 5)

            # 2. Pulsing Warning Halo Ring expanding from the Drone
            halo_r = 20 + int((pygame.time.get_ticks() // 15) % 28)
            halo_surf = pygame.Surface((120, 120), pygame.SRCALPHA)
            pygame.draw.circle(halo_surf, (255, 30, 30, 180), (60, 60), halo_r, 2)
            screen.blit(halo_surf, (int(drone.x) - 60, int(drone.y) - 60))

            # 3. Flashing Top-Center HUD Alarm Banner
            banner_w, banner_h = 360, 36
            banner_x = (MAP_WIDTH - banner_w) // 2
            banner_y = 16
            banner_bg = (170, 15, 15) if flash_on else (80, 10, 10)
            pygame.draw.rect(screen, banner_bg, (banner_x, banner_y, banner_w, banner_h), 0, 6)
            pygame.draw.rect(screen, RED, (banner_x, banner_y, banner_w, banner_h), 2, 6)
            
            warn_txt = FONT_BOLD.render("[!] WARNING: DRONE OUT OF HOUSE!", True, WHITE)
            screen.blit(warn_txt, (banner_x + (banner_w - warn_txt.get_width()) // 2, banner_y + 11))

    # ==========================================
    # TELEMETRY GLASS COCKPIT SIDEBAR UI
    # ==========================================
    sidebar_x = MAP_WIDTH
    sidebar_w = WINDOW_WIDTH - MAP_WIDTH
    pygame.draw.rect(screen, PANEL_BG, (sidebar_x, 0, sidebar_w, WINDOW_HEIGHT))
    pygame.draw.line(screen, BLUE, (sidebar_x, 0), (sidebar_x, WINDOW_HEIGHT), 2)

    # Header
    screen.blit(FONT_TITLE.render("DRONE MISSION CONSOLE", True, CYAN), (sidebar_x + 18, 20))
    pygame.draw.line(screen, DARK_GRAY, (sidebar_x + 18, 44), (WINDOW_WIDTH - 18, 44), 1)

    # Flight State Mode Determination (Blueprint Geofence Check)
    geofence_breached = False
    if drone_placed:
        if not is_inside_house_blueprint(drone.x, drone.y):
            geofence_breached = True
        drone.geofence_breached = geofence_breached

    if geofence_breached:
        flash_text = (pygame.time.get_ticks() // 300) % 2 == 0
        mode_text = "OUT OF HOUSE"
        mode_color = RED if flash_text else (190, 60, 60)
    elif patrol_mode:
        if target in WAYPOINTS:
            wpt_idx = WAYPOINTS.index(target)
            status_tag = " [SCANNING]" if destination_hold_time > 0 else f" [{wpt_idx + 1}/{len(WAYPOINTS)}]"
            mode_text = f"PATROL: {WAYPOINT_NAMES[wpt_idx]}{status_tag}"
        else:
            mode_text = "AUTO PATROL ALL ROOMS"
        mode_color = GREEN
    elif destination_hold_time > 0:
        mode_text, mode_color = "OBJECTIVE HOVER DELAY", BLUE
    elif drone.is_docking:
        if distance_to_home < 8 and drone.battery >= 35.0:
            mode_text, mode_color = "STANDBY (READY TO FLY)", BLUE
        elif distance_to_home < 8:
            mode_text, mode_color = "FAST INDUCTIVE CHARGE", AMBER
        else:
            mode_text, mode_color = "RETURNING TO CHARGER", AMBER
    elif drone_placed:
        mode_text, mode_color = "MANUAL AUTOPILOT", TEXT_WHITE
    else:
        mode_text, mode_color = "AWAITING DEPLOYMENT", GRAY

    current_speed_kph = math.hypot(drone.vx, drone.vy) * 10 if drone_placed else 0.0

    # Telemetry Rows
    sys_status_str = "OUT OF HOUSE!" if geofence_breached else ("ONLINE" if drone_placed else "OFFLINE")
    sys_status_col = RED if (not drone_placed or geofence_breached) else GREEN
    radar_det_str = "OUT OF HOUSE!" if geofence_breached else ("CRITICAL PROXIMITY" if sensor_collision else "CLEAR ENROUTE")

    telemetry_data = [
        ("SYSTEM STATUS:", sys_status_str, sys_status_col),
        ("FLIGHT MODE:", mode_text, mode_color),
        ("TELEMETRY SPEED:", f"{current_speed_kph:.1f} km/h", TEXT_WHITE),
        ("DISTANCE TO DOCK:", f"{distance_to_home * 0.1:.1f} m" if drone_placed else "0.0 m", TEXT_WHITE),
        ("RADAR DETECTIONS:", radar_det_str, RED if (sensor_collision or geofence_breached) else GREEN),
        ("ACTIVE WAYPOINTS:", f"{len(path)} remaining" if len(path) > 0 else "Direct / Holding", CYAN if len(path) > 0 else GRAY)
    ]

    ty = 60
    for label, val, col in telemetry_data:
        screen.blit(FONT_SMALL.render(label, True, GRAY), (sidebar_x + 18, ty))
        screen.blit(FONT_BOLD.render(val, True, col), (sidebar_x + 18, ty + 15))
        ty += 38

    # Power Reserves Bar
    ty += 5
    screen.blit(FONT_SMALL.render("ONBOARD POWER RESERVES:", True, GRAY), (sidebar_x + 18, ty))
    ty += 18
    bat_pct = drone.battery / 100.0
    bat_color = GREEN if drone.battery > 50 else (AMBER if drone.battery > 20 else RED)
    
    bar_rect = pygame.Rect(sidebar_x + 18, ty, 244, 18)
    pygame.draw.rect(screen, DARK_GRAY, bar_rect, 0, 3)
    if drone.battery > 0:
        pygame.draw.rect(screen, bat_color, (sidebar_x + 20, ty + 2, int(240 * bat_pct), 14), 0, 2)
    pygame.draw.rect(screen, GRAY, bar_rect, 1, 3)
    
    charge_label = " (+0.35%/f)" if distance_to_home < 8 and drone.battery < 100 else ""
    screen.blit(FONT_BOLD.render(f"{drone.battery:.1f}%{charge_label}", True, TEXT_WHITE), (sidebar_x + 18, ty + 22))

    # Sidebar Action Buttons
    ty = 360
    buttons = [
        (btn_turbo_rect, "CORE TURBO BOOST [T]", BTN_ACTIVE if drone.turbo_boost else BTN_INACTIVE, TEXT_WHITE),
        (btn_radar_rect, "HIGH-RES RADAR [R]", BTN_ACTIVE if drone.high_res_radar else BTN_INACTIVE, TEXT_WHITE),
        (btn_patrol_rect, "AUTO PATROL [P]", BTN_ACTIVE if patrol_mode else BTN_INACTIVE, TEXT_WHITE),
        (btn_rth_rect, "RETURN TO DOCK [H]", (180, 100, 30) if drone.is_docking else BTN_INACTIVE, TEXT_WHITE),
        (btn_clear_rect, "CLEAR OBSTACLES [C]", BTN_INACTIVE, TEXT_WHITE)
    ]

    for rect, text, bg_col, txt_col in buttons:
        pygame.draw.rect(screen, bg_col, rect, 0, 4)
        pygame.draw.rect(screen, GRAY, rect, 1, 4)
        t_surf = FONT_BOLD.render(text, True, txt_col)
        screen.blit(t_surf, (rect.centerx - t_surf.get_width() // 2, rect.centery - t_surf.get_height() // 2))

    # Controls Sandbox Guide Footer
    pygame.draw.line(screen, DARK_GRAY, (sidebar_x + 18, 550), (WINDOW_WIDTH - 18, 550), 1)
    controls_help = [
        "[Left Click]   Spawn Obstacle Block",
        "[Right Click]  Direct Vector Target",
        "[P Key]        Toggle Patrol Route",
        "[H Key]        Emergency Return Home",
        "[C Key]        Clear Dynamic Obstacles"
    ]
    cy = 562
    for line in controls_help:
        screen.blit(FONT_SMALL.render(line, True, GRAY), (sidebar_x + 18, cy))
        cy += 24

    pygame.display.update()
    clock.tick(60)

pygame.quit()
