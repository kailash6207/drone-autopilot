import pygame
import math

from config import *
from drone import Drone
from image_processing import process_floorplan
from pathfinding import astar

pygame.init()
pygame.font.init()

# Setup Core Fonts
FONT_MAIN = pygame.font.SysFont("Consolas", 16)
FONT_BOLD = pygame.font.SysFont("Consolas", 18, bold=True)

screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
pygame.display.set_caption("AI Autonomous Drone - Command Console")
clock = pygame.time.Clock()

# Systems Setup
drone = Drone()
drone_placed = False 

binary_map = process_floorplan("images/house.png")
floorplan_image = pygame.image.load("images/house.png")
floorplan_image = pygame.transform.scale(floorplan_image, (MAP_WIDTH, WINDOW_HEIGHT))

path = []
target = None
obstacles = []

# Tracker to capture the absolute closest the drone got to its current target
min_distance_recorded = float('inf')

# Timestamp tracker for the destination pause feature
destination_hold_time = 0  

# Destination Ring Indicator Animation Properties
ring_radius = 0
ring_alpha = 255

WAYPOINTS = [
    (150, 200), (150, 600), (450, 200),
    (550, 200), (550, 600), (800, 400), (800, 600)
]
patrol_queue = []
patrol_mode = False
sensor_collision = False

# Interactive UI Button Boundaries
btn_turbo_rect = pygame.Rect(MAP_WIDTH + 20, 440, 240, 35)
btn_radar_rect = pygame.Rect(MAP_WIDTH + 20, 490, 240, 35)

running = True

while running:
    # Clear screen frame backgrounds
    screen.fill(PANEL_BG)
    screen.blit(floorplan_image, (0, 0))

    # Calculate real-time tracking distance to dock station for use in events loop
    distance_to_home = math.sqrt((drone.x - drone.home_x)**2 + (drone.y - drone.home_y)**2) if drone_placed else 0.0

    # ==========================================
    # EVENTS MANAGEMENT & INPUT HANDLING
    # ==========================================
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

        # Toggle Patrol Mode via Keyboard Input
        if event.type == pygame.KEYDOWN and drone_placed:
            if event.key == pygame.K_p and (not drone.is_docking or drone.battery >= 35.0):
                patrol_mode = not patrol_mode
                if patrol_mode:
                    drone.is_docking = False  
                    patrol_queue = WAYPOINTS.copy()
                    min_distance_recorded = float('inf')
                    destination_hold_time = 0
                else:
                    patrol_queue, path = [], []
                    target = None

        # Process Mouse Clicks
        if event.type == pygame.MOUSEBUTTONDOWN:
            mx, my = pygame.mouse.get_pos()
            
            # SIDEBAR DASHBOARD INTERACTIONS
            if mx >= MAP_WIDTH and drone_placed:
                if btn_turbo_rect.collidepoint(mx, my):
                    drone.turbo_boost = not drone.turbo_boost
                elif btn_radar_rect.collidepoint(mx, my):
                    drone.high_res_radar = not drone.high_res_radar

            # MAP CANVAS INTERACTIONS
            elif 0 <= mx < MAP_WIDTH and 0 <= my < WINDOW_HEIGHT:
                # Spawn base station and drone on first left-click
                if not drone_placed:
                    if event.button == 1: 
                        if binary_map[my][mx] == 0:
                            drone.x, drone.y = mx, my
                            drone.home_x, drone.home_y = mx, my
                            drone_placed = True
                else:
                    # LEFT-CLICK: Spawn a perfectly grid-locked obstacle block
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
                            
                            if target:
                                start_node = (int(drone.x // GRID_SIZE), int(drone.y // GRID_SIZE))
                                goal_node = (int(target[0] // GRID_SIZE), int(target[1] // GRID_SIZE))
                                path = astar(binary_map, start_node, goal_node, GRID_SIZE)

                    # RIGHT-CLICK: Set Navigation Target Destination
                    elif event.button == 3 and not patrol_mode:
                        if binary_map[my][mx] == 0:
                            # Allow breakout from charger if above threshold
                            if not drone.is_docking or drone.battery >= 35.0:
                                drone.is_docking = False  
                                target = (mx, my)
                                path = []                             # FIXED: Clear active path leftovers
                                min_distance_recorded = float('inf')  # Reset tracker
                                destination_hold_time = 0             # Reset hover timer
                                ring_radius = 0
                                ring_alpha = 255
                                start_node = (int(drone.x // GRID_SIZE), int(drone.y // GRID_SIZE))
                                goal_node = (int(mx // GRID_SIZE), int(my // GRID_SIZE))
                                path = astar(binary_map, start_node, goal_node, GRID_SIZE)

    # ==========================================
    # CORE ENGINE CALCULATIONS & PHYSICS
    # ==========================================
    if drone_placed:
        # HARD CORE BATTERY DIE SAFEGUARD
        if drone.battery <= 0.0:
            drone.x, drone.y = drone.home_x, drone.home_y
            drone.vx, drone.vy = 0.0, 0.0
            drone.is_docking = True
            patrol_mode = False
            target, path = None, []
            destination_hold_time = 0

        # TRUE EMERGENCY BATTERY RUNHOME
        if drone.battery <= 20.0 and drone.battery > 0 and not drone.is_docking and distance_to_home > 15:
            drone.is_docking = True
            patrol_mode = False
            target = (drone.home_x, drone.home_y)
            min_distance_recorded = float('inf')
            destination_hold_time = 0
            start_node = (int(drone.x // GRID_SIZE), int(drone.y // GRID_SIZE))
            goal_node = (int(drone.home_x // GRID_SIZE), int(drone.home_y // GRID_SIZE))
            path = astar(binary_map, start_node, goal_node, GRID_SIZE)

        # PROXIMITY CHARGER GRID CONNECTION
        if distance_to_home < 6:
            drone.battery = min(100.0, drone.battery + 0.3) 
            if drone.battery >= 100.0 and drone.is_docking:
                drone.is_docking = False
                target = None
                destination_hold_time = 0

        if patrol_mode and len(path) == 0 and len(patrol_queue) > 0:
            next_destination = patrol_queue.pop(0)
            target = next_destination
            min_distance_recorded = float('inf')
            destination_hold_time = 0
            start_node = (int(drone.x // GRID_SIZE), int(drone.y // GRID_SIZE))
            goal_node = (int(target[0] // GRID_SIZE), int(target[1] // GRID_SIZE))
            path = astar(binary_map, start_node, goal_node, GRID_SIZE)
            if len(patrol_queue) == 0:
                patrol_queue = WAYPOINTS.copy()

        # ==========================================================
        # FOLLOW PATH TRACKING ROUTINE
        # ==========================================================
        target_node_x, target_node_y = None, None
        
        if len(path) > 0:
            next_node = path[0]
            target_node_x = (next_node[0] * GRID_SIZE) + (GRID_SIZE // 2)
            target_node_y = (next_node[1] * GRID_SIZE) + (GRID_SIZE // 2)

            if len(path) > 6:
                future_node = path[6]
                fx = (future_node[0] * GRID_SIZE) + (GRID_SIZE // 2)
                fy = (future_node[1] * GRID_SIZE) + (GRID_SIZE // 2)
                
                angle_now = math.atan2(target_node_y - drone.y, target_node_x - drone.x)
                angle_future = math.atan2(fy - target_node_y, fx - target_node_x)
                angle_diff = abs((angle_future - angle_now + math.pi) % (2 * math.pi) - math.pi)
                
                if angle_diff > 0.4: 
                    drone.max_speed *= 0.25

            current_speed = math.sqrt(drone.vx**2 + drone.vy**2)
            arrival_radius = 8 + int(current_speed * 2.5)

            if math.sqrt((target_node_x - drone.x)**2 + (target_node_y - drone.y)**2) <= arrival_radius:
                path.pop(0)
                
        # ==========================================================
        # AUTOMATIC TIMED RETURN-TO-HOME TRIGGER
        # ==========================================================
        elif target is not None:
            dist_to_final_target = math.sqrt((target[0] - drone.x)**2 + (target[1] - drone.y)**2)
            
            if dist_to_final_target < min_distance_recorded:
                min_distance_recorded = dist_to_final_target

            is_overshooting = (dist_to_final_target > min_distance_recorded + 1.5) and (min_distance_recorded < 22)
            is_dead_on = (dist_to_final_target <= 8)

            if not is_dead_on and not is_overshooting:
                # Still traveling toward target coordinate
                target_node_x, target_node_y = target[0], target[1]
            else:
                # Arrived at destination waypoint footprint
                if distance_to_home > 12:
                    # Capture timestamp if it's the first frame of arrival
                    if destination_hold_time == 0:
                        print("Target reached. Holding position for 2 seconds...")
                        destination_hold_time = pygame.time.get_ticks()
                    
                    # Wait for 2000ms countdown window to elapse
                    if pygame.time.get_ticks() - destination_hold_time >= 2000:
                        print("Timer complete. Returning to charging dock.")
                        drone.is_docking = True
                        target = (drone.home_x, drone.home_y)
                        min_distance_recorded = float('inf')
                        destination_hold_time = 0  # Reset timer
                        start_node = (int(drone.x // GRID_SIZE), int(drone.y // GRID_SIZE))
                        goal_node = (int(drone.home_x // GRID_SIZE), int(drone.home_y // GRID_SIZE))
                        path = astar(binary_map, start_node, goal_node, GRID_SIZE)
                    else:
                        # Force dead hover/stabilization during pause window
                        target_node_x, target_node_y = None, None
                else:
                    # Safely back inside home charging dock range
                    target = None
                    destination_hold_time = 0

        # Apply physics engine forces
        if drone.battery > 0.0:
            drone.update_physics(target_node_x, target_node_y)

        # Onboard Radar Sweep Visualizations
        max_scan_range = 220 if drone.high_res_radar else 120
        sensor_collision = False
        map_height = len(binary_map)
        map_width = len(binary_map[0])
        radar_sectors = [False] * 8  

        for angle in range(0, 360, 15):
            radians = math.radians(angle)
            sector_index = int((angle + 22.5) % 360) // 45

            for distance in range(20, max_scan_range, 6):
                sx = int(drone.x + math.cos(radians) * distance)
                sy = int(drone.y + math.sin(radians) * distance)

                if sx < 0 or sy < 0 or sx >= map_width or sy >= map_height:
                    break
                if binary_map[sy][sx] > 0:
                    pygame.draw.circle(screen, RED, (sx, sy), 2)
                    sensor_collision = True
                    radar_sectors[sector_index] = True
                    break
                else:
                    pygame.draw.circle(screen, BLUE, (sx, sy), 1)

        # ==========================================
        # VISUAL RENDER PIPELINE
        # ==========================================
        for obs in obstacles:
            pygame.draw.rect(screen, AMBER, obs)
            pygame.draw.rect(screen, RED, obs, 1)

        pygame.draw.rect(screen, AMBER, (drone.home_x - 10, drone.home_y - 10, 20, 20), 2)
        pygame.draw.circle(screen, AMBER, (drone.home_x, drone.home_y), 4)

        if target:
            pygame.draw.circle(screen, RED, target, 6)
            if ring_alpha > 0:
                surf = pygame.Surface((100, 100), pygame.SRCALPHA)
                pygame.draw.circle(surf, (255, 0, 0, ring_alpha), (50, 50), ring_radius, 2)
                screen.blit(surf, (target[0] - 50, target[1] - 50))
                ring_radius = (ring_radius + 2) % 40
                ring_alpha = max(0, ring_alpha - 6)

        radar_radius = 42
        pos_x, pos_y = int(drone.x), int(drone.y)
        for i in range(8):
            start_angle = math.radians(i * 45 - 22.5)
            stop_angle = math.radians((i + 1) * 45 - 22.5)
            ring_color = RED if radar_sectors[i] else GREEN
            arc_rect = pygame.Rect(pos_x - radar_radius, pos_y - radar_radius, radar_radius * 2, radar_radius * 2)
            pygame.draw.arc(screen, ring_color, arc_rect, -stop_angle, -start_angle, 3)

        drone.draw(screen)

    # ==========================================
    # TELEMETRY DASHBOARD SIDEBAR PANEL UI
    # ==========================================
    pygame.draw.rect(screen, DARK_GRAY, (MAP_WIDTH, 0, WINDOW_WIDTH - MAP_WIDTH, WINDOW_HEIGHT))
    pygame.draw.line(screen, BLUE, (MAP_WIDTH, 0), (MAP_WIDTH, WINDOW_HEIGHT), 3)

    screen.blit(FONT_BOLD.render("DRONE SYSTEM DIAGNOSTICS", True, BLUE), (MAP_WIDTH + 20, 30))
    pygame.draw.line(screen, GRAY, (MAP_WIDTH + 20, 55), (WINDOW_WIDTH - 20, 55), 1)

    current_speed_kph = math.sqrt(drone.vx**2 + drone.vy**2) * 10 if drone_placed else 0.0
    
    geofence_breached = False
    if drone_placed:
        if drone.x < 25 or drone.x > (MAP_WIDTH - 25) or drone.y < 25 or drone.y > (WINDOW_HEIGHT - 25):
            geofence_breached = True

    # Smart UI Telemetry Text Switches
    if geofence_breached:
        mode_text = "GEOFENCE BREACHED!"
        mode_color = RED
    elif patrol_mode:
        mode_text = "AUTO PATROL SURVEILLANCE"
        mode_color = GREEN
    elif destination_hold_time > 0:
        mode_text = "OBJECTIVE HOVER DELAY..."
        mode_color = BLUE
    elif drone.is_docking:
        if distance_to_home < 6 and drone.battery >= 35.0:
            mode_text = "STANDBY (READY TO FLY)"
            mode_color = BLUE
        elif distance_to_home < 6:
            mode_text = "FAST CHARGING ON LOCK..."
            mode_color = AMBER
        else:
            mode_text = "RETURNING TO CHARGING DOCK"
            mode_color = AMBER
    elif drone_placed:
        mode_text = "MANUAL GUIDANCE"
        mode_color = TEXT_WHITE
    else:
        mode_text = "AWAITING INITIAL DEPLOYMENT"
        mode_color = GRAY

    metrics = [
        ("SYSTEM STATUS:", "ONLINE" if drone_placed else "OFFLINE", GREEN if drone_placed else RED),
        ("FLIGHT MODE:", mode_text, mode_color),
        ("TELEMETRY VELOCITY:", f"{current_speed_kph:.1f} km/h", TEXT_WHITE),
        ("RADAR COLLISION ALERTS:", "OUT OF BOUNDS ERROR" if geofence_breached else ("CRITICAL PROXIMITY" if sensor_collision else "CLEAR"), RED if (sensor_collision or geofence_breached) else GREEN),
    ]

    text_y = 80
    for title, val, color in metrics:
        screen.blit(FONT_MAIN.render(title, True, GRAY), (MAP_WIDTH + 20, text_y))
        screen.blit(FONT_BOLD.render(val, True, color), (MAP_WIDTH + 20, text_y + 18))
        text_y += 50

    if geofence_breached and pygame.time.get_ticks() % 1000 < 500:
        alert_surf = FONT_BOLD.render("WARNING: DRONE OUTSIDE HOUSE PERIMETER!", True, RED)
        screen.blit(alert_surf, (MAP_WIDTH // 2 - 180, WINDOW_HEIGHT // 2 - 10))

    screen.blit(FONT_MAIN.render("ONBOARD POWER RESERVES:", True, GRAY), (MAP_WIDTH + 20, 285))
    bat_color = GREEN if drone.battery > 50 else (AMBER if drone.battery > 20 else RED)
    pygame.draw.rect(screen, DARK_GRAY, (MAP_WIDTH + 20, 305, 240, 22), 0)
    pygame.draw.rect(screen, GRAY, (MAP_WIDTH + 20, 305, 240, 22), 2)
    if drone.battery > 0:
        pygame.draw.rect(screen, bat_color, (MAP_WIDTH + 23, 308, int(234 * (drone.battery / 100)), 16), 0)
    screen.blit(FONT_BOLD.render(f"{drone.battery:.1f}%", True, TEXT_WHITE), (MAP_WIDTH + 20, 332))

    pygame.draw.line(screen, GRAY, (MAP_WIDTH + 20, 365), (WINDOW_WIDTH - 20, 365), 1)
    screen.blit(FONT_BOLD.render("HARDWARE MODULE UPGRADES:", True, TEXT_WHITE), (MAP_WIDTH + 20, 380))

    turbo_color = BTN_ACTIVE if drone.turbo_boost else BTN_INACTIVE
    pygame.draw.rect(screen, turbo_color, btn_turbo_rect, 0, 4)
    pygame.draw.rect(screen, GRAY, btn_turbo_rect, 1, 4)
    screen.blit(FONT_BOLD.render("CORE TURBO BOOST", True, TEXT_WHITE), (MAP_WIDTH + 50, 448))

    radar_color = BTN_ACTIVE if drone.high_res_radar else BTN_INACTIVE
    pygame.draw.rect(screen, radar_color, btn_radar_rect, 0, 4)
    pygame.draw.rect(screen, GRAY, btn_radar_rect, 1, 4)
    screen.blit(FONT_BOLD.render("HIGH-RES RADAR SENSOR", True, TEXT_WHITE), (MAP_WIDTH + 30, 498))

    pygame.draw.line(screen, GRAY, (MAP_WIDTH + 20, 550), (WINDOW_WIDTH - 20, 550), 1)
    screen.blit(FONT_MAIN.render("[L-Click Map]  Spawn Block Obstacle", True, GRAY), (MAP_WIDTH + 20, 575))
    screen.blit(FONT_MAIN.render("[R-Click Map]  Smooth Vector Routing", True, GRAY), (MAP_WIDTH + 20, 605))
    screen.blit(FONT_MAIN.render("[P Key]        Toggle Auto Patrol", True, GRAY), (MAP_WIDTH + 20, 635))

    pygame.display.update()
    clock.tick(60)

pygame.quit()