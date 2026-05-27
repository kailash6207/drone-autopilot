import math
import heapq

def astar(binary_map, start, goal, grid_size=1):
    """
    Master Upgraded A* Pathfinding with enhanced iteration headroom 
    and adaptive goal-node relaxation to prevent wall-deadlocks.
    """
    start = (int(start[0]), int(start[1]))
    goal = (int(goal[0]), int(goal[1]))
    
    pixel_height = len(binary_map)
    pixel_width = len(binary_map[0]) if pixel_height > 0 else 0
    
    map_width = pixel_width // grid_size
    map_height = pixel_height // grid_size
    
    def IsValidPixel(x, y):
        return 0 <= x < pixel_width and 0 <= y < pixel_height

    def IsGridWalkable(gx, gy):
        px = int((gx * grid_size) + (grid_size // 2))
        py = int((gy * grid_size) + (grid_size // 2))
        if not IsValidPixel(px, py):
            return False
        return binary_map[py][px] == 0

    # UPGRADE: If destination is placed directly inside a wall thickness,
    # automatically find the nearest open walkable pixel cell.
    if not IsGridWalkable(goal[0], goal[1]):
        found_clearance = False
        for r in range(1, 6):
            for dx in range(-r, r + 1):
                for dy in range(-r, r + 1):
                    tx, ty = goal[0] + dx, goal[1] + dy
                    if 0 <= tx < map_width and 0 <= ty < map_height:
                        if IsGridWalkable(tx, ty):
                            goal = (tx, ty)
                            found_clearance = True
                            break
                if found_clearance: break
            if found_clearance: break
        if not found_clearance:
            return []

    # Adaptive start node recovery
    if not IsGridWalkable(start[0], start[1]):
        escaped = False
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                check_x = start[0] + dx
                check_y = start[1] + dy
                if 0 <= check_x < map_width and 0 <= check_y < map_height:
                    if IsGridWalkable(check_x, check_y):
                        start = (check_x, check_y)
                        escaped = True
                        break
            if escaped:
                break
        if not escaped:
            return []

    open_set = []
    heapq.heappush(open_set, (0, start))
    came_from = {}
    g_score = {start: 0}
    
    def get_heuristic(a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])
        
    f_score = {start: get_heuristic(start, goal)}
    
    # UPGRADE: Increased loop headroom significantly so the pathfinder 
    # has the stamina to explore completely around long partition walls.
    max_iterations = 50000 
    iterations = 0
    raw_path = []
    found_path = False
    
    while len(open_set) > 0:
        iterations += 1
        if iterations > max_iterations:
            break
            
        current_f, current = heapq.heappop(open_set)
        
        if current == goal:
            found_path = True
            while current in came_from:
                raw_path.append(current)
                current = came_from[current]
            raw_path.reverse()
            break
            
        cx, cy = current
        directions = [(cx, cy - 1), (cx, cy + 1), (cx - 1, cy), (cx + 1, cy)]
        neighbors = []
        
        for nx, ny in directions:
            if 0 <= nx < map_width and 0 <= ny < map_height:
                if IsGridWalkable(nx, ny):
                    base_cost = 1
                    proximity_penalty = 0
                    
                    px = int((nx * grid_size) + (grid_size // 2))
                    py = int((ny * grid_size) + (grid_size // 2))
                    
                    scan_radius = 8
                    for dx in range(-scan_radius, scan_radius + 1):
                        for dy in range(-scan_radius, scan_radius + 1):
                            target_px = px + dx
                            target_py = py + dy
                            if IsValidPixel(target_px, target_py) and binary_map[target_py][target_px] > 0:
                                dist = (dx**2 + dy**2)**0.5
                                if dist > 0:
                                    proximity_penalty += int(480 / dist)
                                
                    total_cost = base_cost + proximity_penalty
                    neighbors.append(((nx, ny), total_cost))
                    
        for next_node, cost in neighbors:
            tentative_g = g_score[current] + cost
            if next_node not in g_score or tentative_g < g_score[next_node]:
                came_from[next_node] = current
                g_score[next_node] = tentative_g
                f = tentative_g + get_heuristic(next_node, goal)
                f_score[next_node] = f
                heapq.heappush(open_set, (f, next_node))
                            
    if not found_path or len(raw_path) == 0:
        return []

    # Adaptive Tapered Corridor Raycaster Smoother
    def clear_grid_line_of_sight(node_a, node_b):
        x1, y1 = node_a
        x2, y2 = node_b
        
        dx = abs(x2 - x1)
        dy = abs(y2 - y1)
        sx = 1 if x1 < x2 else -1
        sy = 1 if y1 < y2 else -1
        err = dx - dy
        
        while True:
            dist_to_goal = math.sqrt((x1 - goal[0])**2 + (y1 - goal[1])**2)
            
            if dist_to_goal < 8:
                bound_size = 1
            elif dist_to_goal < 20:
                bound_size = 3
            else:
                bound_size = 5
                
            for bx in range(-bound_size, bound_size + 1):
                for by in range(-bound_size, bound_size + 1):
                    tx = x1 + bx
                    ty = y1 + by
                    if 0 <= tx < map_width and 0 <= ty < map_height:
                        if not IsGridWalkable(tx, ty):
                            return False
                    else:
                        return False
                        
            if x1 == x2 and y1 == y2:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x1 += sx
            if e2 < dx:
                err += dx
                y1 += sy
        return True

    smoothed_path = [raw_path[0]]
    current_index = 0
    
    while current_index < len(raw_path) - 1:
        next_best_index = current_index + 1
        for look_ahead in range(len(raw_path) - 1, current_index, -1):
            if clear_grid_line_of_sight(raw_path[current_index], raw_path[look_ahead]):
                next_best_index = look_ahead
                break
        smoothed_path.append(raw_path[next_best_index])
        current_index = next_best_index

    if smoothed_path[-1] != goal:
        smoothed_path.append(goal)

    return smoothed_path