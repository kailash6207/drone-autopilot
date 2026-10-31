import math
import heapq
from collections import deque

_cached_dist_map = None
_cached_grid_size = None

def compute_distance_map(binary_map, grid_size):
    """
    Computes a multi-source BFS Euclidean/grid distance transform representing
    clearance from the nearest wall in units of grid cells.
    Runs in O(V) in ~30ms, enabling O(1) clearance and repulsion lookups.
    """
    pixel_height = len(binary_map)
    pixel_width = len(binary_map[0]) if pixel_height > 0 else 0
    map_w = pixel_width // grid_size
    map_h = pixel_height // grid_size

    dist_map = [[float('inf')] * map_w for _ in range(map_h)]
    q = deque()

    for gy in range(map_h):
        for gx in range(map_w):
            px = gx * grid_size + grid_size // 2
            py = gy * grid_size + grid_size // 2
            # Wall or perimeter boundary
            if (0 <= px < pixel_width and 0 <= py < pixel_height and binary_map[py][px] == 1) or \
               gx == 0 or gx == map_w - 1 or gy == 0 or gy == map_h - 1:
                dist_map[gy][gx] = 0
                q.append((gx, gy))

    while q:
        cx, cy = q.popleft()
        cd = dist_map[cy][cx]
        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            nx, ny = cx + dx, cy + dy
            if 0 <= nx < map_w and 0 <= ny < map_h:
                if dist_map[ny][nx] > cd + 1:
                    dist_map[ny][nx] = cd + 1
                    q.append((nx, ny))

    return dist_map

def invalidate_distance_cache():
    """Invalidates the cached distance map (call when obstacles are spawned)."""
    global _cached_dist_map, _cached_grid_size
    _cached_dist_map = None
    _cached_grid_size = None

def get_distance_map(binary_map, grid_size):
    """Retrieves or lazily builds the distance transform field."""
    global _cached_dist_map, _cached_grid_size
    if _cached_dist_map is None or _cached_grid_size != grid_size:
        _cached_dist_map = compute_distance_map(binary_map, grid_size)
        _cached_grid_size = grid_size
    return _cached_dist_map

def astar(binary_map, start, goal, grid_size=4):
    """
    8-Directional Center-Line Biased A* Pathfinding with continuous wall clearance
    repulsion field and distance-aware raycaster smoothing.
    """
    start = (int(start[0]), int(start[1]))
    goal = (int(goal[0]), int(goal[1]))

    dist_map = get_distance_map(binary_map, grid_size)
    map_h = len(dist_map)
    map_w = len(dist_map[0]) if map_h > 0 else 0

    # Minimum safe clearance for drone (chassis radius ~12px => 3 cells = 12px)
    MIN_SAFE_CELLS = 3

    # Snap start or goal if located too close to a wall boundary
    def snap_to_safe(node):
        nx, ny = node
        if 0 <= nx < map_w and 0 <= ny < map_h and dist_map[ny][nx] >= MIN_SAFE_CELLS:
            return node
        best = None
        best_score = float('inf')
        for r in range(1, 10):
            for dx in range(-r, r + 1):
                for dy in range(-r, r + 1):
                    tx, ty = nx + dx, ny + dy
                    if 0 <= tx < map_w and 0 <= ty < map_h and dist_map[ty][tx] >= 1:
                        # Prioritize closest cell that has maximum wall clearance
                        score = (dx * dx + dy * dy) - dist_map[ty][tx] * 3.0
                        if score < best_score:
                            best_score = score
                            best = (tx, ty)
            if best:
                return best
        return node

    start = snap_to_safe(start)
    goal = snap_to_safe(goal)

    open_set = []
    counter = 0
    heapq.heappush(open_set, (0.0, counter, start))
    came_from = {}
    g_score = {start: 0.0}

    def heuristic(a, b):
        # Euclidean distance with subtle tie-breaker to favor direct straight lines
        return math.hypot(a[0] - b[0], a[1] - b[1]) * 1.001

    # 8-Directional movement (cardinals cost 1.0, diagonals cost ~1.414)
    moves = [
        (-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
        (-1, -1, 1.4142), (-1, 1, 1.4142), (1, -1, 1.4142), (1, 1, 1.4142)
    ]

    max_iterations = 60000
    iterations = 0
    found_path = False

    while open_set:
        iterations += 1
        if iterations > max_iterations:
            break

        current_f, _, current = heapq.heappop(open_set)

        if current == goal:
            found_path = True
            break

        cx, cy = current
        for dx, dy, move_cost in moves:
            nx, ny = cx + dx, cy + dy
            if 0 <= nx < map_w and 0 <= ny < map_h:
                d_wall = dist_map[ny][nx]
                if d_wall < 1:
                    continue

                # Prevent diagonal corner cutting through diagonal wall corners
                if dx != 0 and dy != 0:
                    if dist_map[cy][nx] < 1 or dist_map[ny][cx] < 1:
                        continue

                # Strong Wall & Corner Repulsion
                # Keeps drone at least 5-6 cells (20-24px) away from corners
                repulsion = 0.0
                if d_wall < 6:
                    repulsion = (6 - d_wall) * 5.0
                elif d_wall < 11:
                    repulsion = (11 - d_wall) * 0.8

                tentative_g = g_score[current] + move_cost + repulsion
                node = (nx, ny)

                if node not in g_score or tentative_g < g_score[node]:
                    g_score[node] = tentative_g
                    came_from[node] = current
                    h = heuristic(node, goal)
                    counter += 1
                    heapq.heappush(open_set, (tentative_g + h, counter, node))

    if not found_path:
        return []

    # Reconstruct raw path from goal to start
    current = goal
    raw_path = [current]
    while current in came_from:
        current = came_from[current]
        raw_path.append(current)
    raw_path.reverse()

    # Corner-Safe Raycaster Line-of-Sight Corridor Smoother
    def clear_line_of_sight(p1, p2):
        x1, y1 = p1
        x2, y2 = p2
        dist = int(math.hypot(x2 - x1, y2 - y1))
        steps = max(dist * 2, 1)
        # Ensure shortcuts never slice closer than 5 cells (20px) to any corner wall
        req_clearance = min(5, max(3, min(dist_map[y1][x1], dist_map[y2][x2])))
        for i in range(steps + 1):
            t = i / steps
            x = int(round(x1 + t * (x2 - x1)))
            y = int(round(y1 + t * (y2 - y1)))
            if not (0 <= x < map_w and 0 <= y < map_h):
                return False
            if dist_map[y][x] < req_clearance:
                return False
        return True

    smoothed_path = [raw_path[0]]
    current_index = 0

    while current_index < len(raw_path) - 1:
        next_best_index = current_index + 1
        for look_ahead in range(len(raw_path) - 1, current_index, -1):
            if clear_line_of_sight(raw_path[current_index], raw_path[look_ahead]):
                next_best_index = look_ahead
                break
        smoothed_path.append(raw_path[next_best_index])
        current_index = next_best_index

    if smoothed_path[-1] != goal:
        smoothed_path.append(goal)

    return smoothed_path
