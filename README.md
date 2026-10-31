# 🚁 Autonomous Drone Autopilot & Navigation Simulator

<div align="center">

[![Python](https://img.shields.io/badge/Python-3.10%20%7C%203.11%20%7C%203.12-3776AB?style=for-the-badge&logo=python&logoColor=white)](https://www.python.org/)
[![Pygame](https://img.shields.io/badge/Pygame-2.6+-1B5E20?style=for-the-badge&logo=pygame&logoColor=white)](https://www.pygame.org/)
[![Pillow](https://img.shields.io/badge/Pillow-Image%20Processing-00599C?style=for-the-badge&logo=python&logoColor=white)](https://python-pillow.org/)
[![NumPy](https://img.shields.io/badge/NumPy-Numerical%20Engine-013243?style=for-the-badge&logo=numpy&logoColor=white)](https://numpy.org/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg?style=for-the-badge)](https://opensource.org/licenses/MIT)
[![Maintenance](https://img.shields.io/badge/Maintained%3F-Yes-green.svg?style=for-the-badge)](https://github.com/kailash6207/drone-autopilot)

**A real-time autonomous robotics simulator built with Pygame featuring computer-vision floorplan ingestion, 8-directional center-line biased A\* pathfinding, corner-safe raycasting, 2D vector flight aerodynamics, 360° proximity radar, architectural geofencing ("OUT OF HOUSE" alert), and autonomous fail-safe docking.**

[Key Features](#-key-features) •
[System Architecture](#-system-architecture--pipeline-flowchart) •
[Mission Lifecycle](#-mission-lifecycle-state-machine) •
[Pathfinding & Smoothing](#-algorithmic-deep-dive--workflows) •
[Flight Physics](#-flight-physics--kinetics-engine-workflow) •
[Obstacle Avoidance](#-dynamic-in-flight-obstacle-injection-workflow) •
[Radar Array](#-360-octant-radar-sensor-array-workflow) •
[Flight Controls](#-flight-controls--sandbox-guide) •
[Installation](#-installation--getting-started)

</div>

---

## 📌 Overview

**Drone Autopilot** is a 2D autonomous robotic simulation environment designed to model realistic indoor unmanned aerial vehicle (UAV) navigation through complex architectural spaces. 

Instead of relying on idealized point-mass movement or trivial grid paths, the simulator implements an integrated robotics stack:
- **Floorplan Computer Vision Pipeline:** Ingests architectural blueprints with automatic morphological wall dilation, featuring a resilient pure-Python PIL fallback for restricted environments (e.g., Windows Smart App Control).
- **Center-Line Biased 8-Directional A\* Planner:** Multi-source BFS distance transform repels paths away from tight doorframes and corners, routing the drone naturally through hallway centers with Euclidean heuristics.
- **Corner-Safe Raycaster Smoother:** Line-of-sight raycaster with a guaranteed $20\text{ px}$ corner safety clearance converts jagged grid steps into direct, collision-free waypoints without corner clipping.
- **Kinetic Vector Aerodynamics & Overshoot Control:** Smooth velocity lerp physics, proactive corner braking, and an $85\text{ px}$ proportional deceleration zone eliminate Turbo Mode overshoot and trajectory drift.
- **Architectural Geofence & Alert System:** Dynamic flood-fill boundary detection triggers immediate **"OUT OF HOUSE"** warnings, flashing HUD alert banners, and a strobe halo when leaving the indoor envelope.
- **Fail-Safe Mission State Machine:** Autonomous mission lifecycle managing launch, point-to-point guidance, automated patrol loops, 2-second hover telemetry checks, emergency low-battery return-to-home (RTH), and fast inductive docking.
- **360° Octant Radar & HUD Diagnostics:** 24-ray radar sweep with real-time radial collision warning arcs around the drone chassis and live glass-cockpit telemetry.

---

## 🌟 Key Features

- **🧭 8-Directional Diagonal A\* with Center-Line Bias:** Euclidean distance heuristic with precomputed Euclidean BFS distance transform ($\max(0, 40 - \text{dist\_map}) \times 1.4$) pulling trajectories safely to corridor center-lines.
- **📐 Corner-Safe Line-of-Sight Raycasting:** Multi-sample corridor raycaster enforcing a minimum $20\text{ px}$ clearance around corners, eliminating edge clipping.
- **🛸 Velocity Lerping & Drift Elimination:** Smooth exponential velocity lerp (`0.14`), proportional arrival deceleration zone ($85\text{ px}$), active target lock, and potential-field obstacle repulsion cushion ($20\text{ px}$) prevent Turbo Mode drift.
- **⚠️ Real-Time "OUT OF HOUSE" Geofence:** Architectural flood-fill geofence triggers audio-visual alerts, pulsing border highlights, and a drone warning halo if the craft exits the house footprint.
- **📡 360° Octant Proximity Radar:** 24-ray radial scan with 8-sector status arcs around the chassis and toggleable range ($120\text{ px}$ Standard vs. $220\text{ px}$ High-Res).
- **🔋 Autonomous Mission Lifecycle:** Automated 2-second target hover validation, low-power emergency RTH ($\le 20\%$), inductive pad fast-recharge ($+0.3\%/\text{frame}$), and departure safety check ($\ge 35\%$).
- **🧱 Live Interactive Sandbox:** Dynamic in-flight obstacle spawning with instantaneous path recalculation, obstacle clearing (`[C]`), protected home base station, and one-key return to dock (`[H]`).

---

## 🏗️ System Architecture & Pipeline Flowchart

A neat, streamlined overview of the end-to-end simulation pipeline:

```mermaid
flowchart LR
    A["🖼️ Vision Pipeline<br/>(Dilation & Binarization)"] --> B["🧭 A* Planner<br/>(Wall Repulsion Field)"]
    B --> C["📐 Raycaster Smoother<br/>(LOS Corridor Pruning)"]
    C --> D["🛸 Flight Physics<br/>(Inertia & Drag Damping)"]
    D --> E["📡 360° Radar & HUD<br/>(Telemetry & Collision)"]

    Inputs["🎮 User Inputs<br/>(Clicks, 'P', Toggles)"] -.-> B
    Inputs -.-> D
    D -.->|"Battery <= 20%"| B
```

---

## 🔄 Mission Lifecycle State Machine

A concise view of the autonomous state transitions governing flight, hover delays, and docking:

```mermaid
stateDiagram-v2
    [*] --> Standby: Deploy at Base Station
    Standby --> InFlight: Target Clicked / Patrol (P) [Battery >= 35%]
    InFlight --> Hover: Destination Reached
    Hover --> Homing: 2.0s Telemetry Hold Done
    InFlight --> Homing: Low Battery Warning (<= 20%)
    Homing --> Standby: Reached Dock (< 6px)
    Standby --> Standby: Fast Charging (+0.3%/frame)
```

---

## 🔬 Algorithmic Deep Dive & Workflows

### 1. Master A* Pathfinding Engine Workflow
Upgraded 8-directional A\* search with precomputed Euclidean BFS distance transform for corridor centerline bias:

```mermaid
flowchart TD
    A["Start & Target Coords"] --> B{"Inside Wall?"}
    B -- Yes --> C["Snap to Nearest Open Cell"]
    B -- No --> D["Push Start into Priority Queue"]
    C --> D
    D --> E["Pop Lowest Cost Node: f = g + h + penalty"]
    E --> F{"Goal Reached?"}
    F -- Yes --> G["Reconstruct Path"]
    F -- No --> H["Expand 8-Directional Neighbors (Diag: √2)"]
    H --> I["Add BFS Centerline Penalty: max(0, 40 - dist_map) * 1.4"]
    I --> D
```

#### Mathematical Cost Formulation
Candidate cells $n$ are evaluated using an 8-directional diagonal movement model with a precalculated Euclidean distance transform:

$$f(n) = g(n) + h(n) + \text{Penalty}_{\text{proximity}}(n)$$

Where the Euclidean heuristic $h(n)$ and the corridor centerline penalty are formulated as:

$$h(n) = \sqrt{(x - x_{\text{goal}})^2 + (y - y_{\text{goal}})^2}$$

$$\text{Penalty}_{\text{proximity}}(n) = \max\left(0, 40 - \text{dist\_map}[y, x]\right) \times 1.4$$

*Note: $\text{dist\_map}$ is precomputed once per map/obstacle modification via a multi-source Breadth-First Search (BFS), keeping per-node pathfinding overhead $O(1)$.*

---

### 2. Corner-Safe Corridor Raycaster Path Smoother
Prunes intermediate grid nodes while guaranteeing a minimum $20\text{ px}$ clearance around corners to prevent clipping:

```mermaid
flowchart TD
    A["Raw A* Path Nodes"] --> B["Greedy Lookahead (Farthest Reachable Node)"]
    B --> C["Raycast Line-of-Sight with 20px Clearance Envelope"]
    C --> D{"Clearance Violated at Any Step?"}
    D -- Yes --> E["Step Target Backward Along Path"]
    D -- No --> F["Connect Waypoint Directly & Prune Intermediate Nodes"]
    E --> C
    F --> G{"Reached Goal?"}
    G -- No --> B
    G -- Yes --> H["Smooth, Corner-Safe Trajectory"]
```

---

## 🛸 Flight Physics & Kinetics Engine Workflow

Every $60\text{ FPS}$ frame integrates smooth velocity lerping, proactive corner braking, and an arrival deceleration envelope:

```mermaid
flowchart TD
    A["Next Waypoint & Drone State"] --> B["Calculate Heading Angle: atan2(dy, dx)"]
    B --> C{"Corner Turn Ahead?"}
    C -- Yes --> D["Clamp Maximum Speed (Speed Limit = 3.6 px/frame)"]
    C -- No --> E["Calculate Distance to Target Destination"]
    D --> E
    E --> F{"Distance < 85px (Arrival Zone)?"}
    F -- Yes --> G["Scale Target Speed Linearly (v = max_speed * dist / 85)"]
    F -- No --> H["Cruising Target Speed"]
    G --> I["Smooth Velocity Lerp: v = lerp(v, target_v, 0.14)"]
    H --> I
    I --> J["Apply Obstacle Repulsion Cushion (20px envelope)"]
    J --> K["Integrate Velocity & Update Position"]
    K --> L["Target Reached (< 3px)? -> Lock Position (Zero Drift)"]
```

---

## 🧱 Dynamic In-Flight Obstacle Injection Workflow

Real-time obstacle spawning with instantaneous mid-flight path recalculation:

```mermaid
sequenceDiagram
    autonumber
    actor User as User
    participant Grid as Binary Map
    participant Planner as A* Planner
    participant Drone as Flight Controller

    User->>Grid: Left-Click to Spawn Obstacle Block
    Note over Grid: Guard: Reject if on Charging Pad
    Grid->>Planner: Trigger Real-Time Re-Plan
    Planner->>Planner: Compute & Smooth New Route
    Planner->>Drone: Hot-Swap Active Flight Path
```

---

## 📡 360° Octant Radar Sensor Array Workflow

The collision radar casts 24 radial rays to build an 8-octant spatial collision map around the drone chassis:

```mermaid
flowchart TD
    A["Drone Center Point"] --> B["Cast 24 Radial Rays (15° Steps)"]
    B --> C["Step Ray outward (120px Normal / 220px High-Res)"]
    C --> D{"Hit Obstacle?"}
    D -- Yes --> E["Mark Sector Red (Alert)"]
    D -- No --> F["Mark Sector Green (Clear)"]
    E & F --> G["Draw 8-Octant HUD Arcs & Update Sidebar Alerts"]
```

---

## 🎮 Flight Controls & Sandbox Guide

| Input | Action | Context & Behavior |
| :--- | :--- | :--- |
| **Left-Click (Map)** | **Initial Drone Deployment** | Spawns the drone and sets its permanent home charging dock station (must be clicked on open floor). |
| **Left-Click (Map)** | **Spawn Obstacle Block** | When deployed, spawns a $36\times 36\text{ px}$ obstacle block. Triggers immediate dynamic path recalculation in real-time. Protected: Cannot place over base station. |
| **Right-Click (Map)** | **Set Target Destination** | Computes smoothed vector trajectory to the clicked coordinate. If docked, initiates departure (requires $\ge 35\%$ battery). |
| **`[P]` Key / UI Button** | **Toggle Auto Patrol** | Starts/stops autonomous surveillance loop through 7 residential waypoints across the property. |
| **`[H]` Key / UI Button** | **Return To Dock (RTH)** | Instantly commands the drone to abort the active mission and return to the charging pad. |
| **`[C]` Key / UI Button** | **Clear Dynamic Obstacles** | Clears all injected obstacle blocks and restores original floorplan with instant path update. |
| **`[T]` Key / UI Button** | **Toggle Turbo Drive** | Increases speed ceiling ($2.4\times$) and acceleration; dynamically manages battery drain. |
| **`[R]` Key / UI Button** | **Toggle Radar Range** | Extends 360° sensor raycasting envelope from $125\text{ px}$ to $220\text{ px}$. |

---

## 🖥️ Telemetry & Dashboard Diagnostics

The persistent right-side HUD panel displays real-time glass-cockpit flight metrics:

```
+------------------------------------------------+
|           DRONE SYSTEM DIAGNOSTICS             |
+------------------------------------------------+
| SYSTEM STATUS:          ONLINE / OFFLINE       |
| FLIGHT MODE:            STANDBY / CRUISE / ... |
| TELEMETRY VELOCITY:     0.0 - 45.0 km/h        |
| RADAR COLLISION ALERTS: CLEAR / CRITICAL PROX  |
+------------------------------------------------+
| ONBOARD POWER RESERVES:                        |
| [=======================       ] 78.4%         |
+------------------------------------------------+
| HARDWARE MODULE UPGRADES:                      |
| [ CORE TURBO BOOST        ] (ON/OFF)           |
| [ HIGH-RES RADAR SENSOR   ] (ON/OFF)           |
+------------------------------------------------+
| INTERACTIVE COMMANDS:                          |
| [L-Click Map] Spawn Block Obstacle             |
| [R-Click Map] Smooth Vector Routing            |
| [P Key]       Toggle Auto Patrol               |
+------------------------------------------------+
```

### Telemetry State Descriptions
- **`AWAITING INITIAL DEPLOYMENT`**: Simulator initialized; waiting for initial left-click to place charging pad.
- **`STANDBY (READY TO FLY)`**: Drone safely docked on base pad with $\ge 35\%$ battery cushion.
- **`FAST CHARGING ON LOCK...`**: Drone docked on pad with $< 35\%$ battery; charging at $+0.3\%/\text{frame}$.
- **`MANUAL GUIDANCE`**: Autonomous navigation actively tracking a user-designated right-click waypoint.
- **`AUTO PATROL SURVEILLANCE`**: Continuous autonomous patrol executing multi-room surveillance waypoints.
- **`OBJECTIVE HOVER DELAY...`**: Reached target coordinates; holding position for 2,000 ms telemetry check.
- **`RETURNING TO CHARGING DOCK`**: Mission complete or low battery; returning to base station pad.
- **`[!] WARNING: DRONE OUT OF HOUSE!`**: Flashing alert banner, yellow/red screen border strobe, and drone halo triggered if the drone crosses outside the indoor architectural footprint.

---

## 📂 Project Structure

```bash
drone-autopilot/
├── config.py                        # Constants, window dimensions, grid resolution, colors & speed settings
├── drone.py                         # Drone flight physics, velocity lerping, corner braking & quadcopter rendering
├── image_processing.py              # Floorplan ingestion, wall inflation & pure PIL fallback parser
├── main.py                          # Simulation loop, HUD diagnostics, geofence monitor & event dispatcher
├── pathfinding.py                   # 8-directional A* engine, BFS Euclidean distance transform & corridor raycaster
├── requirements.txt                 # Python dependencies (pygame, Pillow, numpy)
├── run.bat                          # One-click Windows / WSL launcher script
├── run.ps1                          # PowerShell launcher script
├── Drone_Autopilot_Presentation.pptx # Project architecture presentation deck
├── house.png                        # Architectural floorplan source image
├── debug_map.png                    # Diagnostic binary map output
└── images/
    └── house.png                    # Scaled floorplan asset for runtime rendering
```

---

## 🚀 Installation & Getting Started

### Prerequisites
- **Python 3.10+** (Tested on Python 3.10, 3.11, and 3.12)
- Operating System: Windows, macOS, or Linux / WSL2

### 1. Clone the Repository
```bash
git clone https://github.com/kailash6207/drone-autopilot.git
cd drone-autopilot
```

### 2. Set Up a Virtual Environment (Recommended)
```bash
# On Windows
python -m venv venv
venv\Scripts\activate

# On macOS / Linux
python3 -m venv venv
source venv/bin/activate
```

### 3. Install Dependencies
```bash
pip install -r requirements.txt
```

> **Requirements:**
> - `pygame>=2.5.0` (Rendering engine & window management)
> - `Pillow>=9.0.0` (PIL image processing, morphological filtering & wall inflation)
> - `numpy>=1.20.0` (Numerical engine, with pure PIL fallback if C-extensions are restricted)

### 4. Run the Simulator
```bash
# Direct Python execution:
python main.py
# or
py main.py

# On Windows with Smart App Control (SAC) / DLL Restrictions:
# Simply run the included batch launcher via WSL:
run.bat
# or in PowerShell:
.\run.ps1
```

---

## ⚙️ Configuration & Customization

All primary operational parameters can be customized in [`config.py`](file:///D:/VSCODE/drone-autopilot/config.py):

| Parameter | Default | Description |
| :--- | :--- | :--- |
| `WINDOW_WIDTH` | `1300` | Total application window width in pixels (Map + HUD sidebar). |
| `MAP_WIDTH` | `1000` | Width allocated to the floorplan navigation canvas. |
| `WINDOW_HEIGHT` | `700` | Window and canvas height in pixels. |
| `GRID_SIZE` | `4` | Discretization resolution in pixels per pathfinding grid cell. |
| `DRONE_SPEED` | `5.2` | Base cruising speed constant for drone flight. |
| `DRONE_SIZE` | `20` | Reference diameter for drone collision footprint. |

### Using a Custom Floorplan
1. Place your floorplan image in `images/` (PNG, JPG, or BMP format).
2. Ensure black/dark pixels represent walls and white/light pixels represent open walkable space.
3. Update the file path in [`main.py`](file:///D:/VSCODE/drone-autopilot/main.py#L24-L26):
   ```python
   binary_map = process_floorplan("images/your_floorplan.png")
   floorplan_image = pygame.image.load("images/your_floorplan.png")
   ```

### Modifying Surveillance Waypoints
To alter the automated patrol route, edit the `WAYPOINTS` array in [`main.py`](file:///D:/VSCODE/drone-autopilot/main.py#L42-L45):
```python
WAYPOINTS = [
    (150, 200), (150, 600), (450, 200),
    (550, 200), (550, 600), (800, 400), (800, 600)
]
```

---

## 🗺️ Roadmap & Future Enhancements

- [ ] **Dynamic Moving Obstacles:** Add non-player entities (pets, humans) with real-time Velocity Obstacle (VO) evasion.
- [ ] **Multi-Agent Swarm Mode:** Coordinate multiple autonomous drones with inter-agent collision avoidance (ORCA/RVO).
- [ ] **Fog of War & SLAM:** Real-time Simultaneous Localization and Mapping (LIDAR raycast unmasking of unknown layouts).
- [ ] **Full PID Controller:** Altitude and thrust vector tuning with configurable proportional, integral, and derivative gains.
- [ ] **3D Flight Simulation:** Expand into 3D voxel spaces with 6-DOF (Degrees of Freedom) aerodynamics.

---

## 🤝 Contributing

Contributions, issues, and feature requests are welcome!

1. Fork the Project: `git checkout -b feature/AmazingFeature`
2. Commit your Changes: `git commit -m 'Add some AmazingFeature'`
3. Push to the Branch: `git push origin feature/AmazingFeature`
4. Open a Pull Request

---

## 📜 License

Distributed under the **MIT License**. See `LICENSE` for more information.

---

<div align="center">
Developed by <b><a href="https://github.com/kailash6207">Kailash</a></b> • Built with Python & Pygame
</div>
