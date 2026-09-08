# 🚁 Autonomous Drone Autopilot & Navigation Simulator

<div align="center">

[![Python](https://img.shields.io/badge/Python-3.10%20%7C%203.11%20%7C%203.12-3776AB?style=for-the-badge&logo=python&logoColor=white)](https://www.python.org/)
[![Pygame](https://img.shields.io/badge/Pygame-2.6+-1B5E20?style=for-the-badge&logo=pygame&logoColor=white)](https://www.pygame.org/)
[![OpenCV](https://img.shields.io/badge/OpenCV-Computer%20Vision-5C3EE8?style=for-the-badge&logo=opencv&logoColor=white)](https://opencv.org/)
[![NumPy](https://img.shields.io/badge/NumPy-Numerical%20Engine-013243?style=for-the-badge&logo=numpy&logoColor=white)](https://numpy.org/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg?style=for-the-badge)](https://opensource.org/licenses/MIT)
[![Maintenance](https://img.shields.io/badge/Maintained%3F-Yes-green.svg?style=for-the-badge)](https://github.com/kailash6207/drone-autopilot)

**A real-time autonomous robotics simulator built with Pygame featuring computer-vision floorplan ingestion, center-line biased A\* pathfinding, adaptive corridor raycasting, 2D vector flight physics, 360° proximity radar, and autonomous fail-safe docking.**

[Key Features](#-key-features) •
[System Architecture Flowchart](#-system-architecture--pipeline-flowchart) •
[Mission Lifecycle State Machine](#-mission-lifecycle-state-machine) •
[Pathfinding & Smoothing Workflows](#-algorithmic-deep-dive--workflows) •
[Physics & Aerodynamics Loop](#-flight-physics--kinetics-engine-workflow) •
[Dynamic Obstacle Avoidance Workflow](#-dynamic-in-flight-obstacle-injection-workflow) •
[Sensory Radar Workflow](#-360-octant-radar-sensor-array-workflow) •
[Controls & Sandbox](#-flight-controls--sandbox-guide) •
[Installation](#-installation--getting-started)

</div>

---

## 📌 Overview

**Drone Autopilot** is a 2D autonomous robotic simulation environment designed to model realistic indoor unmanned aerial vehicle (UAV) navigation through complex architectural spaces. 

Instead of relying on idealized point-mass movement or trivial grid paths, the simulator implements an integrated robotics stack:
- **Floorplan Computer Vision Pipeline:** Extracts walkable corridors and inflates obstacle boundaries from raw architectural drawings.
- **Center-Line Biased A\* Planner:** Dynamic wall proximity penalty field repels paths away from tight doorframes and corners, routing the drone naturally through hallway centers.
- **Adaptive Corridor Raycaster Smoother:** Tapered line-of-sight corridor projection converts jagged grid steps into smooth continuous flight paths.
- **Kinetic Vector Flight Mechanics:** Inertial acceleration vectors, velocity caps, angular rotational interpolation, and high-traction aerodynamic drag eliminate drift and overshoot.
- **Fail-Safe Mission State Machine:** Autonomous mission lifecycle managing launch, point-to-point guidance, automated patrol loops, 2-second hover telemetry checks, emergency low-battery return-to-home (RTH), and fast inductive docking.
- **360° Octant Radar & HUD Diagnostics:** 24-ray radar sweep with real-time radial collision warning arcs around the drone chassis and live glass-cockpit telemetry.

---

## 🌟 Key Features

### 🧭 1. Center-Line Biased A* Search Engine
- **50,000 Iteration Computation Headroom:** Capable of navigating deep multi-room partition deadlocks, complex residential floorplans, and narrow doorways without timeouts.
- **Dynamic Inverse-Distance Wall Proximity Penalty:** Evaluates an 8-pixel clearance field around every grid candidate cell ($\sum \frac{480}{\text{dist}}$), pulling generated trajectories toward corridor center-lines.
- **Self-Healing Node Recovery:** If an objective or starting position is clicked inside wall thickness, the pathfinder automatically relaxes the target to the nearest open, valid grid cell within a 6-pixel radius.

### 📐 2. Adaptive Corridor Raycaster Path Smoother
- **Line-of-Sight Bresenham Raycasting:** Greedy lookahead pruning eliminates orthogonal grid steps into optimized straight-line flight vectors.
- **Tapered Corridor Clearance Corridors:** Dynamically scales obstacle clearance bounding envelopes ($1\text{ px}$ at docking/target arrival, $3\text{ px}$ in transition, $5\text{ px}$ during cruising) to preserve clearance during high-speed transit while enabling surgical navigation near tight targets.

### 🛸 3. Inertial Vector Aerodynamics & Physics
- **Rotational Heading Interpolation:** Smooth angular transitions ($\Delta \theta \times 0.15$) pointing the drone's nose cone directly along the velocity vector.
- **Kinetic Drag & Deceleration Modeling:** High-traction friction factor (`0.80`) provides snappy cornering and eliminates sluggish drift.
- **Predictive Curvature Deceleration:** Analyzes future waypoints (6 steps ahead); if path angular delta exceeds $0.4\text{ rad}$, maximum speed is throttled by $75\%$ to execute sharp, controlled turns.

### 📡 4. 360° Proximity Radar Sensor HUD
- **24-Ray Radial Scan Array:** Emits sensor rays in $15^\circ$ angular increments across all $360^\circ$.
- **8-Sector Octant Collision Arcs:** Renders radial status arcs around the chassis (Green = Clear, Red = Proximity Alert).
- **Dual-Mode Scan Range:** Instant toggle between Standard ($120\text{ px}$) and High-Res ($220\text{ px}$) sensor coverage.

### 🔋 5. Power Management & Autonomous Docking Lifecycle
- **Dynamic Battery Discharge:** Power consumption scales dynamically with movement and hardware overdrives ($0.015\%/\text{frame}$ normal, $0.040\%/\text{frame}$ turbo).
- **Automated 2-Second Hover Check:** Holds position and validates telemetry for 2,000 ms upon reaching target before initiating return-to-home.
- **Low-Power RTH Fail-Safe:** Automatically aborts active missions and plots an emergency path to dock when reserves hit $\le 20\%$.
- **Fast Inductive Charging Pad:** Rapid recharge ($+0.3\%/\text{frame}$) upon dock alignment ($< 6\text{ px}$), enforcing a strict $\ge 35\%$ safety cushion before permitting subsequent departures.

### 🧱 6. Real-Time Interactive Sandbox
- **In-Flight Dynamic Obstacle Injection:** Left-click anywhere on the map to spawn grid-locked obstacles; in-flight paths recalculate instantaneously.
- **Base Station Clearance Guard:** Intelligent validation prevents accidental obstacle placement over the charging pad.
- **Geofence Boundary Monitoring:** Detects perimeter threshold breaches ($25\text{ px}$ margin) and triggers pulsing HUD alerts.

---

## 🏗️ System Architecture & Pipeline Flowchart

The following flowchart details the end-to-end data pipeline from raw floorplan ingestion to real-time rendering:

```mermaid
flowchart TD
    subgraph S1 ["🖼️ Phase 1: Computer Vision Ingestion"]
        A["Input: house.png Floorplan"] --> B["Grayscale Conversion ('L')"]
        B --> C["Scale to Window Canvas (1000 x 700 px)"]
        C --> D["Morphological Inflation: ImageFilter.MinFilter(3) Pass 1"]
        D --> E["Morphological Inflation: ImageFilter.MinFilter(3) Pass 2"]
        E --> F["Thresholding: Matrix Pixels < 50 => Obstacle (1), else Walkable (0)"]
        F --> G["binary_map: 2D Occupancy Matrix"]
    end

    subgraph S2 ["🎮 Phase 2: User Input & Mission Dispatcher"]
        H1["Left-Click (Initial)"] --> I1["Spawn Base Station Pad & Drone"]
        H2["Left-Click (Active)"] --> I2["Inject 36x36 px Obstacle Block"]
        H3["Right-Click (Active)"] --> I3["Set New Objective Coordinates"]
        H4["Press 'P' Key"] --> I4["Engage 7-Waypoint Patrol Queue"]
        H5["Sidebar UI Buttons"] --> I5["Toggle Turbo Boost / High-Res Radar"]
    end

    subgraph S3 ["🧭 Phase 3: Path Planning & Smoothing"]
        G --> J["A* Planner (50,000 Iteration Cap)"]
        I3 & I4 --> J
        I2 -.->|"Triggers Live Re-Plan"| J
        J --> K["Compute Wall Proximity Repulsion Field (8px Radius)"]
        K --> L["Raw Discrete Grid Waypoints"]
        L --> M["Adaptive Tapered Corridor Raycaster (Bresenham LOS)"]
        M --> N["Optimized Smooth Vector Trajectory"]
    end

    subgraph S4 ["🛸 Phase 4: Flight Kinetics & Telemetry"]
        N --> O["Trajectory Tracker & Curvature Lookahead"]
        O --> P["Vector Acceleration: dx, dy => Heading Angle"]
        P --> Q["Aerodynamic Drag Damping (friction = 0.80)"]
        Q --> R["Velocity Clamping & Position Integration"]
        R --> S["Dynamic Battery Discharge Simulation"]
    end

    subgraph S5 ["📡 Phase 5: Sensors, Safety & HUD"]
        R --> T["360° 24-Ray Octant Radar Array"]
        T --> U["8-Sector Collision Evaluation"]
        R --> V["Geofence Perimeter Validator (25px Boundary)"]
        S --> W["Low Battery Fail-Safe Monitor (<= 20% => RTH)"]
        W & U & V --> X["Glass Cockpit Sidebar Diagnostics Display"]
        R --> Y["Procedural Quadcopter Drawing Pipeline"]
    end

    S1 --> S3
    S2 --> S3
    S3 --> S4
    S4 --> S5
```

---

## 🔄 Mission Lifecycle State Machine

The drone's internal autopilot state machine governs mission lifecycles, emergency interrupts, and battery fail-safes:

```mermaid
stateDiagram-v2
    [*] --> OFFLINE: Simulation Booted
    OFFLINE --> STANDBY_DOCKED: Left-Click on Map (Deploy Charging Base & Drone)

    state STANDBY_DOCKED {
        [*] --> FAST_CHARGING: Battery < 100%
        FAST_CHARGING --> READY_TO_LAUNCH: Battery >= 35%
        READY_TO_LAUNCH --> FAST_CHARGING: Battery Top-Off (+0.3%/frame)
    }

    STANDBY_DOCKED --> MANUAL_NAVIGATION: Right-Click Target (Battery >= 35%)
    STANDBY_DOCKED --> PATROL_SURVEILLANCE: Press 'P' Key (Battery >= 35%)

    state MANUAL_NAVIGATION {
        [*] --> CRUISE_SPEED: Follow Smoothed Path Waypoints
        CRUISE_SPEED --> CORNERING_BRAKE: Path Curvature > 0.4 rad
        CORNERING_BRAKE --> CRUISE_SPEED: Turn Executed (Speed Restored)
    }

    state PATROL_SURVEILLANCE {
        [*] --> PATROL_WAYPOINT: Route to Current Queue Node
        PATROL_WAYPOINT --> PATROL_WAYPOINT: Pop Node & Select Next Waypoint
    }

    MANUAL_NAVIGATION --> HOVER_TELEMETRY_CHECK: Arrived at Destination Target
    PATROL_SURVEILLANCE --> HOVER_TELEMETRY_CHECK: Final Patrol Waypoint Reached

    state HOVER_TELEMETRY_CHECK {
        [*] --> HOLD_STATION: Counter-Thrust Damping Active
        HOLD_STATION --> COUNTDOWN: Hold Position for 2,000 ms
    }

    HOVER_TELEMETRY_CHECK --> RETURN_TO_HOME_RTH: 2.0s Timer Elapses
    MANUAL_NAVIGATION --> RETURN_TO_HOME_RTH: Battery <= 20% (Low Power Emergency)
    PATROL_SURVEILLANCE --> RETURN_TO_HOME_RTH: Battery <= 20% (Low Power Emergency)

    state RETURN_TO_HOME_RTH {
        [*] --> PLAN_RTH_PATH: Compute Optimal Route to Base Pad
        PLAN_RTH_PATH --> HOMING_FLIGHT: Transit to Dock Coordinates
    }

    RETURN_TO_HOME_RTH --> STANDBY_DOCKED: Distance to Home < 6 px
    MANUAL_NAVIGATION --> CRITICAL_SHUTDOWN: Battery <= 0%
    PATROL_SURVEILLANCE --> CRITICAL_SHUTDOWN: Battery <= 0%
    CRITICAL_SHUTDOWN --> STANDBY_DOCKED: Emergency Base Reset
```

---

## 🔬 Algorithmic Deep Dive & Workflows

### 1. Master A* Pathfinding Engine Workflow
The simulator uses an upgraded A\* pathfinder with adaptive start/goal node relaxation, wall clearance repulsion, and high iteration headroom:

```mermaid
flowchart TD
    StartA["Input: start(x,y), goal(x,y), binary_map"] --> CheckGoal{"Is goal node walkable?"}
    
    CheckGoal -- No --> GoalRelax["Spiral Search (Radius 1 to 5 px) for nearest open cell"]
    GoalRelax --> GoalFound{"Open cell found?"}
    GoalFound -- No --> ReturnFail["Return Empty Path: []"]
    GoalFound -- Yes --> UpdateGoal["Set goal = Nearest Open Cell"]
    CheckGoal -- Yes --> CheckStart{"Is start node walkable?"}
    
    UpdateGoal --> CheckStart
    CheckStart -- No --> StartRelax["3x3 Neighborhood Search for open cell"]
    StartRelax --> StartFound{"Open cell found?"}
    StartFound -- No --> ReturnFail
    StartFound -- Yes --> UpdateStart["Set start = Nearest Open Cell"]
    CheckStart -- Yes --> InitQueue["Push (0, start) to open_set Priority Queue<br/>Initialize g_score[start] = 0<br/>Initialize came_from = {}"]

    UpdateStart --> InitQueue
    InitQueue --> Loop{"open_set not empty AND<br/>iterations < 50,000?"}
    
    Loop -- No --> ReturnFail
    Loop -- Yes --> PopNode["Pop node with lowest f_score from open_set"]
    PopNode --> IsGoal{"Is current node == goal?"}
    
    IsGoal -- Yes --> ReconstructPath["Trace came_from back to start<br/>Reverse path array<br/>Forward to Raycaster Smoother"]
    IsGoal -- No --> GenNeighbors["Generate 4-Way Neighbors:<br/>(x, y-1), (x, y+1), (x-1, y), (x+1, y)"]
    
    GenNeighbors --> NeighborLoop["For each neighbor (nx, ny) within map bounds"]
    NeighborLoop --> IsWalkable{"Is neighbor cell walkable?"}
    
    IsWalkable -- No --> NextNeighbor["Continue to next neighbor"]
    IsWalkable -- Yes --> CalcCost["base_cost = 1<br/>proximity_penalty = 0<br/>Scan 8-pixel neighborhood around cell"]
    
    CalcCost --> WallScan["For all pixels (px+dx, py+dy) in 8px radius:<br/>If pixel is Wall => proximity_penalty += floor(480 / dist)"]
    WallScan --> TotalCost["cost = base_cost + proximity_penalty<br/>tentative_g = g_score[current] + cost"]
    
    TotalCost --> BetterPath{"tentative_g < g_score[neighbor]?"}
    BetterPath -- Yes --> UpdateScores["came_from[neighbor] = current<br/>g_score[neighbor] = tentative_g<br/>f_score[neighbor] = tentative_g + Manhattan(neighbor, goal)<br/>Push neighbor to open_set"]
    BetterPath -- No --> NextNeighbor
    UpdateScores --> NextNeighbor
    NextNeighbor --> Loop
```

### Mathematical Cost Formulation
Every candidate cell $n$ is priced dynamically to naturally pull flight corridors toward room center-lines:

$$f(n) = g(n) + h(n) + \text{Penalty}_{\text{proximity}}(n)$$

$$\text{Penalty}_{\text{proximity}}(n) = \sum_{dx=-8}^{8} \sum_{dy=-8}^{8} \left\lfloor \frac{480}{\sqrt{dx^2 + dy^2}} \right\rfloor \quad \forall (px+dx, py+dy) \in \text{Walls}$$

---

### 2. Adaptive Tapered Corridor Raycaster Path Smoother
Standard A\* output produces jagged 90-degree orthogonal step lines. Our greedy corridor raycaster evaluates line-of-sight clearance across varying corridor bounding envelopes:

```mermaid
flowchart TD
    InRaw["Input: raw_path from A* [P0, P1, ..., Pn]"] --> InitSmooth["current_index = 0<br/>smoothed_path = [ raw_path[0] ]"]
    
    InitSmooth --> OuterLoop{"current_index < len(raw_path) - 1?"}
    OuterLoop -- No --> CheckFinal{"smoothed_path[-1] == goal?"}
    CheckFinal -- No --> AppendGoal["smoothed_path.append(goal)"]
    CheckFinal -- Yes --> OutputPath["Output: Optimized Smoothed Flight Path"]
    AppendGoal --> OutputPath

    OuterLoop -- Yes --> SetLookAhead["look_ahead = len(raw_path) - 1 (Greedy Search from End)"]
    SetLookAhead --> InnerLoop{"look_ahead > current_index?"}
    
    InnerLoop -- No --> StepForward["current_index = current_index + 1<br/>smoothed_path.append(raw_path[current_index])"]
    StepForward --> OuterLoop

    InnerLoop -- Yes --> CalcDistGoal["Calculate Distance: dist_to_goal = Euclidean(P_current, Goal)"]
    CalcDistGoal --> SelectBound{"Evaluate Distance Envelope"}
    
    SelectBound -- "dist < 8 px" --> Bound1["bound_size = 1 px<br/>(Surgical Target Alignment)"]
    SelectBound -- "8 <= dist < 20 px" --> Bound3["bound_size = 3 px<br/>(Intermediate Transition)"]
    SelectBound -- "dist >= 20 px" --> Bound5["bound_size = 5 px<br/>(Wide Corridor Cruising)"]

    Bound1 & Bound3 & Bound5 --> CastRay["Bresenham Raycaster from P_current to P_lookahead"]
    CastRay --> CheckPixels["For every ray coordinate (rx, ry):<br/>Check bounding box: (rx ± bound_size, ry ± bound_size)"]
    
    CheckPixels --> HitWall{"Any wall detected in bounding box?"}
    HitWall -- Yes --> DecrementLookAhead["Ray Blocked!<br/>look_ahead = look_ahead - 1"]
    DecrementLookAhead --> InnerLoop
    
    HitWall -- No --> RayClear["Clear Line of Sight Confirmed!<br/>smoothed_path.append(raw_path[look_ahead])<br/>current_index = look_ahead"]
    RayClear --> OuterLoop
```

---

## 🛸 Flight Physics & Kinetics Engine Workflow

Every simulation frame ($60\text{ FPS}$), vector physics equations integrate thrust, rotational orientation, and aerodynamic drag damping:

```mermaid
flowchart TD
    FrameTick["Clock Tick (60 FPS)"] --> CheckPower{"Is Battery > 0%?"}
    CheckPower -- No --> ZeroCutoff["Drone Inoperable: vx = 0, vy = 0"]
    CheckPower -- Yes --> EvalUpgrades["Read Hardware Flags:<br/>Turbo Boost: 3x Speed, 3x Accel, 2.67x Drain<br/>Normal: 1x Speed, 1x Accel, 1x Drain"]
    
    EvalUpgrades --> BatteryDrain["Apply Dynamic Battery Discharge:<br/>battery = battery - drain_rate"]
    BatteryDrain --> HasTarget{"Has active target coordinate?"}
    
    HasTarget -- Yes --> CalcHeading["dx = target_x - x<br/>dy = target_y - y<br/>target_angle = atan2(dy, dx)"]
    CalcHeading --> ApplyThrust["Apply Vector Thrust Force:<br/>vx += cos(angle) * acceleration<br/>vy += sin(angle) * acceleration"]
    HasTarget -- No --> DragOnly["No Active Thrust (Drift / Station Keeping)"]
    
    ApplyThrust & DragOnly --> VelocityClamp{"speed = sqrt(vx² + vy²) > max_speed?"}
    VelocityClamp -- Yes --> Clamp["vx = (vx / speed) * max_speed<br/>vy = (vy / speed) * max_speed"]
    VelocityClamp -- No --> ApplyDrag["Apply Aerodynamic Drag Friction:<br/>vx *= 0.80<br/>vy *= 0.80"]
    Clamp --> ApplyDrag
    
    ApplyDrag --> PosUpdate["Update Coordinates:<br/>x += vx<br/>y += vy"]
    PosUpdate --> RotInterp["Smooth Heading Rotation:<br/>diff = (target_angle - angle + 180) % 360 - 180<br/>angle += diff * 0.15"]
    
    RotInterp --> CheckArrival{"Distance to next node <= arrival_radius?<br/>arrival_radius = 8 + floor(speed * 2.5)"}
    CheckArrival -- Yes --> PopNode["Pop node from active path list"]
    CheckArrival -- No --> EndPhysics["Physics Frame Complete"]
    PopNode --> EndPhysics
```

---

## 🧱 Dynamic In-Flight Obstacle Injection Workflow

Users can dynamically manipulate the floorplan environment during flight. The diagram below illustrates how obstacles are verified and real-time paths are regenerated:

```mermaid
sequenceDiagram
    autonumber
    actor User as User (Map Canvas)
    participant Dispatcher as Event Dispatcher
    participant BasePad as Docking Guard Validator
    participant Grid as Binary Occupancy Map
    participant Planner as Master A* Pathfinder
    participant Drone as Drone Flight Controller

    User->>Dispatcher: Left-Click on Canvas (mx, my)
    Dispatcher->>BasePad: Validate Coordinate (Is point within Charging Pad?)
    alt Point Overlaps Charging Pad
        BasePad-->>Dispatcher: REJECT ("Cannot block charging pad area!")
    else Coordinate Safe
        BasePad->>Grid: Write 36x36 px Obstacle Block (Value = 1)
        Grid-->>Dispatcher: Obstacle Registered in Matrix
        alt Drone Has Active Destination Target
            Dispatcher->>Planner: Request Emergency Path Recalculation
            Note over Planner: Start: (drone.x // 4, drone.y // 4)<br/>Goal: (target.x // 4, target.y // 4)
            Planner->>Grid: Query Updated Occupancy & Proximity Field
            Grid-->>Planner: Return Walkable Cells & Repulsion Costs
            Planner->>Planner: A* Search + Tapered Corridor Raycaster
            Planner-->>Drone: Inject New Smoothed Vector Trajectory
            Note over Drone: Seamless Mid-Flight Path Correction!
        end
    end
```

---

## 📡 360° Octant Radar Sensor Array Workflow

The collision radar casts 24 radial rays to build an 8-octant spatial collision map around the drone chassis:

```mermaid
flowchart TD
    StartRadar["Initiate 360° Radar Sweep"] --> ConfigRange{"High-Res Radar Active?"}
    ConfigRange -- Yes --> Set220["max_scan_range = 220 px"]
    ConfigRange -- No --> Set120["max_scan_range = 120 px"]
    
    Set220 & Set120 --> InitOctants["Initialize 8 Octant Sectors: [False, ..., False]<br/>Loop angles: 0° to 360° in 15° steps (24 Rays)"]
    
    InitOctants --> CalcOctant["Compute Sector Index:<br/>sector = floor( (angle + 22.5°) % 360° / 45° )"]
    CalcOctant --> RayMarch["March Ray: dist = 20 px to max_scan_range (Step = 6 px)"]
    
    RayMarch --> SamplePixel["Sample Pixel Coordinate:<br/>sx = x + cos(angle) * dist<br/>sy = y + sin(angle) * dist"]
    
    SamplePixel --> BoundsCheck{"Within Map Canvas Bounds?"}
    BoundsCheck -- No --> BreakRay["Break Ray (Reached Perimeter)"]
    BoundsCheck -- Yes --> HitCheck{"binary_map[sy][sx] > 0 (Obstacle Hit)?"}
    
    HitCheck -- Yes --> FlagSector["Draw Red Collision Point<br/>radar_sectors[sector] = True<br/>sensor_collision = True"]
    FlagSector --> NextRay["Advance to Next Radial Angle"]
    
    HitCheck -- No --> DrawRay["Draw Blue Ray Particle"]
    DrawRay --> DistanceCheck{"dist reached max_scan_range?"}
    DistanceCheck -- No --> RayMarch
    DistanceCheck -- Yes --> NextRay
    
    NextRay --> AllDone{"All 24 Rays Evaluated?"}
    AllDone -- No --> CalcOctant
    AllDone -- Yes --> RenderArcs["Draw 8-Octant HUD Arcs around Drone:<br/>If sector is True => Draw Red Arc (Alert)<br/>If sector is False => Draw Green Arc (Clear)"]
    RenderArcs --> UpdateHUD["Push Telemetry Alert to Sidebar Display"]
```

---

## 🎮 Flight Controls & Sandbox Guide

| Input | Action | Context & Behavior |
| :--- | :--- | :--- |
| **Left-Click (Map)** | **Initial Drone Deployment** | Spawns the drone and sets its permanent home charging dock station (must be clicked on open floor). |
| **Left-Click (Map)** | **Spawn Obstacle Block** | When deployed, spawns a $36\times 36\text{ px}$ obstacle block. Triggers immediate dynamic path recalculation in real-time. Protected: Cannot place over base station. |
| **Right-Click (Map)** | **Set Target Destination** | Computes smoothed vector trajectory to the clicked coordinate. If docked, initiates departure (requires $\ge 35\%$ battery). |
| **`[P]` Key** | **Toggle Auto Patrol** | Starts/stops autonomous surveillance loop through 7 residential waypoints across the property. |
| **Turbo Boost (UI)** | **Toggle Turbo Drive** | Increases speed ceiling ($3\times$) and acceleration ($3\times$); increases battery consumption ($2.67\times$). |
| **High-Res Radar (UI)** | **Toggle Radar Range** | Extends 360° sensor raycasting envelope from $120\text{ px}$ to $220\text{ px}$. |

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
- **`GEOFENCE BREACHED!`**: Warning triggered when drone approaches within $25\text{ px}$ of simulator boundary.

---

## 📂 Project Structure

```bash
drone-autopilot/
├── config.py             # Global constants, window dimensions, grid resolution & color schemes
├── drone.py              # Drone physics model, kinetic equations, drag damping & quadcopter renderer
├── image_processing.py   # Floorplan ingestion, scaling, morphological dilation & grid binarization
├── main.py               # Main simulation loop, event dispatcher, HUD dashboard & mission state machine
├── pathfinding.py        # Master A* planner, proximity cost field & tapered corridor raycaster
├── requirements.txt      # Python dependencies (pygame, opencv-python, numpy, Pillow)
├── house.png             # Architectural floorplan source image
├── debug_map.png         # Diagnostic visual map output
└── images/
    └── house.png         # Scaled floorplan asset for runtime rendering
```

---

## 🚀 Installation & Getting Started

### Prerequisites
- **Python 3.10+** (Tested on Python 3.10, 3.11, and 3.12)
- Operating System: Windows, macOS, or Linux

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

> **Requirements include:**
> - `pygame` (Rendering engine & window management)
> - `opencv-python` (Computer vision image handling)
> - `numpy` (High-performance array operations)
> - `Pillow` (PIL morphological image filtering & wall inflation)

### 4. Run the Simulator
```bash
# Windows
py main.py
# or
python main.py

# macOS / Linux
python3 main.py
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
| `DRONE_SPEED` | `4` | Base maximum speed constant for drone movement. |
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
