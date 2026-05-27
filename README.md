# Drone Autopilot: Autonomous Navigation Simulator

An interactive 2D drone simulation built with Pygame that demonstrates advanced autonomous flight mechanics, vector-based physics, and real-time pathfinding. The system features an intelligent control state machine designed to navigate tight floorplans, manage onboard power reserves, and handle automatic mission docking sequences.

---

## Key Features

* **Advanced Autonomous Navigation:** Powered by a customized A* search algorithm featuring an increased computational budget to navigate deep room partitions and complex architectural dead-ends.
* **Inertial Momentum Stabilization:** Physics-engine tracking that identifies kinetic overshoots at targets and applies immediate counter-thrust damping to ensure structural stability.
* **Automatic Timed Docking:** An autonomous mission lifecycle where the drone locks onto a target, completes a 2-second telemetry hover check upon arrival, and instantly routes itself back to the home base station to charge.
* **Proximity Sensor Array:** A real-time, 360-degree sector-based radar system that feeds obstacle collision alerts straight into a persistent sidebar diagnostic HUD.

---

## System Blueprint

* **main.py:** Core engine loop, system state updates, input management, and panel UI rendering.
* **drone.py:** Vector physics engine, velocity components acceleration, and friction braking.
* **pathfinding.py:** Coordinate grid validation, center-line layout pricing, and path smoothing.
* **image_processing.py:** Floorplan image binarization, extracting pixel arrays into walkable data arrays.
* **config.py:** Universal constants, window dimensions, grid configurations, and theme colors.

---

## Installation & Setup Guide

### Step 1: Open project folder in VSCode
Open your terminal and navigate to your project directory:
```bash
cd drone-autopilot

---

pip install -r requirements.txt

---

py main.py

---

Command Console Controls
[Left-Click Map] Spawn a grid-locked obstacle structure in real-time to challenge the drone's vector routing.

[Right-Click Map] Set an objective destination point. If docked, this triggers a charging pad breakout (requires a safe 35% battery cushion).

[P Key] Toggle the autonomous surveillance loop through hardcoded house waypoints.

[UI Sidebar Buttons] Manually toggle Core Turbo Boost or High-Res Radar Sensors to instantly alter engine flight characteristics.
