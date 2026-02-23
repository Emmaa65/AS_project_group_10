# OMPL Planner Package - 3D Cave Exploration

## Übersicht

Dieses Package implementiert autonome 3D-Höhlenerkundung mit einer State Machine für die Transition von Waypoint-Navigation zu Frontier-basierter Exploration.

Die **Frontier-Erkennung** (3D Punkte aus OctoMap) wird durch die `navigation_pkg/frontier_exploration` Node bereitgestellt.

## Architektur

### Node-Struktur

```
┌─────────────────────────────────────────────────────────────┐
│  existing: basic_waypoint_node (Phase 1: fly to cave)      │
│    - subscribes: "current_state_est" (Odometry)            │
│    - publishes: "trajectory" (PolynomialTrajectory4D)       │
└────────────────────────┬────────────────────────────────────┘
                         │
                         ▼
┌─────────────────────────────────────────────────────────────┐
│  existing: navigation_pkg/frontier_exploration              │
│    - subscribes: "octomap_full" (OctoMap from octomap_server)
│    - publishes: "frontier_points_3d" (PointCloud2)         │
│    - Detects 3D frontier points at explored/unexplored boundary
└────────────────────────┬────────────────────────────────────┘
                         │
                         ▼
┌─────────────────────────────────────────────────────────────┐
│  ompl_planner_pkg: Two coordinated nodes                    │
├─────────────────────────────────────────────────────────────┤
│                                                              │
│  1. exploration_manager_node (STATE MACHINE)                │
│     ├─ subscribes:                                          │
│     │  - "current_state_est" (Odometry)                    │
│     │  - "frontier_points_3d" (PointCloud2)                │
│     ├─ publishes:                                           │
│     │  - "target_frontier" (PointStamped)                  │
│     │  - "exploration_markers" (MarkerArray for RViz)      │
│     │                                                       │
│     └─ STATE TRANSITIONS:                                   │
│        INITIALIZATION → WAYPOINT_NAVIGATION →              │
│        WAITING_AT_ENTRANCE → AUTONOMOUS_EXPLORATION →      │
│        EXPLORATION_COMPLETE                                │
│                                                              │
│  2. rrt_path_planner_node                                   │
│     ├─ subscribes:                                          │
│     │  - "target_frontier" (PointStamped)                  │
│     │  - "current_state_est" (Odometry)                    │
│     │  - "octomap_point_cloud_centers" (PointCloud2)       │
│     └─ publishes:                                           │
│        - "trajectory" (PolynomialTrajectory4D)              │
│        - "planned_path_markers" (MarkerArray for RViz)      │
│                                                              │
└────────────────────────┬────────────────────────────────────┘
                         │
                         ▼
┌─────────────────────────────────────────────────────────────┐
│  existing: trajectory_executor                              │
│    - subscribes: "trajectory" (PolynomialTrajectory4D)      │
│    - publishes: "desired_state" (MultiDOFJointTrajectory)  │
└────────────────────────┬────────────────────────────────────┘
                         │
                         ▼
┌─────────────────────────────────────────────────────────────┐
│  existing: controller_node                                  │
│    - subscribes: "desired_state" + "current_state_est"     │
│    - publishes: Motor commands (mav_msgs/Actuators)        │
└─────────────────────────────────────────────────────────────┘
```

## Topics

### Core Exploration Topics

| Topic | Type | Source → Dest | Description |
|-------|------|---------|-------------|
| `frontier_points_3d` | PointCloud2 | navigation_pkg → exploration_manager | 3D frontier points from OctoMap boundary |
| `target_frontier` | PointStamped | exploration_manager → rrt_path_planner | Selected target frontier for planning |
| `trajectory` | PolynomialTrajectory4D | rrt_path_planner → trajectory_executor | Planned path in polynomial format |

### Shared Topics (with existing system)

| Topic | Type | Description |
|-------|------|-------------|
| `current_state_est` | Odometry | Current drone position/velocity (from simulator/SLAM) |
| `octomap_full` | Octomap | Full 3D occupancy map from octomap_server |
| `octomap_point_cloud_centers` | PointCloud2 | OctoMap occupancy centers (for visualization/collision checking) |

### Debug Topics

| Topic | Type | Description |
|-------|------|-------------|
| `exploration_markers` | MarkerArray | Debugging markers (cave entrance, current pos, frontiers) |
| `planned_path_markers` | MarkerArray | Planned path visualization |

## Nodes im Detail

### 1. frontier_exploration_node (navigation_pkg) ⭐ EXTERNAL

**Funktion:** Extrahiert 3D Grenzpunkte zwischen erkundeter und unerkundeter Raum aus dem OctoMap.

**Publisher:** `frontier_points_3d` (PointCloud2)

**Implementierung:** Siehe `navigation_pkg/src/frontier_exploration.cpp`

Wenn nicht vorhanden, kopiere die `frontier_exploration` Node von diesem Package oder starten Sie sie separat:
```bash
ros2 run navigation_pkg frontier_exploration
```

### 2. exploration_manager_node (ompl_planner_pkg) ⭐ OUR NODE

**Funktion:** Zentrale Steuerung - orchestriert den Übergang von Waypoint-Navigation zu autonomer Exploration und selektiert die beste Frontier.

**State Machine:**

```
INITIALIZATION
  ↓ (when first odometry received)
WAYPOINT_NAVIGATION
  ↓ (when distance_to_cave_entrance < tolerance)
WAITING_AT_ENTRANCE
  ↓ (immediate)
AUTONOMOUS_EXPLORATION
  ├─ Select best frontier (info_gain / distance)
  ├─ Publish target_frontier
  └─ Wait for arrival → select next frontier
  ↓ (when no more frontiers)
EXPLORATION_COMPLETE
```

**Parameter:**
- `cave_entrance_*` [m]: Koordinaten des Höhleneingangs (must match waypoint!)
- `entrance_reach_tolerance` [m]: Distance tolerance to declare "at entrance"
- `frontier_update_rate` [Hz]: Wie oft Frontier neu evaluiert
- `min_frontier_distance` [m]: Ignoriere zu nahe Frontiers
- `max_frontier_distance` [m]: Ignoriere zu weit entfernte Frontiers

**Frontier-Scoring:** 
```
score = info_gain / (distance + epsilon)
```
Wählt Frontier mit höchstem Score (Gewinn pro Entfernung).

### 3. rrt_path_planner_node (ompl_planner_pkg) ⭐ OUR NODE

**Funktion:** Plant Pfad vom aktuellen Ort zum Ziel-Frontier mit RRT* oder straight-line (Phase 1) und konvertiert zu PolynomialTrajectory.

**Parameter:**
- `max_planning_time` [s]: Max Zeit für RRT* planning
- `step_size` [m]: RRT step size
- `max_iterations` [int]: Max RRT iterations
- `max_velocity` [m/s]: Constraint für Trajectory generation
- `max_acceleration` [m/s²]: Constraint für Trajectory generation

**Phase 1 Implementierung:**
- Simple straight-line interpolation (kein echtes RRT*)
- TODO: Integration mit multidim_rrt_cpp für echte RRT* planning

## Parameter-File

Alle Parameter für `ompl_planner_pkg` sind zentral in `config/exploration_params.yaml` definiert.

Die `frontier_exploration` Node (navigation_pkg) hat ihr eigenes Parameter-File in `navigation_pkg/config/`.

## Starten des Systems

### Komplettes System (avec Waypoints + Exploration):

```bash
# Terminal 1: Simulator/Main System
ros2 launch system_bringup main.launch.py

# Terminal 2: OctoMap Builder
ros2 launch octomap_mapping octomap_mapping.launch.py

# Terminal 3: Frontier Detection (from navigation_pkg)
ros2 run navigation_pkg frontier_exploration

# Terminal 4: Exploration (ompl_planner_pkg)
ros2 launch ompl_planner_pkg exploration.launch.py

# Terminal 5: RViz Visualization
rviz2 -d /path/to/config.rviz
```

### Für Debugging: Individual Nodes

```bash
# Nur exploration manager
ros2 run ompl_planner_pkg exploration_manager_node

# Nur RRT path planner  
ros2 run ompl_planner_pkg rrt_path_planner_node
```

### RViz Marker Topics zum Visualisieren

Füge in RViz hinzu:
- `exploration_markers` → Sehen cave entrance + aktuelle Pos + Frontiers
- `planned_path_markers` → Sehen geplanten Pfad
- `frontier_points_3d` → Alle Frontier PointCloud
- `octomap_full` → Das 3D OctoMap

## Phase 1 vs Future Work

### Jetzt implementiert (Phase 1):
- ✅ State Machine für Waypoint ↔ Exploration
- ✅ Frontier-Erkennung aus OctoMap (via navigation_pkg)
- ✅ Frontier-Selection (score-based)
- ✅ Straight-line path planning
- ✅ Trajectory generation & tracking

### TODO (Phase 2):
- ❌ Echtes RRT* Planning (Integration mit multidim_rrt_cpp)
- ❌ Visited exploration tracking (avoid revisiting)
- ❌ Collision detection/avoidance
- ❌ Loop-closure und map merging
- ❌ Battery awareness und return-to-base
- ❌ Adaptive frontier evaluation (info-rich frontiers bevorzugen)

## Verbindung zu bestehenden Packages

| Package | Verwendung |
|---------|-----------|
| `basic_waypoint_pkg` | Fliegt zu Höhleneingang mit vordefinierten Waypoints |
| `navigation_pkg` | Frontier-Erkennung (3D) aus OctoMap **← DIESE NODE NUTZEN** |
| `octomap_server` | Erstellt 3D OctoMap aus Kameras |
| `octomap_mapping` | ROS2 OctoMap Integration |
| `multidim_rrt_cpp` | (TODO) RRT* Planning - noch zu integrieren |
| `mav_trajectory_generation` | Polynomial trajectory generation |
| `controller_pkg` | Low-level UAV control |

## Troubleshooting

### Frontier-Punkte werden nicht empfangen
- Prüfen ob `frontier_points_3d` gepublished wird: `ros2 topic echo /frontier_points_3d`
- Prüfen ob `frontier_exploration` Node läuft: `ros2 node list | grep frontier`
- Prüfen ob OctoMap gepublished wird: `ros2 topic echo /octomap_full`

### Drohne fliegt nicht zur Frontier
- Überprüfen ob `target_frontier` gepublished wird: `ros2 topic echo /target_frontier`
- Überprüfen ob `trajectory` gepublished wird: `ros2 topic echo /trajectory`
- Prüfen state machine state: `ros2 topic echo /exploration_markers`

### Zu langsam
- `frontier_update_rate` in exploration_manager erhöhen (kostet CPU)
- `step_size` im RRT erhöhen (vereinfacht Pfad)
- `max_iterations` reduzieren

## Authors

Autonomous Systems Project Group 10
