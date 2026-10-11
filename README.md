# Trajectory Planning: Frenet-Frame Overtaking Simulation

A Python simulation of autonomous-vehicle local trajectory planning. A self-driving **ego vehicle** follows a spline-defined road, adapts its speed to the traffic ahead, and **overtakes a slower vehicle** by choosing the lowest-cost, collision-free, dynamically feasible trajectory at every planning step.

The planner is built on the **Frenet optimal trajectory** method (Werling et al.), in which motion is planned in road-aligned coordinates instead of raw (x, y). The repository also contains the building blocks the planner is made of (cubic splines, quintic and quartic polynomials), each runnable on its own.

> Course project: *Wait for Pedestrian + Overtake*, Masters in AI Engineering of Autonomous Systems (June 2024). The slide deck is included as `Project_Pedestrian_Overtake.pptx`.

---

## Table of Contents

1. [Project goals](#project-goals)
2. [How it works](#how-it-works)
3. [Repository structure](#repository-structure)
4. [Installation](#installation)
5. [Running the simulations](#running-the-simulations)
6. [Module reference](#module-reference)
7. [Configuration and tuning](#configuration-and-tuning)
8. [The overtaking scenario in detail](#the-overtaking-scenario-in-detail)
9. [Known limitations](#known-limitations)
10. [Roadmap](#roadmap)
11. [Credits and references](#credits-and-references)

---

## Project goals

The scenarios explored in this project:

1. Wait for a pedestrian on a zebra crossing.
2. Drive on again after the pedestrian leaves.
3. Overtake another vehicle and return to the original lane.
4. Overtake a vehicle **while on a curve**.

The code in this repository implements goals 3 and 4: overtaking on a curved, spline-defined road. See [Known limitations](#known-limitations) for the state of the pedestrian logic.

---

## How it works

### 1. Reference path: cubic splines
The road centerline is defined by a handful of waypoints. A **2D cubic spline parameterised by arc length** turns them into a smooth curve, and gives position, heading (yaw) and curvature at any distance `s` along the road.

### 2. Frenet coordinates
Instead of planning in Cartesian (x, y), positions are described relative to the road centerline:

| Symbol | Meaning |
|--------|---------|
| `s` | Longitudinal position: distance travelled along the centerline |
| `d` | Lateral position: signed offset from the centerline |

This makes the problem follow the road's curvature automatically and lets longitudinal and lateral motion be planned **independently**.

### 3. Candidate trajectory generation
At every planning step, the ego vehicle generates a large set of candidate trajectories:

- **Lateral motion `d(t)`**: a **quintic polynomial** that takes the car from its current lateral state (position, velocity, acceleration) to a sampled target offset with zero lateral velocity and acceleration.
- **Longitudinal motion `s(t)`**: a **quartic polynomial** (velocity-keeping) that takes the car from its current speed and acceleration to a sampled target speed.

The candidates are the combination of every sampled target offset, planning horizon and target speed.

### 4. Back to the real world
Each (s, d) candidate is converted to global (x, y) coordinates through the spline, and yaw, segment length and curvature are derived along it.

### 5. Feasibility filtering
A candidate is **discarded** if it violates any of:

- maximum speed,
- maximum acceleration,
- maximum curvature,
- a **collision check** against the predicted path of the obstacle vehicle.

### 6. Cost-based selection
Each surviving candidate is scored and the cheapest one wins:

```
C_lat = K_J · Σ jerk_d²  +  K_T · T  +  K_D · d_end²
C_lon = K_J · Σ jerk_s²  +  K_T · T  +  K_D · (V_target − v_end)²
C_tot = K_LAT · C_lat  +  K_LON · C_lon
```

Low jerk (comfort), short horizon, ending near the centerline, and reaching the target speed are all rewarded. Only the first step of the winning trajectory is executed before the whole process **replans** (receding-horizon control).

### 7. Dynamic speed adjustment
The ego vehicle's target speed depends on how far ahead the obstacle is. This is what turns "follow the road" into "close in on, pass, and drop back behind" the slower car:

| Situation | Target speed |
|-----------|--------------|
| Obstacle more than 20 m ahead | 30 km/h (cruise) |
| Obstacle 0–20 m ahead | 50 km/h (speed up to overtake) |
| Obstacle already behind | 30 km/h (resume cruise) |

### Pipeline overview

```mermaid
flowchart LR
    A[Waypoints] --> B[Cubic spline road]
    B --> C[Frenet candidates<br/>quintic d(t) x quartic s(t)]
    C --> D[Convert to x,y<br/>yaw, curvature]
    D --> E{Constraints and<br/>collision check}
    E -- rejected --> X[Discard]
    E -- feasible --> F[Cost function]
    F --> G[Lowest-cost path]
    G --> H[Execute first step,<br/>update state, replan]
    H --> C
    O[Obstacle planner] -.predicted path.-> E
```

---

## Repository structure

```
Trajectory-planning/
├── CubicSplinePlanner/
│   └── cubic_spline_planner.py          # 1D / 2D cubic splines (position, yaw, curvature)
├── QuinticPolynomialsPlanner/
│   └── quintic_polynomials_planner.py   # Quintic polynomial start→goal planner (standalone demo)
├── FrenetOptimalTrajectory/
│   └── frenet_optimal_trajectory.py     # Baseline Frenet planner with static obstacles
├── Overtake/
│   ├── main.py                          # Simulation loop and animation
│   ├── ego_vehicle.py                   # Ego planner: polynomials, Frenet paths, checks, cost, speed logic
│   ├── obs_vehicle.py                   # Obstacle vehicle planner
│   └── utilis.py                        # Shared constants (limits, sampling, cost weights, dimensions)
├── Project_Pedestrian_Overtake.pptx     # Project presentation
└── README.md
```

---

## Installation

**Requirements**

- Python 3.9 or newer (developed with Python 3.11)
- [NumPy](https://numpy.org/)
- [Matplotlib](https://matplotlib.org/)

```bash
git clone https://github.com/PremKishore27/Trajectory-planning.git
cd Trajectory-planning

# optional but recommended
python -m venv .venv
source .venv/bin/activate        # Windows: .venv\Scripts\activate

pip install numpy matplotlib
```

A graphical Matplotlib backend is needed for the live animations (the default on most desktop setups).

---

## Running the simulations

### Main project: overtaking scenario

```bash
python Overtake/main.py
```

An animated window shows:

- the road boundaries and dashed centerline,
- the **ego vehicle** (blue rectangle) and the **obstacle vehicle** (red rectangle), drawn with their real heading,
- the reference path, the obstacle's path (red) and the ego's chosen path (green),
- the 10 lowest-cost candidate trajectories (dashed),
- live ego and obstacle speeds in the plot title.

Press **Esc** to quit. Set `show_animation = False` in `Overtake/main.py` to run headless.

### Individual building blocks

```bash
python CubicSplinePlanner/cubic_spline_planner.py                         # spline demo: path, yaw, curvature
python QuinticPolynomialsPlanner/quintic_polynomials_planner.py           # start→goal quintic trajectory
python FrenetOptimalTrajectory/frenet_optimal_trajectory.py               # Frenet planner with static obstacles
```

---

## Module reference

### `CubicSplinePlanner/cubic_spline_planner.py`
- **`CubicSpline1D(x, y)`**: natural cubic spline with `calc_position`, `calc_first_derivative`, `calc_second_derivative`. Returns `None` outside its domain.
- **`CubicSpline2D(x, y)`**: spline over arc length `s`, giving `calc_position(s)`, `calc_yaw(s)` and `calc_curvature(s)`.
- **`calc_spline_course(x, y, ds)`**: samples a dense course from waypoints.

### `QuinticPolynomialsPlanner/quintic_polynomials_planner.py`
- **`QuinticPolynomial`**: fixes position, velocity and acceleration at both ends of a time interval; exposes the position and its first three derivatives (velocity, acceleration, jerk).
- **`quintic_polynomials_planner(...)`**: plans a trajectory between two full states (position, yaw, speed, acceleration), searching over durations until acceleration and jerk limits are met.

### `FrenetOptimalTrajectory/frenet_optimal_trajectory.py`
Reference implementation of the Frenet method against a list of **static point obstacles** on a spline road. Contains `QuarticPolynomial`, `FrenetPath`, candidate generation, `check_paths`, and `frenet_optimal_planning`.

### `Overtake/ego_vehicle.py`
The ego planner used by the main simulation, extended from the baseline:
- adds a configurable target speed `V` per planning call,
- samples a **wider lateral range** (`-MAX_ROAD_WIDTH` to `3 × MAX_ROAD_WIDTH`) so the car can move into the overtaking lane,
- performs a **time-indexed collision check** against the obstacle's predicted path (see below),
- applies the distance-based speed logic in `parameter(...)`.

### `Overtake/obs_vehicle.py`
`obstacle_planning(...)` plans the obstacle vehicle with the same Frenet machinery at a lower target speed (15 km/h) and its own speed/acceleration limits, so it moves along the road as a predictable, slow-moving lead vehicle.

### `Overtake/utilis.py`
Single place for all constants: vehicle dimensions, speed/acceleration/curvature limits, sampling resolution, cost weights.

### `Overtake/main.py`
Defines the road, initialises both vehicles, runs the receding-horizon loop (plan obstacle → plan ego → advance both one step → draw), and handles visualisation.

---

## Configuration and tuning

All main parameters live in [`Overtake/utilis.py`](Overtake/utilis.py).

| Parameter | Default | Meaning |
|-----------|---------|---------|
| `MAX_EGO_SPEED` | 60 km/h | Speed ceiling for the ego vehicle |
| `MAX_EGO_ACCEL` | 2.0 m/s² | Acceleration limit for ego candidates |
| `MAX_OBS_SPEED` | 15 km/h | Speed ceiling for the obstacle vehicle |
| `MAX_CURVATURE` | 10.0 1/m | Curvature limit |
| `MAX_ROAD_WIDTH` | 3.0 m | Lateral sampling extent |
| `D_ROAD_W` | 1.0 m | Lateral sampling step |
| `DT` | 0.2 s | Time step |
| `MIN_T` / `MAX_T` | 4.0 / 4.4 s | Planning horizon range |
| `D_T_S` / `N_S_SAMPLE` | 5 km/h / 4 | Target-speed sampling step and count |
| `ego_length` × `ego_width` | 5 m × 2 m | Ego dimensions |
| `obs_length` × `obs_width` | 5 m × 2 m | Obstacle dimensions |
| `K_J`, `K_T`, `K_D` | 0.1, 0.1, 1.5 | Jerk, time and deviation weights |
| `K_LAT`, `K_LON` | 1.0, 0.5 | Lateral vs. longitudinal cost weights |
| `SIM_LOOP` | 1000 | Maximum number of simulation steps |

Scenario setup is in `Overtake/main.py`:

- **Road shape**: edit the `wx`, `wy` waypoint lists.
- **Obstacle**: start position `obs_s0` and speed `obs_speed` (default 40 m ahead, 15 km/h).
- **Ego**: initial speed `c_speed` (default 30 km/h).
- **Speed-adaptation rule**: edit `parameter(...)` in `ego_vehicle.py`.

**Tuning hints**

- Increase `K_D` to keep the ego closer to the centerline and make it less eager to overtake.
- Increase `K_J` for smoother, more comfortable manoeuvres at the cost of agility.
- Increase `MAX_EGO_ACCEL` or the speed-sampling range if the ego cannot complete the pass.
- A finer `D_ROAD_W` produces smoother lateral choices but multiplies the number of candidates (and compute time).

---

## The overtaking scenario in detail

- **Road**: a 250 m spline road with a gentle S-bend over its first ~100 m followed by a straight section. Visual boundaries are drawn 7.5 m to the left and 2.5 m to the right of the centerline, giving a 10 m wide road with room to pass.
- **Obstacle**: starts 40 m ahead at 15 km/h on the centerline.
- **Ego**: starts at 30 km/h at the beginning of the road.
- **Behaviour**: the ego approaches, accelerates to 50 km/h once within 20 m, sweeps out into the free lateral space, passes, and settles back to cruising speed once ahead, always selecting the cheapest feasible candidate.
- **Collision check**: for each candidate, positions are compared **time step by time step** with the obstacle's predicted path. A candidate is rejected if the squared distance falls below `(0.7·L)² + (0.7·W)²`, an inflated safety envelope around the vehicle dimensions.

---

## Known limitations

These are inherent to the approach and the current implementation:

- **Perfect information**: positions of other vehicles are known exactly; there is no sensor simulation (LiDAR, radar, camera) or noise.
- **Predictable obstacles**: the obstacle follows its own planned path and does not react to the ego or change lanes unexpectedly.
- **Pedestrian handling**: the pedestrian and zebra-crossing logic described in the slide deck (`add_zebra_crossing_and_pedestrian`) is not part of the Python sources in this repository, which implement the overtaking behaviour.
- **Simplified speed rule**: the distance trigger compares the obstacle's x-coordinate to the ego's arc-length `s`, which is only an approximation on curved roads.
- **Point-distance collision model**: a single inflated-distance test is used instead of exact oriented-rectangle overlap.
- **Weights are hand-tuned**: cost weights are set manually and are scenario-specific.
- **Structured roads only**: Frenet planning is best suited to well-marked roads like highways, less to open or unstructured spaces.
- **Termination**: the run ends when the course ends or no valid path remains. The "Goal" distance test is exact (`<= 0.0`), so it is effectively never triggered.

---

## Roadmap

- **Reinforcement learning**: replace manually tuned cost weights and the fixed speed rule with a learned policy, for adaptivity in dynamic and uncertain environments.
- **Learned collision detection and avoidance** inside the planning loop.
- **Richer pedestrian model**: variable speed, sudden stops, multiple pedestrians.
- **Reactive obstacles**: vehicles that change lanes or respond to the ego.
- **Sensor simulation and fusion**: noisy perception feeding the planner.
- **Adaptive cost functions** that change with context (speed, traffic density, road type).
- **Tooling**: add a `requirements.txt`, unit tests for the polynomial and spline modules, and a `.gitignore` for `__pycache__/`.

---

## Credits and references

The spline, polynomial and baseline Frenet modules are based on the open-source [PythonRobotics](https://github.com/AtsushiSakai/PythonRobotics) project by **Atsushi Sakai**, and keep their original attribution headers. The overtaking scenario (ego and obstacle planners, dynamic speed logic, collision handling, parameters and visualisation) and the project presentation are the work of **Prem Kishore**.

- M. Werling, J. Ziegler, S. Kammel, S. Thrun, *Optimal Trajectory Generation for Dynamic Street Scenarios in a Frenet Frame*, IEEE ICRA, 2010.
- Local path planning and motion control for AGV in positioning, IEEE/RSJ IROS (the quintic polynomial reference).
- A. Sakai et al., *PythonRobotics: a Python code collection of robotics algorithms*, 2018.

---

## License

No license file is currently included. Consider adding one (e.g. MIT) to state how others may use the code. Note that PythonRobotics is MIT-licensed, which any derived code should respect.
