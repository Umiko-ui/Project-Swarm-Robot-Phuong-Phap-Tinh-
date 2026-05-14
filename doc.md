# Swarm Robotics System Documentation

## 1. Overview
This system implements a hierarchical swarm control architecture including:
- SwarmController (main loop)
- FormationController (local interactions)
- VirtualRegionController (shape generation)
- Robots (agents)

---

## 2. SwarmControllerStable
### Description
Main controller that coordinates observation, planning, and control.
---
### Types
- `Vector2`: np.ndarray shape (2,) → 2D vector [x, y]
- `Positions`: np.ndarray shape (N, 2) → positions of N robots
---
### Variables
- `C_meas`: Vector2 — measured center of swarm  
- `C_obs`: Vector2 — filtered (observed) center  
- `C_plan`: Vector2 — planned center  
- `V_plan`: Vector2 — velocity of planned center  
---
### Functions
#### compute_measurement()
**Description:**
Compute the geometric center of the swarm from follower robot positions.
**Inputs:**
- None (uses `self.followers`)
**Outputs:**
- Updates `self.C_meas: Vector2`
**Details:**
- Computes mean of all follower positions
- Stores result in delay buffer (`self.buffer`)
**Notes:**
- First step in control loop
---
#### update_observer()
**Description:**
Apply a first-order low-pass filter to reduce noise and measurement delay.
**Inputs:**
- Internal buffer (`self.buffer`)
**Outputs:**
- Updates `self.C_obs: Vector2`
**Details:**
- Uses exponential smoothing:
  C_obs = (1 - alpha) * C_obs + alpha * delayed_measurement
**Notes:**
- Stabilizes input to planner
---

#### update_planner(target)
**Description:**
Update the planned center using second-order damped dynamics.
**Inputs:**
- `target: Vector2` — desired center
**Outputs:**
- Updates:
  - `self.C_plan: Vector2`
  - `self.V_plan: Vector2`
**Details:**
- Computes:
  - error = target - C_plan  
  - acceleration = 2 * error - 2 * velocity  
- Applies velocity saturation (`v_max`)
- Integrates using timestep `dt`
**Notes:**
- Produces smooth motion toward target
---

#### update_followers()
**Description:**
Update positions of follower robots based on formation targets.
**Inputs:**
- None (uses internal state)
**Outputs:**
- Updates robot positions (`self.followers`)
**Details:**
1. Generate target positions using `VirtualRegionController`
2. Get current positions of all robots
3. Call `FormationController.update_positions(...)`
4. Apply new positions to each robot
**Notes:**
- This step implements local interaction + formation tracking
- Uses C++ backend for performance


# 3. FormationController

## Description

Controls follower robot positions using local interactions and target tracking.

This module enforces:

- Cohesion toward formation targets
- Local spacing regulation
- Collision avoidance through interaction forces
- Scalable neighbor-based swarm dynamics

The core dynamics are implemented in a C++ backend (`formation_cpp`) for performance.

---

## Types

- `Vector2`: np.ndarray shape `(2,)` → 2D vector
- `Positions`: np.ndarray shape `(N, 2)` → positions of N robots
- `AdjacencyMatrix`: np.ndarray shape `(N, N)`
- `Neighbors`: `list[list[int]]`

---

## Parameters

- `kc: float` — target tracking gain
- `kf: float` — interaction force gain
- `Rs: float` — base desired spacing
- `dt: float` — simulation timestep

---

## Functions

### build_neighbors(p_current)

#### Signature

```python
build_neighbors(
    p_current: Positions
) -> Neighbors
```

#### Description

Construct local neighbor sets using spatial hashing and a uniform spatial grid.

#### Inputs

- `p_current: Positions` — current robot positions

#### Outputs

- `neighbors: list[list[int]]` — nearby robot indices for each robot

#### Details

- Uses a uniform spatial grid (`SpatialGrid`)
- Grid cell size equals interaction radius:

$$r_c = 3R_s$$

- Neighbor search is restricted to:
  - current cell
  - adjacent surrounding cells

- Computational complexity is approximately:

$$O(N)$$

instead of:

$$O(N^2)$$

#### Notes

- Enables scalable swarm simulation
- Designed for large robot populations
- Neighbor interactions are local only

---

### update_positions(`p_current`, `p_target`, `Delta`, `kc`, `kf`, `Rs`, `dt`)

#### Signature

```python
update_positions(
    p_current: Positions,
    p_target: Positions,
    Delta: AdjacencyMatrix,
    kc: float,
    kf: float,
    Rs: float,
    dt: float
) -> Positions
```

#### Description

Compute updated robot positions using:

- target attraction
- local interaction forces
- second-order damped dynamics

The implementation uses a C++ backend accelerated with `pybind11`.

#### Inputs

- `p_current: Positions` — current robot positions
- `p_target: Positions` — desired target positions
- `Delta: AdjacencyMatrix` — formation scaling matrix
- `kc: float` — target attraction gain
- `kf: float` — interaction gain
- `Rs: float` — base desired spacing
- `dt: float` — simulation timestep

#### Outputs

- `p_new: Positions` — updated robot positions

---

## Internal Dynamics

### 1. Target Tracking Force

Each robot is attracted toward its target:

$$F_i^{\mathrm{target}} = k_c \left( p_i^{\mathrm{target}} - p_i \right)$$

where:

- `p_i` is the current robot position
- `p_i^{target}` is the desired target position

---

### 2. Neighbor Interaction Force

For nearby robots, the controller computes a spacing regulation force.

Desired pairwise distance:

$$d_{ij}^{\mathrm{desired}} = R_s \Delta_{ij}$$

Distance error:

$$e_{ij} = \|p_j - p_i\| - d_{ij}^{\mathrm{desired}}$$

Exponential weighting:

$$w_{ij} = \exp\left( -\frac{\|p_j-p_i\|}{d_{ij}^{\mathrm{desired}}} \right)$$

Combined interaction model:

$$f_{ij} = 0.7k_f e_{ij} + 0.3k_f e_{ij} w_{ij}$$

Force saturation:

$$F_{ij} = 20 \tanh\left( \frac{f_{ij}}{20} \right)$$

Applied directional force:

$$\mathbf{F}_{ij} = F_{ij} \frac{p_j-p_i}{\|p_j-p_i\|}$$

---

### 3. Force Scaling

To prevent numerical instability, forces are globally scaled when the maximum force magnitude becomes too large.

If:

$$\max_i \|F_i\| > 10$$

then:

$$\mathrm{scale} = \frac{10}{\max_i \|F_i\|}$$

---

### 4. Second-Order Dynamics

Velocity memory is preserved between iterations.

Velocity update:

$$v_i^{t+1} = 0.92v_i^t + \mathrm{scale} \cdot F_i \cdot dt$$

Position integration:

$$p_i^{t+1} = p_i^t + v_i^{t+1} dt$$

---

## Numerical Stability Features

The implementation includes several safeguards:

- Non-finite value detection (`NaN`, `Inf`)
- Distance lower bounds
- Desired distance clamping
- Force saturation using `tanh`
- Velocity sanitization
- Output sanitization
- Global force normalization

---

## Notes

- Uses spatial hashing for scalable local interactions
- Complexity is approximately linear in swarm size
- Supports arbitrary formation topologies through `Delta`
- Maintains smooth motion using damped second-order dynamics
- Designed for large-scale swarm simulations and real-time execution



## 4. VirtualRegionController
### Description
Generates the geometric structure (circle or ellipse) that defines the swarm formation.
This module adapts the formation shape based on:
- Available free space
- Obstacle configuration
- Direction toward goal
---
### Types
- `Vector2`: np.ndarray shape (2,)
- `OpenSights`: list of obstacle boundary segments
---
### Parameters
- `base_radius`: float — nominal swarm radius  
- `safety_margin`: float — clearance from obstacles  
---
### State Variables
- `current_shape`: str — "CIRCLE" or "ELLIPSE"  
- `params`: dict containing:
  - `R`: radius (circle)
  - `a`: major axis (ellipse)
  - `b`: minor axis (ellipse)
  - `angle`: orientation (radians)
---
### Functions
#### _calculate_free_space(current_center, goal, open_sights)
**Signature:**
`_calculate_free_space(Vector2, Vector2, OpenSights) -> (float, float)`
**Description:**
Estimate available free space and optimal passage direction.
**Inputs:**
- `current_center: Vector2`  
- `goal: Vector2`  
- `open_sights`: obstacle boundary points  
**Outputs:**
- `free_width: float` — width of navigable gap  
- `passage_angle: float` — direction to pass through gap  
**Details:**
- Default direction: toward goal
- If obstacles detected:
  1. Extract obstacle boundary points
  2. Cluster into two groups (KMeans)
  3. Compute gap width between clusters
  4. Choose direction perpendicular to obstacle line
**Notes:**
- Key step for obstacle-aware formation deformation
---

#### update_structural_parameters(current_center, goal, open_sights)
**Signature:**
`update_structural_parameters(Vector2, Vector2, OpenSights) -> (shape, params)`
**Description:**
Update formation shape (circle or ellipse) based on environment.
**Inputs:**
- `current_center: Vector2`  
- `goal: Vector2`  
- `open_sights`: sensed obstacle data  
**Outputs:**
- `current_shape`: str  
- `params`: dict with shape parameters  
**Details:**
1. Compute free space width
2. Compare with swarm diameter
3. If wide:
   → use circle  
4. If narrow:
   → use ellipse:
   - minor axis: `b = free_width / 2`
   - major axis: `a = R² / b` (area preservation)
   - apply upper bound on `a`
   - align with passage angle
**Notes:**
- Prevents collision in narrow passages
- Maintains formation coherence
---

#### get_robot_target_positions(current_center, num_robots)
**Signature:**
`get_robot_target_positions(Vector2, int) -> Positions`
**Description:**
Generate target positions for all robots based on current formation shape.
**Inputs:**
- `current_center: Vector2`  
- `num_robots: int`  
**Outputs:**
- `targets: Positions`  
**Details:**
- Calls C++ backend (`formation_cpp`)
- Distributes robots evenly along:
  - circle (if wide)
  - ellipse (if narrow)
**Notes:**
- Ensures consistent formation geometry
- Abstracts shape generation from control logic
---

## 5. Data Flow
1. Get robot positions
2. Compute `C_meas`
3. Filter → `C_obs`
4. Plan → `C_target`
5. Smooth → `C_plan`
6. Generate shape
7. Generate targets
8. Update robots
---

## 6. Notes
- Neighbor interactions are local (radius-based)
- Core computation is implemented in C++