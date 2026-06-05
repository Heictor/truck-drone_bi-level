# Bi-Level Truck–Drone Last-Mile Delivery Benchmark (MATLAB, CPU/GPU Hybrid)

A MATLAB application for simulating and benchmarking **bi-level optimization** strategies for the **Truck–Drone Routing Problem (TDRP)**. The framework models truck parking/dispatch decisions (leader problem) coupled with drone mission optimization (follower problem), and benchmarks hybrid metaheuristic pairings across multiple scenarios.

The implementation supports **CPU parallelism** (`parfor`) and a **GPU/CPU hybrid** execution mode for the upper-level continuous metaheuristics.

---

## Contents

- [Problem Overview](#problem-overview)
- [Bi-Level Mathematical Model](#bi-level-mathematical-model)
	- [Lower-Level: Truck Routing (Leader)](#lower-level-truck-routing-leader)
	- [Upper-Level: Drone Control & Path Planning (Follower)](#upper-level-drone-control--path-planning-follower)
	- [Coupling Mechanism](#coupling-mechanism)
- [Algorithms](#algorithms)
	- [Lower-Level Algorithms](#lower-level-algorithms)
	- [Upper-Level Algorithms](#upper-level-algorithms)
	- [Hybrid Pairings Benchmarked](#hybrid-pairings-benchmarked)
- [Software Architecture](#software-architecture)
- [Repository Layout](#repository-layout)
- [Requirements](#requirements)
- [Quick Start](#quick-start)
- [Running the Benchmark](#running-the-benchmark)
	- [CPU Baseline](#cpu-baseline)
	- [GPU Benchmark](#gpu-benchmark)
- [Benchmark Scenarios & Metrics](#benchmark-scenarios--metrics)
- [Parallel Execution Notes](#parallel-execution-notes)
- [Reproducibility](#reproducibility)
- [Known Issues & Implementation Notes](#known-issues--implementation-notes)
- [License / Citation](#license--citation)

---

## Problem Overview

The system models a last-mile “tactical wave” where:

- A fleet of trucks selects parking/launch points around a constrained urban region (e.g., truck no-go zone).
- Each truck launches a drone that must complete a delivery mission to a target location.
- Truck cost depends on **driving cost** and **waiting cost**, where waiting cost depends on the **optimized drone mission time**.
- Drone mission cost monetizes not only flight time, but also safety/operational risk and battery degradation.

The core challenge is that evaluating a candidate truck solution requires solving an embedded drone optimization problem for each truck, resulting in a computationally expensive nested structure.

---

## Bi-Level Mathematical Model

The model is formulated as a Stackelberg leader–follower problem:

- **Leader (lower level):** truck parking selection to minimize operational cost.
- **Follower (upper level):** drone mission optimization (including PID gain tuning) given the selected parking point.

### Lower-Level: Truck Routing (Leader)

Let:

- $N$ be the set of candidate parking nodes.
- $N_T$ be the number of trucks (and drone missions).
- $P = \{p_1, p_2, \dots, p_{N_T}\} \subset N$ be the selected truck parking spots.
- $d(\text{depot}, p_t)$ be the shortest feasible truck travel distance (computed by A* on a grid map).
- $V_{truck}$ be truck speed.
- $C_{km}$ be the cost per km and $C_{hr}$ the cost per hour for waiting.
- $T_{drone,t}^*$ be the optimized drone mission time (upper-level output).

**Objective**

$$
\min_{P \subset N} J_{Truck} = \sum_{t=1}^{N_T} \left( C_{Travel,t} + C_{Wait,t} \right)
$$

with:

- Travel cost:

$$
C_{Travel,t} = C_{km} \cdot d(\text{depot}, p_t)
$$

- Waiting cost:

$$
C_{Wait,t} = C_{hr} \cdot \max\left(0,\, T_{drone,t}^* - \frac{d(\text{depot}, p_t)}{V_{truck}}\right)
$$

**Constraints**

$$
p_t \in N,\quad \forall t \in \{1,\dots,N_T\}
$$

$$
p_i \neq p_j,\quad \forall i \neq j
$$

$$
V_{truck,min} \le V_{truck} \le V_{truck,max}
$$

$$
T_{truck,t} + T_{wait,t} \le T_{max,window}
$$

---

### Upper-Level: Drone Control & Path Planning (Follower)

Given $p_t$, the drone must navigate to its assigned target while minimizing a monetized mission cost. The decision variables are the PID gains:

$$
PID = \{K_p, K_i, K_d\}
$$

A continuous-time integral form for the drone objective is:

$$
\min_{PID} J_{Drone} = \int_{0}^{T_{flight}} \left(
w_1 \cdot \|\vec{v}(t)\| + w_2 \cdot t + w_3 \cdot P(p(t)) + \frac{w_4}{B(t)} + w_5 \cdot S(p(t))
\right) dt
$$

In the implementation, this is discretized and expressed as a sum of four monetized components:

$$
J_{Drone} = C_{Time} + C_{Battery} + C_{Obstacle} + C_{Signal}
$$

#### Cost components

**1) Time cost**

$$
C_{Time} = T_{flight} \cdot C_{drone,hr}
$$

**2) Battery degradation cost**

$$
C_{Battery} = \left( \sum_{k=1}^{K} \frac{1}{B_{rem}(k)} \right) \cdot C_{batt,pen}
$$

where $B_{rem}(k)$ is remaining battery fraction at step $k$.

**3) Obstacle risk cost**

$$
C_{Obstacle} = P_{obs} \cdot C_{obs,pen}
$$

where $P_{obs}$ is a collision/obstacle intersection indicator or count (implementation-dependent).

**4) Signal risk cost**

Signal penalty is distance-based with thresholds $R_1$ and $R_2$:

$$
S(p(t))=
\begin{cases}
0 & \text{if } d_{T}(p(t))\le R_{1} \\
k_{1}\cdot(d_{T}(p(t))-R_{1})^{2} & \text{if } R_{1}<d_{T}(p(t))\le R_{2} \\
k_{2} & \text{if } d_{T}(p(t))>R_{2}
\end{cases}
$$

#### Drone constraints

Obstacle clearance:

$$
\text{dist}(\text{Path}(k), O_j) \ge R_{safety},\quad \forall k,\forall j
$$

Battery reserve:

$$
B(t) \ge B_{min}
$$

Discharge rate limit:

$$
\frac{dB}{dt} \le \dot{B}_{max}
$$

Payload-adjusted speed constraint:

$$
V_{drone}(w_{pkg}) \le V_{base}\cdot\left(1 - f_{payload}\frac{w_{pkg}}{w_{max}}\right)
$$

---

### Coupling Mechanism

The coupling variable is the **optimized drone mission time** $T_{drone,t}^*$, which directly influences the truck waiting cost:

- Lower level proposes $p_t$.
- Upper level returns $T_{drone,t}^*(p_t)$ and the optimized cost components.
- Lower level uses $T_{drone,t}^*$ to compute waiting penalties and evaluate $J_{Truck}$.

---

## Algorithms

### Lower-Level Algorithms

- **GA (Genetic Algorithm):** permutation-based encoding of candidate parking nodes, selection/crossover/mutation.
- **ACO (Ant Colony Optimization):** pheromone-guided constructive search over node permutations.
- **ALNS (Adaptive Large Neighborhood Search):** destroy/repair operators with adaptive operator weighting + acceptance criterion.

### Upper-Level Algorithms

- **PSO (Particle Swarm Optimization)**
- **BOA (Bat Optimization Algorithm)**
- **BSO (Brain Storm Optimization)**
- **CMA-ES (Covariance Matrix Adaptation Evolution Strategy)**

### Hybrid Pairings Benchmarked

- GA–BOA
- GA–PSO
- ACO–BSO
- ALNS–PSO
- ACO–CMA-ES

---

## Software Architecture

1. **Benchmark Orchestrator** (`run_benchmark*.m`)
	- Builds a job list: scenarios × algorithms × runs.
	- Executes jobs in parallel with `parfor`.
	- Aggregates results into a summary CSV.

2. **Single Simulation Runner** (`runSingleSimulation*.m`)
	- Generates scenario instance (targets, obstacles, maps).
	- Runs lower-level algorithm to select parking nodes.
	- Evaluates lower-level candidates using a fast or full cost pipeline.
	- For the final solution, runs full upper-level optimization and produces plots + MAT outputs.

3. **Core Geometry / Path / Simulation**
	- A* pathfinding for truck and drone 2D trajectories.
	- Drone flight simulation (time stepping) + cost accumulation.

4. **Parallel & GPU Execution**
	- CPU parallelism via `parfor`.
	- GPU acceleration used selectively for upper-level population update steps via `gpuArray`.

---

## Repository Layout
├── run_benchmark.m

├── run_benchmark_GPU.m

├── runSingleSimulation.m

├── runSingleSimulation_GPU.m

├── helpers/

│   ├── AStarPath.m

│   ├── getNeighbors.m

│   ├── priority_queue.m

│   └── ...

├── algorithms/

│   ├── lower/

│   └── upper/

├── outputs/

│   ├── Benchmark_Results_Final/

│   └── Benchmark_Results_GPU/

└── README.md


---

## Requirements

- MATLAB (recent versions; must support `parfor`)
- Parallel Computing Toolbox (for `parfor`)
- For GPU mode:
	- CUDA-capable NVIDIA GPU
	- MATLAB GPU support (Parallel Computing Toolbox GPU features)

---

## Quick Start

Open MATLAB in the repository root.

PROJECT_ROOT = fileparts(mfilename('fullpath'));

addpath(genpath(PROJECT_ROOT));


(Optional) Start the parallel pool:

if isempty(gcp('nocreate'))

parpool('local');

end

pctRunOnAll addpath(genpath(PROJECT_ROOT));


---

## Running the Benchmark

### CPU Baseline
run_benchmark

### GPU Benchmark
run_benchmark_GPU


---

## Reproducibility

Each run should set a deterministic seed, for example:

$$
\text{seed} = 1000 \cdot s_{idx} + run_{idx}
$$

---

## Known Issues & Implementation Notes

### 1) Complex-number artifacts in costs

Some runs may produce values like `X + 0i` due to numerical behavior. Enforce real outputs:

final_total_cost = real(sum(final_JTruck_costs) + sum(final_JDrone_costs));


### 2) `parfor` worker visibility of helper functions

To avoid `Undefined function ...` errors inside `parfor`, keep helper utilities as standalone `.m` files on the MATLAB path (e.g., `AStarPath.m`, `getNeighbors.m`), and propagate the path to workers via `pctRunOnAll`.

### 3) GPU acceleration boundaries

GPU acceleration targets upper-level population update steps (vector math). If fitness evaluation calls CPU-only routines (e.g., A* pathfinding, sequential flight simulation), CPU↔GPU transfers (`gather`) are expected and GPU utilization may not reach 100%.

---

## License / Citation
