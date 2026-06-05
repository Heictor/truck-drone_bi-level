\# Bi-Level Truck–Drone Last-Mile Delivery Benchmark (MATLAB, CPU/GPU Hybrid)



A MATLAB application for simulating and benchmarking \*\*bi-level optimization\*\* strategies for a \*\*Truck–Drone Routing Problem (TDRP)\*\*. The framework models truck parking/dispatch decisions (leader problem) coupled with drone mission optimization (follower problem), and benchmarks hybrid metaheuristic pairings across multiple scenarios.



The implementation supports \*\*CPU parallelism\*\* (`parfor`) and a \*\*GPU/CPU hybrid\*\* execution mode for the upper-level continuous metaheuristics.



---



\## Contents



\- \[Problem Overview](#problem-overview)

\- \[Bi-Level Mathematical Model](#bi-level-mathematical-model)

&nbsp; - \[Lower-Level: Truck Routing (Leader)](#lower-level-truck-routing-leader)

&nbsp; - \[Upper-Level: Drone Control \& Path Planning (Follower)](#upper-level-drone-control--path-planning-follower)

&nbsp; - \[Coupling Mechanism](#coupling-mechanism)

\- \[Algorithms](#algorithms)

&nbsp; - \[Lower-Level Algorithms](#lower-level-algorithms)

&nbsp; - \[Upper-Level Algorithms](#upper-level-algorithms)

&nbsp; - \[Hybrid Pairings Benchmarked](#hybrid-pairings-benchmarked)

\- \[Software Architecture](#software-architecture)

\- \[Repository Layout (Suggested)](#repository-layout-suggested)

\- \[Requirements](#requirements)

\- \[Quick Start](#quick-start)

\- \[Running the Benchmark](#running-the-benchmark)

&nbsp; - \[CPU Baseline](#cpu-baseline)

&nbsp; - \[GPU Benchmark](#gpu-benchmark)

\- \[Benchmark Scenarios \& Metrics](#benchmark-scenarios--metrics)

\- \[Parallel Execution Notes](#parallel-execution-notes)

\- \[Reproducibility](#reproducibility)

\- \[Known Issues \& Implementation Notes](#known-issues--implementation-notes)

\- \[License / Citation](#license--citation)



---



\## Problem Overview



The system models a last-mile “tactical wave” where:



\- A fleet of trucks selects parking/launch points around a constrained urban region (e.g., truck no-go zone).

\- Each truck launches a drone that must complete a delivery mission to a target location.

\- Truck cost depends on \*\*driving cost\*\* and \*\*waiting cost\*\*, where waiting cost depends on the \*\*optimized drone mission time\*\*.

\- Drone mission cost monetizes not only flight time, but also safety/operational risk and battery degradation.



The core challenge is that evaluating a candidate truck solution requires solving an embedded drone optimization problem for each truck, resulting in a computationally expensive nested structure.



---



\## Bi-Level Mathematical Model



The model is formulated as a Stackelberg leader–follower problem:



\- \*\*Leader (lower level):\*\* truck parking selection to minimize operational cost.

\- \*\*Follower (upper level):\*\* drone mission optimization (including PID gain tuning) given the selected parking point.



\### Lower-Level: Truck Routing (Leader)



Let:

\- \\( N \\) be the set of candidate parking nodes.

\- \\( N\_T \\) be the number of trucks (and drone missions).

\- \\( P = \\{p\_1, p\_2, \\dots, p\_{N\_T}\\} \\subset N \\) be the selected truck parking spots.

\- \\( d(\\text{depot}, p\_t) \\) be the shortest feasible truck travel distance (computed by A\* on a grid map).

\- \\( V\_{truck} \\) be truck speed.

\- \\( C\_{km} \\) be the cost per km and \\( C\_{hr} \\) the cost per hour for waiting.

\- \\( T\_{drone,t}^\* \\) be the optimized drone mission time (upper level output).



\*\*Objective:\*\*

\\\[

\\min\_{P \\subset N} J\_{Truck} = \\sum\_{t=1}^{N\_T} \\left( C\_{Travel,t} + C\_{Wait,t} \\right)

\\]

with:

\- Travel cost:

\\\[

C\_{Travel,t} = C\_{km} \\cdot d(\\text{depot}, p\_t)

\\]

\- Waiting cost:

\\\[

C\_{Wait,t} = C\_{hr} \\cdot \\max\\left(0,\\, T\_{drone,t}^\* - \\frac{d(\\text{depot}, p\_t)}{V\_{truck}}\\right)

\\]



\*\*Constraints:\*\*

\\\[

p\_t \\in N,\\quad \\forall t \\in \\{1,\\dots,N\_T\\}

\\]

\\\[

p\_i \\neq p\_j,\\quad \\forall i \\neq j

\\]

\\\[

V\_{truck,min} \\le V\_{truck} \\le V\_{truck,max}

\\]

\\\[

T\_{truck,t} + T\_{wait,t} \\le T\_{max,window}

\\]



---



\### Upper-Level: Drone Control \& Path Planning (Follower)



Given \\( p\_t \\), the drone must navigate to its assigned target while minimizing a monetized mission cost. The decision variables are the PID gains:

\\\[

PID = \\{K\_p, K\_i, K\_d\\}

\\]



A continuous-time integral form for the drone objective is:

\\\[

\\min\_{PID} J\_{Drone} = \\int\_{0}^{T\_{flight}} \\left(

w\_1 \\cdot \\|\\vec{v}(t)\\| + w\_2 \\cdot t + w\_3 \\cdot P(p(t)) + \\frac{w\_4}{B(t)} + w\_5 \\cdot S(p(t))

\\right) dt

\\]



In the implementation, this is discretized and expressed as a sum of four monetized components:

\\\[

J\_{Drone} = C\_{Time} + C\_{Battery} + C\_{Obstacle} + C\_{Signal}

\\]



\#### Cost components



\*\*1) Time cost\*\*

\\\[

C\_{Time} = T\_{flight} \\cdot C\_{drone,hr}

\\]



\*\*2) Battery degradation cost\*\*

\\\[

C\_{Battery} = \\left( \\sum\_{k=1}^{K} \\frac{1}{B\_{rem}(k)} \\right) \\cdot C\_{batt,pen}

\\]

where \\( B\_{rem}(k) \\) is remaining battery fraction at step \\(k\\).



\*\*3) Obstacle risk cost\*\*

\\\[

C\_{Obstacle} = P\_{obs} \\cdot C\_{obs,pen}

\\]

where \\(P\_{obs}\\) is a collision/obstacle intersection indicator or count (implementation-dependent).



\*\*4) Signal risk cost\*\*

Signal penalty is distance-based with thresholds \\(R\_1\\) and \\(R\_2\\):

\\\[

S(p(t))=

\\begin{cases}

0 \& \\text{if } d\_{T}(p(t))\\le R\_{1} \\\\

k\_{1}\\cdot(d\_{T}(p(t))-R\_{1})^{2} \& \\text{if } R\_{1}<d\_{T}(p(t))\\le R\_{2} \\\\

k\_{2} \& \\text{if } d\_{T}(p(t))>R\_{2}

\\end{cases}

\\]



\#### Drone constraints



Obstacle clearance:

\\\[

\\text{dist}(\\text{Path}(k), O\_j) \\ge R\_{safety},\\quad \\forall k,\\forall j

\\]



Battery reserve:

\\\[

B(t) \\ge B\_{min}

\\]



Discharge rate limit:

\\\[

\\frac{dB}{dt} \\le \\dot{B}\_{max}

\\]



Payload-adjusted speed constraint:

\\\[

V\_{drone}(w\_{pkg}) \\le V\_{base}\\cdot\\left(1 - f\_{payload}\\frac{w\_{pkg}}{w\_{max}}\\right)

\\]



---



\### Coupling Mechanism



The coupling variable is the \*\*optimized drone mission time\*\* \\(T\_{drone,t}^\*\\), which directly influences the truck waiting cost:



\- Lower level proposes \\(p\_t\\).

\- Upper level returns \\(T\_{drone,t}^\*(p\_t)\\) and the optimized cost components.

\- Lower level uses \\(T\_{drone,t}^\*\\) to compute waiting penalties and evaluate \\(J\_{Truck}\\).



This creates a nested evaluation structure: each candidate truck solution requires solving \\(N\_T\\) embedded upper-level optimizations.



---



\## Algorithms



\### Lower-Level Algorithms



Lower-level solves a discrete combinatorial selection problem over candidate parking nodes:



\- \*\*GA (Genetic Algorithm):\*\* permutation-based encoding of candidate parking nodes, selection/crossover/mutation.

\- \*\*ACO (Ant Colony Optimization):\*\* pheromone-guided constructive search over node permutations.

\- \*\*ALNS (Adaptive Large Neighborhood Search):\*\* destroy/repair operators with adaptive operator weighting + acceptance criterion.



\### Upper-Level Algorithms



Upper-level solves a low-dimensional continuous optimization (PID tuning):



\- \*\*PSO (Particle Swarm Optimization)\*\*

\- \*\*BOA (Bat Optimization Algorithm)\*\*

\- \*\*BSO (Brain Storm Optimization)\*\*

\- \*\*CMA-ES (Covariance Matrix Adaptation Evolution Strategy)\*\*



Upper-level is called for each truck mission, typically after an initial A\* path generation step.



\### Hybrid Pairings Benchmarked



The benchmark compares hybrid pairs (lower-level + upper-level), e.g.:



\- GA–BOA

\- GA–PSO

\- ACO–BSO

\- ALNS–PSO

\- ACO–CMAES



---



\## Software Architecture



The benchmark is structured as:



1\. \*\*Benchmark Orchestrator\*\* (`run\_benchmark\*.m`)

&nbsp;  - Builds a job list: scenarios × algorithms × runs.

&nbsp;  - Executes jobs in parallel with `parfor`.

&nbsp;  - Aggregates results into summary CSV.

&nbsp;  - Runs statistical analysis (Friedman / Wilcoxon) when applicable.



2\. \*\*Single Simulation Runner\*\* (`runSingleSimulation\*.m`)

&nbsp;  - Generates scenario instance (targets, obstacles, maps).

&nbsp;  - Runs lower-level algorithm to select parking nodes.

&nbsp;  - Evaluates lower-level candidates using a fast or full cost pipeline.

&nbsp;  - For final solution, runs full upper-level optimization and produces plots + MAT outputs.



3\. \*\*Core Geometry / Path / Physics\*\*

&nbsp;  - A\* pathfinding for truck and drone 2D trajectories.

&nbsp;  - Drone flight simulation (time stepping) + cost accumulation.



4\. \*\*Parallel \& GPU Execution\*\*

&nbsp;  - CPU parallelism via `parfor`.

&nbsp;  - GPU acceleration used selectively for upper-level population updates via `gpuArray` (with CPU-side fitness evaluation where needed).



---



\## Repository Layout (Suggested)



> Adjust paths/names to match the repository organization.



