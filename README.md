# POCO: Pairwise Obstacle-free Conflict Oracle

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![C++ Standard](https://img.shields.io/badge/C%2B%2B-17-blue.svg)](https://en.cppreference.com/w/cpp/17)

> **Official C++ Implementation** for the paper:  
> **"Hostile, Compatible, or Free: A Constant Time Classification of Pairwise Shortest Path Conflicts in Obstacle-Free MAPF"**

## 📖 Overview

**POCO** is a high-performance algorithmic library that solves a fundamental challenge in Multi-Agent Pathfinding (MAPF): **how to quickly and systematically classify the intrinsic conflict relationship between any two agents.**

Based on rigorous geometric proofs, POCO provides an **$O(1)$ analytical methodology** to determine the conflict type between two agents over *all* their possible shortest paths, bypassing the combinatorial explosion of brute-force enumeration.

### The Ternary Classification

POCO classifies any agent pair into three exhaustive types:
* 🔴 **Hostile**: Conflict is **inevitable**. All pairs of shortest paths conflict.
* 🟢 **Free**: Conflict is **impossible**. No pairs of shortest paths conflict.
* 🟡 **Compatible**: Conflict is **avoidable**. Both conflicting and non-conflicting path pairs exist.

## 📋 Assumptions & Model

The conflict classification logic relies on the following theoretical assumptions. The library is valid **only** when these conditions are met:

* **Grid Environment**: Agents operate on a 4-neighbor grid graph where each vertex corresponds to integer coordinates $(x, y)$.
* **Obstacle-Free**: The environment is assumed to be free of static obstacles (the algorithm is analytical and does not perform collision checks against a map).
* **Unit-Sized Agents**: Each agent occupies a single vertex (a $1 \times 1$ grid cell) at any timestep.
* **Shortest Path Constraint**: Each agent is assigned a start and goal vertex and is strictly constrained to travel along a **Manhattan shortest path**.
* **Greedy Movement**: Agents move in discrete timesteps. For any timestep $t < T_i$ (before reaching the goal), an agent **must** move to an adjacent vertex that strictly decreases its Manhattan distance to the goal (i.e., waiting is not allowed mid-path).
* **Terminal Waiting**: Upon reaching its goal at timestep $T_i$, an agent remains stationary at $g_i$ for all subsequent timesteps.

## 🚀 Key Features

* **Constant-Time Classification**: Determines the conflict relationship (Hostile/Compatible/Free) in **$O(1)$** time using only start and goal coordinates.
* **MVC Heuristic**: Provides a built-in tight lower bound $LB$ for the **Minimum Vertex Cover (MVC)**. Also exposes a flexible interface to inject external exact solvers (e.g., NuMVC, Gurobi) for high-precision heuristics.
* **High Performance**: Optimized for large-scale instances. Capable of updating graphs and calculating heuristics for 1000+ agents in milliseconds.
* **Easy Integration**: Designed as a lightweight, header-only core that can be integrated into coupled solvers (e.g., LaCAM) or decoupled solvers (e.g., CBS).

## 🛠️ Build & Install

POCO is built using CMake.

```bash
git clone https://github.com/sonoffreewind/poco.git

cd poco

cmake -B build && make -C build

# Run performance benchmarks
build/benchmark 

# Run simple usage examples
build/simple_examples 

# Run basic functionality tests
build/test_basic 

# Run conflict oracle tests
build/test_oracle
```

## 💻 Quick Start

### 1. Pairwise Conflict Check ($O(1)$)

Use `getConflictRelation` to instantly classify the relationship between two agents.

```cpp
#include "conflict.hpp"

Point s1(0, 0), g1(5, 0); // Agent 1
Point s2(2, 0), g2(2, 5); // Agent 2

// O(1) Classification
ConflictRelation rel = getConflictRelation(s1, g1, s2, g2);

if (rel == ConflictRelation::Hostile) {
    // Conflict is inevitable (Cost will increase)
} else if (rel == ConflictRelation::Free) {
    // No conflict possible
} else {
    // Compatible: Coordination required
}
```

### 2. Dynamic Heuristic Calculation

Use `ConflictOracle` to maintain a dynamic conflict graph and compute the MVC heuristic.

```cpp
#include "conflictoracle.hpp"

// Initialize oracle with static goals (one-time setup)
ConflictOracle oracle(starts, goals);

// Get conflict realtion of agent i and agent j
ConflictRelation rel = oracle.get_agent_relation(i,j);

// Update agents to new_starts and get hostile graph's mvc size
int mvc_val, hostile_edges, compatible_edges;
// true = Calculate Fast Lower Bound (Built-in)
// false = Calculate Exact MVC (Requires external solver injection)
oracle.update_calmvc(new_starts, true, mvc_val, hostile_edges, compatible_edges);

// Use mvc_val as a heuristic metric
// such as an additional part of the evaluation function of the A* shortest path algorithm
f = g + h + mvc_val;
```
### 3. Advanced: Injecting an Exact MVC Solver

By default, POCO calculates a fast lower bound ($LB$) for the Minimum Vertex Cover. To obtain the exact MVC size (which provides a more accurate heuristic but takes longer), you need to inject an external solver (e.g., NuMVC, Gurobi).

**1. Define the Solver Callback**
The solver function must match the following signature:
```cpp
/**
 * @param edges A list of edges (pairs of vertex indices) in the conflict graph.
 * @param num_vertices The total number of vertices (agents).
 * @return The size of the minimum vertex cover.
 */
int my_mvc_solver(const std::vector<std::pair<int, int>>& edges, int num_vertices);
```

**2. Register the Solver**
```cpp
// ... inside your main code ...
ConflictOracle oracle(starts, goals);

// Register your solver wrapper (e.g., calling NuMVC)
oracle.set_mvc_solver(my_mvc_solver);

// Now, setting use_lb=false will trigger your injected solver
int mvc;
// ...
oracle.update_calmvc(new_starts, false, mvc, hostile_edges, compatible_edges);
```
## 🔗 Related Projects

This library serves as the core engine for our experiments. For the complete solver integration and reproduction of the paper's results, please visit:

* **[LaCAM3-POCO](https://github.com/sonoffreewind/lacam3-poco)**: The LaCAM3 solver integrated with POCO, demonstrating significant improvements in solution quality and search efficiency.

## 📚 Citation

If you use POCO in your research, please cite our paper:

```bibtex
@article{Guo2025POCO,
  title={Hostile, Compatible, or Free: A Constant Time Classification of Pairwise Shortest Path Conflicts in Obstacle-Free MAPF},
  author={Lifeng Guo and Changhong Lu},
  journal={Under Review},
  year={2025}
}
```

## 📄 License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.