# MATLAB Genetic Algorithm Path Planner

**GA-PathPlanner** is a self-contained MATLAB project that finds collision-free, near-shortest paths for a point robot moving on a 500 × 500 binary map.
It is written from scratch – no call to MATLAB’s `ga` function – and demonstrates how core genetic-algorithm building blocks can be combined into a robust motion-planning pipeline.&#x20;

---

## Problem model

* **Map** Binary image (`true` = free, `false` = obstacle).

* **Start / goal** Fixed at `[1 1]` and `[500 500]` (easily changed).

* **Chromosome** A path is a fixed-length vector of 2 × *n* floating-point genes

  ```
  [x1 y1  x2 y2  …  xn yn]    1 ≤ xi, yi ≤ 500
  ```

  joined with straight segments start → P1 → … → Pn → goal.

* **Fitness** Euclidean length + obstacle penalty
  `f = Lpath + α · Lobstacle`, where `α` is auto-scaled to dominate length whenever collisions occur.

---

## Algorithmic core

| Stage           | Implementation notes                                                                                                                                       |
| --------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **Selection**   | Three interchangeable schemes: roulette wheel, tournament (configurable size / replacement), and rank based.                                               |
| **Crossover**   | Two operators: <br>• *k*-point (adaptive choice of *k* ∈ {1, 2, 3}) <br>• Arithmetic blending (`αP1 + (1-α)P2`) applied gene-wise with random `α ∈ [0,1]`. |
| **Mutation**    | Two operators: <br>• Uniform random reset on selected genes <br>• Gaussian creep (`gene += N(0, σ)`, with σ proportional to map size).                     |
| **Elitism**     | Best individual is copied verbatim to the next generation when enabled.                                                                                    |
| **Termination** | Stops when the best fitness stagnates for *m* generations or after a maximum generation budget.                                                            |

All operators are vectorised; loops appear only where clarity demands it.&#x20;

---

## Repository layout

```
GA-PathPlanner/
├── main.m                     % entry point
├── operators/
│   ├── selection_rws.m
│   ├── selection_tournament.m
│   ├── selection_rank.m
│   ├── crossover_kpoint.m
│   ├── crossover_blend.m
│   ├── mutate_reset.m
│   └── mutate_gaussian.m
├── fitness.m                  % length + obstacle penalty
├── utils/
│   ├── generate_random_map.m  % quick test maps
│   └── plot_path.m            % visual overlay
└── examples/
    └── random_map.bmp
```

---

## Quick start

```matlab
% Run inside MATLAB
>> main
```

You will be prompted for:

1. Selection method (0 = roulette, 1 = tournament, 2 = rank)
2. Crossover operator (0 or 1)
3. Mutation operator (0 or 1)

The script then loads or generates a binary map, evolves a population until convergence, and displays execution time plus a figure showing the best path overlay.

---

## Performance tips

* Population size and generation cap are defined near the top of `main.m`.
* Vectorised fitness evaluation processes the entire population in just a few matrix operations.
* Obstacle penalty scaling is automatic, but you can override `alphaPenalty` in `fitness.m` for stricter collision avoidance.

---

## Extending the planner

| Idea                        | Where to start                                                                             |
| --------------------------- | ------------------------------------------------------------------------------------------ |
| Variable-length chromosomes | Replace the fixed gene count with a delimiter symbol and adjust crossover and mutation.    |
| Real robot footprint        | Dilate obstacles by the robot radius (`imdilate`) prior to optimisation.                   |
| Multi-objective GA          | Keep length and clearance as separate objectives and apply nondominated sorting (NSGA-II). |
| Parallel evaluation         | Wrap fitness calls in `parfor`; all functions are side-effect free.                        |

---

## License

MIT. Experiment, benchmark, and integrate the planner into your own robotics or optimisation projects. Pull requests for new operators or performance improvements are very welcome.
