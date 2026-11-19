# DSA-CPO: Diversity- and Stagnation-Aware Crested Porcupine Optimizer

MATLAB code of **DSA-CPO** and of the original **CPO**, with two applications:
the bi-objective ZDT1 benchmark and the 3-D UAV base-station (UAV-BS)
placement problem of

> R. El Taani "An Adaptive Crested Porcupine
> Optimizer for 3D UAV Base Station Placement," ICCEIS 2026.

## Folder structure

```
DSA-CPO/
├── main_ZDT1.m                 DSA-CPO on the ZDT1 benchmark       <- start here
├── main_UAV.m                  DSA-CPO vs CPO on one UAV-BS scenario
├── reproduce_paper_results.m   DSA-CPO and CPO on the 15 scenarios of the paper
├── algorithms/
│   ├── CPO.m                   original Crested Porcupine Optimizer
│   └── DSA_CPO.m               proposed DSA-CPO
└── problems/
    ├── ZDT1.m                  ZDT1 benchmark (two objectives)
    ├── UAV_Problem_Setup.m     UAV-BS placement instance (area, users, bounds)
    ├── UAV_Fitness.m           UAV-BS placement fitness and metrics
    └── plot_deployment.m       top view of a deployment
```

Open MATLAB in the `DSA-CPO` folder and run one of the three `main` scripts;
they add the two sub-folders to the path themselves.
Requirements: MATLAB R2018b or later, no toolbox.

## The two algorithms

Both files have the same interface and share the same defences (sight, sound,
odor, physical attack), cyclic population reduction, greedy selection and
clipping at the bounds:

```matlab
[bestF, bestX, curve]       = CPO(N0, MaxFEs, lb, ub, dim, fobj)
[bestF, bestX, curve, info] = DSA_CPO(N0, MaxFEs, lb, ub, dim, fobj)
```

| | CPO | DSA-CPO |
|---|---|---|
| Exploration probability P_E | fixed, 1/2 | adapted once per iteration (below) |
| Extra function evaluations | – | none |
| Extra output | – | `info`: trace of P_E, diversity, stagnation, population size |

DSA-CPO computes

```
P_E = clip( P0(t) + (kD + kS*S) * (Dref(t) - Div/Div(0)), Pmin, Pmax )
P0(t)   = Pstart + (Pend - Pstart) * t/Tmax
Dref(t) = Dref0 * (1 - t/Tmax)
```

where Div is the normalised diversity of the active population and S in [0, 1]
the stagnation of the best fitness over the last Ws iterations. A population
more concentrated than the reference explores more, a more dispersed one
exploits more, and stagnation amplifies the correction.

`fobj` takes a 1 x dim row vector and must return a non-negative value,
because the odor and physical-attack defences use f_i / sum_k f_k.

```matlab
addpath('algorithms');
fobj = @(x) sum(x.^2);                                   % any f >= 0
[bestF, bestX, curve, info] = DSA_CPO(30, 30000, -100, 100, 10, fobj);
plot(info.fe, info.PE)                                   % P_E during the run
```

### Parameters (`opts`, optional 7th input, default values used in the paper)

| Field | Default | Meaning | Used by |
|---|---|---|---|
| `Nmin` | round(2N'/3) | minimum population size (20 for N' = 30) | both |
| `T` | 2 | number of population-reduction cycles | both |
| `alpha` | 0.2 | convergence-speed factor | both |
| `Tf` | 0.8 | odor / physical-attack trade-off | both |
| `Pstart`, `Pend` | 0.6, 0.3 | linear baseline P0(t) | DSA-CPO |
| `Dref0` | 0.5 | initial reference diversity | DSA-CPO |
| `kD`, `kS` | 0.5, 0.3 | diversity gain, stagnation gain | DSA-CPO |
| `Pmin`, `Pmax` | 0.1, 0.9 | clipping of P_E | DSA-CPO |
| `Ws` | 10 | stagnation window (iterations) | DSA-CPO |
| `deltaS` | 1e-3 | relative improvement regarded as normal progress | DSA-CPO |

The DSA-CPO parameters were set once and used unchanged in all experiments.

## ZDT1 benchmark (`main_ZDT1.m`)

ZDT1 (30 variables in [0, 1]) has two objectives, while DSA-CPO is a
single-objective optimizer. The script approximates the Pareto front by
decomposition: for 50 weight vectors w = (lambda, 1 - lambda), DSA-CPO
minimises the Tchebycheff function

```
g(x | w) = max( w1 |f1(x) - z1|, w2 |f2(x) - z2| ),   z = (0, 0)
```

with 5,000 evaluations per weight vector. Each run gives one point of the
front. The script prints IGD, GD and the hypervolume (reference point
(1.1, 1.1); 0.8767 for the exact front) and plots the obtained points against
the true front f2 = 1 - sqrt(f1). Set `optimizer = @CPO` to run CPO instead.

## UAV-BS placement (`main_UAV.m`, `reproduce_paper_results.m`)

N_d UAV-BSs at (X_j, Y_j, h_j) serve N_u ground users in a W x L area. The
decision vector is x = [X_1..X_N, Y_1..Y_N, h_1..h_N], the footprint radius is
R_j = h_j tan(45 deg), and the fitness to minimise is

```
F = wC (1 - C) + wQ (1 - Q) + wE * Ebar,   (wC, wQ, wE) = (1, 0.25, 0.25)
```

with C the fraction of covered users, Q the normalised coverage margin and
Ebar the mean normalised altitude.

```matlab
addpath('algorithms', 'problems');
P    = UAV_Problem_Setup(5, 30, 1);           % 5 UAV-BSs, 30 users, user seed 1
fobj = @(x) UAV_Fitness(x, P);
[bestF, bestX] = DSA_CPO(30, 30000, P.lb, P.ub, P.dim, fobj);
[F, out] = UAV_Fitness(bestX, P);             % coverage, altitudes, ...
plot_deployment(bestX, P, 'DSA-CPO');

P = UAV_Problem_Setup(5, 30, 201, 'dist', 'clustered');   % clustered users
```

### Reproducing the paper

* 15 scenarios: N_d = 1..8 with N_u = 30, and N_u = 5..40 with N_d = 5;
  100 m x 100 m, h in [5, 25] m, user seed 1.
* Population 30, 30,000 evaluations, 30 runs; run r uses
  `rng(1000 + r, 'twister')` for both algorithms (same initial population).

Expected mean fitness over the 30 runs (MATLAB, Apple silicon):

| N_d | N_u | DSA-CPO | CPO |
|---|---|---|---|
| 1 | 30 | 1.1350 | 1.1352 |
| 2 | 30 | 0.8733 | 0.8719 |
| 3 | 30 | 0.6581 | 0.6562 |
| 4 | 30 | 0.4698 | 0.4672 |
| 5 | 30 | 0.3919 | 0.3963 |
| 6 | 30 | 0.3563 | 0.3656 |
| 7 | 30 | 0.3339 | 0.3439 |
| 8 | 30 | 0.3175 | 0.3278 |
| 5 | 5 | 0.2580 | 0.2608 |
| 5 | 10 | 0.2931 | 0.2933 |
| 5 | 15 | 0.3212 | 0.3212 |
| 5 | 20 | 0.3645 | 0.3672 |
| 5 | 25 | 0.3756 | 0.3813 |
| 5 | 35 | 0.3931 | 0.3950 |
| 5 | 40 | 0.4197 | 0.4205 |

A run takes about 0.15 s for either algorithm.

## CPO implementation

`CPO.m` follows the equations published in [1]. It differs from the MATLAB
code distributed by the authors of [1] in three points, found by a
line-by-line comparison:

| Official code | Effect | `CPO.m` and `DSA_CPO.m` |
|---|---|---|
| `U2=rand(1,dim)<0.5*2-1;` | `0.5*2-1` is 0, so the sign vector is always 0: the perturbations of the odor and physical-attack defences vanish | sign vector `2*(rand(1,dim)<0.5)-1` in {-1, +1} |
| `rem(t,Tmax/T)/Tmax/T` | operator precedence divides by Tmax*T instead of Tmax/T, so the population barely shrinks | `rem(fe,cycleLen)/cycleLen`, `cycleLen = MaxFEs/T` |
| random re-initialisation of out-of-bound coordinates | differs from the other algorithms compared in the paper | clipping at the bounds |

On the 15 scenarios of the paper, this implementation is significantly better
than the official code in 14 scenarios (mean fitness 4.8 % lower on average,
up to 9.2 %).

## Citation

Please cite the paper above (see `CITATION.cff`) and the original CPO [1].
For ZDT1, cite [2].

[1] M. Abdel-Basset, R. Mohamed and M. Abouhawwash, "Crested Porcupine
Optimizer: A new nature-inspired metaheuristic," *Knowledge-Based Systems*,
vol. 284, 111257, 2024.

[2] E. Zitzler, K. Deb and L. Thiele, "Comparison of multiobjective
evolutionary algorithms: Empirical results," *Evolutionary Computation*,
vol. 8, no. 2, pp. 173–195, 2000.

## License

MIT, see `LICENSE`.
