# Inexact Predictor Feedback for Multi-Input Nonlinear Systems
### Reproduction of Fang & Zhang (Automatica, 2024)

[![Python Tests](https://github.com/KK1182112KK/fang-2024-reproduction/actions/workflows/python-ci.yml/badge.svg)](https://github.com/KK1182112KK/fang-2024-reproduction/actions/workflows/python-ci.yml)
[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)

## Overview

This repository reproduces the numerical example from Fang & Zhang (2024), which proposes **inexact predictor feedback** for multi-input nonlinear systems with distinct unknown input delays. Instead of requiring one predictor per input channel, a single predictor with a fixed constant horizon D0 robustly compensates all delays simultaneously.

## Key Results

| Scenario | Final |x(t_end)| | Behavior |
|----------|--------------------:|----------|
| Inexact predictor (D0=0.45) | ~1e-4 | Exponential convergence |
| Without compensation | Overflow | Divergent |
| Delay-free baseline | ~1e-5 | Exponential convergence |

## Methods

**System** (Eq. 35-36): 2-input nonaffine nonlinear system with delays D1=0.4, D2=0.5

**Inexact predictor**: Fixed horizon D0=0.45, predictor ODE integrated from t-D0 to t using stored control history. Feedback: u_i(t) = -[kp_i, kd_i] * P(t)

**Key equation**: P(theta) = x(t) + integral from t-D0 to theta of f(P(s), u1(s), u2(s)) ds

**Solver**: Forward Euler, dt=0.001, validated with O(dt) convergence order analysis.

## Quick Start

### MATLAB
```matlab
cd matlab
run('run_all.m')         % simulation + figures
run('run_all.m', 'test') % validation tests
```

### Python
```bash
pip install -r python/requirements.txt
python python/run_all.py              # simulation
python python/run_all.py --mode test  # validation tests
python python/run_all.py --mode fig   # generate figures
```

## Validation

| Test | Description | Result |
|------|-------------|--------|
| Convergence | x(t) -> 0 under predictor feedback | Pass |
| Divergence | Uncompensated system diverges | Pass |
| Predictor accuracy | P(0) matches independent integration | Pass |
| Convergence order | O(dt) verified across 4 step sizes | Pass |
| Exponential decay | log(norm(x)) has negative slope | Pass |
| D0 sweep | D0 in {0.4, 0.42, 0.45, 0.48, 0.5} all converge | Pass |

## Report

See [`docs/report.tex`](docs/report.tex) for the full technical report.

## Citation

```bibtex
@article{FangZhang2024,
  author  = {Qin Fang and Zhengqiang Zhang},
  title   = {Inexact predictor feedback for multi-input nonlinear systems with distinct input delays},
  journal = {Automatica},
  year    = {2024},
  volume  = {159},
  pages   = {111399},
  doi     = {10.1016/j.automatica.2023.111399}
}
```

## License

This project is licensed under the MIT License -- see [LICENSE](LICENSE) for details.
