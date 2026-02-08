# Specification: Inexact Predictor Feedback for Multi-Input Nonlinear Systems

## Reference
- **Authors**: Qin Fang, Zhengqiang Zhang
- **Title**: Inexact predictor feedback for multi-input nonlinear systems with distinct input delays
- **Venue**: Automatica, Vol. 159, Article 111399
- **Year**: 2024 (available online Nov 2023)
- **DOI**: 10.1016/j.automatica.2023.111399

## Problem Statement

The paper addresses stabilization of multi-input nonlinear systems where each input channel
has a different unknown constant delay. Instead of exact predictor feedback (which requires
one predictor per channel and knowledge of each delay), the authors propose **inexact predictor
feedback**: a single predictor with a fixed constant horizon D0 that robustly compensates
all delays simultaneously, provided max{|D_lower - D0|, |D_upper - D0|} <= epsilon.

## System Model (Numerical Example, Section 6)

Nonaffine nonlinear system with two delayed inputs (Eq. 35-36):

$$\dot{x}_1(t) = x_2(t)$$

$$\dot{x}_2(t) = -x_1(t) + \sin(x_1(t)) + x_2(t) + u_1(t - D_1) + \tanh(u_1(t - D_1)) + \sin(u_2(t - D_2))$$

where:
- $x(t) = [x_1(t), x_2(t)]^T \in \mathbb{R}^2$ is the state
- $u_1(t), u_2(t)$ are two control inputs
- $D_1 = 0.4$, $D_2 = 0.5$ are the distinct input delays

## Controller Design

### Delay-Free Controllers (Eq. 37-38)

PD controllers that stabilize the delay-free system:

$$u_1(t) = -[k_{p1}, k_{d1}] \cdot x(t) = -(3.0 \, x_1(t) + 5.0 \, x_2(t))$$

$$u_2(t) = -[k_{p2}, k_{d2}] \cdot x(t) = -(10 \, x_1(t) + 10 \, x_2(t))$$

### Inexact Predictor Feedback (Eq. 39-40)

$$u_1(t) = -[k_{p1}, k_{d1}] \cdot P(t) = -(3.0 \, P_1(t) + 5.0 \, P_2(t))$$

$$u_2(t) = -[k_{p2}, k_{d2}] \cdot P(t) = -(10 \, P_1(t) + 10 \, P_2(t))$$

### Predictor (Eq. 3, specialized to this example)

The predictor P(theta) satisfies, for theta in [t - D0, t]:

$$P(\theta) = Z(t) + \int_{t-D_0}^{\theta} f(P(s), U_1(s), U_2(s)) \, ds$$

with initial condition $P(t - D_0) = x(t)$ (current state).

Equivalently, this is the ODE initial value problem:

$$\frac{dP}{d\theta} = f(P(\theta), u_1(\theta), u_2(\theta)), \quad P(t - D_0) = x(t)$$

where the dynamics f are:

$$f_1(P, u_1, u_2) = P_2$$

$$f_2(P, u_1, u_2) = -P_1 + \sin(P_1) + P_2 + u_1 + \tanh(u_1) + \sin(u_2)$$

The predicted state is $y(t) = P(t)$, i.e., the predictor integrated from $\theta = t - D_0$ to $\theta = t$.

**Key insight**: For $\theta \in [t - D_0, t]$, the controls $u_1(\theta), u_2(\theta)$ are **past values** that are already known (stored in history). The predictor uses these known past controls to propagate the state forward by D0.

## Parameters

| Parameter | Value | Description |
|-----------|-------|-------------|
| D1 | 0.4 | Delay on channel 1 |
| D2 | 0.5 | Delay on channel 2 |
| D0 | 0.45 | Fixed constant prediction horizon |
| kp1 | 3.0 | Proportional gain, channel 1 |
| kd1 | 5.0 | Derivative gain, channel 1 |
| kp2 | 10.0 | Proportional gain, channel 2 |
| kd2 | 10.0 | Derivative gain, channel 2 |
| x(0) | [1.0, 1.0]^T | Initial state |
| u_i(tau) | 0 for tau < 0 | Control pre-history |

## Figures to Reproduce

| Figure | Description | Status |
|--------|-------------|--------|
| Fig. 1 | State x1, x2 under inexact predictor feedback | [ ] |
| Fig. 2 | Controls u1, u2 under inexact predictor feedback | [ ] |
| Fig. 3 | State x1, x2 without delay compensation (divergent) | [ ] |
| Fig. 4 | Controls u1, u2 without delay compensation | [ ] |
| Fig. 5 | State x1, x2 of delay-free system (baseline) | [ ] |
| Fig. 6 | Controls u1, u2 of delay-free system | [ ] |
| Fig. 7 | Distributed actuator state u1(z,t) — 3D surface | [ ] |
| Fig. 8 | Distributed actuator state u2(z,t) — 3D surface | [ ] |

## Three Simulation Scenarios

1. **Inexact predictor feedback** (Figs. 1-2): System (35-36) with controllers (39-40) using predictor (3) with D0 = 0.45. Should converge.
2. **Without delay compensation** (Figs. 3-4): System (35-36) with delay-free controllers (37-38) applied directly. Should diverge.
3. **Delay-free system** (Figs. 5-6): System (35-36) with D1 = D2 = 0 and controllers (37-38). Should converge (baseline).

## Distributed Actuator States (Figs. 7-8)

The PDE representation (Eq. 5-7) defines:

$$u_i(z, t) = U_i(t + D_i(z - 1)), \quad z \in [0, 1]$$

This is the transport PDE $D_i \partial u_i / \partial t = \partial u_i / \partial z$ with boundary $u_i(1, t) = U_i(t)$.

For Fig. 7-8: plot $u_i(z, t)$ as a 3D surface over $z \in [0,1]$ and $t \geq 0$.
This is simply a time-shifted version of the control signal: $u_i(z, t) = U_i(t - D_i(1-z))$.

## Success Criteria

1. **Convergence under predictor feedback**: x(t) -> 0 exponentially (match Fig. 1 qualitatively)
2. **Divergence without compensation**: State grows unbounded (match Fig. 3)
3. **Delay-free convergence**: Similar transient to predictor case (match Fig. 5)
4. **Quantitative match**: State trajectories, control signals, and convergence rates should visually match paper figures
5. **Exponential decay**: ||x(t)|| should decay approximately exponentially after initial transient
6. **Predictor accuracy**: P(t) at t=0 should match independent forward integration of the ODE
