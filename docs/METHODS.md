# Numerical Methods & Implementation Notes

## ODE Solver

**Forward Euler** for all time integration (system dynamics, predictor ODE).

Rationale:
- System is non-stiff for the given parameters (no extreme eigenvalues)
- Forward Euler is simple, transparent, and sufficient for dt <= 0.001
- Matches the Ponomarev 2016 reproduction approach
- RK4 available as cross-validation reference

## Discretization

### Time stepping
- **dt = 0.001** (primary): Fine enough for O(dt) Euler to give <1% error
- Time horizon: t_end = 10.0 (paper figures show approximately this range)
- N_t = t_end/dt + 1 = 10001 time steps

### Predictor integration
- The predictor ODE is integrated from theta = t_k - D0 to theta = t_k at each time step
- N_pred = round(D0/dt) sub-steps for the predictor integration
- ds = D0/N_pred (predictor sub-step size)
- At each sub-step j, query the stored control history: u_i(t_k - D0 + j*ds)

### Delay handling
- Pre-allocate control history arrays: u1_full[N_t + N_D1], u2_full[N_t + N_D2]
- N_Di = round(Di/dt) offset for each channel
- Control pre-history: u_i(tau) = 0 for tau < 0
- Lookup: u_i(t_query) via index = round(t_query/dt) + N_Di, clamped to valid range

**Alternative (simpler) delay approach**: Use a single large u_full array for each channel
with offset N_Dmax = round(max(D1,D2)/dt), but per-channel offsets are more precise.

### Distributed actuator state (Figs. 7-8)
- u_i(z, t) = U_i(t - Di*(1-z)) — simply a time-shifted lookup into the control history
- Spatial grid: z in [0, 1] with N_z = 51 points
- Temporal grid: same as simulation time steps (or subsampled for plotting)

## Numerical Pitfalls

1. **Predictor control lookup boundary**: At early times t < D0, the predictor
   queries controls at times theta < 0. These fall in the pre-history (u_i = 0).
   Must handle the index clamping correctly.

2. **Divergent scenario**: Without delay compensation (Figs. 3-4), the system diverges.
   Need to cap the simulation if |x| exceeds a threshold (e.g., 1e6) to avoid overflow.

3. **Non-affine terms**: The system has tanh(u1) and sin(u2) — nonlinear in control.
   This means the system is non-affine, which is important for the predictor: the
   predictor uses the actual control values, not linearized.

4. **Predictor horizon vs actual delays**: D0 = 0.45 does NOT match either D1 = 0.4
   or D2 = 0.5 exactly. The mismatch |Di - D0| = 0.05 for both channels. This is
   the "inexact" nature of the approach. The predictor compensates approximately.

## Validation Strategy

1. **Predictor accuracy at t=0**: Independently integrate the predictor ODE with
   known initial conditions and pre-history. Compare with simulation's P(0).

2. **Convergence order**: Run with dt = {0.004, 0.002, 0.001, 0.0005} and compare
   against reference solution at dt = 0.00025. Expect O(dt) for Euler.

3. **Three-way comparison**: Verify that:
   - Predictor feedback converges (Fig. 1)
   - Uncompensated diverges (Fig. 3)
   - Delay-free converges faster (Fig. 5)

4. **Exponential decay rate**: After initial transient, fit log(||x||) vs t and
   verify negative slope.

5. **Delay sweep**: Vary D0 and verify convergence for D0 near the actual delays,
   divergence for D0 far away (tests the epsilon condition).

6. **Cross-method validation**: Compare Euler solution at fine dt with RK4 at
   coarser dt to confirm both agree.

## Performance Notes

- **Runtime estimate**: 10001 time steps x (N_pred = 450 sub-steps per predictor) = ~4.5M flops/step
- Total: ~45 billion flops. With vectorization, expect ~5-10 seconds in MATLAB, ~15-30 seconds in Python.
- The predictor is the dominant cost (inner loop at each time step).
- No parallelization needed; sequential is sufficient.
