"""
Delay-ignorant feedback for 2-input nonlinear system.
Fang & Zhang (2024), Section 6.

System (Eq. 35-36):
    dx1/dt = x2
    dx2/dt = -x1 + sin(x1) + x2 + u1(t-D1) + tanh(u1(t-D1)) + sin(u2(t-D2))

Control: delay-ignorant feedback (no predictor):
    u1(t) = -(kp1*x1(t) + kd1*x2(t))
    u2(t) = -(kp2*x1(t) + kd2*x2(t))

The system still has delays D1, D2 on the inputs, but the controller
ignores them. This is expected to destabilise the system.

Includes overflow protection: simulation stops if norm(x) > 1e6.
"""

import numpy as np


def _lookup_u(t_query, dt, u_full, N_delay):
    """Lookup u from full history array."""
    idx = int(round(t_query / dt)) + N_delay
    idx = max(0, min(idx, len(u_full) - 1))
    return u_full[idx]


def run_uncompensated(params=None):
    """Run uncompensated (delay-ignorant) simulation.

    Parameters
    ----------
    params : dict, optional
        Override default parameters. Keys:
            D1    : float, delay on channel 1 (default: 0.4)
            D2    : float, delay on channel 2 (default: 0.5)
            kp1   : float, proportional gain ch 1 (default: 3.0)
            kd1   : float, derivative gain ch 1 (default: 5.0)
            kp2   : float, proportional gain ch 2 (default: 10.0)
            kd2   : float, derivative gain ch 2 (default: 10.0)
            x0    : array-like, initial state [x1, x2] (default: [1, 1])
            dt    : float, time step (default: 0.001)
            t_end : float, simulation end time (default: 10.0)

    Returns
    -------
    dict with keys: t, x_hist, u1_hist, u2_hist
    """
    if params is None:
        params = {}

    D1    = params.get('D1', 0.4)
    D2    = params.get('D2', 0.5)
    kp1   = params.get('kp1', 3.0)
    kd1   = params.get('kd1', 5.0)
    kp2   = params.get('kp2', 10.0)
    kd2   = params.get('kd2', 10.0)
    x0    = np.asarray(params.get('x0', [1.0, 1.0]), dtype=float).ravel()
    dt    = params.get('dt', 0.001)
    t_end = params.get('t_end', 10.0)

    # Derived quantities
    Nt   = int(round(t_end / dt)) + 1
    N_D1 = int(round(D1 / dt))
    N_D2 = int(round(D2 / dt))

    # Pre-allocate
    t      = np.arange(Nt) * dt
    x_hist = np.zeros((Nt, 2))

    # Control history with pre-history (zeros for t < 0)
    u1_full = np.zeros(Nt + N_D1)
    u2_full = np.zeros(Nt + N_D2)

    # Initial conditions
    x_hist[0, :] = x0

    # Track actual simulation length (may stop early on overflow)
    k_final = Nt

    # Main time-stepping loop
    for k in range(Nt - 1):
        x1_k = x_hist[k, 0]
        x2_k = x_hist[k, 1]
        t_k  = t[k]

        # --- Overflow protection ---
        if np.linalg.norm([x1_k, x2_k]) > 1e6:
            print(f'  Uncompensated: overflow at t = {t_k:.3f}, stopping.')
            k_final = k + 1  # include current step
            break

        # --- Control law (no predictor, uses current state directly) ---
        u1_k = -(kp1 * x1_k + kd1 * x2_k)
        u2_k = -(kp2 * x1_k + kd2 * x2_k)

        # Store controls
        u1_full[N_D1 + k] = u1_k
        u2_full[N_D2 + k] = u2_k

        # --- System dynamics with actual delays ---
        u1_delayed = _lookup_u(t_k - D1, dt, u1_full, N_D1)
        u2_delayed = _lookup_u(t_k - D2, dt, u2_full, N_D2)

        dx1 = x2_k
        dx2 = (-x1_k + np.sin(x1_k) + x2_k
               + u1_delayed + np.tanh(u1_delayed) + np.sin(u2_delayed))

        # Euler step
        x_hist[k + 1, 0] = x1_k + dt * dx1
        x_hist[k + 1, 1] = x2_k + dt * dx2

    # Truncate on overflow
    if k_final < Nt:
        t      = t[:k_final]
        x_hist = x_hist[:k_final, :]
        u1_hist = u1_full[N_D1:N_D1 + k_final].copy()
        u2_hist = u2_full[N_D2:N_D2 + k_final].copy()
    else:
        # Hold last control value
        u1_full[N_D1 + Nt - 1] = u1_full[N_D1 + Nt - 2]
        u2_full[N_D2 + Nt - 1] = u2_full[N_D2 + Nt - 2]
        u1_hist = u1_full[N_D1:N_D1 + Nt].copy()
        u2_hist = u2_full[N_D2:N_D2 + Nt].copy()

    print(f'  Uncompensated: |x(t_end)| = {np.linalg.norm(x_hist[-1]):.4e}'
          f'  (t_end = {t[-1]:.3f})')

    return {
        't': t,
        'x_hist': x_hist,
        'u1_hist': u1_hist,
        'u2_hist': u2_hist,
    }
