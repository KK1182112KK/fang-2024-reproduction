"""
Delay-free system with direct state feedback.
Fang & Zhang (2024), Section 6.

System (no delays, D1=D2=0):
    dx1/dt = x2
    dx2/dt = -x1 + sin(x1) + x2 + u1 + tanh(u1) + sin(u2)

Control (delay-free feedback):
    u1(t) = -(kp1*x1(t) + kd1*x2(t))
    u2(t) = -(kp2*x1(t) + kd2*x2(t))

This is the ideal baseline case: the same gains used in the predictor
design, but applied without any transport delay. Should converge.
"""

import numpy as np


def run_delay_free(params=None):
    """Run delay-free baseline simulation.

    Parameters
    ----------
    params : dict, optional
        Override default parameters. Keys:
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

    kp1   = params.get('kp1', 3.0)
    kd1   = params.get('kd1', 5.0)
    kp2   = params.get('kp2', 10.0)
    kd2   = params.get('kd2', 10.0)
    x0    = np.asarray(params.get('x0', [1.0, 1.0]), dtype=float).ravel()
    dt    = params.get('dt', 0.001)
    t_end = params.get('t_end', 10.0)

    # Derived quantities
    Nt = int(round(t_end / dt)) + 1

    # Pre-allocate
    t       = np.arange(Nt) * dt
    x_hist  = np.zeros((Nt, 2))
    u1_hist = np.zeros(Nt)
    u2_hist = np.zeros(Nt)

    # Initial conditions
    x_hist[0, :] = x0

    # Main time-stepping loop
    for k in range(Nt - 1):
        x1_k = x_hist[k, 0]
        x2_k = x_hist[k, 1]

        # --- Control law (delay-free, applied immediately) ---
        u1_k = -(kp1 * x1_k + kd1 * x2_k)
        u2_k = -(kp2 * x1_k + kd2 * x2_k)

        u1_hist[k] = u1_k
        u2_hist[k] = u2_k

        # --- System dynamics (no delay) ---
        dx1 = x2_k
        dx2 = (-x1_k + np.sin(x1_k) + x2_k
               + u1_k + np.tanh(u1_k) + np.sin(u2_k))

        # Euler step
        x_hist[k + 1, 0] = x1_k + dt * dx1
        x_hist[k + 1, 1] = x2_k + dt * dx2

    # Final control (hold last value)
    u1_hist[Nt - 1] = u1_hist[Nt - 2]
    u2_hist[Nt - 1] = u2_hist[Nt - 2]

    print(f'  Delay-free: |x(t_end)| = {np.linalg.norm(x_hist[-1]):.4e}')

    return {
        't': t,
        'x_hist': x_hist,
        'u1_hist': u1_hist,
        'u2_hist': u2_hist,
    }
