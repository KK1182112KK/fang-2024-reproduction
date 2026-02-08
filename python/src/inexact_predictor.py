"""
Inexact predictor feedback for 2-input nonlinear system.
Fang & Zhang (2024), Section 6, Eq. 35-40.

System:
    dx1/dt = x2
    dx2/dt = -x1 + sin(x1) + x2 + u1(t-D1) + tanh(u1(t-D1)) + sin(u2(t-D2))

Inexact predictor: uses a single nominal delay D0 in [D1, D2] instead of
exact per-channel delays. Predictor ODE integrated from t-D0 to t.

Feedback:
    u1(t) = -(kp1*P1(t) + kd1*P2(t))
    u2(t) = -(kp2*P1(t) + kd2*P2(t))
where P = [P1, P2] is the predicted state at time t.
"""

import numpy as np


def _lookup_u(t_query, dt, u_full, N_delay):
    """Lookup u from full history array (including pre-history).

    u_full is indexed so that u_full[N_delay] = u(0),
    u_full[0] = u(-D), etc.
    Index for time t_query: idx = round(t_query / dt) + N_delay
    """
    idx = int(round(t_query / dt)) + N_delay
    idx = max(0, min(idx, len(u_full) - 1))
    return u_full[idx]


def run_inexact_predictor(params=None):
    """Run inexact predictor feedback simulation.

    Parameters
    ----------
    params : dict, optional
        Override default parameters. Keys:
            D1    : float, delay on channel 1 (default: 0.4)
            D2    : float, delay on channel 2 (default: 0.5)
            D0    : float, predictor horizon (default: 0.45)
            kp1   : float, proportional gain ch 1 (default: 3.0)
            kd1   : float, derivative gain ch 1 (default: 5.0)
            kp2   : float, proportional gain ch 2 (default: 10.0)
            kd2   : float, derivative gain ch 2 (default: 10.0)
            x0    : array-like, initial state [x1, x2] (default: [1, 1])
            dt    : float, time step (default: 0.001)
            t_end : float, simulation end time (default: 10.0)

    Returns
    -------
    dict with keys: t, x_hist, u1_hist, u2_hist, P_hist
    """
    if params is None:
        params = {}

    D1    = params.get('D1', 0.4)
    D2    = params.get('D2', 0.5)
    D0    = params.get('D0', 0.45)
    kp1   = params.get('kp1', 3.0)
    kd1   = params.get('kd1', 5.0)
    kp2   = params.get('kp2', 10.0)
    kd2   = params.get('kd2', 10.0)
    x0    = np.asarray(params.get('x0', [1.0, 1.0]), dtype=float).ravel()
    dt    = params.get('dt', 0.001)
    t_end = params.get('t_end', 10.0)

    # Derived quantities
    Nt     = int(round(t_end / dt)) + 1
    N_D1   = int(round(D1 / dt))
    N_D2   = int(round(D2 / dt))
    N_pred = int(round(D0 / dt))

    # Pre-allocate
    t      = np.arange(Nt) * dt
    x_hist = np.zeros((Nt, 2))
    P_hist = np.zeros((Nt, 2))

    # Control history arrays with pre-history for negative times.
    # u1_full: indices 0:N_D1 are pre-history (u=0 for t<0)
    #          index N_D1 + k stores u1 at time step k
    u1_full = np.zeros(Nt + N_D1)
    u2_full = np.zeros(Nt + N_D2)

    # Initial conditions
    x_hist[0, :] = x0

    # Main time-stepping loop
    for k in range(Nt - 1):
        x1_k = x_hist[k, 0]
        x2_k = x_hist[k, 1]
        t_k  = t[k]

        # === Step 1: Solve predictor ODE from theta = t_k - D0 to t_k ===
        # Initial condition: P(t_k - D0) = x(:,k)
        P1 = x1_k
        P2 = x2_k
        d_theta = D0 / N_pred  # predictor sub-step size (= dt)

        for j in range(N_pred):
            theta_j = t_k - D0 + j * d_theta

            # Look up u1(theta_j) and u2(theta_j) from stored history.
            u1_theta = _lookup_u(theta_j, dt, u1_full, N_D1)
            u2_theta = _lookup_u(theta_j, dt, u2_full, N_D2)

            # Predictor dynamics (same as plant dynamics)
            dP1 = P2
            dP2 = -P1 + np.sin(P1) + P2 + u1_theta + np.tanh(u1_theta) + np.sin(u2_theta)

            # Euler step
            P1 = P1 + d_theta * dP1
            P2 = P2 + d_theta * dP2

        # Store predicted state
        P_hist[k, :] = [P1, P2]

        # === Step 2: Compute control from predicted state ===
        u1_k = -(kp1 * P1 + kd1 * P2)
        u2_k = -(kp2 * P1 + kd2 * P2)

        # === Step 3: Store controls in history ===
        u1_full[N_D1 + k] = u1_k
        u2_full[N_D2 + k] = u2_k

        # === Step 4: Advance system dynamics (Euler) ===
        # The plant uses ACTUAL delayed controls: u1(t_k - D1), u2(t_k - D2)
        u1_delayed = _lookup_u(t_k - D1, dt, u1_full, N_D1)
        u2_delayed = _lookup_u(t_k - D2, dt, u2_full, N_D2)

        dx1 = x2_k
        dx2 = (-x1_k + np.sin(x1_k) + x2_k
               + u1_delayed + np.tanh(u1_delayed) + np.sin(u2_delayed))

        x_hist[k + 1, 0] = x1_k + dt * dx1
        x_hist[k + 1, 1] = x2_k + dt * dx2

    # Final step: hold last predictor and control values
    P_hist[Nt - 1, :] = P_hist[Nt - 2, :]
    u1_full[N_D1 + Nt - 1] = u1_full[N_D1 + Nt - 2]
    u2_full[N_D2 + Nt - 1] = u2_full[N_D2 + Nt - 2]

    # Extract control histories aligned with t
    u1_hist = u1_full[N_D1:N_D1 + Nt].copy()
    u2_hist = u2_full[N_D2:N_D2 + Nt].copy()

    print(f'  Inexact predictor: |x(t_end)| = {np.linalg.norm(x_hist[-1]):.4e}')

    return {
        't': t,
        'x_hist': x_hist,
        'u1_hist': u1_hist,
        'u2_hist': u2_hist,
        'P_hist': P_hist,
    }
