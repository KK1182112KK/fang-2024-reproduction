"""
Validation tests for inexact predictor feedback.
Fang & Zhang (2024), Section 6.
"""

import sys
import os
import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from src.inexact_predictor import run_inexact_predictor


class TestConvergence:
    """test_convergence -- State converges to zero under inexact predictor."""

    def test_state_converges_to_zero(self):
        """State norm at t_end should be below 0.1."""
        res = run_inexact_predictor()
        final_norm = np.linalg.norm(res['x_hist'][-1])
        print(f'    |x(t_end)| = {final_norm:.4e}')
        assert final_norm < 0.1, \
            f'State did not converge: |x(t_end)| = {final_norm:.4e}'

    def test_x1_converges(self):
        """x1 component should converge to zero."""
        res = run_inexact_predictor()
        assert abs(res['x_hist'][-1, 0]) < 0.1, \
            f'x1 did not converge: x1(t_end) = {res["x_hist"][-1, 0]:.4e}'

    def test_x2_converges(self):
        """x2 component should converge to zero."""
        res = run_inexact_predictor()
        assert abs(res['x_hist'][-1, 1]) < 0.1, \
            f'x2 did not converge: x2(t_end) = {res["x_hist"][-1, 1]:.4e}'


class TestPredictorAtT0:
    """test_predictor_at_t0 -- Verify predictor ODE at t=0 vs independent integration."""

    def test_predictor_matches_independent(self):
        """Predictor at t=0 should match independent forward integration."""
        D0  = 0.45
        x0  = np.array([1.0, 1.0])
        dt  = 0.001

        N_pred = int(round(D0 / dt))
        d_theta = D0 / N_pred

        # At t=0, the predictor integrates from theta = -D0 to 0.
        # All past controls are 0 (pre-history).
        P1, P2 = x0[0], x0[1]
        for j in range(N_pred):
            u1_theta = 0.0  # pre-history
            u2_theta = 0.0  # pre-history
            dP1 = P2
            dP2 = -P1 + np.sin(P1) + P2 + u1_theta + np.tanh(u1_theta) + np.sin(u2_theta)
            P1 = P1 + d_theta * dP1
            P2 = P2 + d_theta * dP2

        P_manual = np.array([P1, P2])

        # Run full simulation and extract P(0)
        res = run_inexact_predictor({'x0': x0.tolist(), 'dt': dt, 'D0': D0})
        P_sim = res['P_hist'][0, :]

        err = np.linalg.norm(P_manual - P_sim)
        print(f'    P_manual = [{P_manual[0]:.6f}, {P_manual[1]:.6f}]')
        print(f'    P_sim    = [{P_sim[0]:.6f}, {P_sim[1]:.6f}]')
        print(f'    |error|  = {err:.4e}')

        assert err < 1e-10, \
            f'Predictor at t=0 does not match independent integration: err = {err:.4e}'


class TestControlInitial:
    """test_control_initial -- First control values are correct."""

    def test_first_u1_correct(self):
        """u1(0) should match -(kp1*P1(0) + kd1*P2(0))."""
        res = run_inexact_predictor()
        kp1 = 3.0
        kd1 = 5.0
        P1_0 = res['P_hist'][0, 0]
        P2_0 = res['P_hist'][0, 1]
        u1_expected = -(kp1 * P1_0 + kd1 * P2_0)
        u1_actual = res['u1_hist'][0]
        err = abs(u1_expected - u1_actual)
        print(f'    u1(0) expected = {u1_expected:.6f}')
        print(f'    u1(0) actual   = {u1_actual:.6f}')
        print(f'    |error|        = {err:.4e}')
        assert err < 1e-10, f'u1(0) mismatch: err = {err:.4e}'

    def test_first_u2_correct(self):
        """u2(0) should match -(kp2*P1(0) + kd2*P2(0))."""
        res = run_inexact_predictor()
        kp2 = 10.0
        kd2 = 10.0
        P1_0 = res['P_hist'][0, 0]
        P2_0 = res['P_hist'][0, 1]
        u2_expected = -(kp2 * P1_0 + kd2 * P2_0)
        u2_actual = res['u2_hist'][0]
        err = abs(u2_expected - u2_actual)
        print(f'    u2(0) expected = {u2_expected:.6f}')
        print(f'    u2(0) actual   = {u2_actual:.6f}')
        print(f'    |error|        = {err:.4e}')
        assert err < 1e-10, f'u2(0) mismatch: err = {err:.4e}'


class TestDtRefinement:
    """test_dt_refinement -- O(dt) convergence order estimate."""

    def test_convergence_order(self):
        """Average convergence order should be at least 0.8."""
        params_base = {'x0': [1.0, 1.0], 't_end': 5.0}

        dt_vals = [0.004, 0.002, 0.001, 0.0005]

        # Reference: dt = 0.00025
        params_ref = {**params_base, 'dt': 0.00025}
        res_ref = run_inexact_predictor(params_ref)

        errors = np.zeros(len(dt_vals))
        for i, dt_i in enumerate(dt_vals):
            params_i = {**params_base, 'dt': dt_i}
            res_i = run_inexact_predictor(params_i)

            # Interpolate reference onto this grid
            x_ref_interp = np.zeros_like(res_i['x_hist'])
            for col in range(2):
                x_ref_interp[:, col] = np.interp(
                    res_i['t'], res_ref['t'], res_ref['x_hist'][:, col])
            errors[i] = np.max(np.abs(res_i['x_hist'] - x_ref_interp))
            print(f'    dt = {dt_i:.5f}: max error = {errors[i]:.4e}')

        # Compute convergence orders between successive refinements
        print('    Convergence orders:')
        orders = []
        for i in range(len(dt_vals) - 1):
            if errors[i + 1] > 1e-14:
                ratio = dt_vals[i] / dt_vals[i + 1]
                order = np.log(errors[i] / errors[i + 1]) / np.log(ratio)
                orders.append(order)
                print(f'      dt {dt_vals[i]:.5f} -> {dt_vals[i+1]:.5f}: '
                      f'order = {order:.2f}')

        if orders:
            avg_order = np.mean(orders)
            print(f'    Average order = {avg_order:.2f} (expected ~1.0)')
            assert avg_order > 0.8, \
                f'Average convergence order too low: {avg_order:.2f}'


class TestExponentialDecay:
    """test_exponential_decay -- Negative slope of log(norm(x)) after transient."""

    def test_negative_slope(self):
        """log(||x||) should have a negative slope after initial transient."""
        res = run_inexact_predictor()
        t = res['t']
        x_norm = np.linalg.norm(res['x_hist'], axis=1)

        # Use data after initial transient (t > 2.0)
        mask = t > 2.0
        t_tail = t[mask]
        x_norm_tail = x_norm[mask]

        # Remove any near-zero values to avoid log issues
        valid = x_norm_tail > 1e-15
        t_valid = t_tail[valid]
        log_norm = np.log(x_norm_tail[valid])

        # Linear fit: log(||x||) ~ a*t + b
        if len(t_valid) > 10:
            coeffs = np.polyfit(t_valid, log_norm, 1)
            slope = coeffs[0]
            print(f'    Slope of log(||x||) for t > 2.0: {slope:.4f}')
            assert slope < -0.1, \
                f'Exponential decay not observed: slope = {slope:.4f}'
        else:
            pytest.skip('Not enough valid data points for exponential decay test.')
