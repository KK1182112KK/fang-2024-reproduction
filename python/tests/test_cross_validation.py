"""
Cross-validation tests: Euler vs RK4.
Fang & Zhang (2024), Section 6.

Verifies that Euler integration at small dt agrees with a simple RK4
implementation of the same dynamics.
"""

import sys
import os
import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from src.inexact_predictor import run_inexact_predictor, _lookup_u
from src.delay_free import run_delay_free


class TestEulerVsRK4DelayFree:
    """test_euler_vs_rk4_delay_free -- Euler and RK4 agree on delay-free system."""

    def test_agreement(self):
        """Euler at dt=0.001 and RK4 at dt=0.01 should agree within 1e-2."""
        # Euler reference (fine dt)
        res_euler = run_delay_free({'dt': 0.001, 't_end': 5.0})

        # RK4 implementation of delay-free system
        kp1, kd1 = 3.0, 5.0
        kp2, kd2 = 10.0, 10.0
        x0 = np.array([1.0, 1.0])
        dt_rk4 = 0.01
        t_end = 5.0
        Nt_rk4 = int(round(t_end / dt_rk4)) + 1

        t_rk4 = np.arange(Nt_rk4) * dt_rk4
        x_rk4 = np.zeros((Nt_rk4, 2))
        x_rk4[0, :] = x0

        def rhs(x):
            """Right-hand side of delay-free system."""
            x1, x2 = x
            u1 = -(kp1 * x1 + kd1 * x2)
            u2 = -(kp2 * x1 + kd2 * x2)
            dx1 = x2
            dx2 = -x1 + np.sin(x1) + x2 + u1 + np.tanh(u1) + np.sin(u2)
            return np.array([dx1, dx2])

        for k in range(Nt_rk4 - 1):
            xk = x_rk4[k, :]
            k1 = rhs(xk)
            k2 = rhs(xk + 0.5 * dt_rk4 * k1)
            k3 = rhs(xk + 0.5 * dt_rk4 * k2)
            k4 = rhs(xk + dt_rk4 * k3)
            x_rk4[k + 1, :] = xk + (dt_rk4 / 6.0) * (k1 + 2*k2 + 2*k3 + k4)

        # Interpolate Euler onto RK4 time grid
        x_euler_interp = np.zeros_like(x_rk4)
        for col in range(2):
            x_euler_interp[:, col] = np.interp(
                t_rk4, res_euler['t'], res_euler['x_hist'][:, col])

        max_err = np.max(np.abs(x_euler_interp - x_rk4))
        print(f'    Max |Euler - RK4| = {max_err:.4e}')
        assert max_err < 1e-2, \
            f'Euler and RK4 disagree too much: max error = {max_err:.4e}'


class TestEulerVsRK4Predictor:
    """test_euler_vs_rk4_predictor -- Euler predictor at two dt values converge."""

    def test_fine_vs_coarse(self):
        """Euler at dt=0.0005 and dt=0.002 should agree within 0.05."""
        res_fine = run_inexact_predictor({'dt': 0.0005, 't_end': 5.0})
        res_coarse = run_inexact_predictor({'dt': 0.002, 't_end': 5.0})

        # Interpolate fine onto coarse grid
        x_fine_interp = np.zeros_like(res_coarse['x_hist'])
        for col in range(2):
            x_fine_interp[:, col] = np.interp(
                res_coarse['t'], res_fine['t'], res_fine['x_hist'][:, col])

        max_err = np.max(np.abs(x_fine_interp - res_coarse['x_hist']))
        print(f'    Max |fine - coarse| = {max_err:.4e}')
        assert max_err < 0.05, \
            f'Euler predictor dt refinement mismatch: max error = {max_err:.4e}'


class TestRK4DelayFreeConverges:
    """test_rk4_delay_free_converges -- RK4 delay-free system converges."""

    def test_converges(self):
        """RK4 delay-free should converge, confirming gain design."""
        kp1, kd1 = 3.0, 5.0
        kp2, kd2 = 10.0, 10.0
        x0 = np.array([1.0, 1.0])
        dt = 0.01
        t_end = 10.0
        Nt = int(round(t_end / dt)) + 1

        x = np.zeros((Nt, 2))
        x[0, :] = x0

        def rhs(xv):
            x1, x2 = xv
            u1 = -(kp1 * x1 + kd1 * x2)
            u2 = -(kp2 * x1 + kd2 * x2)
            dx1 = x2
            dx2 = -x1 + np.sin(x1) + x2 + u1 + np.tanh(u1) + np.sin(u2)
            return np.array([dx1, dx2])

        for k in range(Nt - 1):
            xk = x[k, :]
            k1 = rhs(xk)
            k2 = rhs(xk + 0.5 * dt * k1)
            k3 = rhs(xk + 0.5 * dt * k2)
            k4 = rhs(xk + dt * k3)
            x[k + 1, :] = xk + (dt / 6.0) * (k1 + 2*k2 + 2*k3 + k4)

        final_norm = np.linalg.norm(x[-1])
        print(f'    RK4 delay-free: |x(t_end)| = {final_norm:.4e}')
        assert final_norm < 0.1, \
            f'RK4 delay-free did not converge: |x(t_end)| = {final_norm:.4e}'
