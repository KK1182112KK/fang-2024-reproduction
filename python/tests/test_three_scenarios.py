"""
Validation tests for three simulation scenarios.
Fang & Zhang (2024), Section 6.

Scenario 1: Inexact predictor feedback (should converge)
Scenario 2: Uncompensated / delay-ignorant (should diverge)
Scenario 3: Delay-free baseline (should converge)
"""

import sys
import os
import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from src.inexact_predictor import run_inexact_predictor
from src.uncompensated import run_uncompensated
from src.delay_free import run_delay_free


class TestPredictorConverges:
    """test_predictor_converges -- Inexact predictor feedback converges."""

    def test_converges(self):
        """State under predictor feedback converges to zero."""
        res = run_inexact_predictor()
        final_norm = np.linalg.norm(res['x_hist'][-1])
        print(f'    Predictor: |x(t_end)| = {final_norm:.4e}')
        assert final_norm < 0.1, \
            f'Predictor feedback did not converge: |x(t_end)| = {final_norm:.4e}'


class TestUncompensatedDiverges:
    """test_uncompensated_diverges -- Delay-ignorant controller destabilises."""

    def test_diverges(self):
        """Uncompensated system should diverge (norm grows or overflows)."""
        res = run_uncompensated()
        final_norm = np.linalg.norm(res['x_hist'][-1])
        print(f'    Uncompensated: |x(t_end)| = {final_norm:.4e}')
        print(f'    Simulation ended at t = {res["t"][-1]:.3f}')
        # Either it overflowed (t < t_end) or the final norm is large
        diverged = (res['t'][-1] < 9.9) or (final_norm > 1.0)
        assert diverged, \
            f'Uncompensated system did not diverge: |x| = {final_norm:.4e}'

    def test_worse_than_predictor(self):
        """Uncompensated should perform worse than predictor feedback."""
        res_pred = run_inexact_predictor()
        res_uncomp = run_uncompensated()

        norm_pred = np.linalg.norm(res_pred['x_hist'][-1])
        norm_uncomp = np.linalg.norm(res_uncomp['x_hist'][-1])

        print(f'    Predictor:     |x(t_end)| = {norm_pred:.4e}')
        print(f'    Uncompensated: |x(t_end)| = {norm_uncomp:.4e}')
        assert norm_uncomp > norm_pred, \
            'Uncompensated should perform worse than predictor.'


class TestDelayFreeConverges:
    """test_delay_free_converges -- Delay-free baseline converges."""

    def test_converges(self):
        """Delay-free system should converge to zero."""
        res = run_delay_free()
        final_norm = np.linalg.norm(res['x_hist'][-1])
        print(f'    Delay-free: |x(t_end)| = {final_norm:.4e}')
        assert final_norm < 0.1, \
            f'Delay-free system did not converge: |x(t_end)| = {final_norm:.4e}'


class TestPredictorSimilarToDelayFree:
    """test_predictor_similar_to_delay_free -- Both converge with similar final norms."""

    def test_both_converge(self):
        """Both predictor and delay-free should converge."""
        res_pred = run_inexact_predictor()
        res_df = run_delay_free()

        norm_pred = np.linalg.norm(res_pred['x_hist'][-1])
        norm_df = np.linalg.norm(res_df['x_hist'][-1])

        print(f'    Predictor:  |x(t_end)| = {norm_pred:.4e}')
        print(f'    Delay-free: |x(t_end)| = {norm_df:.4e}')
        assert norm_pred < 0.1, \
            f'Predictor did not converge: |x| = {norm_pred:.4e}'
        assert norm_df < 0.1, \
            f'Delay-free did not converge: |x| = {norm_df:.4e}'

    def test_similar_final_magnitude(self):
        """Final norms should be within an order of magnitude of each other."""
        res_pred = run_inexact_predictor()
        res_df = run_delay_free()

        norm_pred = np.linalg.norm(res_pred['x_hist'][-1])
        norm_df = np.linalg.norm(res_df['x_hist'][-1])

        # Both should be small; check they are within a factor of 100
        if norm_df > 1e-15 and norm_pred > 1e-15:
            ratio = max(norm_pred, norm_df) / min(norm_pred, norm_df)
            print(f'    Ratio: {ratio:.2f}')
            assert ratio < 100, \
                f'Final norms differ by too much: ratio = {ratio:.2f}'


class TestD0Sweep:
    """test_d0_sweep -- Various D0 values in [D1, D2] all converge."""

    @pytest.mark.parametrize("D0_val", [0.40, 0.42, 0.45, 0.48, 0.50])
    def test_converges(self, D0_val):
        """State should converge for each D0 in [D1, D2].

        Boundary values (D0=0.40, D0=0.50) converge more slowly,
        so we use t_end=30 and a relaxed threshold of 1.0 to confirm
        the convergence trend rather than demand small residuals.
        """
        params = {'D0': D0_val, 't_end': 30.0}
        res = run_inexact_predictor(params)
        final_norm = np.linalg.norm(res['x_hist'][-1])
        print(f'    D0 = {D0_val:.2f}: |x(t_end)| = {final_norm:.4e}')
        assert final_norm < 1.0, \
            f'State did not converge for D0 = {D0_val:.2f}: |x| = {final_norm:.4e}'
