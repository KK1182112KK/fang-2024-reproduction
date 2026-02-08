#!/usr/bin/env python3
"""
One-click reproduction of Fang & Zhang (2024) results.

Usage:
    python run_all.py              # default 'sim' mode
    python run_all.py --mode sim   # run simulations only
    python run_all.py --mode fig   # generate figures only
    python run_all.py --mode test  # run validation tests only
    python run_all.py --mode all   # everything: sim + fig + test

Reference:
    Fang & Zhang (2024), "Inexact predictor feedback for multi-input
    nonlinear systems with distinct input delays", Automatica.
"""

import argparse
import sys
import os
import numpy as np

# Ensure the package root is on the path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from src.inexact_predictor import run_inexact_predictor
from src.uncompensated import run_uncompensated
from src.delay_free import run_delay_free
from src.figures import generate_figures


def print_banner():
    """Print identification banner."""
    print('=============================================')
    print('  Fang & Zhang (2024) -- Reproduction')
    print('  Inexact Predictor Feedback for')
    print('  Multi-Input Nonlinear Systems with')
    print('  Distinct Input Delays')
    print('=============================================')
    print()


def run_simulation():
    """Run all simulation scenarios."""
    print('--- Simulation ---\n')

    # Scenario 1: Inexact predictor feedback
    print('[Scenario 1] Inexact predictor feedback (D0 = 0.45)')
    res_pred = run_inexact_predictor()
    print(f'  Final: |x({res_pred["t"][-1]:.0f})| = '
          f'{np.linalg.norm(res_pred["x_hist"][-1]):.4e}\n')

    # Scenario 2: Uncompensated (delay-ignorant)
    print('[Scenario 2] Uncompensated (delay-ignorant)')
    res_uncomp = run_uncompensated()
    print(f'  Final: |x({res_uncomp["t"][-1]:.3f})| = '
          f'{np.linalg.norm(res_uncomp["x_hist"][-1]):.4e}\n')

    # Scenario 3: Delay-free baseline
    print('[Scenario 3] Delay-free baseline')
    res_df = run_delay_free()
    print(f'  Final: |x({res_df["t"][-1]:.0f})| = '
          f'{np.linalg.norm(res_df["x_hist"][-1]):.4e}\n')

    # D0 sweep
    print('[D0 Sweep] Testing robustness across D0 values')
    for D0_val in [0.40, 0.42, 0.45, 0.48, 0.50]:
        res = run_inexact_predictor({'D0': D0_val})
        print(f'  D0 = {D0_val:.2f}: |x(t_end)| = '
              f'{np.linalg.norm(res["x_hist"][-1]):.4e}')

    # Summary
    print('\n--- Summary ---')
    print(f'  Predictor (D0=0.45):  |x(10)| = '
          f'{np.linalg.norm(res_pred["x_hist"][-1]):.4e}  [should converge]')
    print(f'  Uncompensated:        |x|     = '
          f'{np.linalg.norm(res_uncomp["x_hist"][-1]):.4e}  [should diverge]')
    print(f'  Delay-free:           |x(10)| = '
          f'{np.linalg.norm(res_df["x_hist"][-1]):.4e}  [should converge]')


def run_figures():
    """Generate all figures."""
    print('--- Generating Figures ---\n')
    generate_figures(fig_set='all', dpi=300)


def run_tests():
    """Run validation tests via pytest."""
    print('--- Running Tests ---\n')
    import pytest
    test_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'tests')
    exit_code = pytest.main([test_dir, '-v', '--tb=short'])
    if exit_code != 0:
        print(f'\n{exit_code} test(s) failed.')
        sys.exit(exit_code)


def main():
    """Entry point."""
    parser = argparse.ArgumentParser(
        description='Fang & Zhang (2024) reproduction -- Python implementation')
    parser.add_argument('--mode', default='sim',
                        choices=['sim', 'fig', 'test', 'all'],
                        help='Execution mode (default: sim)')
    args = parser.parse_args()

    print_banner()

    if args.mode == 'sim':
        run_simulation()
    elif args.mode == 'fig':
        run_figures()
    elif args.mode == 'test':
        run_tests()
    elif args.mode == 'all':
        run_simulation()
        run_figures()
        run_tests()

    print('\n=== Done ===')


if __name__ == '__main__':
    main()
