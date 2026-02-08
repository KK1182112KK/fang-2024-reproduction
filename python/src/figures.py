"""
Generate all figures for Fang & Zhang (2024) reproduction.

Figures:
    Fig 1-2: Inexact predictor feedback -- states and controls
    Fig 3-4: Uncompensated (delay-ignorant) -- states and controls
    Fig 5-6: Delay-free baseline -- states and controls
    Fig 7-8: Distributed actuator states u_i(z,t) as 3D surface plots
"""

import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

from src.inexact_predictor import run_inexact_predictor
from src.uncompensated import run_uncompensated
from src.delay_free import run_delay_free


def _ensure_dir(path):
    """Create directory if it does not exist."""
    os.makedirs(path, exist_ok=True)


def _results_dir():
    """Return the path to the results directory."""
    return os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'results')


def _save_figure(fig, filepath, dpi):
    """Save figure and close it."""
    fig.savefig(filepath, dpi=dpi, bbox_inches='tight')
    plt.close(fig)


def _make_fig1(results_dir, dpi):
    """Fig 1: Inexact predictor -- state trajectories."""
    print('  Generating Fig 1 (Inexact predictor -- states)...')

    res = run_inexact_predictor()

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(9, 4))

    ax1.plot(res['t'], res['x_hist'][:, 0], 'b', linewidth=1.5)
    ax1.axhline(y=0, color='k', linestyle='--', linewidth=0.5)
    ax1.set_xlabel('t')
    ax1.set_ylabel('$x_1(t)$')
    ax1.set_title('State $x_1$')
    ax1.grid(True)

    ax2.plot(res['t'], res['x_hist'][:, 1], 'r', linewidth=1.5)
    ax2.axhline(y=0, color='k', linestyle='--', linewidth=0.5)
    ax2.set_xlabel('t')
    ax2.set_ylabel('$x_2(t)$')
    ax2.set_title('State $x_2$')
    ax2.grid(True)

    fig.suptitle('Figure 1: Inexact Predictor Feedback -- States (Fang & Zhang 2024)')

    filepath = os.path.join(results_dir, 'fig1_predictor_states.png')
    _save_figure(fig, filepath, dpi)
    print(f'    Saved fig1_predictor_states.png')


def _make_fig2(results_dir, dpi):
    """Fig 2: Inexact predictor -- control inputs."""
    print('  Generating Fig 2 (Inexact predictor -- controls)...')

    res = run_inexact_predictor()

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(9, 4))

    ax1.plot(res['t'], res['u1_hist'], 'b', linewidth=1.5)
    ax1.axhline(y=0, color='k', linestyle='--', linewidth=0.5)
    ax1.set_xlabel('t')
    ax1.set_ylabel('$u_1(t)$')
    ax1.set_title('Control $u_1$')
    ax1.grid(True)

    ax2.plot(res['t'], res['u2_hist'], 'r', linewidth=1.5)
    ax2.axhline(y=0, color='k', linestyle='--', linewidth=0.5)
    ax2.set_xlabel('t')
    ax2.set_ylabel('$u_2(t)$')
    ax2.set_title('Control $u_2$')
    ax2.grid(True)

    fig.suptitle('Figure 2: Inexact Predictor Feedback -- Controls (Fang & Zhang 2024)')

    filepath = os.path.join(results_dir, 'fig2_predictor_controls.png')
    _save_figure(fig, filepath, dpi)
    print(f'    Saved fig2_predictor_controls.png')


def _make_fig3(results_dir, dpi):
    """Fig 3: Uncompensated -- state trajectories."""
    print('  Generating Fig 3 (Uncompensated -- states)...')

    res = run_uncompensated()

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(9, 4))

    ax1.plot(res['t'], res['x_hist'][:, 0], 'b', linewidth=1.5)
    ax1.set_xlabel('t')
    ax1.set_ylabel('$x_1(t)$')
    ax1.set_title('State $x_1$')
    ax1.grid(True)

    ax2.plot(res['t'], res['x_hist'][:, 1], 'r', linewidth=1.5)
    ax2.set_xlabel('t')
    ax2.set_ylabel('$x_2(t)$')
    ax2.set_title('State $x_2$')
    ax2.grid(True)

    fig.suptitle('Figure 3: Uncompensated (Delay-Ignorant) -- States')

    filepath = os.path.join(results_dir, 'fig3_uncompensated_states.png')
    _save_figure(fig, filepath, dpi)
    print(f'    Saved fig3_uncompensated_states.png')


def _make_fig4(results_dir, dpi):
    """Fig 4: Uncompensated -- control inputs."""
    print('  Generating Fig 4 (Uncompensated -- controls)...')

    res = run_uncompensated()

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(9, 4))

    ax1.plot(res['t'], res['u1_hist'], 'b', linewidth=1.5)
    ax1.set_xlabel('t')
    ax1.set_ylabel('$u_1(t)$')
    ax1.set_title('Control $u_1$')
    ax1.grid(True)

    ax2.plot(res['t'], res['u2_hist'], 'r', linewidth=1.5)
    ax2.set_xlabel('t')
    ax2.set_ylabel('$u_2(t)$')
    ax2.set_title('Control $u_2$')
    ax2.grid(True)

    fig.suptitle('Figure 4: Uncompensated (Delay-Ignorant) -- Controls')

    filepath = os.path.join(results_dir, 'fig4_uncompensated_controls.png')
    _save_figure(fig, filepath, dpi)
    print(f'    Saved fig4_uncompensated_controls.png')


def _make_fig5(results_dir, dpi):
    """Fig 5: Delay-free -- state trajectories."""
    print('  Generating Fig 5 (Delay-free -- states)...')

    res = run_delay_free()

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(9, 4))

    ax1.plot(res['t'], res['x_hist'][:, 0], 'b', linewidth=1.5)
    ax1.axhline(y=0, color='k', linestyle='--', linewidth=0.5)
    ax1.set_xlabel('t')
    ax1.set_ylabel('$x_1(t)$')
    ax1.set_title('State $x_1$')
    ax1.grid(True)

    ax2.plot(res['t'], res['x_hist'][:, 1], 'r', linewidth=1.5)
    ax2.axhline(y=0, color='k', linestyle='--', linewidth=0.5)
    ax2.set_xlabel('t')
    ax2.set_ylabel('$x_2(t)$')
    ax2.set_title('State $x_2$')
    ax2.grid(True)

    fig.suptitle('Figure 5: Delay-Free Baseline -- States')

    filepath = os.path.join(results_dir, 'fig5_delayfree_states.png')
    _save_figure(fig, filepath, dpi)
    print(f'    Saved fig5_delayfree_states.png')


def _make_fig6(results_dir, dpi):
    """Fig 6: Delay-free -- control inputs."""
    print('  Generating Fig 6 (Delay-free -- controls)...')

    res = run_delay_free()

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(9, 4))

    ax1.plot(res['t'], res['u1_hist'], 'b', linewidth=1.5)
    ax1.axhline(y=0, color='k', linestyle='--', linewidth=0.5)
    ax1.set_xlabel('t')
    ax1.set_ylabel('$u_1(t)$')
    ax1.set_title('Control $u_1$')
    ax1.grid(True)

    ax2.plot(res['t'], res['u2_hist'], 'r', linewidth=1.5)
    ax2.axhline(y=0, color='k', linestyle='--', linewidth=0.5)
    ax2.set_xlabel('t')
    ax2.set_ylabel('$u_2(t)$')
    ax2.set_title('Control $u_2$')
    ax2.grid(True)

    fig.suptitle('Figure 6: Delay-Free Baseline -- Controls')

    filepath = os.path.join(results_dir, 'fig6_delayfree_controls.png')
    _save_figure(fig, filepath, dpi)
    print(f'    Saved fig6_delayfree_controls.png')


def _make_fig7(results_dir, dpi):
    """Fig 7: Distributed actuator state u_1(z,t) -- 3D surface."""
    print('  Generating Fig 7 (Actuator state u_1(z,t) surface)...')

    res = run_inexact_predictor()

    # Distributed actuator state: u_i(z,t) = U_i(t - D_i*(1-z))
    D1 = 0.4
    dt = res['t'][1] - res['t'][0]

    # Subsample time for manageable surface plot
    t_sub_idx = np.arange(0, len(res['t']), 10)
    t_sub = res['t'][t_sub_idx]
    Nz = 51
    z_vec = np.linspace(0, 1, Nz)

    # Build u1_full with pre-history for lookup
    N_D1 = int(round(D1 / dt))
    Nt = len(res['t'])
    u1_full = np.zeros(Nt + N_D1)
    u1_full[N_D1:N_D1 + Nt] = res['u1_hist']

    # Compute surface: u_1(z, t) = U_1(t - D1*(1-z))
    surf_data = np.zeros((Nz, len(t_sub)))
    for iz in range(Nz):
        for it in range(len(t_sub)):
            t_query = t_sub[it] - D1 * (1 - z_vec[iz])
            idx = int(round(t_query / dt)) + N_D1
            idx = max(0, min(idx, len(u1_full) - 1))
            surf_data[iz, it] = u1_full[idx]

    fig = plt.figure(figsize=(7, 5))
    ax = fig.add_subplot(111, projection='3d')
    T_mesh, Z_mesh = np.meshgrid(t_sub, z_vec)
    ax.plot_surface(T_mesh, Z_mesh, surf_data, cmap='viridis', edgecolor='none')
    ax.set_xlabel('t')
    ax.set_ylabel('z')
    ax.set_zlabel('$u_1(z,t)$')
    ax.set_title('Figure 7: Distributed Actuator State $u_1(z,t)$')
    ax.view_init(elev=30, azim=-30)

    filepath = os.path.join(results_dir, 'fig7_actuator_u1_surface.png')
    _save_figure(fig, filepath, dpi)
    print(f'    Saved fig7_actuator_u1_surface.png')


def _make_fig8(results_dir, dpi):
    """Fig 8: Distributed actuator state u_2(z,t) -- 3D surface."""
    print('  Generating Fig 8 (Actuator state u_2(z,t) surface)...')

    res = run_inexact_predictor()

    # Distributed actuator state: u_2(z,t) = U_2(t - D2*(1-z))
    D2 = 0.5
    dt = res['t'][1] - res['t'][0]

    # Subsample time for manageable surface plot
    t_sub_idx = np.arange(0, len(res['t']), 10)
    t_sub = res['t'][t_sub_idx]
    Nz = 51
    z_vec = np.linspace(0, 1, Nz)

    # Build u2_full with pre-history for lookup
    N_D2 = int(round(D2 / dt))
    Nt = len(res['t'])
    u2_full = np.zeros(Nt + N_D2)
    u2_full[N_D2:N_D2 + Nt] = res['u2_hist']

    # Compute surface: u_2(z, t) = U_2(t - D2*(1-z))
    surf_data = np.zeros((Nz, len(t_sub)))
    for iz in range(Nz):
        for it in range(len(t_sub)):
            t_query = t_sub[it] - D2 * (1 - z_vec[iz])
            idx = int(round(t_query / dt)) + N_D2
            idx = max(0, min(idx, len(u2_full) - 1))
            surf_data[iz, it] = u2_full[idx]

    fig = plt.figure(figsize=(7, 5))
    ax = fig.add_subplot(111, projection='3d')
    T_mesh, Z_mesh = np.meshgrid(t_sub, z_vec)
    ax.plot_surface(T_mesh, Z_mesh, surf_data, cmap='viridis', edgecolor='none')
    ax.set_xlabel('t')
    ax.set_ylabel('z')
    ax.set_zlabel('$u_2(z,t)$')
    ax.set_title('Figure 8: Distributed Actuator State $u_2(z,t)$')
    ax.view_init(elev=30, azim=-30)

    filepath = os.path.join(results_dir, 'fig8_actuator_u2_surface.png')
    _save_figure(fig, filepath, dpi)
    print(f'    Saved fig8_actuator_u2_surface.png')


# Dispatch table
_FIG_MAKERS = {
    'fig1': _make_fig1,
    'fig2': _make_fig2,
    'fig3': _make_fig3,
    'fig4': _make_fig4,
    'fig5': _make_fig5,
    'fig6': _make_fig6,
    'fig7': _make_fig7,
    'fig8': _make_fig8,
}


def generate_figures(fig_set='all', dpi=300):
    """Generate figures for Fang & Zhang (2024) reproduction.

    Parameters
    ----------
    fig_set : str
        'all' or specific: 'fig1', 'fig2', ..., 'fig8'
    dpi : int
        Resolution for saving (default: 300)
    """
    results_dir = _results_dir()
    _ensure_dir(results_dir)

    fig_set = fig_set.lower()

    if fig_set == 'all':
        for maker in _FIG_MAKERS.values():
            maker(results_dir, dpi)
    elif fig_set in _FIG_MAKERS:
        _FIG_MAKERS[fig_set](results_dir, dpi)
    else:
        raise ValueError(f'Unknown figure set: {fig_set}')
