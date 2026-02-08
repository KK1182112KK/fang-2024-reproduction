function generate_figures(fig_set, dpi)
%GENERATE_FIGURES  Generate all figures for Fang & Zhang (2024) reproduction.
%
%  generate_figures()
%  generate_figures(fig_set, dpi)
%
%  Arguments:
%    fig_set — 'all' (default), or specific: 'fig1','fig2',...,'fig8'
%    dpi     — resolution for saving (default 300)
%
%  Figures:
%    Fig 1-2: Inexact predictor feedback — states and controls
%    Fig 3-4: Uncompensated (delay-ignorant) — states and controls
%    Fig 5-6: Delay-free baseline — states and controls
%    Fig 7-8: Distributed actuator states u_i(z,t) as 3D surface plots

    if nargin < 1 || isempty(fig_set), fig_set = 'all'; end
    if nargin < 2 || isempty(dpi), dpi = 300; end

    results_dir = fullfile(fileparts(mfilename('fullpath')), '..', 'results');
    if ~exist(results_dir, 'dir')
        mkdir(results_dir);
    end

    switch lower(fig_set)
        case 'all'
            make_fig1(results_dir, dpi);
            make_fig2(results_dir, dpi);
            make_fig3(results_dir, dpi);
            make_fig4(results_dir, dpi);
            make_fig5(results_dir, dpi);
            make_fig6(results_dir, dpi);
            make_fig7(results_dir, dpi);
            make_fig8(results_dir, dpi);
        case 'fig1', make_fig1(results_dir, dpi);
        case 'fig2', make_fig2(results_dir, dpi);
        case 'fig3', make_fig3(results_dir, dpi);
        case 'fig4', make_fig4(results_dir, dpi);
        case 'fig5', make_fig5(results_dir, dpi);
        case 'fig6', make_fig6(results_dir, dpi);
        case 'fig7', make_fig7(results_dir, dpi);
        case 'fig8', make_fig8(results_dir, dpi);
        otherwise
            error('Unknown figure set: %s', fig_set);
    end
end

%% ====================================================================
%  Fig 1: Inexact predictor — state trajectories
%  ====================================================================
function make_fig1(results_dir, dpi)
    fprintf('  Generating Fig 1 (Inexact predictor — states)...\n');

    res = run_inexact_predictor();

    fig = figure('Position', [100 100 900 400], 'Visible', 'off');

    subplot(1,2,1);
    plot(res.t, res.x_hist(:,1), 'b', 'LineWidth', 1.5);
    hold on; grid on;
    yline(0, 'k--', 'LineWidth', 0.5);
    xlabel('t'); ylabel('x_1(t)');
    title('State x_1');

    subplot(1,2,2);
    plot(res.t, res.x_hist(:,2), 'r', 'LineWidth', 1.5);
    hold on; grid on;
    yline(0, 'k--', 'LineWidth', 0.5);
    xlabel('t'); ylabel('x_2(t)');
    title('State x_2');

    sgtitle('Figure 1: Inexact Predictor Feedback — States (Fang & Zhang 2024)');

    save_figure(fig, fullfile(results_dir, 'fig1_predictor_states.png'), dpi);
    fprintf('    Saved fig1_predictor_states.png\n');
end

%% ====================================================================
%  Fig 2: Inexact predictor — control inputs
%  ====================================================================
function make_fig2(results_dir, dpi)
    fprintf('  Generating Fig 2 (Inexact predictor — controls)...\n');

    res = run_inexact_predictor();

    fig = figure('Position', [100 100 900 400], 'Visible', 'off');

    subplot(1,2,1);
    plot(res.t, res.u1_hist, 'b', 'LineWidth', 1.5);
    hold on; grid on;
    yline(0, 'k--', 'LineWidth', 0.5);
    xlabel('t'); ylabel('u_1(t)');
    title('Control u_1');

    subplot(1,2,2);
    plot(res.t, res.u2_hist, 'r', 'LineWidth', 1.5);
    hold on; grid on;
    yline(0, 'k--', 'LineWidth', 0.5);
    xlabel('t'); ylabel('u_2(t)');
    title('Control u_2');

    sgtitle('Figure 2: Inexact Predictor Feedback — Controls (Fang & Zhang 2024)');

    save_figure(fig, fullfile(results_dir, 'fig2_predictor_controls.png'), dpi);
    fprintf('    Saved fig2_predictor_controls.png\n');
end

%% ====================================================================
%  Fig 3: Uncompensated — state trajectories
%  ====================================================================
function make_fig3(results_dir, dpi)
    fprintf('  Generating Fig 3 (Uncompensated — states)...\n');

    res = run_uncompensated();

    fig = figure('Position', [100 100 900 400], 'Visible', 'off');

    subplot(1,2,1);
    plot(res.t, res.x_hist(:,1), 'b', 'LineWidth', 1.5);
    hold on; grid on;
    xlabel('t'); ylabel('x_1(t)');
    title('State x_1');

    subplot(1,2,2);
    plot(res.t, res.x_hist(:,2), 'r', 'LineWidth', 1.5);
    hold on; grid on;
    xlabel('t'); ylabel('x_2(t)');
    title('State x_2');

    sgtitle('Figure 3: Uncompensated (Delay-Ignorant) — States');

    save_figure(fig, fullfile(results_dir, 'fig3_uncompensated_states.png'), dpi);
    fprintf('    Saved fig3_uncompensated_states.png\n');
end

%% ====================================================================
%  Fig 4: Uncompensated — control inputs
%  ====================================================================
function make_fig4(results_dir, dpi)
    fprintf('  Generating Fig 4 (Uncompensated — controls)...\n');

    res = run_uncompensated();

    fig = figure('Position', [100 100 900 400], 'Visible', 'off');

    subplot(1,2,1);
    plot(res.t, res.u1_hist, 'b', 'LineWidth', 1.5);
    hold on; grid on;
    xlabel('t'); ylabel('u_1(t)');
    title('Control u_1');

    subplot(1,2,2);
    plot(res.t, res.u2_hist, 'r', 'LineWidth', 1.5);
    hold on; grid on;
    xlabel('t'); ylabel('u_2(t)');
    title('Control u_2');

    sgtitle('Figure 4: Uncompensated (Delay-Ignorant) — Controls');

    save_figure(fig, fullfile(results_dir, 'fig4_uncompensated_controls.png'), dpi);
    fprintf('    Saved fig4_uncompensated_controls.png\n');
end

%% ====================================================================
%  Fig 5: Delay-free — state trajectories
%  ====================================================================
function make_fig5(results_dir, dpi)
    fprintf('  Generating Fig 5 (Delay-free — states)...\n');

    res = run_delay_free();

    fig = figure('Position', [100 100 900 400], 'Visible', 'off');

    subplot(1,2,1);
    plot(res.t, res.x_hist(:,1), 'b', 'LineWidth', 1.5);
    hold on; grid on;
    yline(0, 'k--', 'LineWidth', 0.5);
    xlabel('t'); ylabel('x_1(t)');
    title('State x_1');

    subplot(1,2,2);
    plot(res.t, res.x_hist(:,2), 'r', 'LineWidth', 1.5);
    hold on; grid on;
    yline(0, 'k--', 'LineWidth', 0.5);
    xlabel('t'); ylabel('x_2(t)');
    title('State x_2');

    sgtitle('Figure 5: Delay-Free Baseline — States');

    save_figure(fig, fullfile(results_dir, 'fig5_delayfree_states.png'), dpi);
    fprintf('    Saved fig5_delayfree_states.png\n');
end

%% ====================================================================
%  Fig 6: Delay-free — control inputs
%  ====================================================================
function make_fig6(results_dir, dpi)
    fprintf('  Generating Fig 6 (Delay-free — controls)...\n');

    res = run_delay_free();

    fig = figure('Position', [100 100 900 400], 'Visible', 'off');

    subplot(1,2,1);
    plot(res.t, res.u1_hist, 'b', 'LineWidth', 1.5);
    hold on; grid on;
    yline(0, 'k--', 'LineWidth', 0.5);
    xlabel('t'); ylabel('u_1(t)');
    title('Control u_1');

    subplot(1,2,2);
    plot(res.t, res.u2_hist, 'r', 'LineWidth', 1.5);
    hold on; grid on;
    yline(0, 'k--', 'LineWidth', 0.5);
    xlabel('t'); ylabel('u_2(t)');
    title('Control u_2');

    sgtitle('Figure 6: Delay-Free Baseline — Controls');

    save_figure(fig, fullfile(results_dir, 'fig6_delayfree_controls.png'), dpi);
    fprintf('    Saved fig6_delayfree_controls.png\n');
end

%% ====================================================================
%  Fig 7: Distributed actuator state u_1(z,t) — 3D surface
%  ====================================================================
function make_fig7(results_dir, dpi)
    fprintf('  Generating Fig 7 (Actuator state u_1(z,t) surface)...\n');

    res = run_inexact_predictor();

    % Distributed actuator state: u_i(z,t) = U_i(t - D_i*(1-z))
    % where z in [0,1] is the spatial coordinate and U_i is the control input.
    D1 = 0.4;
    dt = res.t(2) - res.t(1);

    % Subsample time for manageable surface plot
    t_sub_idx = 1:10:length(res.t);
    t_sub = res.t(t_sub_idx);
    Nz = 51;
    z_vec = linspace(0, 1, Nz);

    % Build u1_full with pre-history for lookup
    N_D1 = round(D1 / dt);
    Nt = length(res.t);
    u1_full = zeros(Nt + N_D1, 1);
    u1_full(N_D1 + (1:Nt)) = res.u1_hist;

    % Compute surface: u_1(z, t) = U_1(t - D1*(1-z))
    surf_data = zeros(Nz, length(t_sub));
    for iz = 1:Nz
        for it = 1:length(t_sub)
            t_query = t_sub(it) - D1 * (1 - z_vec(iz));
            idx = round(t_query / dt) + N_D1 + 1;
            idx = max(1, min(idx, length(u1_full)));
            surf_data(iz, it) = u1_full(idx);
        end
    end

    fig = figure('Position', [100 100 700 500], 'Visible', 'off');
    surf(t_sub, z_vec, surf_data, 'EdgeColor', 'none');
    colorbar;
    xlabel('t'); ylabel('z'); zlabel('u_1(z,t)');
    title('Figure 7: Distributed Actuator State u_1(z,t)');
    view([-30 30]);

    save_figure(fig, fullfile(results_dir, 'fig7_actuator_u1_surface.png'), dpi);
    fprintf('    Saved fig7_actuator_u1_surface.png\n');
end

%% ====================================================================
%  Fig 8: Distributed actuator state u_2(z,t) — 3D surface
%  ====================================================================
function make_fig8(results_dir, dpi)
    fprintf('  Generating Fig 8 (Actuator state u_2(z,t) surface)...\n');

    res = run_inexact_predictor();

    % Distributed actuator state: u_2(z,t) = U_2(t - D2*(1-z))
    D2 = 0.5;
    dt = res.t(2) - res.t(1);

    % Subsample time for manageable surface plot
    t_sub_idx = 1:10:length(res.t);
    t_sub = res.t(t_sub_idx);
    Nz = 51;
    z_vec = linspace(0, 1, Nz);

    % Build u2_full with pre-history for lookup
    N_D2 = round(D2 / dt);
    Nt = length(res.t);
    u2_full = zeros(Nt + N_D2, 1);
    u2_full(N_D2 + (1:Nt)) = res.u2_hist;

    % Compute surface: u_2(z, t) = U_2(t - D2*(1-z))
    surf_data = zeros(Nz, length(t_sub));
    for iz = 1:Nz
        for it = 1:length(t_sub)
            t_query = t_sub(it) - D2 * (1 - z_vec(iz));
            idx = round(t_query / dt) + N_D2 + 1;
            idx = max(1, min(idx, length(u2_full)));
            surf_data(iz, it) = u2_full(idx);
        end
    end

    fig = figure('Position', [100 100 700 500], 'Visible', 'off');
    surf(t_sub, z_vec, surf_data, 'EdgeColor', 'none');
    colorbar;
    xlabel('t'); ylabel('z'); zlabel('u_2(z,t)');
    title('Figure 8: Distributed Actuator State u_2(z,t)');
    view([-30 30]);

    save_figure(fig, fullfile(results_dir, 'fig8_actuator_u2_surface.png'), dpi);
    fprintf('    Saved fig8_actuator_u2_surface.png\n');
end

%% ====================================================================
%  Utility functions
%  ====================================================================

function save_figure(fig, filepath, dpi)
    print(fig, filepath, '-dpng', sprintf('-r%d', dpi));
    close(fig);
end

function val = get_field(s, name, default)
    if isfield(s, name)
        val = s.(name);
    else
        val = default;
    end
end
