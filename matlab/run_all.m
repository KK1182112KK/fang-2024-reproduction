%% RUN_ALL  One-click reproduction of all results
%
%  Usage:
%    run_all           — Run simulation + generate figures
%    run_all('test')   — Run validation tests only
%    run_all('all')    — Run everything (simulation + tests + figures)
%
%  This script is designed for both interactive use and CI pipelines.

function run_all(mode)
    if nargin < 1, mode = 'sim'; end

    % Setup paths
    root = fileparts(mfilename('fullpath'));
    addpath(genpath(fullfile(root, 'src')));

    fprintf('=== Paper Reproduction: run_all ===\n');
    fprintf('Mode: %s\n\n', mode);

    switch lower(mode)
        case 'sim'
            run_simulation();
            make_figures();
        case 'test'
            run_tests();
        case 'all'
            run_simulation();
            make_figures();
            run_tests();
        otherwise
            error('Unknown mode: %s. Use ''sim'', ''test'', or ''all''.', mode);
    end

    fprintf('\n=== Done ===\n');
end

function run_simulation()
    fprintf('--- Running simulations ---\n');

    fprintf('\n[1/3] Inexact predictor feedback:\n');
    res_pred = run_inexact_predictor();

    fprintf('\n[2/3] Uncompensated (delay-ignorant):\n');
    res_uncomp = run_uncompensated();

    fprintf('\n[3/3] Delay-free baseline:\n');
    res_free = run_delay_free();

    fprintf('\n--- Simulations complete ---\n');
end

function make_figures()
    fprintf('--- Generating figures ---\n');
    generate_figures('all', 300);
    fprintf('--- Figures complete ---\n');
end

function run_tests()
    fprintf('--- Running tests ---\n');
    results = runtests('tests', 'IncludeSubfolders', true);
    disp(results);
    n_failed = sum([results.Failed]);
    if n_failed > 0
        error('%d test(s) failed.', n_failed);
    end
end
