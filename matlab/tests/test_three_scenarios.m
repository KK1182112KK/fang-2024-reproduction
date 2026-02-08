function tests = test_three_scenarios
%TEST_THREE_SCENARIOS  Comparative tests for predictor, uncompensated, and delay-free.
%
%  Validates the expected qualitative behaviour of each control strategy:
%    - Inexact predictor: converges
%    - Uncompensated (delay-ignorant): diverges or stays far from origin
%    - Delay-free baseline: converges
%  Also tests robustness of the predictor across a range of D0 values.
%
%  Run with:  results = runtests('test_three_scenarios');

    tests = functiontests(localfunctions);
end

%% -----------------------------------------------------------------------
%  test_predictor_converges
%  -----------------------------------------------------------------------
function test_predictor_converges(testCase)
    fprintf('\n--- test_predictor_converges ---\n');

    p.dt    = 0.001;
    p.t_end = 10.0;
    r = run_inexact_predictor(p);

    final_norm = norm(r.x_hist(end,:));
    fprintf('  Inexact predictor final |x|: %.6e\n', final_norm);

    verifyLessThan(testCase, final_norm, 0.1, ...
        'Inexact predictor should converge to near the origin.');
end

%% -----------------------------------------------------------------------
%  test_uncompensated_diverges
%  -----------------------------------------------------------------------
%  The uncompensated controller (delay-ignorant) should either overflow
%  or have a much larger final state norm than the predictor.
function test_uncompensated_diverges(testCase)
    fprintf('\n--- test_uncompensated_diverges ---\n');

    p.dt    = 0.001;
    p.t_end = 10.0;

    r_pred   = run_inexact_predictor(p);
    r_uncomp = run_uncompensated(p);

    norm_pred   = norm(r_pred.x_hist(end,:));
    norm_uncomp = norm(r_uncomp.x_hist(end,:));

    fprintf('  Predictor final |x|:      %.6e\n', norm_pred);
    fprintf('  Uncompensated final |x|:  %.6e\n', norm_uncomp);
    fprintf('  Simulation stopped at t = %.3f (uncompensated)\n', r_uncomp.t(end));

    % The uncompensated system should be significantly worse.
    % Either it overflowed (t(end) < t_end) or the final norm is larger.
    overflow_detected = r_uncomp.t(end) < p.t_end - p.dt;
    worse_norm = norm_uncomp > norm_pred;

    fprintf('  Overflow detected: %d\n', overflow_detected);
    fprintf('  Worse norm:        %d\n', worse_norm);

    verifyTrue(testCase, overflow_detected || worse_norm, ...
        'Uncompensated system should diverge or have larger final norm than predictor.');
end

%% -----------------------------------------------------------------------
%  test_delay_free_converges
%  -----------------------------------------------------------------------
function test_delay_free_converges(testCase)
    fprintf('\n--- test_delay_free_converges ---\n');

    p.dt    = 0.001;
    p.t_end = 10.0;
    r = run_delay_free(p);

    final_norm = norm(r.x_hist(end,:));
    fprintf('  Delay-free final |x|: %.6e\n', final_norm);

    verifyLessThan(testCase, final_norm, 0.1, ...
        'Delay-free system should converge to near the origin.');
end

%% -----------------------------------------------------------------------
%  test_predictor_similar_to_delay_free
%  -----------------------------------------------------------------------
%  The inexact predictor and delay-free final norms should be in the same
%  order of magnitude (both converge, though potentially at different rates).
function test_predictor_similar_to_delay_free(testCase)
    fprintf('\n--- test_predictor_similar_to_delay_free ---\n');

    p.dt    = 0.001;
    p.t_end = 10.0;

    r_pred = run_inexact_predictor(p);
    r_free = run_delay_free(p);

    norm_pred = norm(r_pred.x_hist(end,:));
    norm_free = norm(r_free.x_hist(end,:));

    fprintf('  Predictor final |x|:  %.6e\n', norm_pred);
    fprintf('  Delay-free final |x|: %.6e\n', norm_free);

    % Both should be small; check that the ratio of the larger to the
    % smaller does not exceed 1000 (same order of magnitude, loosely).
    if norm_pred < 1e-15 && norm_free < 1e-15
        % Both essentially zero — pass trivially
        fprintf('  Both norms negligible.\n');
        return;
    end

    larger  = max(norm_pred, norm_free);
    smaller = max(min(norm_pred, norm_free), 1e-15);  % guard against zero
    ratio = larger / smaller;

    fprintf('  Ratio (larger/smaller): %.2f\n', ratio);

    verifyLessThan(testCase, ratio, 1000, ...
        'Predictor and delay-free final norms should be within 3 orders of magnitude.');
end

%% -----------------------------------------------------------------------
%  test_D0_sweep
%  -----------------------------------------------------------------------
%  The inexact predictor should tolerate different nominal delays D0
%  between D1 and D2.  All should converge.
function test_D0_sweep(testCase)
    fprintf('\n--- test_D0_sweep ---\n');

    D0_values = [0.40, 0.42, 0.45, 0.48, 0.50];

    p.dt    = 0.001;
    p.t_end = 10.0;

    all_converged = true;
    for i = 1:length(D0_values)
        p.D0 = D0_values(i);
        r = run_inexact_predictor(p);

        fn = norm(r.x_hist(end,:));
        converged = fn < 0.1;
        fprintf('  D0 = %.2f  =>  |x(end)| = %.6e  converged = %d\n', ...
                D0_values(i), fn, converged);

        if ~converged
            all_converged = false;
        end
    end

    verifyTrue(testCase, all_converged, ...
        'Inexact predictor should converge for all D0 in [D1, D2].');
end
