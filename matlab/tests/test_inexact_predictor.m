function tests = test_inexact_predictor
%TEST_INEXACT_PREDICTOR  Unit tests for the inexact predictor controller.
%
%  Tests convergence, predictor initialisation, control computation,
%  time-step refinement order, and exponential decay of the state norm.
%
%  Run with:  results = runtests('test_inexact_predictor');

    tests = functiontests(localfunctions);
end

%% -----------------------------------------------------------------------
%  test_convergence
%  -----------------------------------------------------------------------
%  After t_end = 10 the state should be near the origin.
function test_convergence(testCase)
    fprintf('\n--- test_convergence ---\n');

    p.dt    = 0.001;
    p.t_end = 10.0;
    r = run_inexact_predictor(p);

    final_norm = norm(r.x_hist(end,:));
    fprintf('  Final state norm: %.6e\n', final_norm);

    verifyLessThan(testCase, final_norm, 0.1, ...
        'State should converge close to the origin by t=10.');
end

%% -----------------------------------------------------------------------
%  test_predictor_at_t0
%  -----------------------------------------------------------------------
%  At t = 0 the predictor integrates from theta = -D0 to 0.
%  Because all control pre-history is zero, we can reproduce this
%  integration independently.  P(−D0) = x0 = [1;1], u1=u2=0,
%  dynamics: dP1 = P2, dP2 = −P1 + sin(P1) + P2.
function test_predictor_at_t0(testCase)
    fprintf('\n--- test_predictor_at_t0 ---\n');

    D0  = 0.45;
    dt  = 0.001;
    x0  = [1; 1];

    % Run the full simulation (only need step k=1 result)
    p.dt    = dt;
    p.t_end = 0.1;       % short run, we only need P_hist(1,:)
    p.D0    = D0;
    r = run_inexact_predictor(p);

    P_sim = r.P_hist(1,:);   % predictor output at t=0
    fprintf('  P_hist(1,:) from simulation: [%.10f, %.10f]\n', P_sim(1), P_sim(2));

    % Manual integration: Euler, same dt, u1=u2=0 everywhere
    N_pred = round(D0 / dt);
    P1 = x0(1);
    P2 = x0(2);
    for j = 0:N_pred-1
        dP1 = P2;
        dP2 = -P1 + sin(P1) + P2;   % u1=u2=0 -> tanh(0)=0, sin(0)=0
        P1 = P1 + dt * dP1;
        P2 = P2 + dt * dP2;
    end
    P_manual = [P1, P2];
    fprintf('  Manual integration result:   [%.10f, %.10f]\n', P_manual(1), P_manual(2));

    err = norm(P_sim - P_manual);
    fprintf('  Error: %.4e\n', err);

    verifyLessThan(testCase, err, 1e-10, ...
        'Predictor at t=0 should match manual Euler integration exactly.');
end

%% -----------------------------------------------------------------------
%  test_control_initial
%  -----------------------------------------------------------------------
%  At k=1 the control is computed from the predictor output P:
%    u1(0) = -(kp1*P1 + kd1*P2),   u2(0) = -(kp2*P1 + kd2*P2)
function test_control_initial(testCase)
    fprintf('\n--- test_control_initial ---\n');

    kp1 = 3;  kd1 = 5;
    kp2 = 10; kd2 = 10;

    p.dt    = 0.001;
    p.t_end = 0.1;
    r = run_inexact_predictor(p);

    P1 = r.P_hist(1,1);
    P2 = r.P_hist(1,2);

    u1_expected = -(kp1*P1 + kd1*P2);
    u2_expected = -(kp2*P1 + kd2*P2);

    u1_actual = r.u1_hist(1);
    u2_actual = r.u2_hist(1);

    fprintf('  u1: expected = %.10f, actual = %.10f\n', u1_expected, u1_actual);
    fprintf('  u2: expected = %.10f, actual = %.10f\n', u2_expected, u2_actual);

    err_u1 = abs(u1_actual - u1_expected);
    err_u2 = abs(u2_actual - u2_expected);

    verifyLessThan(testCase, err_u1, 1e-12, ...
        'u1(0) should match -(kp1*P1 + kd1*P2) exactly.');
    verifyLessThan(testCase, err_u2, 1e-12, ...
        'u2(0) should match -(kp2*P1 + kd2*P2) exactly.');
end

%% -----------------------------------------------------------------------
%  test_dt_refinement
%  -----------------------------------------------------------------------
%  The Euler scheme is first-order.  We check that halving dt roughly
%  halves the error relative to a fine reference solution.
function test_dt_refinement(testCase)
    fprintf('\n--- test_dt_refinement ---\n');

    t_end = 5.0;

    % Reference solution with very fine dt
    p_ref.dt    = 0.00025;
    p_ref.t_end = t_end;
    r_ref = run_inexact_predictor(p_ref);

    dt_list = [0.004, 0.002, 0.001, 0.0005];
    err = zeros(size(dt_list));

    for i = 1:length(dt_list)
        p_i.dt    = dt_list(i);
        p_i.t_end = t_end;
        r_i = run_inexact_predictor(p_i);

        % Compute max error at common time points
        % Sample at the coarsest grid of (reference, current)
        Nt_i = length(r_i.t);
        ratio = round(dt_list(i) / p_ref.dt);
        max_err = 0;
        for k = 1:Nt_i
            idx_ref = (k-1)*ratio + 1;
            if idx_ref > length(r_ref.t), break; end
            e = norm(r_i.x_hist(k,:) - r_ref.x_hist(idx_ref,:));
            if e > max_err, max_err = e; end
        end
        err(i) = max_err;
        fprintf('  dt = %.5f  =>  max error = %.6e\n', dt_list(i), err(i));
    end

    % Estimate convergence orders from successive pairs
    orders = zeros(length(dt_list)-1, 1);
    for i = 1:length(dt_list)-1
        orders(i) = log(err(i)/err(i+1)) / log(dt_list(i)/dt_list(i+1));
        fprintf('  Order (dt %.5f -> %.5f): %.3f\n', ...
                dt_list(i), dt_list(i+1), orders(i));
    end

    avg_order = mean(orders);
    fprintf('  Average convergence order: %.3f\n', avg_order);

    verifyGreaterThan(testCase, avg_order, 0.8, ...
        'Average convergence order should be at least 0.8 (Euler is O(dt)).');
end

%% -----------------------------------------------------------------------
%  test_exponential_decay
%  -----------------------------------------------------------------------
%  After the initial transient (t > 2), the log of the state norm should
%  have a negative slope, indicating exponential-like decay.
function test_exponential_decay(testCase)
    fprintf('\n--- test_exponential_decay ---\n');

    p.dt    = 0.001;
    p.t_end = 10.0;
    r = run_inexact_predictor(p);

    % Extract portion after transient
    idx_start = find(r.t >= 2.0, 1, 'first');
    t_tail = r.t(idx_start:end);
    norms  = sqrt(r.x_hist(idx_start:end, 1).^2 + r.x_hist(idx_start:end, 2).^2);

    % Remove any exactly-zero entries to avoid log issues
    valid = norms > 0;
    t_valid = t_tail(valid);
    log_norms = log(norms(valid));

    % Linear regression: log(||x||) ~ slope * t + intercept
    n = length(t_valid);
    t_mean = mean(t_valid);
    ln_mean = mean(log_norms);
    slope = sum((t_valid - t_mean) .* (log_norms - ln_mean)) / ...
            sum((t_valid - t_mean).^2);

    fprintf('  Log-norm slope for t > 2: %.4f\n', slope);

    verifyLessThan(testCase, slope, 0, ...
        'Log(||x||) slope should be negative after transient (exponential decay).');
end
