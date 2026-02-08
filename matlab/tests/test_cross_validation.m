function tests = test_cross_validation
%TEST_CROSS_VALIDATION  Cross-method validation of the simulation.
%
%  Compares the existing Euler-based simulation against an independent
%  RK4 implementation to verify numerical correctness.
%
%  Run with:  results = runtests('test_cross_validation');

    tests = functiontests(localfunctions);
end

%% -----------------------------------------------------------------------
%  test_euler_vs_rk4
%  -----------------------------------------------------------------------
%  Run the existing Euler code at dt=0.001 and an independent RK4
%  implementation at dt=0.005.  Despite RK4 using a 5x larger step,
%  its higher order should make it accurate enough to serve as a
%  reference.  Max state error should be below 0.05.
function test_euler_vs_rk4(testCase)
    fprintf('\n--- test_euler_vs_rk4 ---\n');

    % Euler simulation (production code)
    p_euler.dt    = 0.001;
    p_euler.t_end = 10.0;
    r_euler = run_inexact_predictor(p_euler);

    % Independent RK4 simulation
    r_rk4 = run_rk4_inexact_predictor(0.005, 10.0);

    % Compare at RK4 time points
    dt_euler = p_euler.dt;
    dt_rk4   = 0.005;
    Nt_rk4   = length(r_rk4.t);

    max_err = 0;
    for k = 1:Nt_rk4
        % Find corresponding Euler index
        idx_euler = round(r_rk4.t(k) / dt_euler) + 1;
        if idx_euler > length(r_euler.t), break; end

        e = norm(r_rk4.x(k,:) - r_euler.x_hist(idx_euler,:));
        if e > max_err, max_err = e; end
    end

    fprintf('  Max state error (Euler dt=0.001 vs RK4 dt=0.005): %.6e\n', max_err);
    fprintf('  Euler final |x|: %.6e\n', norm(r_euler.x_hist(end,:)));
    fprintf('  RK4   final |x|: %.6e\n', norm(r_rk4.x(end,:)));

    verifyLessThan(testCase, max_err, 0.05, ...
        'Euler (dt=0.001) and RK4 (dt=0.005) should agree within 0.05.');
end

%% =======================================================================
%  INDEPENDENT RK4 IMPLEMENTATION
%  =======================================================================
%  This is a self-contained RK4 version of the inexact predictor simulation.
%  It mirrors the logic of run_inexact_predictor but uses classical
%  Runge-Kutta for the plant dynamics (the predictor sub-integration
%  also uses RK4 with its own sub-steps).
function result = run_rk4_inexact_predictor(dt, t_end)
    % Parameters (hardcoded to match defaults)
    D1  = 0.4;
    D2  = 0.5;
    D0  = 0.45;
    kp1 = 3.0;  kd1 = 5.0;
    kp2 = 10.0; kd2 = 10.0;
    x0  = [1; 1];

    Nt    = round(t_end / dt) + 1;
    N_D1  = round(D1 / dt);
    N_D2  = round(D2 / dt);
    N_pred = round(D0 / dt);

    t_vec  = (0:Nt-1)' * dt;
    x_hist = zeros(Nt, 2);

    u1_full = zeros(Nt + N_D1, 1);
    u2_full = zeros(Nt + N_D2, 1);

    x_hist(1,:) = x0(:)';

    for k = 1:Nt-1
        x_k = x_hist(k,:)';
        t_k = t_vec(k);

        % === Predictor (RK4 sub-integration) ===
        P = x_k;
        d_theta = D0 / N_pred;

        for j = 0:N_pred-1
            theta_j = t_k - D0 + j * d_theta;

            % RK4 for predictor
            k1 = pred_rhs(P, theta_j, dt, u1_full, u2_full, N_D1, N_D2);
            k2 = pred_rhs(P + 0.5*d_theta*k1, theta_j + 0.5*d_theta, dt, u1_full, u2_full, N_D1, N_D2);
            k3 = pred_rhs(P + 0.5*d_theta*k2, theta_j + 0.5*d_theta, dt, u1_full, u2_full, N_D1, N_D2);
            k4 = pred_rhs(P + d_theta*k3, theta_j + d_theta, dt, u1_full, u2_full, N_D1, N_D2);

            P = P + (d_theta/6) * (k1 + 2*k2 + 2*k3 + k4);
        end

        % === Control from predicted state ===
        u1_k = -(kp1 * P(1) + kd1 * P(2));
        u2_k = -(kp2 * P(1) + kd2 * P(2));

        u1_full(N_D1 + k) = u1_k;
        u2_full(N_D2 + k) = u2_k;

        % === Plant dynamics (RK4 step) ===
        f_plant = @(x_loc) plant_rhs(x_loc, t_k, dt, D1, D2, u1_full, u2_full, N_D1, N_D2);

        pk1 = f_plant(x_k);
        pk2 = f_plant(x_k + 0.5*dt*pk1);
        pk3 = f_plant(x_k + 0.5*dt*pk2);
        pk4 = f_plant(x_k + dt*pk3);

        x_hist(k+1,:) = (x_k + (dt/6)*(pk1 + 2*pk2 + 2*pk3 + pk4))';
    end

    % Hold last control
    u1_full(N_D1 + Nt) = u1_full(N_D1 + Nt - 1);
    u2_full(N_D2 + Nt) = u2_full(N_D2 + Nt - 1);

    result.t  = t_vec;
    result.x  = x_hist;
    result.u1 = u1_full(N_D1 + (1:Nt));
    result.u2 = u2_full(N_D2 + (1:Nt));

    fprintf('  RK4 inexact predictor: |x(t_end)| = %.4e\n', norm(x_hist(end,:)));
end

%% Predictor right-hand side
function dP = pred_rhs(P, theta, dt, u1_full, u2_full, N_D1, N_D2)
    u1_theta = lookup_u_local(theta, dt, u1_full, N_D1);
    u2_theta = lookup_u_local(theta, dt, u2_full, N_D2);

    dP = [P(2); ...
          -P(1) + sin(P(1)) + P(2) + u1_theta + tanh(u1_theta) + sin(u2_theta)];
end

%% Plant right-hand side (uses delayed controls at fixed time t_k)
function dx = plant_rhs(x_loc, t_k, dt, D1, D2, u1_full, u2_full, N_D1, N_D2)
    u1_del = lookup_u_local(t_k - D1, dt, u1_full, N_D1);
    u2_del = lookup_u_local(t_k - D2, dt, u2_full, N_D2);

    dx = [x_loc(2); ...
          -x_loc(1) + sin(x_loc(1)) + x_loc(2) ...
          + u1_del + tanh(u1_del) + sin(u2_del)];
end

%% Lookup helper (matches source code logic)
function u_val = lookup_u_local(t_query, dt, u_full, N_delay)
    idx = round(t_query / dt) + N_delay + 1;
    idx = max(1, min(idx, length(u_full)));
    u_val = u_full(idx);
end
