function result = run_inexact_predictor(params)
%RUN_INEXACT_PREDICTOR  Inexact predictor feedback for 2-input nonlinear system.
%
%  result = run_inexact_predictor()
%  result = run_inexact_predictor(params)
%
%  System (Fang & Zhang 2024, Eq. 35-36):
%    dx1/dt = x2
%    dx2/dt = -x1 + sin(x1) + x2 + u1(t-D1) + tanh(u1(t-D1)) + sin(u2(t-D2))
%
%  Inexact predictor: uses a single nominal delay D0 in [D1, D2] instead
%  of exact per-channel delays. Predictor ODE integrated from t-D0 to t.
%
%  Feedback:
%    u1(t) = -(kp1*P1(t) + kd1*P2(t))
%    u2(t) = -(kp2*P1(t) + kd2*P2(t))
%  where P = [P1; P2] is the predicted state at time t.
%
%  Returns struct with fields:
%    t       — (Nt x 1)  time vector
%    x_hist  — (Nt x 2)  state trajectory [x1, x2]
%    u1_hist — (Nt x 1)  control input channel 1
%    u2_hist — (Nt x 1)  control input channel 2
%    P_hist  — (Nt x 2)  predicted state trajectory [P1, P2]

    %% Default parameters
    if nargin < 1, params = struct(); end

    D1    = get_field(params, 'D1', 0.4);
    D2    = get_field(params, 'D2', 0.5);
    D0    = get_field(params, 'D0', 0.45);
    kp1   = get_field(params, 'kp1', 3.0);
    kd1   = get_field(params, 'kd1', 5.0);
    kp2   = get_field(params, 'kp2', 10.0);
    kd2   = get_field(params, 'kd2', 10.0);
    x0    = get_field(params, 'x0', [1; 1]);
    dt    = get_field(params, 'dt', 0.001);
    t_end = get_field(params, 't_end', 10.0);

    %% Derived quantities
    Nt    = round(t_end / dt) + 1;
    N_D1  = round(D1 / dt);       % steps in delay D1
    N_D2  = round(D2 / dt);       % steps in delay D2
    N_pred = round(D0 / dt);      % predictor sub-steps

    %% Pre-allocate
    t      = (0:Nt-1)' * dt;
    x_hist = zeros(Nt, 2);
    P_hist = zeros(Nt, 2);

    % Control history arrays with pre-history for negative times.
    % u1_full: indices 1:N_D1 are pre-history (u=0 for t<0)
    %          index N_D1 + k stores u1 at time step k
    % u2_full: indices 1:N_D2 are pre-history (u=0 for t<0)
    %          index N_D2 + k stores u2 at time step k
    u1_full = zeros(Nt + N_D1, 1);
    u2_full = zeros(Nt + N_D2, 1);

    % Initial conditions
    x_hist(1,:) = x0(:)';

    %% Main time-stepping loop
    for k = 1:Nt-1
        x1_k = x_hist(k, 1);
        x2_k = x_hist(k, 2);
        t_k  = t(k);

        % === Step 1: Solve predictor ODE from theta = t_k - D0 to t_k ===
        % Initial condition: P(t_k - D0) = x(:,k)
        P1 = x1_k;
        P2 = x2_k;
        d_theta = D0 / N_pred;     % predictor sub-step size (= dt)

        for j = 0:N_pred-1
            theta_j = t_k - D0 + j * d_theta;

            % Look up u1(theta_j) and u2(theta_j) from stored history.
            % IMPORTANT: at theta_j = t_k, u(t_k) is not yet computed.
            % The predictor only uses past values up to t_k - dt.
            u1_theta = lookup_u(theta_j, dt, u1_full, N_D1);
            u2_theta = lookup_u(theta_j, dt, u2_full, N_D2);

            % Predictor dynamics (same as plant dynamics)
            dP1 = P2;
            dP2 = -P1 + sin(P1) + P2 + u1_theta + tanh(u1_theta) + sin(u2_theta);

            % Euler step
            P1 = P1 + d_theta * dP1;
            P2 = P2 + d_theta * dP2;
        end

        % Store predicted state
        P_hist(k,:) = [P1, P2];

        % === Step 2: Compute control from predicted state ===
        u1_k = -(kp1 * P1 + kd1 * P2);
        u2_k = -(kp2 * P1 + kd2 * P2);

        % === Step 3: Store controls in history ===
        u1_full(N_D1 + k) = u1_k;
        u2_full(N_D2 + k) = u2_k;

        % === Step 4: Advance system dynamics (Euler) ===
        % The plant uses ACTUAL delayed controls: u1(t_k - D1), u2(t_k - D2)
        u1_delayed = lookup_u(t_k - D1, dt, u1_full, N_D1);
        u2_delayed = lookup_u(t_k - D2, dt, u2_full, N_D2);

        dx1 = x2_k;
        dx2 = -x1_k + sin(x1_k) + x2_k ...
              + u1_delayed + tanh(u1_delayed) + sin(u2_delayed);

        x_hist(k+1, 1) = x1_k + dt * dx1;
        x_hist(k+1, 2) = x2_k + dt * dx2;
    end

    % Final step: hold last predictor and control values
    P_hist(Nt,:) = P_hist(Nt-1,:);
    u1_full(N_D1 + Nt) = u1_full(N_D1 + Nt - 1);
    u2_full(N_D2 + Nt) = u2_full(N_D2 + Nt - 1);

    %% Pack output
    result.t       = t;
    result.x_hist  = x_hist;
    result.u1_hist = u1_full(N_D1 + (1:Nt));
    result.u2_hist = u2_full(N_D2 + (1:Nt));
    result.P_hist  = P_hist;

    fprintf('  Inexact predictor: |x(t_end)| = %.4e\n', norm(x_hist(end,:)));
end

%% Helper: lookup u from full history array (including pre-history)
function u_val = lookup_u(t_query, dt, u_full, N_delay)
    % u_full is indexed so that u_full(N_delay + 1) = u(0),
    % u_full(1) = u(-D), etc.
    % Index for time t_query: idx = round(t_query / dt) + N_delay + 1
    idx = round(t_query / dt) + N_delay + 1;
    idx = max(1, min(idx, length(u_full)));
    u_val = u_full(idx);
end

%% Helper: get field with default
function val = get_field(s, name, default)
    if isfield(s, name)
        val = s.(name);
    else
        val = default;
    end
end
