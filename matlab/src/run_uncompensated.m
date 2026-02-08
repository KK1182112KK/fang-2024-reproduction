function result = run_uncompensated(params)
%RUN_UNCOMPENSATED  Delay-ignorant feedback for 2-input nonlinear system.
%
%  result = run_uncompensated()
%  result = run_uncompensated(params)
%
%  System (Fang & Zhang 2024, Eq. 35-36):
%    dx1/dt = x2
%    dx2/dt = -x1 + sin(x1) + x2 + u1(t-D1) + tanh(u1(t-D1)) + sin(u2(t-D2))
%
%  Control: delay-ignorant feedback (no predictor):
%    u1(t) = -(kp1*x1(t) + kd1*x2(t))
%    u2(t) = -(kp2*x1(t) + kd2*x2(t))
%
%  The system still has delays D1, D2 on the inputs, but the controller
%  ignores them. This is expected to destabilise the system.
%
%  Includes overflow protection: simulation stops if norm(x) > 1e6.
%
%  Returns struct with fields:
%    t       — (Nt x 1)  time vector (may be truncated on overflow)
%    x_hist  — (Nt x 2)  state trajectory [x1, x2]
%    u1_hist — (Nt x 1)  control input channel 1
%    u2_hist — (Nt x 1)  control input channel 2

    %% Default parameters
    if nargin < 1, params = struct(); end

    D1    = get_field(params, 'D1', 0.4);
    D2    = get_field(params, 'D2', 0.5);
    kp1   = get_field(params, 'kp1', 3.0);
    kd1   = get_field(params, 'kd1', 5.0);
    kp2   = get_field(params, 'kp2', 10.0);
    kd2   = get_field(params, 'kd2', 10.0);
    x0    = get_field(params, 'x0', [1; 1]);
    dt    = get_field(params, 'dt', 0.001);
    t_end = get_field(params, 't_end', 10.0);

    %% Derived quantities
    Nt   = round(t_end / dt) + 1;
    N_D1 = round(D1 / dt);
    N_D2 = round(D2 / dt);

    %% Pre-allocate
    t      = (0:Nt-1)' * dt;
    x_hist = zeros(Nt, 2);

    % Control history with pre-history (zeros for t < 0)
    u1_full = zeros(Nt + N_D1, 1);
    u2_full = zeros(Nt + N_D2, 1);

    % Initial conditions
    x_hist(1,:) = x0(:)';

    % Track actual simulation length (may stop early on overflow)
    k_final = Nt;

    %% Main time-stepping loop
    for k = 1:Nt-1
        x1_k = x_hist(k, 1);
        x2_k = x_hist(k, 2);
        t_k  = t(k);

        % --- Overflow protection ---
        if norm([x1_k, x2_k]) > 1e6
            fprintf('  Uncompensated: overflow at t = %.3f, stopping.\n', t_k);
            k_final = k;
            break;
        end

        % --- Control law (no predictor, uses current state directly) ---
        u1_k = -(kp1 * x1_k + kd1 * x2_k);
        u2_k = -(kp2 * x1_k + kd2 * x2_k);

        % Store controls
        u1_full(N_D1 + k) = u1_k;
        u2_full(N_D2 + k) = u2_k;

        % --- System dynamics with actual delays ---
        u1_delayed = lookup_u(t_k - D1, dt, u1_full, N_D1);
        u2_delayed = lookup_u(t_k - D2, dt, u2_full, N_D2);

        dx1 = x2_k;
        dx2 = -x1_k + sin(x1_k) + x2_k ...
              + u1_delayed + tanh(u1_delayed) + sin(u2_delayed);

        % Euler step
        x_hist(k+1, 1) = x1_k + dt * dx1;
        x_hist(k+1, 2) = x2_k + dt * dx2;
    end

    % Truncate on overflow
    if k_final < Nt
        t      = t(1:k_final);
        x_hist = x_hist(1:k_final, :);
        u1_full_trunc = u1_full(N_D1 + (1:k_final));
        u2_full_trunc = u2_full(N_D2 + (1:k_final));
    else
        % Hold last control value
        u1_full(N_D1 + Nt) = u1_full(N_D1 + Nt - 1);
        u2_full(N_D2 + Nt) = u2_full(N_D2 + Nt - 1);
        u1_full_trunc = u1_full(N_D1 + (1:Nt));
        u2_full_trunc = u2_full(N_D2 + (1:Nt));
    end

    %% Pack output
    result.t       = t;
    result.x_hist  = x_hist;
    result.u1_hist = u1_full_trunc;
    result.u2_hist = u2_full_trunc;

    fprintf('  Uncompensated: |x(t_end)| = %.4e  (t_end = %.3f)\n', ...
            norm(x_hist(end,:)), t(end));
end

%% Helper: lookup u from full history array
function u_val = lookup_u(t_query, dt, u_full, N_delay)
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
