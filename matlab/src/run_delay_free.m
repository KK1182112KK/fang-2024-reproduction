function result = run_delay_free(params)
%RUN_DELAY_FREE  Delay-free system with direct state feedback.
%
%  result = run_delay_free()
%  result = run_delay_free(params)
%
%  System (no delays, D1=D2=0):
%    dx1/dt = x2
%    dx2/dt = -x1 + sin(x1) + x2 + u1 + tanh(u1) + sin(u2)
%
%  Control (delay-free feedback):
%    u1(t) = -(kp1*x1(t) + kd1*x2(t))
%    u2(t) = -(kp2*x1(t) + kd2*x2(t))
%
%  This is the ideal baseline case: the same gains used in the predictor
%  design, but applied without any transport delay. Should converge.
%
%  Returns struct with fields:
%    t       — (Nt x 1)  time vector
%    x_hist  — (Nt x 2)  state trajectory [x1, x2]
%    u1_hist — (Nt x 1)  control input channel 1
%    u2_hist — (Nt x 1)  control input channel 2

    %% Default parameters
    if nargin < 1, params = struct(); end

    kp1   = get_field(params, 'kp1', 3.0);
    kd1   = get_field(params, 'kd1', 5.0);
    kp2   = get_field(params, 'kp2', 10.0);
    kd2   = get_field(params, 'kd2', 10.0);
    x0    = get_field(params, 'x0', [1; 1]);
    dt    = get_field(params, 'dt', 0.001);
    t_end = get_field(params, 't_end', 10.0);

    %% Derived quantities
    Nt = round(t_end / dt) + 1;

    %% Pre-allocate
    t       = (0:Nt-1)' * dt;
    x_hist  = zeros(Nt, 2);
    u1_hist = zeros(Nt, 1);
    u2_hist = zeros(Nt, 1);

    % Initial conditions
    x_hist(1,:) = x0(:)';

    %% Main time-stepping loop
    for k = 1:Nt-1
        x1_k = x_hist(k, 1);
        x2_k = x_hist(k, 2);

        % --- Control law (delay-free, applied immediately) ---
        u1_k = -(kp1 * x1_k + kd1 * x2_k);
        u2_k = -(kp2 * x1_k + kd2 * x2_k);

        u1_hist(k) = u1_k;
        u2_hist(k) = u2_k;

        % --- System dynamics (no delay) ---
        dx1 = x2_k;
        dx2 = -x1_k + sin(x1_k) + x2_k ...
              + u1_k + tanh(u1_k) + sin(u2_k);

        % Euler step
        x_hist(k+1, 1) = x1_k + dt * dx1;
        x_hist(k+1, 2) = x2_k + dt * dx2;
    end

    % Final control (hold last value)
    u1_hist(Nt) = u1_hist(Nt-1);
    u2_hist(Nt) = u2_hist(Nt-1);

    %% Pack output
    result.t       = t;
    result.x_hist  = x_hist;
    result.u1_hist = u1_hist;
    result.u2_hist = u2_hist;

    fprintf('  Delay-free: |x(t_end)| = %.4e\n', norm(x_hist(end,:)));
end

%% Helper: get field with default
function val = get_field(s, name, default)
    if isfield(s, name)
        val = s.(name);
    else
        val = default;
    end
end
