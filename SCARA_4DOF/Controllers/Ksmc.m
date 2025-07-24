function u = Ksmc(t, x, param)
% Ksmc - Sliding Mode Controller (SMC) for 4-DOF SCARA robot
%
% Inputs:
%   t - Current simulation time (s)
%   x - State vector [q; dq] of size (8x1)
%
% Output:
%   u - Virtual control input (desired joint accelerations)

    % Generate reference trajectories
    [qd, dqd, ddqd] = generate_reference(t, param);

    % Extract current joint positions and velocities
    q  = x(1:4);
    dq = x(5:8);

    % Compute tracking errors
    e  = q - qd;
    de = dq - dqd;

    % Define sliding surface
    Lambda = diag([20 10 19 10]);     % Convergence rate
    s = de + Lambda * e;           % Sliding variable

    % Compute control law
    K   = diag([40 20 30 20]);       % Robustness gain
    phi = 0.1;                     % Boundary layer thickness
    u_eq  = ddqd - Lambda * de;    % Equivalent control
    u_dis = -K * sat(s / phi);     % Discontinuous (robust) term
    u = u_eq + u_dis;              % Total virtual control

    % Apply safety bounds to control output
    u = max(min(u, 100), -100);

    % Sanity check for numerical issues
    if any(isnan(u)) || any(isinf(u))
        error("SMC output contains NaN or Inf at time %.3f", t);
    end
end

function y = sat(x)
% sat - Saturation function to reduce chattering
%   Limits elements of x to the range [-1, 1]
    y = min(max(x, -1), 1);
end
