function v = Kflc(t, x, param)
% Kflc - Feedback Linearization virtual control law for SCARA robot
%
% Inputs:
%   t - Current simulation time (s)
%   x - Current state vector [q; dq] (8x1)
%
% Output:
%   v - Virtual control input (desired joint accelerations)

    % Reference trajectory: position, velocity, and acceleration
    [qd, dqd, ddqd] = generate_reference(t, param);

    % Extract current joint positions and velocities
    q  = x(1:4);
    dq = x(5:8);

    % Compute tracking errors
    e  = q  - qd;     % Position error
    de = dq - dqd;    % Velocity error

    % Feedback linearization controller gains
    Kp = diag([550 300 600 600]);   % Proportional gains
    Kd = diag([195 160  190  200]);  % Derivative gains

    % Virtual control law: desired joint accelerations
    v = ddqd - Kd * de - Kp * e;
end
