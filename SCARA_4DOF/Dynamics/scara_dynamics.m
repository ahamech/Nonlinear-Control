function dx = scara_dynamics(t, x, u, param)
% SCARA_DYNAMICS
% Computes the nonlinear dynamics of a 4-DOF SCARA robot in open-loop form.
%
% Inputs:
%   t     - Current simulation time (s)
%   x     - State vector [q1; q2; d3; q4; dq1; dq2; dd3; dq4]
%   u     - Control input vector (torques/forces)
%   param - Structure of physical parameters
%
% Output:
%   dx    - Derivative of state vector [dq; ddq]

    % --- Extract joint states ---
    q  = x(1:4);   % Joint positions
    dq = x(5:8);   % Joint velocities

    % --- Get dynamic model matrices ---
    [M, C, G, Phi] = scara_dynamics_matrices(q, dq, param);

    % --- Compute joint accelerations ---
    ddq = M \ (u - C - G - Phi);

    % --- Return state derivative ---
    dx = [dq; ddq];
end
