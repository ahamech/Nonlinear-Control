function dx = scara_smc_dynamics(t, x, u, param, disturbance_func)
% scara_smc_dynamics - Computes state derivatives for a 4-DOF SCARA robot under SMC.
%
% Inputs:
%   t                - Current time (s)
%   x                - State vector [q; dq] (8x1)
%   u                - Virtual control input (desired joint accelerations)
%   param            - Structure of robot physical parameters
%   disturbance_func - (Optional) Function handle for external disturbances
%
% Output:
%   dx               - Time derivative of state vector [dq; ddq]

    % Joint positions and velocities
    q  = x(1:4);
    dq = x(5:8);

    % Compute dynamics: Mass, Coriolis, Gravity, and friction
    [M, C, G, Phi] = scara_dynamics_matrices(q, dq, param);

    % External disturbance
    if nargin < 5 || isempty(disturbance_func)
        d = zeros(4,1);
    else
        d = disturbance_func(t);
    end

    % Apply virtual control input and compute joint accelerations
    ddq = u - M \ d;

    % Return state derivatives
    dx = [dq; ddq];
end
