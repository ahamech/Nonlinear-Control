function dx = scara_flc_dynamics(t, x, u, param, disturbance_func)
% SCARA_FLC_DYNAMICS
% Computes the dynamics of the SCARA robot under feedback linearization control.
%
% Inputs:
%   t                - Current time (s)
%   x                - State vector [q; dq]
%   u                - Desired joint accelerations from FLC controller
%   param            - Structure of physical parameters
%   disturbance_func - Function handle for external disturbance (optional)
%
% Output:
%   dx               - Time derivative of state vector

    % --- Extract joint positions and velocities ---
    q  = x(1:4);
    dq = x(5:8);

    % --- Get dynamic model matrices ---
    [M, C, G, Phi] = scara_dynamics_matrices(q, dq, param);

    % --- External disturbance ---
    if nargin < 5 || isempty(disturbance_func)
        d = zeros(4,1);
    else
        d = disturbance_func(t);
    end

    % --- Compute total torque and resulting acceleration ---
    tau = M*u + C + G + Phi;
    ddq = M \ (tau - C - G - Phi - d);

    % --- Return state derivative ---
    dx = [dq; ddq];
end
