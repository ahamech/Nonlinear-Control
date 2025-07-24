function [param, tspan, x0, controller_type, disturbance_func] = scara_get_settings()
% SCARA_GET_SETTINGS
% Central configuration function for the SCARA robot simulation and control setup.
%
% Outputs:
%   param             - Structure containing physical parameters of the robot
%   tspan             - Simulation time interval [start_time, end_time]
%   x0                - Initial state vector [q; dq]
%   controller_type   - Controller selection: 'flc', 'smc', or 'openloop'
%   disturbance_func  - Function handle for external disturbance (optional)

    % --- Physical parameters ---
    param = struct( ...
        'm1', 3.5, ...      % Mass of link 1 (kg)
        'm2', 2.8, ...      % Mass of link 2 (kg)
        'm3', 1.5, ...      % Mass of prismatic link 3 (kg)
        'm4', 1.2, ...      % Mass of rotary link 4 (kg)
        'l1', 0.5, ...      % Length of link 1 (m)
        'l2', 0.35, ...     % Length of link 2 (m)
        'r4', 0.05, ...     % Radius of rotary disk (link 4) (m)
        'g', 9.81, ...      % Gravitational acceleration (m/s^2)
        'b1', 0.2, ...      % Viscous friction coefficient for joint 1
        'b2', 0.15, ...     % Viscous friction coefficient for joint 2
        'b3', 0.1, ...      % Viscous friction coefficient for joint 3
        'b4', 0.05, ...     % Viscous friction coefficient for joint 4
        'd1', 0.25, ...     % Center of mass distance for link 1 (m)
        'd2', 0.175 ...     % Center of mass distance for link 2 (m)
    );

    % --- Simulation time span ---
    tspan = [0, 40];  % Simulate from 0 to 40 seconds

    % --- Initial condition ---
    %x0 = zeros(8, 1);  % [q1; q2; d3; q4; dq1; dq2; dd3; dq4] = 0
    x0 = [0.2; 0.25; 0.25; 0.25; 0.2; 0.25; 0.2; 0];

    % --- Controller selection ---
    controller_type = 'openloop';  % Options: 'flc', 'smc', or 'openloop'

    % --- Reference type selection ---
    param.reference_type = 'sine';   % Options: 'sine' or 'step'

    % --- External disturbance function ---
    disturbance_func = @(t) [ ...
        0.3 * sin(3 * t); ...
        0.2 * cos(4 * t); ...
        0.1 * sin(5 * t); ...
        0.02 * cos(2 * t) ...
    ];
    % disturbance_func = @(t) [ ...
    %     0; ...
    %     0; ...
    %     0; ...
    %     0 ...
    % ];

    % --- Auto-run simulation only if called from base workspace ---
    if isscalar(dbstack)
        disp('🔁 scara_get_settings called from base workspace. Running simulation...');
        scara_dynamics_sim;
    end
end
