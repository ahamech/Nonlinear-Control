function [t, x, actual_ddq_all] = scara_dynamics_sim_compare(controller_type)
% scara_dynamics_sim_compare - Simulates SCARA robot dynamics under various controllers.
%
% Input:
%   controller_type - Control strategy: 'flc', 'smc', or 'openloop'
%
% Outputs:
%   t               - Time vector
%   x               - State trajectory [q; dq] over time
%   actual_ddq_all  - Actual joint accelerations at each time step

    % Load robot parameters, time span, initial state, and disturbance
    [param, tspan, x0, ~, disturbance_func] = scara_get_settings();

    % Select system dynamics based on controller type
    switch lower(controller_type)
        case 'flc'
            dynamics_func = @(t, x_curr) scara_flc_dynamics(t, x_curr, Kflc(t, x_curr, param), param, disturbance_func);
        case 'smc'
            dynamics_func = @(t, x_curr) scara_smc_dynamics(t, x_curr, Ksmc(t, x_curr, param), param, disturbance_func);
        case 'openloop'
            dynamics_func = @(t, x_curr) scara_dynamics(t, x_curr, generate_openloop_input(t, param), param);
        otherwise
            error('Invalid controller type: %s. Choose "flc", "smc", or "openloop".', controller_type);
    end

    % Simulate the system using ODE solver
    [t, x] = ode45(dynamics_func, tspan, x0);

    % Compute actual joint accelerations for each time step
    actual_ddq_all = zeros(length(t), 4);
    for i = 1:length(t)
        q_curr = x(i, 1:4)';
        dq_curr = x(i, 5:8)';
        [M_curr, C_curr, G_curr, Phi_curr] = scara_dynamics_matrices(q_curr, dq_curr, param);
        d_curr = disturbance_func(t(i));

        switch lower(controller_type)
            case 'flc'
                u_curr = Kflc(t(i), [q_curr; dq_curr], param);
                actual_ddq_all(i, :) = (u_curr - (M_curr \ d_curr))';
            case 'smc'
                u_curr = Ksmc(t(i), [q_curr; dq_curr], param);
                actual_ddq_all(i, :) = (u_curr - (M_curr \ d_curr))';
            case 'openloop'
                u_curr = generate_openloop_input(t(i));
                actual_ddq_all(i, :) = (M_curr \ (u_curr - C_curr - G_curr - Phi_curr - d_curr))';
        end
    end
end
