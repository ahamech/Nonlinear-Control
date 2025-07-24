clc; clear; close all;

% Add all subfolders (Dynamics, Controllers, etc.) to MATLAB path
addpath(genpath(fileparts(mfilename('fullpath'))));

% Set default font for all plots globally
set(groot, 'defaultAxesFontName', 'Times New Roman');
set(groot, 'defaultTextFontName', 'Times New Roman');
set(groot, 'defaultLegendFontName', 'Times New Roman');
set(groot, 'defaultColorbarFontName', 'Times New Roman');

% --- Load SCARA robot configuration and parameters ---
[param, tspan, x0, controller_type, disturbance_func] = scara_get_settings();

% --- Select dynamics function based on controller type ---
switch controller_type
    case 'flc'
        dynamics_func = @(t, x) scara_flc_dynamics(t, x, Kflc(t, x, param), param, disturbance_func);
    case 'smc'
        dynamics_func = @(t, x) scara_smc_dynamics(t, x, Ksmc(t, x, param), param, disturbance_func);
    case 'openloop'
        dynamics_func = @(t, x) scara_dynamics(t, x, generate_openloop_input(t, param), param);
    otherwise
        error('Invalid controller type.');
end

% --- Solve system using ode45 ---
[t, x] = ode45(dynamics_func, tspan, x0);

% --- Plot joint trajectories over time ---
figure;
plot(t, x(:,1:4), 'LineWidth', 1.5);
xlabel('Time (s)', 'Interpreter','latex');
ylabel('Joint States', 'Interpreter','latex');
legend({'q_1 (rad)', 'q_2 (rad)', 'q_3 (m)', 'q_4 (rad)'});
grid on;
title(['SCARA - ', upper(controller_type), ' Simulation']);

% --- Compute reference trajectory at all time steps ---
qd_all   = zeros(length(t), 4);   % Desired joint positions
dqd_all  = zeros(length(t), 4);   % Desired joint velocities
ddqd_all = zeros(length(t), 4);   % Desired joint accelerations

% --- Also compute actual accelerations for plotting ---
actual_ddq_all = zeros(length(t), 4); % NEW: Initialize actual accelerations
for i = 1:length(t)
    [qd, dqd, ddqd] = generate_reference(t(i), param);
    qd_all(i, :)   = qd';
    dqd_all(i, :)  = dqd';
    ddqd_all(i, :) = ddqd';

    % Re-calculate ddq at each step using the inverse dynamics equation for accuracy
    q_curr = x(i, 1:4)';
    dq_curr = x(i, 5:8)';

    [M_curr, C_curr, G_curr, Phi_curr] = scara_dynamics_matrices(q_curr, dq_curr, param);

    % Determine the control input 'u' at this time step
    switch controller_type
        case 'flc'
            u_curr = Kflc(t(i), [q_curr; dq_curr], param);
            % For FLC/SMC, ddq_actual = u - M_inv * d (effectively)
            d_curr = disturbance_func(t(i));
            actual_ddq_all(i, :) = (u_curr - (M_curr \ d_curr))';
        case 'smc'
            u_curr = Ksmc(t(i), [q_curr; dq_curr], param);
            d_curr = disturbance_func(t(i));
            actual_ddq_all(i, :) = (u_curr - (M_curr \ d_curr))';
        case 'openloop'
            u_curr = generate_openloop_input(t(i), param);
            d_curr = disturbance_func(t(i)); % Assuming openloop also has disturbance for consistency
            actual_ddq_all(i, :) = (M_curr \ (u_curr - C_curr - G_curr - Phi_curr - d_curr))';
    end
end

% --- Plot actual vs desired trajectories for each joint ---
figure;
for i = 1:4
    subplot(2, 2, i)
    plot(t, x(:,i), 'b', 'LineWidth', 1.5); hold on;
    plot(t, qd_all(:,i), 'r--', 'LineWidth', 1.5);
    xlabel('Time (s)', 'Interpreter','latex');
    if i == 3
        ylabel('$\mathrm{q_3}$ (m)', 'Interpreter', 'latex');
    else
        ylabel(['$\mathrm{q_', num2str(i), '}$ (rad)'], 'Interpreter', 'latex');
    end
    legend({'Actual', 'Desired'});
    grid on;
end
sgtitle(['Position Tracking - ', upper(controller_type)]);

% --- Plot tracking error for each joint ---
figure;
for i = 1:4
    subplot(2, 2, i)
    plot(t, x(:,i) - qd_all(:,i), 'k', 'LineWidth', 1.5);
    xlabel('Time (s)', 'Interpreter','latex');
    if i == 3
        ylabel('Error $\mathrm{q_3}$ (m)', 'Interpreter', 'latex');
    else
        ylabel(['Error $\mathrm{q_', num2str(i), '}$ (rad)'], 'Interpreter', 'latex');
    end
    grid on;
end
sgtitle(['Position Tracking Errors - ', upper(controller_type)]);

% --- Plot joint velocity tracking (Actual vs Desired) ---
figure;
for i = 1:4
    subplot(2, 2, i)
    plot(t, x(:, i+4), 'b', 'LineWidth', 1.5); hold on;
    plot(t, dqd_all(:, i), 'r--', 'LineWidth', 1.5);
    xlabel('Time (s)', 'Interpreter','latex');
    if i == 3
        ylabel('$\mathrm{\dot{q}_3}$ (m/s)', 'Interpreter', 'latex');
    else
        ylabel(['$\mathrm{\dot{q}_', num2str(i), '}$ (rad/s)'], 'Interpreter', 'latex');
    end
    legend({'Actual', 'Desired'});
    grid on;
end
sgtitle(['Velocity Tracking - ', upper(controller_type)]);

% --- Plot joint velocity errors ---
figure;
for i = 1:4
    subplot(2, 2, i)
    dq_actual = x(:,4+i);
    dq_desired = dqd_all(:,i);
    plot(t, dq_actual - dq_desired, 'k', 'LineWidth', 1.5);
    xlabel('Time (s)', 'Interpreter','latex');
    if i == 3
        ylabel('Error $\mathrm{\dot{q}_3}$ (m/s)', 'Interpreter', 'latex');
    else
        ylabel(['Error $\mathrm{\dot{q}_', num2str(i), '}$ (rad/s)'], 'Interpreter', 'latex');
    end
    grid on;
end
sgtitle(['Velocity Errors - ', upper(controller_type)]);


% --- Plot joint acceleration tracking (Actual vs Desired) ---
figure;
for i = 1:4
    subplot(2, 2, i)
    plot(t, actual_ddq_all(:, i), 'b', 'LineWidth', 1.5); hold on; % Uses actual_ddq_all
    plot(t, ddqd_all(:, i), 'r--', 'LineWidth', 1.5); % Uses ddqd_all (same length as t)
    xlabel('Time (s)', 'Interpreter','latex');
    if i == 3
        ylabel('$\mathrm{\ddot{q}_3}$ (m/s$\mathrm{^2}$)', 'Interpreter', 'latex');
    else
        ylabel(['$\mathrm{\ddot{q}_', num2str(i), '}$ (rad/s$\mathrm{^2}$)'], 'Interpreter', 'latex');
    end
    legend({'Actual', 'Desired'});
    grid on;
end
sgtitle(['Acceleration Tracking - ', upper(controller_type)]);

% Plot join acceleration errors
figure;
for i = 1:4
    subplot(2, 2, i)
    err_ddq = actual_ddq_all(:, i) - ddqd_all(:, i); % Uses actual_ddq_all and ddqd_all
    plot(t, err_ddq, 'k', 'LineWidth', 1.5);
    xlabel('Time (s)', 'Interpreter','latex');
    if i == 3
        ylabel('Error $\mathrm{\ddot{q}_3}$ (m/s$\mathrm{^2}$)', 'Interpreter', 'latex');
    else
        ylabel(['Error $\mathrm{\ddot{q}_', num2str(i), '}$ (rad/s$\mathrm{^2}$)'], 'Interpreter', 'latex');
    end
    grid on;
end
sgtitle(['Acceleration Errors - ', upper(controller_type)]);
