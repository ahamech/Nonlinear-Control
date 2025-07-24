%% FLC vs SMC: Simulation and Performance Analysis
% Simulates SCARA robot dynamics under FLC and SMC control,
% and compares tracking accuracy and control energy.

clc; clear; close all;

% Add all subfolders (Dynamics, Controllers, etc.) to MATLAB path
addpath(genpath(fileparts(mfilename('fullpath'))));

% Set default font for all plots globally
set(groot, 'defaultAxesFontName', 'Times New Roman'); % For axis labels, tick labels
set(groot, 'defaultTextFontName', 'Times New Roman');  % For titles, sgtitles, legends, text objects

controllers = {'flc', 'smc'};
results = struct();

% Run simulations for each controller
for i = 1:length(controllers)
    ctrl = controllers{i};
    disp(['🔁 Simulating ', upper(ctrl), ' controller...']);
    [t, x, actual_ddq] = scara_dynamics_sim_compare(ctrl);
    results.(ctrl).t = t;
    results.(ctrl).x = x;
    results.(ctrl).actual_ddq = actual_ddq;
end

% Use FLC time vector as reference
t = results.flc.t;
dt = mean(diff(t));

[param, ~, ~, ~, disturbance_func] = scara_get_settings();

% Generate reference trajectories
qd_all   = zeros(length(t), 4);
dqd_all  = zeros(length(t), 4);
ddqd_all = zeros(length(t), 4);
for i = 1:length(t)
    [qd, dqd, ddqd] = generate_reference(t(i), param);
    qd_all(i,:)   = qd';
    dqd_all(i,:)  = dqd';
    ddqd_all(i,:) = ddqd';
end



% Performance Metrics Evaluation
metrics = struct();
for k = 1:length(controllers)
    ctrl = controllers{k};
    x_data = results.(ctrl).x;
    actual_ddq_data = results.(ctrl).actual_ddq;

    % Interpolate SMC results if needed
    if strcmp(ctrl, 'smc')
        x_data = interp1(results.smc.t, x_data, t, 'linear', 'extrap');
        actual_ddq_data = interp1(results.smc.t, actual_ddq_data, t, 'linear', 'extrap');
    end

    q_all = x_data(:,1:4);
    dq_all = x_data(:,5:8);

    % Tracking error
    error_all = q_all - qd_all;
    RMSE = sqrt(mean(error_all.^2));
    MaxError = max(abs(error_all));

    % Control torque and energy
    tau_all = zeros(length(t), 4);
    for j = 1:length(t)
        x_now = [q_all(j,:)'; dq_all(j,:)'];
        q = x_now(1:4); dq = x_now(5:8);
        u = strcmp(ctrl, 'flc') * Kflc(t(j), x_now, param) + strcmp(ctrl, 'smc') * Ksmc(t(j), x_now, param);
        [M, C, G, Phi] = scara_dynamics_matrices(q, dq, param);
        tau_all(j,:) = (M * u + C + G + Phi)';
    end
    control_energy = sum(vecnorm(tau_all, 2, 2).^2) * dt;

    metrics.(ctrl).RMSE = RMSE;
    metrics.(ctrl).MaxError = MaxError;
    metrics.(ctrl).ControlEnergy = control_energy;
    metrics.(ctrl).actual_ddq = actual_ddq_data;

    % Print results
    fprintf('\n📊 Metrics for %s controller:\n', upper(ctrl));
    disp('RMSE:');
    disp('    q1(rad)   q2(rad)   q3(m)     q4(rad)');
    disp(RMSE);
    disp('Max Error:');
    disp('    q1(rad)   q2(rad)   q3(m)     q4(rad)');
    disp(MaxError);
    fprintf('⚡ Control Energy: %.4f J\n', control_energy);
end

% --- Plots ---

% RMSE & Max Error Comparison
figure;
subplot(2,1,1);
bar([metrics.flc.RMSE; metrics.smc.RMSE]');
title('RMSE of Joints (q₁, q₂, q₄ in (rad), q₃ in (m))');
ylabel('RMSE', 'Interpreter','latex');
xticks(1:4);
xticklabels({'$\mathrm{q_1}$', '$\mathrm{q_2}$', '$\mathrm{q_3}$', '$\mathrm{q_4}$'});
set(gca, 'TickLabelInterpreter', 'latex');
legend({'FLC', 'SMC'});
grid on;

subplot(2,1,2);
bar([metrics.flc.MaxError; metrics.smc.MaxError]');
title('Max Error of Joints (q₁, q₂, q₄ in (rad), q₃ in (m))');
ylabel('Max Error', 'Interpreter','latex');
xticks(1:4);
xticklabels({'$\mathrm{q_1}$', '$\mathrm{q_2}$', '$\mathrm{q_3}$', '$\mathrm{q_4}$'});
set(gca, 'TickLabelInterpreter', 'latex');
legend({'FLC', 'SMC'});
grid on;

% Control Energy Comparison
figure;
bar([metrics.flc.ControlEnergy, metrics.smc.ControlEnergy]);
xticks([1 2]);
xticklabels({'FLC', 'SMC'});
set(gca, 'TickLabelInterpreter', 'latex');
ylabel('Control Energy (J)', 'Interpreter','latex'); 
title('Total Control Energy');
grid on;

% Joint Position Tracking
figure;
for i = 1:4
    subplot(2,2,i)
    plot(t, results.flc.x(:,i), 'b', 'LineWidth', 1.4); hold on;
    plot(t, interp1(results.smc.t, results.smc.x(:,i), t), 'r--', 'LineWidth', 1.4);
    plot(t, qd_all(:,i), 'k:', 'LineWidth', 1.2);
    xlabel('Time (s)', 'Interpreter','latex');
    if i == 3
        ylabel('$\mathrm{q_3}$ (m)', 'Interpreter', 'latex');
    else
        ylabel(['$\mathrm{q_', num2str(i), '}$ (rad)'], 'Interpreter', 'latex');
    end
    legend({'FLC', 'SMC', 'Reference'});
    grid on;
end
sgtitle('Joint Position Tracking');

% Position Tracking Errors
figure;
for i = 1:4
    subplot(2, 2, i)
    err_flc = results.flc.x(:, i) - qd_all(:, i);
    err_smc = interp1(results.smc.t, results.smc.x(:, i), t) - qd_all(:, i);
    plot(t, err_flc, 'b', 'LineWidth', 1.4); hold on;
    plot(t, err_smc, 'r--', 'LineWidth', 1.4);
    xlabel('Time (s)', 'Interpreter','latex');
    if i == 3
        ylabel('Error $\mathrm{q_3}$ (m)', 'Interpreter', 'latex');
    else
        ylabel(['Error $\mathrm{q_', num2str(i), '}$ (rad)'], 'Interpreter', 'latex');
    end
    legend({'FLC', 'SMC'});
    grid on;
end
sgtitle('Position Tracking Errors');

% Velocity Tracking
figure;
for i = 1:4
    subplot(2,2,i)
    dq_flc = results.flc.x(:, 4+i);
    dq_smc = interp1(results.smc.t, results.smc.x(:, 4+i), t);
    dq_ref = dqd_all(:, i);
    plot(t, dq_flc, 'b', 'LineWidth', 1.4); hold on;
    plot(t, dq_smc, 'r--', 'LineWidth', 1.4);
    plot(t, dq_ref, 'k:', 'LineWidth', 1.2);
    xlabel('Time (s)', 'Interpreter','latex');
    if i == 3
        ylabel('$\mathrm{\dot{q}_3}$ (m/s)', 'Interpreter', 'latex');
    else
        ylabel(['$\mathrm{\dot{q}_', num2str(i), '}$ (rad/s)'], 'Interpreter', 'latex');
    end
    legend({'FLC', 'SMC', 'Reference'}); 
    grid on;
end
sgtitle('Velocity Tracking');

% Velocity Tracking Errors
figure;
for i = 1:4
    subplot(2, 2, i)
    dq_flc = results.flc.x(:, 4+i);
    dq_smc = interp1(results.smc.t, results.smc.x(:, 4+i), t);
    dq_ref = dqd_all(:, i);
    err_flc = dq_flc - dq_ref;
    err_smc = dq_smc - dq_ref;
    plot(t, err_flc, 'b', 'LineWidth', 1.4); hold on;
    plot(t, err_smc, 'r--', 'LineWidth', 1.4);
    xlabel('Time (s)', 'Interpreter','latex');
    if i == 3
        ylabel('Error $\mathrm{\dot{q}_3}$ (m/s)', 'Interpreter', 'latex');
    else
        ylabel(['Error $\mathrm{\dot{q}_', num2str(i), '}$ (rad/s)'], 'Interpreter', 'latex');
    end
    legend({'FLC', 'SMC'});
    grid on;
end
sgtitle('Velocity Tracking Errors');

% Acceleration Tracking
figure;
for i = 1:4
    subplot(2,2,i)
    ddq_flc = metrics.flc.actual_ddq(:,i);
    ddq_smc = metrics.smc.actual_ddq(:,i);
    ddq_ref = ddqd_all(:, i);
    plot(t, ddq_flc, 'b', 'LineWidth', 1.4); hold on;
    plot(t, ddq_smc, 'r--', 'LineWidth', 1.4);
    plot(t, ddq_ref, 'k:', 'LineWidth', 1.2);
    xlabel('Time (s)', 'Interpreter','latex');
    if i == 3
        ylabel('$\mathrm{\ddot{q}_3}$ (m/s$\mathrm{^2}$)', 'Interpreter', 'latex');
    else
        ylabel(['$\mathrm{\ddot{q}_', num2str(i), '}$ (rad/s$\mathrm{^2}$)'], 'Interpreter', 'latex');
    end
    legend({'FLC', 'SMC', 'Reference'});
    grid on;
end
sgtitle('Acceleration Tracking');

% Acceleration Tracking Errors
figure;
for i = 1:4
    subplot(2, 2, i)
    ddq_flc = metrics.flc.actual_ddq(:,i);
    ddq_smc = metrics.smc.actual_ddq(:,i);
    ddq_ref = ddqd_all(:, i);
    err_flc = ddq_flc - ddq_ref;
    err_smc = ddq_smc - ddq_ref;
    plot(t, err_flc, 'b', 'LineWidth', 1.4); hold on;
    plot(t, err_smc, 'r--', 'LineWidth', 1.4);
    xlabel('Time (s)', 'Interpreter','latex');
    if i == 3
        ylabel('Error $\mathrm{\ddot{q}_3}$ (m/s$\mathrm{^2}$)', 'Interpreter', 'latex');
    else
        ylabel(['Error $\mathrm{\ddot{q}_', num2str(i), '}$ (rad/s$\mathrm{^2}$)'], 'Interpreter', 'latex');
    end
    legend({'FLC', 'SMC'});
    grid on;
end
sgtitle('Acceleration Tracking Errors');




figure;
% Define the desired subplot order for positions and velocities
position_subplots = [1, 2, 5, 6]; % Corresponds to q1, q2, q3, q4
velocity_subplots = [3, 4, 7, 8]; % Corresponds to dq1, dq2, dq3, dq4

% Plot Joint Position Tracking
for i = 1:4
    subplot(2, 4, position_subplots(i));
    plot(t, results.flc.x(:,i), 'b', 'LineWidth', 1.4); hold on;
    plot(t, interp1(results.smc.t, results.smc.x(:,i), t), 'r--', 'LineWidth', 1.4);
    plot(t, qd_all(:,i), 'k:', 'LineWidth', 1.2);
    xlabel('Time (s)', 'Interpreter','latex');
    if i == 3
        ylabel('$\mathrm{q_3}$ (m)', 'Interpreter', 'latex');
    else
        ylabel(['$\mathrm{q_', num2str(i), '}$ (rad)'], 'Interpreter', 'latex');
    end
    % Only add legend to subplot 5
    if position_subplots(i) == 5
        legend({'FLC', 'SMC', 'Reference'});
    end
    grid on;
end

% Plot Velocity Tracking
for i = 1:4
    subplot(2, 4, velocity_subplots(i));
    dq_flc = results.flc.x(:, 4+i);
    dq_smc = interp1(results.smc.t, results.smc.x(:, 4+i), t);
    dq_ref = dqd_all(:, i);
    plot(t, dq_flc, 'b', 'LineWidth', 1.4); hold on;
    plot(t, dq_smc, 'r--', 'LineWidth', 1.4);
    plot(t, dq_ref, 'k:', 'LineWidth', 1.2);
    xlabel('Time (s)', 'Interpreter','latex');
    if i == 3
        ylabel('$\mathrm{\dot{q}_3}$ (m/s)', 'Interpreter', 'latex');
    else
        ylabel(['$\mathrm{\dot{q}_', num2str(i), '}$ (rad/s)'], 'Interpreter', 'latex');
    end
    grid on;
end
sgtitle('Joint Position and Velocity Tracking');
