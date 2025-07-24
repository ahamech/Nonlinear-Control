%% Full Sensitivity Analysis
clc; clear; close all;

% Add all subfolders (Dynamics, Controllers, etc.) to MATLAB path
addpath(genpath(fileparts(mfilename('fullpath'))));

rng(0);  % Fix random seed for reproducibility

% Set default font for all plots globally
set(groot, 'defaultAxesFontName', 'Times New Roman'); 
set(groot, 'defaultTextFontName', 'Times New Roman');  
set(groot, 'defaultLegendFontName', 'Times New Roman'); 
set(groot, 'defaultColorbarFontName', 'Times New Roman');  

%% Step 1: Run Sensitivity Analysis
[param_base, tspan, x0_base, ~, dfunc_base] = scara_get_settings();
controller_types = {'flc', 'smc'};
num_scenarios_param = 3;
num_scenarios_init  = 3;
num_scenarios_dist  = 3;
results = struct();
scenario_id = 1;

for c = 1:length(controller_types)
    ctrl_type = controller_types{c};
    fprintf('\n🚀 Running simulations for controller: %s\n', upper(ctrl_type));

    % Parameter Variation Scenarios
    for i = 1:num_scenarios_param
        param = param_base;
        fields = {'m1','m2','m3','m4','b1','b2','b3','b4','l1','l2','d1','d2','r4'};
        for f = 1:length(fields)
            name = fields{f};
            if strcmp(name, 'r4') || strcmp(name, 'b4')
                delta = 0.05 * param_base.(name) * (2*rand - 1);
            else
                delta = 0.15 * param_base.(name) * (2*rand - 1);
            end
            param.(name) = param_base.(name) + delta;
            if startsWith(name, 'b') || startsWith(name, 'd')
                param.(name) = max(param.(name), 0);
            elseif startsWith(name, 'm') || startsWith(name, 'l') || strcmp(name, 'r4')
                param.(name) = max(param.(name), 0.001);
            end
        end
        param.g = param_base.g + 0.005 * param_base.g * (2*rand - 1);
        x0 = x0_base;
        dfunc = dfunc_base;
        tag = sprintf('ctrl_%s_paramVar_%d', ctrl_type, i);
        results = simulate_and_store(tag, ctrl_type, param, x0, dfunc, tspan, results);
    end

    % Initial Condition Variation Scenarios
    for i = 1:num_scenarios_init
        param = param_base;
        x0 = x0_base + (rand(8,1)-0.5) .* [pi; pi; 0.1; pi; 1; 1; 0.2; 1]; 
        dfunc = dfunc_base;
        tag = sprintf('ctrl_%s_initVar_%d', ctrl_type, i);
        results = simulate_and_store(tag, ctrl_type, param, x0, dfunc, tspan, results);
    end

    % Disturbance Variation Scenarios
    for i = 1:num_scenarios_dist
        param = param_base;
        x0 = x0_base;
        base_vals = [0.3, 0.2, 0.5, 0.1];
        disturb_scale = 1 + 0.2*(2*rand(4,1) - 1);
        dfunc = @(t) base_vals(:) .* disturb_scale .* [sin(3*t); cos(4*t); sin(5*t); cos(2*t)];
        tag = sprintf('ctrl_%s_distVar_%d', ctrl_type, i);
        results = simulate_and_store(tag, ctrl_type, param, x0, dfunc, tspan, results);
    end
end
disp('✅ Sensitivity analysis completed.');
save results_flc_smc_param_saved.mat results
disp('💾 Results (with param) saved to results_flc_smc_param_saved.mat');

%% Step 2: Plot Summary Metrics
scenario_names = fieldnames(results);
num_scenarios = length(scenario_names);
RMSE_all = zeros(num_scenarios, 4);
MaxError_all = zeros(num_scenarios, 4);
Energy_all = zeros(num_scenarios, 1);
labels = cell(num_scenarios,1);

for i = 1:num_scenarios
    s = scenario_names{i};
    RMSE_all(i,:) = results.(s).RMSE;
    MaxError_all(i,:) = results.(s).MaxErr;
    Energy_all(i) = results.(s).Energy;
    labels{i} = format_scenario_name(s);
end

figure;
bar(RMSE_all, 'grouped');
title('RMSE of Joints (q₁, q₂, q₄ in (rad), q₃ in (m))');
ylabel('RMSE', 'Interpreter', 'latex');
xlabel('Scenario');
legend({'$\mathrm{q_1}$', '$\mathrm{q_2}$', '$\mathrm{q_3}$', '$\mathrm{q_4}$'}, 'Interpreter', 'latex');
xticks(1:num_scenarios);
xticklabels(labels); 
set(gca, 'TickLabelInterpreter', 'none');
xtickangle(45); grid on;

figure;
bar(MaxError_all, 'grouped');
title('Max Error of Joints (q₁, q₂, q₄ in (rad), q₃ in (m))');
ylabel('Max Error', 'Interpreter', 'latex');
xlabel('Scenario');
legend({'$\mathrm{q_1}$', '$\mathrm{q_2}$', '$\mathrm{q_3}$', '$\mathrm{q_4}$'}, 'Interpreter', 'latex');
xticks(1:num_scenarios);
xticklabels(labels); 
set(gca, 'TickLabelInterpreter', 'none');
xtickangle(45); grid on;

figure;
bar(Energy_all);
title('Control Energy Across Scenarios');
ylabel('Energy (J)', 'Interpreter', 'latex');
xlabel('Scenario');
xticks(1:num_scenarios);
xticklabels(labels); 
set(gca, 'TickLabelInterpreter', 'none');
xtickangle(45); grid on;

%% Step 3: Plot Tracking

% Define scenario types and their corresponding number of variations
scenario_types_to_plot = {'paramVar', 'initVar', 'distVar'};
num_variations_for_each_type = [num_scenarios_param, num_scenarios_init, num_scenarios_dist];

plot_types_row_labels = {'Position', 'Velocity', 'Acceleration'};
units_rad = {'(rad)', '(rad/s)', '(rad/s$^2$)'};
units_m = {'(m)', '(m/s)', '(m/s$^2$)'};
joint_labels = {'q_1', 'q_2', 'q_3', 'q_4'};

% Loop through each scenario type (Parameter, Initial, Disturbance)
for type_idx = 1:length(scenario_types_to_plot)
    current_type = scenario_types_to_plot{type_idx};
    current_num_variations = num_variations_for_each_type(type_idx);

    % Loop through each variation number (1, 2, 3) within the current type
    for var_num = 1:current_num_variations
        % Construct the tags for FLC and SMC for the current scenario type and variation number
        flc_tag = sprintf('ctrl_flc_%s_%d', current_type, var_num);
        smc_tag = sprintf('ctrl_smc_%s_%d', current_type, var_num);

        % Check if these scenarios exist in results (they should from Step 1)
        if isfield(results, flc_tag) && isfield(results, smc_tag)
            % Get data for FLC
            t_flc = results.(flc_tag).t;
            q_flc = results.(flc_tag).q;
            dq_flc = results.(flc_tag).dq;
            ddq_flc = results.(flc_tag).ddq;
            qd_ref = results.(flc_tag).qd;
            dqd_ref = results.(flc_tag).dqd;
            ddqd_ref = results.(flc_tag).ddqd;

            % Get data for SMC
            t_smc = results.(smc_tag).t;
            q_smc = results.(smc_tag).q;
            dq_smc = results.(smc_tag).dq;
            ddq_smc = results.(smc_tag).ddq;

            fig_title = sprintf('Tracking Performance: %s Variation %d', strrep(current_type, 'Var', ''), var_num);
            figure('Name', fig_title, 'NumberTitle', 'off', 'Position', [100 100 1200 800]);
            sgtitle(fig_title, 'Interpreter', 'latex');

            for i = 1:3 % Row loop (1=Position, 2=Velocity, 3=Acceleration)
                for j = 1:4 % Column loop (Joints 1 to 4)
                    subplot(3, 4, (i-1)*4 + j); 
                    hold on;

                    % Plot Actual FLC, Actual SMC, and Reference
                    switch i
                        case 1 % Position
                            plot(t_flc, q_flc(:,j), 'b', 'DisplayName', 'FLC Actual', 'LineWidth', 1.2);
                            plot(t_smc, q_smc(:,j), 'r--', 'DisplayName', 'SMC Actual', 'LineWidth', 1.2);
                            plot(t_flc, qd_ref(:,j), 'k:', 'DisplayName', 'Reference', 'LineWidth', 1.2);
                            if j == 3 % q3 is prismatic (meters)
                                ylabel_text = ['$\mathrm{', joint_labels{j}, '}$ ', units_m{i}]; 
                            else % q1, q2, q4 are rotational (radians)
                                ylabel_text = ['$\mathrm{', joint_labels{j}, '}$ ', units_rad{i}];
                            end
                        case 2 % Velocity
                            plot(t_flc, dq_flc(:,j), 'b', 'DisplayName', 'FLC Actual', 'LineWidth', 1.2);
                            plot(t_smc, dq_smc(:,j), 'r--', 'DisplayName', 'SMC Actual', 'LineWidth', 1.2);
                            plot(t_flc, dqd_ref(:,j), 'k:', 'DisplayName', 'Reference', 'LineWidth', 1.2);
                            if j == 3 % dq3 is prismatic (m/s)
                                ylabel_text = ['$\mathrm{\dot{', joint_labels{j}, '}}$ ', units_m{i}]; 
                            else % dq1, dq2, dq4 are rotational (rad/s)
                                ylabel_text = ['$\mathrm{\dot{', joint_labels{j}, '}}$ ', units_rad{i}];
                            end
                        case 3 % Acceleration
                            plot(t_flc, ddq_flc(:,j), 'b', 'DisplayName', 'FLC', 'LineWidth', 1.2);
                            plot(t_smc, ddq_smc(:,j), 'r--', 'DisplayName', 'SMC', 'LineWidth', 1.2);
                            plot(t_flc, ddqd_ref(:,j), 'k:', 'DisplayName', 'Reference', 'LineWidth', 1.2);
                            if j == 3 % ddq3 is prismatic (m/s^2)
                                ylabel_text = ['$\mathrm{\ddot{', joint_labels{j}, '}}$ ', units_m{i}];
                            else % ddq1, ddq2, ddq4 are rotational (rad/s^2)
                                ylabel_text = ['$\mathrm{\ddot{', joint_labels{j}, '}}$ ', units_rad{i}];
                            end
                    end
                    
                    xlabel('Time (s)', 'Interpreter', 'latex');
                    ylabel(ylabel_text, 'Interpreter', 'latex');
                    grid on;
                    
                    % Add legend only to the first subplot of each row (for clarity)
                    if (i == 3 && j == 1)
                        legend('Location', 'best', 'Interpreter', 'latex'); 
                    end
         
                end
            end
        else
            warning('Could not find data for FLC or SMC for %s variation %d. Skipping plot.', current_type, var_num);
        end
    end
end


%% Helper Functions
function results = simulate_and_store(tag, ctrl_type, param, x0, disturbance_func, tspan, results)
    switch ctrl_type
        case 'flc'
            dynamics_func = @(t,x) scara_flc_dynamics(t, x, Kflc(t,x, param), param, disturbance_func);
            control_law = @(t,x) Kflc(t,x, param);
        case 'smc'
            dynamics_func = @(t,x) scara_smc_dynamics(t, x, Ksmc(t,x, param), param, disturbance_func);
            control_law = @(t,x) Ksmc(t,x, param);
        otherwise
            error('Unknown controller type: %s', ctrl_type);
    end
    [t, x] = ode45(dynamics_func, tspan, x0);
    dt = mean(diff(t));
    q = x(:,1:4);
    dq = x(:,5:8);
    qd = zeros(length(t),4);
    dqd = zeros(length(t),4);
    ddqd = zeros(length(t),4);
    for k = 1:length(t)
        [qk, dqk, ddqk] = generate_reference(t(k) , param);
        qd(k,:) = qk';
        dqd(k,:) = dqk';
        ddqd(k,:) = ddqk';
    end
    err = q - qd;
    RMSE = sqrt(mean(err.^2));
    MaxErr = max(abs(err));
    ddq = zeros(length(t),4);
    energy = 0; % Initialize energy
    for k = 1:length(t)
        % Extract current state
        q_curr = x(k,1:4)';
        dq_curr = x(k,5:8)';
        % Get virtual control input (desired acceleration)
        uk = control_law(t(k), [q_curr; dq_curr]);
        % Get dynamic matrices at current state
        [M_curr, C_curr, G_curr, Phi_curr] = scara_dynamics_matrices(q_curr, dq_curr, param);
        
        % Calculate actual torques/forces applied by actuators (tau)
        % tau = M * u_virtual + C + G + Phi
        tau_actual = M_curr * uk + C_curr + G_curr + Phi_curr;
        % Calculate actual accelerations (ddq)
        % This is based on the system's actual dynamics including disturbance
        % ddq_actual = M_inv * (tau_actual - C - G - Phi - disturbance)
        % Or, from the perspective of the FLC/SMC dynamics files:
        % ddq = u_virtual - M_inv * d
        % Let's use the definition from scara_flc_dynamics/scara_smc_dynamics.m
        d_curr = disturbance_func(t(k));
        ddq(k,:) = (uk - (M_curr \ d_curr))'; % For FLC/SMC: ddq = u_virtual - M_inv * d
        % Accumulate control energy (sum of squared norm of actual torques)
        energy = energy + norm(tau_actual)^2 * dt;
    end
    
    results.(tag).RMSE = RMSE;
    results.(tag).MaxErr = MaxErr;
    results.(tag).Energy = energy;
    results.(tag).q = q;
    results.(tag).qd = qd;
    results.(tag).dq = dq;
    results.(tag).dqd = dqd;
    results.(tag).ddq = ddq;
    results.(tag).ddqd = ddqd;
    results.(tag).t = t;
    results.(tag).param = param;
end

function out = format_scenario_name(tag)
    tag = strrep(tag, 'ctrl_', '');
    tag = strrep(tag, 'paramVar', 'param Var');
    tag = strrep(tag, 'initVar', 'init Cond');
    tag = strrep(tag, 'distVar', 'disturb Var');
    parts = strsplit(tag, '_');
    controller = upper(parts{1});
    out = sprintf('%s - %s %s', controller, parts{2}, parts{3});
end
