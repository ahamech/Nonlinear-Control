function u = generate_openloop_input(t, param)
% GENERATE_OPENLOOP_INPUT
% Produces open-loop input for the SCARA robot.
% In this basic version, the input is taken as the desired joint accelerations
% from the reference trajectory.
%
% Input:
%   t - Current simulation time (s)
%
% Output:
%   u - Virtual input (desired acceleration vector, 4x1)

    [~, ~, ddqd] = generate_reference(t, param);
    u = ddqd;  % Pure feedforward from reference
end
