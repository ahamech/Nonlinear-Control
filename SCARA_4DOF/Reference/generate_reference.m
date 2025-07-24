function [qd, dqd, ddqd] = generate_reference(t, param)
% GENERATE_REFERENCE
% Generates desired trajectory (position, velocity, acceleration)
% for all 4 joints of the SCARA robot at time t.
%
% Outputs:
%   qd   - Desired joint positions   (4x1)
%   dqd  - Desired joint velocities  (4x1)
%   ddqd - Desired joint accelerations (4x1)

% Unified reference generator: step or sine based on param.reference_type

    switch lower(param.reference_type)
        case 'sine'
            % --- Trajectory parameters ---
            A = [pi/6; pi/8; 0.05; pi/12];       % Amplitude for each joint
            w = [0.5; 0.7; 0.9; 0.6];            % Frequency (rad/s)
            qd   = A .* sin(w * t);              % Position
            dqd  = A .* (w) .* cos(w * t);       % Velocity
            ddqd = -A .* (w.^2) .* sin(w * t);   % Acceleration

        case 'step'
            q1 = (t < 10) * 0.3 + (t >= 10) * 0.4;
            q2 = 0;
            q3 = (t < 10) * 0.1 + (t >= 10) * 0.2;
            q4 = 0;
            qd = [q1; q2; q3; q4];

            dq1 = (t < 10) * 0.4 + (t >= 10) * 0.3;
            dq2 = 0;
            dq3 = (t < 10) * 0.3 + (t >= 10) * 0.5;
            dq4 = 0;
            dqd = [dq1; dq2; dq3; dq4];

            ddqd = zeros(4,1);  % Step trajectory → constant vel → zero accel

        otherwise
            error('Unknown reference type: %s', param.reference_type);
    end
end
