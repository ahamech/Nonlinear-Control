function [M, C, G, Phi] = scara_dynamics_matrices(q, dq, param)
% SCARA_DYNAMICS_MATRICES
% Computes M(q), C(q,dq), G(q), and friction Phi(dq) for a 4-DOF SCARA robot.
%
% Inputs:
%   q     - Joint positions (4x1)
%   dq    - Joint velocities (4x1)
%   param - Robot parameters structure
%
% Outputs:
%   M     - Mass/inertia matrix (4x4)
%   C     - Coriolis and centrifugal vector (4x1)
%   G     - Gravity vector (4x1)
%   Phi   - Viscous friction vector (4x1)

    % Unpack state
    q1 = q(1); q2 = q(2);
    dq1 = dq(1); dq2 = dq(2);

    % Unpack parameters
    m1 = param.m1; m2 = param.m2; m3 = param.m3; m4 = param.m4;
    l1 = param.l1; l2 = param.l2;
    r4 = param.r4;
    g  = param.g;
    b1 = param.b1; b2 = param.b2; b3 = param.b3; b4 = param.b4;
    d1 = param.d1; d2 = param.d2;

    % Inertia terms
    I1 = (1/12) * m1 * l1^2;
    I2 = (1/12) * m2 * l2^2;
    I3 = 0;
    I4 = 0.5 * m4 * r4^2;

    % Dynamic coefficients
    p1 = I1 + I2 + I3 + I4 + m1*d1^2 + m2*(d1^2 + l1^2) + (m3 + m4)*(l1^2 + l2^2);
    p2 = 2 * (l1*d2*m2 + l1*l2*(m3 + m4));
    p3 = I2 + m2*l2^2 + (m3 + m4)*l2^2;
    p4 = m3 + m4;
    p5 = I4;

    % Mass matrix
    M = [p1 + p2*cos(q2), p3 + 0.5*p2*cos(q2), 0, -p5;
         p3 + 0.5*p2*cos(q2), p2,              0, -p5;
         0,                  0,                p4, 0;
        -p5,                -p5,               0,  p5];

    % Coriolis vector
    C1 = -p2*cos(q1)*dq1*dq2 - 0.5*p2*sin(q2)*dq2^2;
    C2 =  0.5*p2*sin(q2)*dq1^2;
    C = [C1; C2; 0; 0];

    % Gravity vector
    G = [0; 0; p4 * g; 0];

    % Friction vector
    Phi = [b1*dq1; b2*dq2; b3*dq(3); b4*dq(4)];
end
