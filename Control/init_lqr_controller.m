%% =========================================================
%  LQR Controller Initialization & Gain Synthesis
%  =========================================================
%  Description:
%    Computes the optimal feedback gain matrix (K) for the 
%    LQR (Linear Quadratic Regulator) variant in the Control 
%    Layer. Uses linearized state-space matrices around hover.
% =========================================================

function [K, S, e] = init_lqr_controller(m, Ixx, Iyy, Izz)
    fprintf('Synthesizing LQR Controller Gains...\n');
    
    % Gravity
    g = 9.81;
    
    % Simplified linearised state-space model around hover
    % States: x = [phi, theta, psi, p, q, r] (Attitude & Body Rates)
    % Inputs: u = [tau_phi, tau_theta, tau_psi] (Torques)
    
    A = zeros(6, 6);
    A(1:3, 4:6) = eye(3);
    
    B = zeros(6, 3);
    B(4, 1) = 1/Ixx;
    B(5, 2) = 1/Iyy;
    B(6, 3) = 1/Izz;
    
    % State weighting matrix (Q)
    % Penalize attitude errors more than rate errors
    q_phi = 100; q_theta = 100; q_psi = 50;
    q_p = 10;    q_q = 10;      q_r = 5;
    Q = diag([q_phi, q_theta, q_psi, q_p, q_q, q_r]);
    
    % Input weighting matrix (R)
    % Penalize excessive control effort (torque)
    r_phi = 1; r_theta = 1; r_psi = 2;
    R = diag([r_phi, r_theta, r_psi]);
    
    % Compute LQR Gain Matrix
    [K, S, e] = lqr(A, B, Q, R);
    
    fprintf('LQR Synthesis Complete. K Matrix:\n');
    disp(K);
    
    % Save to base workspace for Simulink to access
    assignin('base', 'LQR_K_Matrix', K);
end
