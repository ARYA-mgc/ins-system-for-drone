%% =========================================================
%  Real-Time Advanced State Estimation (15-State EKF)
%  GPS-Denied Environment | MATLAB Sensor Fusion
%  =========================================================
%  Description:
%    This script implements an Advanced Inertial Navigation 
%    System (INS) for UAVs in GPS-denied environments. 
%    It uses a 15-state Extended Kalman Filter (EKF) that 
%    dynamically estimates accelerometer and gyroscope biases.
% =========================================================

clc; clear; close all;

%% ── 0. Add src to path ──────────────────────────────────
addpath(genpath(fileparts(mfilename('fullpath'))));
% Ensure all NASA-grade folders are in path
addpath(fullfile(pwd, 'Guidance'));
addpath(fullfile(pwd, 'Control'));
addpath(fullfile(pwd, 'Plant'));
addpath(fullfile(pwd, 'Sensors'));
addpath(fullfile(pwd, 'Estimator'));
addpath(fullfile(pwd, 'Faults'));
addpath(fullfile(pwd, 'Validation'));
addpath(fullfile(pwd, 'Interfaces'));
addpath(fullfile(pwd, 'Runtime'));
addpath(fullfile(pwd, 'Scenarios'));
addpath(fullfile(pwd, 'Logs'));

%% ── 1. Simulation Parameters ────────────────────────────
dt          = 0.01;          % Time step (100 Hz)
T_total     = 30;            % Total simulation time (s)
t           = 0:dt:T_total;
N           = length(t);

fprintf('=== Advanced UAV INS Simulation (15-State) ===\n');
fprintf('Duration : %.1f s  |  dt = %.4f s  |  Rate = %d Hz\n', ...
        T_total, dt, round(1/dt));

%% ── 2. IMU Noise Parameters ─────────────────────────────
imu = imu_noise_params();
% Add custom true biases for the advanced simulation
true_acc_bias = [0.02; -0.015; 0.03]; 
true_gyr_bias = [0.002; -0.001; 0.003];

%% ── 3. True Trajectory Generation ───────────────────────
[pos_true, vel_true, euler_true] = generate_trajectory(t, dt);

%% ── 4. IMU Data Simulation (with noise and biases) ──────
[accel_meas_ideal, gyro_meas_ideal] = simulate_imu(pos_true, vel_true, euler_true, dt, N, imu);
% Add static biases manually to verify estimator
accel_meas = accel_meas_ideal + repmat(true_acc_bias, 1, N);
gyro_meas  = gyro_meas_ideal + repmat(true_gyr_bias, 1, N);

%% ── 5. Advanced EKF Initialisation ──────────────────────
[x_est, P_est] = advanced_ekf_init(pos_true(:,1), vel_true(:,1), euler_true(:,1));

%% ── 6. Storage Arrays ───────────────────────────────────
pos_ekf   = zeros(3, N);
vel_ekf   = zeros(3, N);
euler_ekf = zeros(3, N);
acc_bias_est = zeros(3, N);
gyr_bias_est = zeros(3, N);

pos_ekf(:,1)   = x_est(1:3);
vel_ekf(:,1)   = x_est(4:6);
euler_ekf(:,1) = x_est(7:9);

%% ── 7. Main Estimation Loop ─────────────────────────────
fprintf('\nRunning Advanced 15-State EKF estimation loop...\n');
tic;

for k = 2:N
    %-- 7a. EKF Predict
    [x_est, P_est] = advanced_ekf_predict(x_est, P_est, accel_meas(:,k), ...
                                          gyro_meas(:,k), dt, imu);

    %-- 7b. EKF Update (barometer at 10 Hz, magnetometer at 50 Hz)
    if mod(k, 10) == 0          % barometer @ 10 Hz
        z_baro = pos_true(3,k) + randn * imu.baro_std;
        [x_est, P_est] = advanced_ekf_update_baro(x_est, P_est, z_baro, imu);
    end
    if mod(k, 2) == 0           % magnetometer @ 50 Hz
        z_mag = euler_true(3,k) + randn * imu.mag_std;
        [x_est, P_est] = advanced_ekf_update_mag(x_est, P_est, z_mag, imu);
    end

    %-- 7c. Store results
    pos_ekf(:,k)      = x_est(1:3);
    vel_ekf(:,k)      = x_est(4:6);
    euler_ekf(:,k)    = x_est(7:9);
    acc_bias_est(:,k) = x_est(10:12);
    gyr_bias_est(:,k) = x_est(13:15);
end
elapsed = toc;
fprintf('Loop complete: %.3f s  (%.1f Hz effective)\n', elapsed, N/elapsed);

%% ── 8. Error Analysis & Output ──────────────────────────
fprintf('\n======= Bias Estimation Summary =======\n');
fprintf('True Acc Bias: [%.4f, %.4f, %.4f]\n', true_acc_bias(1), true_acc_bias(2), true_acc_bias(3));
fprintf('Est  Acc Bias: [%.4f, %.4f, %.4f]\n', acc_bias_est(1,end), acc_bias_est(2,end), acc_bias_est(3,end));
fprintf('True Gyr Bias: [%.4f, %.4f, %.4f]\n', true_gyr_bias(1), true_gyr_bias(2), true_gyr_bias(3));
fprintf('Est  Gyr Bias: [%.4f, %.4f, %.4f]\n', gyr_bias_est(1,end), gyr_bias_est(2,end), gyr_bias_est(3,end));
fprintf('=======================================\n');

%% ── 9. Bias Tracking Plot ───────────────────────────────
figure('Name', 'Advanced EKF: Bias Estimation Tracking', 'Color', 'w', 'Position', [100 100 800 600]);

subplot(2,1,1);
plot(t, acc_bias_est(1,:), 'r', 'LineWidth', 1.5); hold on;
plot(t, acc_bias_est(2,:), 'g', 'LineWidth', 1.5);
plot(t, acc_bias_est(3,:), 'b', 'LineWidth', 1.5);
yline(true_acc_bias(1), 'r--', 'LineWidth', 1);
yline(true_acc_bias(2), 'g--', 'LineWidth', 1);
yline(true_acc_bias(3), 'b--', 'LineWidth', 1);
title('Accelerometer Bias Tracking');
legend('Est X', 'Est Y', 'Est Z', 'True X', 'True Y', 'True Z');
ylabel('Bias (m/s^2)'); grid on;

subplot(2,1,2);
plot(t, gyr_bias_est(1,:), 'r', 'LineWidth', 1.5); hold on;
plot(t, gyr_bias_est(2,:), 'g', 'LineWidth', 1.5);
plot(t, gyr_bias_est(3,:), 'b', 'LineWidth', 1.5);
yline(true_gyr_bias(1), 'r--', 'LineWidth', 1);
yline(true_gyr_bias(2), 'g--', 'LineWidth', 1);
yline(true_gyr_bias(3), 'b--', 'LineWidth', 1);
title('Gyroscope Bias Tracking');
legend('Est X', 'Est Y', 'Est Z', 'True X', 'True Y', 'True Z');
xlabel('Time (s)'); ylabel('Bias (rad/s)'); grid on;

fprintf('\nSimulation complete. Check figures for bias estimation tracking.\n');
