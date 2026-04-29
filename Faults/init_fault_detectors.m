%% =========================================================
%  Advanced Fault Detection & Isolation (FDI) Parameters
%  =========================================================
%  Description:
%    Initializes the thresholds and logic matrices for the 
%    Faults layer in Simulink. Includes parameters for EKF 
%    Innovation Mahalanobis distance checks, Motor Thrust 
%    Loss detection, and Battery brown-out triggers.
% =========================================================

function init_fault_detectors()
    fprintf('Loading Advanced Fault Detection Parameters...\n');
    
    % --- 1. EKF Divergence Monitor (Innovation Gating) ---
    % Mahalanobis distance threshold (chi-square distribution, 99% confidence)
    % Used to reject sensor outliers. If distance > threshold, reject measurement.
    faults.ekf.innov_gate_threshold = 9.21;  % For 2 Degrees of Freedom (e.g. XY pos)
    faults.ekf.cov_trace_max        = 5.0;   % Max allowable trace of Covariance matrix before triggering Reset
    
    % --- 2. Motor Thrust Loss Detector ---
    % Detects if RPM command is maxed but vehicle is still losing altitude/attitude
    faults.motor.rpm_saturation_time_limit = 1.5; % Seconds. If saturated for >1.5s -> Failure
    faults.motor.thrust_loss_degrade_ratio = 0.6; % If thrust drops below 60% of expected -> Trigger RTL
    
    % --- 3. Sensor Hard-Fault Detection ---
    % Implausible rates of change (derivatives)
    faults.sensor.max_gyro_rate_change = 50.0; % rad/s^2. Exceeding this implies sensor hit/glitch
    faults.sensor.max_accel_shock      = 16.0; % G's. Crash detection.
    
    % --- 4. Battery & Power Monitors ---
    faults.power.cell_min_voltage  = 3.2; % V per cell. Absolute minimum.
    faults.power.brownout_trigger  = faults.power.cell_min_voltage * 4; % For 4S lipo
    
    % --- 5. Supervisory Fallback Matrix ---
    % Maps detected fault ID to Action ID
    % Faults: 1=EKF Div, 2=Motor Loss, 3=Sensor Glitch, 4=Low Battery
    % Actions: 1=RTL, 2=Emergency Land, 3=Degraded Control (Attitude only)
    faults.supervisor.action_matrix = [
        1, 2;  % EKF Divergence -> Emergency Land
        2, 3;  % Motor Loss     -> Degraded Control / Crash land
        3, 1;  % Sensor Glitch  -> RTL (assuming redundant sensor takes over)
        4, 1   % Low Battery    -> RTL
    ];

    assignin('base', 'faults', faults);
    fprintf('Fault thresholds and supervisor matrices loaded.\n');
end
