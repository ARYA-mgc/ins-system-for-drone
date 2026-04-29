%% =========================================================
%  Master Initialization Script (init_all.m)
%  =========================================================
%  Description:
%    Advanced integration script. Loads all parameters from 
%    every architectural block (Guidance, Control, Plant, 
%    Sensors, Estimator, Faults) into the base workspace.
%    Must be run before compiling the Simulink model.
% =========================================================

function init_all()
    fprintf('\n========================================\n');
    fprintf('  UAV Enterprise Simulation Setup\n');
    fprintf('========================================\n\n');
    
    % 0. Interfaces & Runtime Contracts (Must load first)
    bus_definitions();
    signal_enums();
    units_registry();
    sample_rates();
    
    task_scheduler();
    sensor_timestamps();
    loop_rate_manager();
    latency_injector();
    
    % 1. Plant (Base physical properties needed by others)
    init_plant_params();
    thrust_torque_lut();
    battery_sag_model();
    motor_esc_dynamics();
    aero_drag_model();
    
    % 2. Sensors (Depends on Plant)
    imu_noise_params();
    sensor_bias_drift();
    mag_disturbance_model();
    gps_dropout_sim();
    
    % 3. Estimator (Depends on Sensors)
    covariance_tuning();
    quat_normalization();
    
    % 4. Control (Depends on Plant & Estimator)
    plant_data = evalin('base', 'plant');
    init_lqr_controller(plant_data.mass, plant_data.Ixx, plant_data.Iyy, plant_data.Izz);
    init_pid_gains();
    control_allocation_matrix();
    rate_limiter_setup();
    feedforward_gains();
    
    % 5. Guidance
    init_waypoint_manager();
    init_state_machine();
    calculate_jerk_limits();
    geofence_monitor();
    
    % 6. Faults
    init_fault_detectors();
    thrust_loss_detector();
    ekf_divergence_check();
    sensor_glitch_monitor();
    fallback_supervisor_matrix();
    
    % 7. Codegen
    setup_embedded_coder();
    misra_c_config();
    target_hardware_setup();
    data_type_optimization();
    
    fprintf('\n========================================\n');
    fprintf('  Initialization Complete.\n');
    fprintf('  Ready to build: Drone_NASA_Architecture.slx\n');
    fprintf('========================================\n\n');
end
