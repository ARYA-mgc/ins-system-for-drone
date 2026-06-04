%% =========================================================
%  Master Initialization Script (init_all.m)
%  =========================================================
%  Description:
%    Advanced integration script. Loads all parameters from 
%    every architectural block (Guidance, Control, Plant, 
%    Sensors, Estimator, Faults) into the base workspace.
%    Must be run before compiling the Simulink model.
%
%    Gracefully handles missing toolboxes (Simulink, Aerospace
%    Blockset, Stateflow) so that parameter-only initialization
%    succeeds even in headless CI environments.
% =========================================================

function init_all()
    fprintf('\n========================================\n');
    fprintf('  UAV Enterprise Simulation Setup\n');
    fprintf('========================================\n\n');
    
    errors = {};
    
    % --- 0. Add all subdirectories to MATLAB path ---
    rootDir = fileparts(mfilename('fullpath'));
    folders = {'Interfaces','Runtime','Guidance','Control','Plant', ...
               'Sensors','Estimator','Faults','Validation','Logs', ...
               'Scenarios','Codegen'};
    for k = 1:numel(folders)
        p = fullfile(rootDir, folders{k});
        if isfolder(p)
            addpath(p);
        end
    end
    fprintf('[PATH]  All subdirectories added to MATLAB path.\n\n');
    
    % --- 1. Interface Contracts (Pure MATLAB, no toolbox needed) ---
    errors = safe_call(@bus_definitions,    'Interfaces/bus_definitions',    errors);
    errors = safe_call(@signal_enums,       'Interfaces/signal_enums',       errors);
    errors = safe_call(@units_registry,     'Interfaces/units_registry',     errors);
    errors = safe_call(@sample_rates,       'Interfaces/sample_rates',       errors);
    
    % --- 2. Runtime Schedulers ---
    errors = safe_call(@task_scheduler,     'Runtime/task_scheduler',        errors);
    errors = safe_call(@sensor_timestamps,  'Runtime/sensor_timestamps',     errors);
    errors = safe_call(@loop_rate_manager,  'Runtime/loop_rate_manager',     errors);
    errors = safe_call(@latency_injector,   'Runtime/latency_injector',      errors);
    
    % --- 3. Plant (Base physical properties needed by others) ---
    errors = safe_call(@init_plant_params,  'Plant/init_plant_params',       errors);
    errors = safe_call(@thrust_torque_lut,  'Plant/thrust_torque_lut',       errors);
    errors = safe_call(@battery_sag_model,  'Plant/battery_sag_model',       errors);
    errors = safe_call(@motor_esc_dynamics, 'Plant/motor_esc_dynamics',      errors);
    errors = safe_call(@aero_drag_model,    'Plant/aero_drag_model',         errors);
    
    % --- 4. Sensors ---
    errors = safe_call(@imu_noise_params,       'Sensors/imu_noise_params',       errors);
    errors = safe_call(@sensor_bias_drift,      'Sensors/sensor_bias_drift',      errors);
    errors = safe_call(@mag_disturbance_model,  'Sensors/mag_disturbance_model',  errors);
    errors = safe_call(@gps_dropout_sim,        'Sensors/gps_dropout_sim',        errors);
    
    % --- 5. Estimator ---
    errors = safe_call(@covariance_tuning,  'Estimator/covariance_tuning',   errors);
    errors = safe_call(@quat_normalization, 'Estimator/quat_normalization',  errors);
    
    % --- 6. Control (Depends on Plant) ---
    try
        plant_data = evalin('base', 'plant');
        init_lqr_controller(plant_data.mass, plant_data.Ixx, plant_data.Iyy, plant_data.Izz);
        fprintf('[  OK ] Control/init_lqr_controller\n');
    catch ME
        fprintf('[SKIP ] Control/init_lqr_controller — %s\n', ME.message);
        errors{end+1} = 'Control/init_lqr_controller';
    end
    errors = safe_call(@init_pid_gains,            'Control/init_pid_gains',            errors);
    errors = safe_call(@control_allocation_matrix,  'Control/control_allocation_matrix', errors);
    errors = safe_call(@rate_limiter_setup,         'Control/rate_limiter_setup',        errors);
    errors = safe_call(@feedforward_gains,          'Control/feedforward_gains',         errors);
    
    % --- 7. Guidance ---
    errors = safe_call(@init_waypoint_manager, 'Guidance/init_waypoint_manager', errors);
    errors = safe_call(@init_state_machine,    'Guidance/init_state_machine',    errors);
    errors = safe_call(@calculate_jerk_limits, 'Guidance/calculate_jerk_limits', errors);
    errors = safe_call(@geofence_monitor,      'Guidance/geofence_monitor',      errors);
    
    % --- 8. Faults ---
    errors = safe_call(@init_fault_detectors,      'Faults/init_fault_detectors',      errors);
    errors = safe_call(@thrust_loss_detector,      'Faults/thrust_loss_detector',      errors);
    errors = safe_call(@ekf_divergence_check,      'Faults/ekf_divergence_check',      errors);
    errors = safe_call(@sensor_glitch_monitor,     'Faults/sensor_glitch_monitor',     errors);
    errors = safe_call(@fallback_supervisor_matrix,'Faults/fallback_supervisor_matrix', errors);
    
    % --- 9. Codegen (Requires Simulink & Embedded Coder — skip if absent) ---
    if license('test', 'Simulink') && license('test', 'Real-Time_Workshop')
        errors = safe_call(@setup_embedded_coder,  'Codegen/setup_embedded_coder',  errors);
        errors = safe_call(@misra_c_config,        'Codegen/misra_c_config',        errors);
        errors = safe_call(@target_hardware_setup, 'Codegen/target_hardware_setup', errors);
        errors = safe_call(@data_type_optimization,'Codegen/data_type_optimization',errors);
    else
        fprintf('[SKIP ] Codegen/* — Simulink or Embedded Coder license not available.\n');
    end
    
    % --- Summary ---
    fprintf('\n========================================\n');
    if isempty(errors)
        fprintf('  Initialization Complete (0 errors).\n');
    else
        fprintf('  Initialization Complete (%d modules skipped).\n', numel(errors));
    end
    fprintf('  Ready to build: Drone_NASA_Architecture.slx\n');
    fprintf('========================================\n\n');
end

%% ---- Helper: Safe Call with Logging ----
function errors = safe_call(fn, label, errors)
    try
        fn();
        fprintf('[  OK ] %s\n', label);
    catch ME
        fprintf('[SKIP ] %s — %s\n', label, ME.message);
        errors{end+1} = label;
    end
end
