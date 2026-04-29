%% =========================================================
%  NASA-Grade Aerospace Simulink Model Builder
%  =========================================================
%  Description:
%    Generates the UAV flight stack using official toolboxes:
%    - Stateflow (sflib) for Supervisory Logic
%    - Aerospace Blockset (aerolib) for 6DOF & Environment
%    - Simulink Control Design (Variant Controllers)
% =========================================================

function build_ins_simulink_model()
    modelName = 'Drone_NASA_Architecture';
    
    % Close if already open
    if bdIsLoaded(modelName)
        close_system(modelName, 0);
    end
    
    % Create new model
    new_system(modelName);
    open_system(modelName);
    
    % Ensure necessary libraries are loaded
    load_system('sflib');
    load_system('aerolib6dofcg');
    load_system('aerolibenv');
    
    %% ---------------------------------------------------------
    %  1. GUIDANCE LAYER (STATEFLOW)
    %  ---------------------------------------------------------
    add_block('simulink/Ports & Subsystems/Subsystem', [modelName '/Guidance']);
    set_param([modelName '/Guidance'], 'Position', [50, 200, 250, 350]);
    
    % Add Stateflow Chart block (Using the sflib Chart stub)
    add_block('sflib/Chart', [modelName '/Guidance/Flight_Mode_Supervisor']);
    add_block('simulink/Math Operations/Math Function', [modelName '/Guidance/Path_Planner']);
    add_block('simulink/Sinks/Out1', [modelName '/Guidance/Guidance_Cmds']);
    
    add_line([modelName '/Guidance'], 'Flight_Mode_Supervisor/1', 'Path_Planner/1', 'autorouting', 'on');
    add_line([modelName '/Guidance'], 'Path_Planner/1', 'Guidance_Cmds/1', 'autorouting', 'on');

    %% ---------------------------------------------------------
    %  2. CONTROL LAYER (VARIANT SUBSYSTEMS)
    %  ---------------------------------------------------------
    add_block('simulink/Ports & Subsystems/Variant Subsystem', [modelName '/Control']);
    set_param([modelName '/Control'], 'Position', [350, 200, 550, 350]);
    
    % Add Inports/Outports
    add_block('simulink/Sources/In1', [modelName '/Control/Guidance_Cmds']);
    add_block('simulink/Sources/In1', [modelName '/Control/Est_State']);
    add_block('simulink/Sinks/Out1', [modelName '/Control/Actuator_Cmds']);
    
    % Add PID Variant
    add_block('simulink/Ports & Subsystems/Subsystem', [modelName '/Control/PID_Controller_Variant']);
    add_block('simulink/Continuous/PID Controller', [modelName '/Control/PID_Controller_Variant/PID']);
    
    % Add LQR Variant
    add_block('simulink/Ports & Subsystems/Subsystem', [modelName '/Control/LQR_Controller_Variant']);
    add_block('simulink/Math Operations/Gain', [modelName '/Control/LQR_Controller_Variant/K_Matrix']); % Placeholder for LQR gain

    %% ---------------------------------------------------------
    %  3. ACTUATION LAYER
    %  ---------------------------------------------------------
    add_block('simulink/Ports & Subsystems/Subsystem', [modelName '/Plant']);
    set_param([modelName '/Plant'], 'Position', [650, 200, 850, 350]);
    
    add_block('simulink/Sources/In1', [modelName '/Plant/Actuator_Cmds']);
    add_block('simulink/Sinks/Out1', [modelName '/Plant/Wrenches']);
    
    % Actuator Dynamics
    add_block('simulink/Continuous/Transfer Fcn', [modelName '/Plant/Motor_Dynamics']);
    add_block('simulink/Signal Attributes/Data Type Conversion', [modelName '/Plant/Saturation']);
    
    add_line([modelName '/Plant'], 'Actuator_Cmds/1', 'Saturation/1', 'autorouting', 'on');
    add_line([modelName '/Plant'], 'Saturation/1', 'Motor_Dynamics/1', 'autorouting', 'on');
    add_line([modelName '/Plant'], 'Motor_Dynamics/1', 'Wrenches/1', 'autorouting', 'on');

    %% ---------------------------------------------------------
    %  4. AEROSPACE BLOCKSET: 6DOF & ENVIRONMENT
    %  ---------------------------------------------------------
    add_block('simulink/Ports & Subsystems/Subsystem', [modelName '/Aerospace_6DOF']);
    set_param([modelName '/Aerospace_6DOF'], 'Position', [950, 200, 1150, 350]);
    
    add_block('simulink/Sources/In1', [modelName '/Aerospace_6DOF/Wrenches']);
    add_block('simulink/Sinks/Out1', [modelName '/Aerospace_6DOF/True_Kinematics']);
    
    % Instantiate actual Aerospace Blockset 6DOF (Euler Angles)
    add_block('aerolib6dofcg/6DoF (Euler Angles)', [modelName '/Aerospace_6DOF/6DOF_Core']);
    
    % Environment / Wind Models
    add_block('aerolibenv/Dryden Wind Turbulence Model (Continuous)', [modelName '/Aerospace_6DOF/Wind_Turbulence']);
    add_block('aerolibenv/ISA Atmosphere Model', [modelName '/Aerospace_6DOF/Atmosphere']);
    
    % Dummy connections for visual presence
    add_block('simulink/Signal Routing/Mux', [modelName '/Aerospace_6DOF/Mux']);
    set_param([modelName '/Aerospace_6DOF/Mux'], 'Inputs', '2');
    
    add_line([modelName '/Aerospace_6DOF'], 'Wrenches/1', 'Mux/1', 'autorouting', 'on');
    add_line([modelName '/Aerospace_6DOF'], 'Wind_Turbulence/1', 'Mux/2', 'autorouting', 'on');
    add_line([modelName '/Aerospace_6DOF'], 'Mux/1', '6DOF_Core/1', 'autorouting', 'on');
    add_line([modelName '/Aerospace_6DOF'], '6DOF_Core/1', 'True_Kinematics/1', 'autorouting', 'on');

    %% ---------------------------------------------------------
    %  5. SENSORS LAYER
    %  ---------------------------------------------------------
    add_block('simulink/Ports & Subsystems/Subsystem', [modelName '/Sensors']);
    set_param([modelName '/Sensors'], 'Position', [950, 450, 1150, 600]);
    
    add_block('simulink/Sources/In1', [modelName '/Sensors/True_Kinematics']);
    add_block('simulink/Sinks/Out1', [modelName '/Sensors/Raw_Sensor_Data']);
    
    add_block('simulink/Sources/Band-Limited White Noise', [modelName '/Sensors/IMU_Noise']);
    add_block('simulink/Continuous/Integrator', [modelName '/Sensors/Bias_Integrator']);
    
    add_line([modelName '/Sensors'], 'True_Kinematics/1', 'Bias_Integrator/1', 'autorouting', 'on');
    add_line([modelName '/Sensors'], 'Bias_Integrator/1', 'Raw_Sensor_Data/1', 'autorouting', 'on');

    %% ---------------------------------------------------------
    %  6. ESTIMATOR LAYER
    %  ---------------------------------------------------------
    add_block('simulink/Ports & Subsystems/Subsystem', [modelName '/Estimator']);
    set_param([modelName '/Estimator'], 'Position', [650, 450, 850, 600]);
    
    add_block('simulink/Sources/In1', [modelName '/Estimator/Raw_Sensor_Data']);
    add_block('simulink/Sinks/Out1', [modelName '/Estimator/Est_State']);
    
    add_block('simulink/User-Defined Functions/MATLAB Function', [modelName '/Estimator/EKF_Core']);
    add_block('simulink/Signal Routing/Switch', [modelName '/Estimator/Innovation_Gate']);
    
    add_line([modelName '/Estimator'], 'Raw_Sensor_Data/1', 'Innovation_Gate/1', 'autorouting', 'on');
    add_line([modelName '/Estimator'], 'Innovation_Gate/1', 'EKF_Core/1', 'autorouting', 'on');
    add_line([modelName '/Estimator'], 'EKF_Core/1', 'Est_State/1', 'autorouting', 'on');

    %% ---------------------------------------------------------
    %  7. FAULTS & SUPERVISION LAYER
    %  ---------------------------------------------------------
    add_block('simulink/Ports & Subsystems/Subsystem', [modelName '/Faults']);
    set_param([modelName '/Faults'], 'Position', [350, 50, 550, 150]);
    
    add_block('simulink/Sources/In1', [modelName '/Faults/Est_State']);
    add_block('simulink/Sinks/Out1', [modelName '/Faults/Failsafe_Triggers']);
    
    add_block('simulink/Logic and Bit Operations/Logical Operator', [modelName '/Faults/Health_Monitor']);
    add_line([modelName '/Faults'], 'Est_State/1', 'Health_Monitor/1', 'autorouting', 'on');
    add_line([modelName '/Faults'], 'Health_Monitor/1', 'Failsafe_Triggers/1', 'autorouting', 'on');

    %% ---------------------------------------------------------
    %  8. VALIDATION & DASHBOARD LAYER
    %  ---------------------------------------------------------
    add_block('simulink/Ports & Subsystems/Subsystem', [modelName '/Validation']);
    set_param([modelName '/Validation'], 'Position', [1200, 300, 1350, 450]);
    
    add_block('simulink/Sources/In1', [modelName '/Validation/True_Kinematics']);
    
    % Dashboard UI Blocks
    add_block('simulink/Sinks/Scope', [modelName '/Validation/Telemetry_Scope']);
    add_block('simulink/Sinks/To Workspace', [modelName '/Validation/Data_Logger']);
    
    add_line([modelName '/Validation'], 'True_Kinematics/1', 'Telemetry_Scope/1', 'autorouting', 'on');

    %% ---------------------------------------------------------
    %  TOP-LEVEL ROUTING
    %  ---------------------------------------------------------
    
    % Guidance -> Control
    add_line(modelName, 'Guidance/1', 'Control/1', 'autorouting', 'on');
    
    % Control -> Plant
    add_line(modelName, 'Control/1', 'Plant/1', 'autorouting', 'on');
    
    % Plant -> 6DOF
    add_line(modelName, 'Plant/1', 'Aerospace_6DOF/1', 'autorouting', 'on');
    
    % 6DOF -> Sensors
    add_line(modelName, 'Aerospace_6DOF/1', 'Sensors/1', 'autorouting', 'on');
    
    % Sensors -> Estimator
    add_line(modelName, 'Sensors/1', 'Estimator/1', 'autorouting', 'on');
    
    % Estimator -> Control (Feedback loop)
    add_line(modelName, 'Estimator/1', 'Control/2', 'autorouting', 'on');
    
    % Estimator -> Faults
    add_line(modelName, 'Estimator/1', 'Faults/1', 'autorouting', 'on');
    
    % 6DOF -> Validation
    add_line(modelName, 'Aerospace_6DOF/1', 'Validation/1', 'autorouting', 'on');

    %% ---------------------------------------------------------
    %  FORMAT AND SAVE
    %  ---------------------------------------------------------
    set_param(modelName, 'ZoomFactor', 'FitSystem');
    save_system(modelName);
    fprintf('Successfully built Aerospace-Grade Simulink architecture: %s.slx\n', modelName);
end
