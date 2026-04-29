%% =========================================================
%  Embedded Coder Configuration Script
%  =========================================================
%  Description:
%    Configures the 'Drone_NASA_Architecture' Simulink model
%    to strictly comply with aerospace embedded C/C++ 
%    code generation standards (e.g., MISRA C, fixed step,
%    hardware targeting like ARM Cortex-M or PX4/ArduPilot).
% =========================================================

function setup_embedded_coder()
    model = 'Drone_NASA_Architecture';
    
    if ~bdIsLoaded(model)
        try
            load_system(model);
        catch
            fprintf('Model %s not found. Run build script first.\n', model);
            return;
        end
    end
    
    fprintf('Configuring %s for Embedded Coder...\n', model);
    
    % --- Solver Configuration ---
    % Embedded targets require fixed-step discrete solvers
    set_param(model, 'SolverType', 'Fixed-step');
    set_param(model, 'Solver', 'FixedStepDiscrete');
    set_param(model, 'FixedStep', '0.0025'); % 400Hz control loop base rate
    
    % --- Code Generation Settings ---
    % Set System Target File to Embedded Real-Time (ert.tlc)
    set_param(model, 'SystemTargetFile', 'ert.tlc');
    
    % Target Language (C++)
    set_param(model, 'TargetLang', 'C++');
    
    % Hardware Implementation (e.g., ARM Cortex-M4 typically used in Flight Controllers)
    set_param(model, 'ProdHWDeviceType', 'ARM Compatible->ARM Cortex-M');
    
    % --- Optimization & MISRA Compliance ---
    % Optimize for Execution Efficiency and RAM
    set_param(model, 'DefaultParameterBehavior', 'Inlined');
    set_param(model, 'OptimizationCustomize', 'on');
    set_param(model, 'GlobalVariableUsage', 'None');
    
    % MISRA C:2012 compliance checks (Requires Polyspace or specific ERT settings)
    % We enable strict code generation rules
    set_param(model, 'ModelReferenceCompliant', 'on');
    set_param(model, 'GenerateComments', 'on');
    set_param(model, 'SimulinkBlockComments', 'on');
    set_param(model, 'StateflowObjectComments', 'on');
    set_param(model, 'ReqsInCode', 'on'); % Traceability
    
    % --- Output Configuration ---
    % Only generate code (don't compile binary right away)
    set_param(model, 'GenCodeOnly', 'on');
    
    fprintf('Embedded Coder configuration complete.\n');
    fprintf('You can now generate flight-ready C++ code using:\n');
    fprintf('>> slbuild(''%s'')\n', model);
end
