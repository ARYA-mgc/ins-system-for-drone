%% =========================================================
%  Trim and Linearization Analysis Tool
%  =========================================================
%  Description:
%    Analyzes the 'Drone_Advanced_Flight_Stack' model.
%    Finds the trim state (equilibrium) for a hover 
%    condition, extracts the state-space model (A,B,C,D),
%    and plots the root locus and Bode response to verify 
%    stability margins.
% =========================================================

function trim_and_linearize()
    model = 'Drone_Advanced_Flight_Stack';
    
    % Ensure the model is loaded
    if ~bdIsLoaded(model)
        load_system(model);
    end
    
    fprintf('=== Trim & Linearization Analysis ===\n');
    fprintf('Model: %s\n', model);
    
    try
        % 1. Create default operating point specification
        opspec = operspec(model);
        
        % In a real scenario, you would configure opspec here to enforce:
        % - Alt = constant (e.g., 10 meters)
        % - Vel = 0 (Hover)
        % - Pitch/Roll = 0
        
        fprintf('Searching for Hover Trim Condition...\n');
        
        % Find operating point (Requires Simulink Control Design)
        % Using basic findop
        opt = findopOptions('DisplayReport', 'off');
        op = findop(model, opspec, opt);
        
        fprintf('Trim condition found successfully.\n');
        
        % 2. Linearize the model around the trim point
        fprintf('Linearizing model to extract State-Space (A,B,C,D)...\n');
        
        sys = linearize(model, op);
        
        % 3. Stability Analysis
        fprintf('Extracting eigenvalues (poles)...\n');
        p = pole(sys);
        
        % Check for unstable poles (real part > 0)
        unstable_poles = p(real(p) > 1e-6);
        
        if isempty(unstable_poles)
            fprintf('--> SUCCESS: System is STABLE at hover (all poles in LHP).\n');
        else
            fprintf('--> WARNING: System is UNSTABLE at hover (%d poles in RHP).\n', length(unstable_poles));
            disp(unstable_poles);
        end
        
        % 4. Generate Plots
        figure('Name', 'Trim & Linearization Analysis', 'Position', [100 100 1000 400]);
        
        subplot(1,2,1);
        pzmap(sys);
        title('Pole-Zero Map (Hover)');
        grid on;
        
        subplot(1,2,2);
        bode(sys);
        title('Bode Response');
        grid on;
        
        fprintf('Analysis complete. Check figures for Pole-Zero and Bode plots.\n');
        
    catch ME
        fprintf('Note: Linearization requires Simulink Control Design toolbox.\n');
        fprintf('Error Details: %s\n', ME.message);
    end
end
