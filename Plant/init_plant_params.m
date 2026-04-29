%% =========================================================
%  Plant Dynamics & Actuator Realism Parameterization
%  =========================================================
%  Description:
%    Initializes the physical parameters of the UAV, including
%    mass, inertia, motor lag, aerodynamic drag coefficients,
%    and battery discharge curves. Loads into the base 
%    workspace for the Simulink 'Plant' layer.
% =========================================================

function init_plant_params()
    fprintf('Loading Advanced Plant Dynamics Parameters...\n');
    
    % --- Rigid Body Parameters ---
    plant.mass = 1.25;          % kg
    plant.g    = 9.81;          % m/s^2
    
    % Moments of Inertia (kg*m^2)
    plant.Ixx = 0.021;
    plant.Iyy = 0.022;
    plant.Izz = 0.040;
    plant.I_matrix = diag([plant.Ixx, plant.Iyy, plant.Izz]);
    
    % --- Actuator Realism ---
    % ESC / Motor Delay (First-order transfer function time constant)
    plant.esc_tau_spool_up   = 0.05;  % seconds (faster to spool up)
    plant.esc_tau_spool_down = 0.08;  % seconds (slower to spool down due to inertia)
    
    % Motor limits
    plant.rpm_min = 1000;
    plant.rpm_max = 8500;
    
    % --- Non-Linear Thrust & Torque Mapping (Lookup Tables) ---
    % Simulated bench-test data mapping RPM to Thrust (N) and Torque (N*m)
    plant.rpm_breakpoints = [1000, 2000, 3000, 4000, 5000, 6000, 7000, 8000, 8500];
    plant.thrust_lut_data = [ 0.5,  1.2,  2.5,  4.2,  6.8,  9.9, 13.5, 18.0, 20.5]; % Quadratic curve
    plant.torque_lut_data = [ 0.01, 0.03, 0.08, 0.15, 0.25, 0.40, 0.60, 0.85, 1.00];
    
    % Quad-X Mixer Geometry
    plant.arm_length = 0.25; % meters
    % Mixer matrix: [Thrust; Roll; Pitch; Yaw] mapped from 4 motors
    % (Standard Betaflight/PX4 Quad-X configuration)
    plant.mixer_matrix = [
         1,  1,  1,  1;
        -1, -1,  1,  1;
         1, -1, -1,  1;
        -1,  1, -1,  1
    ];
    plant.inv_mixer = inv(plant.mixer_matrix); % To map control wrenches back to motors

    % --- Aerodynamics ---
    % Translational drag coefficients
    plant.Cd_x = 0.25;
    plant.Cd_y = 0.25;
    plant.Cd_z = 0.50;
    
    % --- Battery Model ---
    plant.bat_cells = 4;
    plant.bat_max_voltage = 4.2 * plant.bat_cells; % 16.8V
    plant.bat_sag_coeff   = 0.05; % V per Amp drawn
    
    % Push to workspace
    assignin('base', 'plant', plant);
    fprintf('Plant parameters successfully loaded to Workspace.\n');
end
