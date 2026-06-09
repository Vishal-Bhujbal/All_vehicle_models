%% Generate_22DOF_YMD_Sequential.m
% Bulletproof sequential batch-simulation script for 22-DOF Simulink model.
% - Uses Accelerator mode (No parsim / low memory)
% - Angles in Radians
% - Split Brake & Throttle commands
% - Direct variable extraction (out.VariableName)

% clear; clc; close all;

%% 1. Configuration & Setup
mdl = 'Vehicle_Model_999_Modular_roll_gradient'; % model name
load_system(mdl);

% Define control inputs to sweep
% Generate the steering sweep in RADIANS
Steer_Cmds_Rad = deg2rad(-20:5:20);           

% Base logical array for longitudinal sweep: [-1 (Full Brake) to +1 (Full Throttle)]
Long_Cmds = [-1.0, -0.5, 0, 0.5, 1.0]; 

% Create grid of all possible input combinations
[Steer_Grid, Long_Grid] = meshgrid(Steer_Cmds_Rad, Long_Cmds);
num_runs = numel(Steer_Grid);

fprintf('Preparing %d sequential maneuvers (Accelerator Mode)...\n', num_runs);

%% 2. Pre-allocate Data Arrays
% We will aggregate ALL time-steps from ALL runs into these master arrays
Ay_all   = [];
Mz_all   = [];
Beta_all = [];

%% 3. Execute Sequential Simulations
% Create a waitbar so you know the script hasn't frozen
h_wait = waitbar(0, 'Initializing Accelerator Mode (First run takes longer)...');

for i = 1:num_runs
    % Update the progress bar
    waitbar(i/num_runs, h_wait, sprintf('Running simulation %d of %d...', i, num_runs));
    
    % Split the Long_Grid(-1 to 1) into Brake(0 to 1) and Accln(0 to 1)
    brake_val = max(0, -Long_Grid(i));
    accln_val = max(0, Long_Grid(i));
    
    % Build the SimulationInput object for THIS specific run
    simIn = Simulink.SimulationInput(mdl);
    simIn = simIn.setVariable('Cmd_Steer', Steer_Grid(i));
    simIn = simIn.setVariable('Cmd_Brake', brake_val);
    simIn = simIn.setVariable('Cmd_Accln', accln_val);
    
    % Force Accelerator mode and set Stop Time
    simIn = simIn.setModelParameter('SimulationMode', 'accelerator');
    simIn = simIn.setModelParameter('StopTime', '4.0');
    
    try
        % Execute the simulation sequentially
        out = sim(simIn);
        
        % =========================================================================
        % DATA EXTRACTION: DIRECT VARIABLE ACCESS
        % Automatically handles both 'Array' and 'Timeseries' save formats
        % =========================================================================
        
        % 1. Extract Lateral Acceleration
        if isa(out.Lateral_accln, 'timeseries')
            Ay_ts = out.Lateral_accln.Data;
        else
            Ay_ts = out.Lateral_accln; 
        end
        
        % 2. Extract Yaw Moment
        if isa(out.Yaw_moment, 'timeseries')
            Mz_ts = out.Yaw_moment.Data;
        else
            Mz_ts = out.Yaw_moment; 
        end
        
        % 3. Extract Sideslip Angle
        if isa(out.Sideslip_angle, 'timeseries')
            Beta_ts = out.Sideslip_angle.Data;
        else
            Beta_ts = out.Sideslip_angle; 
        end
        
        % Append this run's time-series to the master arrays
        Ay_all   = [Ay_all; Ay_ts];
        Mz_all   = [Mz_all; Mz_ts];
        Beta_all = [Beta_all; Beta_ts];
        
    catch ME
        % If the vehicle flips/crashes, catch the error and keep the loop alive!
        warning('Run %d crashed at Steer: %.2f rad, Long: %.1f. Error: %s', ...
            i, Steer_Grid(i), Long_Grid(i), ME.message);
    end
end

% Close the waitbar when finished
close(h_wait);
fprintf('All simulations complete. Generating Dynamic YMD...\n');

%% 4. Plot the Dynamic Phase Portrait
h_fig = figure('Name', '22-DOF Dynamic YMD', 'Color', 'w', 'Position', [100, 100, 950, 700]);
ax = axes('Parent', h_fig);
hold(ax, 'on');
grid(ax, 'on');
box(ax, 'on');

% Create a scatter plot color-coded by the transient Sideslip Angle (\beta)
scatter(ax, Ay_all, Mz_all, 15, Beta_all, 'filled', 'MarkerEdgeAlpha', 0.4);

% Setup Colormap for Sideslip visualization
colormap(ax, jet);
cb = colorbar(ax);
cb.Label.String = 'Transient Sideslip Angle, \beta [rad]';
cb.Label.FontSize = 12;
cb.Label.FontWeight = 'bold';

% Limit color scale to +/- 15 degrees, but strictly in radians bounds (~0.26 rad)
clim(ax, deg2rad([-15, 15])); 

% Add Origin Axes
xline(ax, 0, 'k-', 'LineWidth', 1.5);
yline(ax, 0, 'k-', 'LineWidth', 1.5);

% Formatting
title(ax, 'Dynamic Yaw Moment Phase Portrait', 'FontSize', 16, 'FontWeight', 'bold');
xlabel(ax, 'Lateral Acceleration, A_y [g]', 'FontSize', 14, 'FontWeight', 'bold');
ylabel(ax, 'Total Yaw Moment, M_z [Nm]', 'FontSize', 14, 'FontWeight', 'bold');

% Set a tag so you can overlay specific runs on this later
set(h_fig, 'Tag', 'YMD_Master_Figure'); 
hold(ax, 'off');

fprintf('Done! Master Figure saved with tag "YMD_Master_Figure".\n');