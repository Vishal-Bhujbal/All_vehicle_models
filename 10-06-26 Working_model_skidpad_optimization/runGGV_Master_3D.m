    % SCRIPT: RunGGV_Master_3D.m
    % Purpose: Final GGV Generator with 3D Surface & Explicit Peak Logic.
    % Features:
    %   - Uses 'ax_sensor' and 'ay_sensor' (Physically Correct).
    %   - Captures Transients (Turn-in/Braking spikes).
    %   - Explicit Mode: Prevents Drag from confusing Accel/Brake detection.
    %   - Generates 3D Performance Tube.
    

function [results, Ax_grid, Ay_grid, V_grid] = runGGV_Master_3D(modelName)

    % --- 1. SETUP ---
    if ~bdIsLoaded(modelName), load_system(modelName); end
    
    % Velocity Sweep (m/s)
    velocities = 5:5:45; 
    % ts_zero = timeseries(zeros(726),linspace(0,95,726));
    % Initialize Storage 
    results = struct('v', [], 'ay_max', [], 'ax_max', [], 'ax_brk', []);
    % baseSimIn = baseSimIn.setVariable('Mode', 0);
    % baseSimIn = baseSimIn.setVariable('throttle_ts',ts_zero);
    % baseSimIn = baseSimIn.setVariable('brake_ts',ts_zero);
    % baseSimIn = baseSimIn.setVariable('steer_ts',ts_zero);

    for i = 1:length(velocities)
        v_target = velocities(i);
        baseSimIn = Simulink.SimulationInput(modelName);
        baseSimIn = baseSimIn.setModelParameter('StopTime', '2.5');
        baseSimIn = baseSimIn.setModelParameter('Solver', 'ode23s'); 
        baseSimIn = baseSimIn.setVariable('init_vx', v_target);
        baseSimIn = baseSimIn.setVariable('init_vy', 0);
        baseSimIn = baseSimIn.setVariable('init_r', 0);
        
        % Load Parameters
        current_WB = 1.17;
        
        % --- TEST 1: MAX BRAKING (Explicit 'brake' mode) ---
        simIn = baseSimIn;
        simIn = simIn.setVariable('Steer_Val', 0);
        simIn = simIn.setVariable('Throttle_Val', 0);
        simIn = simIn.setVariable('Brake_Val', 1.0); 
        
        [ax_brk, ~] = Get_Peak_Explicit(simIn, v_target, 'brake', 0, current_WB);
        
        % --- TEST 2: MAX ACCEL (Explicit 'accel' mode) ---
        simIn = baseSimIn;
        simIn = simIn.setVariable('Steer_Val', 0);
        simIn = simIn.setVariable('Throttle_Val', 1.0); 
        simIn = simIn.setVariable('Brake_Val', 0);
        
        [ax_acc, ~] = Get_Peak_Explicit(simIn, v_target, 'accel', 0, current_WB);
        
        % --- TEST 3: MAX LATERAL (Explicit 'lat' mode) ---
        best_ay = 0;
        
        % Optimization: Ramp steer to find limit
        for steer = 0.04:0.04:0.6
            simIn = baseSimIn;
            simIn = simIn.setVariable('Steer_Val', steer);
            simIn = simIn.setVariable('Throttle_Val', 0.5); 
            simIn = simIn.setVariable('Brake_Val', 0);
            
            [ay_curr, ~] = Get_Peak_Explicit(simIn, v_target, 'lat', steer, current_WB);
            
            if ay_curr > best_ay
                best_ay = ay_curr;
            elseif ay_curr < (best_ay * 0.9)
                % Stop if grip drops significantly (passed the peak)
                break; 
            end
        end
        
        % Store Results (Convert to Gs)
        results.v(i)      = v_target;
        results.ax_max(i) = ax_acc / 9.81;
        results.ax_brk(i) = ax_brk / 9.81; % Should be negative
        results.ay_max(i) = best_ay / 9.81;
        
    end
    
    % --- 2. GENERATE 3D SURFACE DATA ---
    % --- 2. GENERATE 3D SURFACE DATA (OPTIMIZED) ---
    theta = linspace(0, 2*pi, 60); 
    
    % 1. Determine dimensions
    n_speeds = length(results.v);
    n_angles = length(theta);
    
    % 2. Pre-allocate the grids (Zeros fill)
    % This reserves memory immediately, eliminating the warning
    V_grid  = zeros(n_speeds, n_angles);
    Ax_grid = zeros(n_speeds, n_angles);
    Ay_grid = zeros(n_speeds, n_angles);
    
    % 3. Fill the grids
    for i = 1:n_speeds
        v_slice = results.v(i);
        ax_pos = results.ax_max(i);
        ax_neg = results.ax_brk(i); 
        ay_lim = results.ay_max(i);
        
        % We fill the i-th row directly instead of appending
        for k = 1:n_angles
            th = theta(k);
            
            % Lateral (Cosine)
            y_val = ay_lim * cos(th);
            
            % Longitudinal (Sine) - Asymmetric blending
            sin_val = sin(th);
            if sin_val >= 0
                x_val = ax_pos * sin_val; % Forward Accel
            else
                x_val = abs(ax_neg) * sin_val; % Braking
            end
            
            % Assign directly to the matrix
            Ax_grid(i, k) = x_val;
            Ay_grid(i, k) = y_val;
            V_grid(i, k)  = v_slice;
        end
    end   
    
    % =========================================================================
    % FUNCTION: EXPLICIT PEAK FINDER (STRICT PEAK ONLY - NO MEAN)
    % =========================================================================
    function [peak_val, peak_yaw] = Get_Peak_Explicit(simIn, target_v, mode, steer_angle, wheelbase)
        peak_val = 0; 
        peak_yaw = 0;
        
        try
            out = sim(simIn);
            
            % Robust Data Fetch
            try t = out.tout; catch, t = out.get('tout'); end
            try vx = out.vx.Data; catch, vx = out.get('vx').Data; end
            try yaw = out.r.Data; catch, try yaw = out.yawrate.Data; catch, yaw = zeros(size(t)); end, end
            
            if strcmp(mode, 'lat')
                try acc = abs(out.ay_sensor.Data); catch, acc = zeros(size(t)); end
            else
                try acc = out.ax_sensor.Data; catch, acc = zeros(size(t)); end
            end
            
            % INITIALIZE EXTREMES
            % We initialize to values that ensure we find the true peak, 
            % even if the car spends most of the time at 0.
            switch mode
                case 'accel'
                    peak_val = -999; % Look for highest positive
                case 'brake'
                    peak_val = 999;  % Look for lowest negative
                case 'lat'
                    peak_val = 0;    % Look for highest absolute
            end
    
            % POINT-BY-POINT SCAN
            valid_points_found = false;
            
            for k = 1:length(t)
                % 1. Velocity Guard (Ignore low-speed noise in high-speed tests)
                if abs(vx(k) - target_v) > 10, continue; end
    
                % 2. Stability Guard (Ignore spins)
                r_theo = (vx(k) * tan(steer_angle)) / wheelbase;
                r_limit = max(0.5, abs(r_theo)*2.5 + 0.2);
                if abs(yaw(k)) > r_limit, continue; end
                
                % 3. Sanity Guard (Ignore simulation explosions)
                if abs(acc(k)) > 45, continue; end 
                
                val = acc(k);
                valid_points_found = true;
                
                % 4. STRICT PEAK LOGIC (No Averaging)
                switch mode
                    case 'accel'
                        if val > peak_val, peak_val = val; end
                    case 'brake'
                        if val < peak_val, peak_val = val; end
                    case 'lat'
                        if val > peak_val, peak_val = val; end
                end
            end
            
            % Handle cases where no valid data was found (e.g., instability immediately)
            if ~valid_points_found
                peak_val = 0;
            elseif peak_val == -999 || peak_val == 999
                peak_val = 0; 
            end
            
        catch ME
            peak_val = 0;
        end
    end
    assignin('base','Mode',1)
end

