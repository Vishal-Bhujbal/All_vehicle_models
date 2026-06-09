function results = plotCurvatureVsSteer_MultiV(modelName,velocity)
% =========================================================================
% FUNCTION: plotCurvatureVsSteer_MultiV
% Purpose:
%   Runs the simulation for multiple velocities and stores curvature vs
%   steering data for each velocity.
%
% Output:
%   results: struct array with fields:
%     - v     : velocity (m/s)
%     - s_deg : steering angle (deg)
%     - kappa : curvature (1/m)
% =========================================================================

    if ~bdIsLoaded(modelName)
        load_system(modelName);
    end

    StopTime = '20';
    Solver   = 'ode23s';
    t_start    = 10;
    v_tar = velocity;

    % Preallocate struct
    results = repmat(struct('v', [], 's_deg', [], 'kappa', []), 1);  
    % Kappa is curvature..

        % --- Simulation Input ---
    simIn = Simulink.SimulationInput(modelName);
    simIn = simIn.setModelParameter('StopTime', StopTime);
    simIn = simIn.setModelParameter('Solver', Solver);
    simIn = simIn.setVariable('V_ref', v_tar);

        % --- Run simulation ---
        out = sim(simIn);

        % --- Fetch curvature + steer ---
        kappa_ts = out.curvature;
        steer_ts = out.steer_input;

        % Ensure timeseries format
        if ~isa(kappa_ts,'timeseries')
            kappa_ts = timeseries(kappa_ts.Data, kappa_ts.Time);
        end
        if ~isa(steer_ts,'timeseries')
            steer_ts = timeseries(steer_ts.Data, steer_ts.Time);
        end

        % Convert steering to degrees
        steer_ts.Data = steer_ts.Data * (180/pi);

        % Trim after 10 sec
        kappa_ts = getsamples(kappa_ts, kappa_ts.Time >= t_start);
        steer_ts = getsamples(steer_ts, steer_ts.Time >= t_start);

        % Resample steering to curvature time base
        t_common = kappa_ts.Time;
        steer_r  = resample(steer_ts, t_common);

        % Store in results
        results.v     = v_tar;
        results.s_deg = steer_r.Data;
        results.kappa = kappa_ts.Data;
    
end
