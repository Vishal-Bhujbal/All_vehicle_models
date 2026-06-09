function SimResults = LapSim_Endurance_for_app(track_file, ggv_file, Par, aero_par)
% RUN_ENDURANCE_SIMULATION Master Endurance Simulation 3D
% Designed for background execution (e.g., MATLAB App Designer)
%
% INPUTS:
%   track_file - String path to the track .csv file
%   ggv_file   - String path to the GGV .mat file
%   Par        - Struct containing vehicle parameters (m, C_rr, TW)
%   aero_par   - Struct containing aerodynamic parameters (Area, rho)
%
% OUTPUT:
%   SimResults - Struct containing all data needed for visualization.

    % --- 0. SIMULATION PARAMETERS ---
    TARGET_DISTANCE_M = 22000; % 22 km
    MASS_KG           = Par.m;
    CdA               = aero_par.Area;
    RHO               = aero_par.rho;
    ROLLING_RES       = Par.C_rr;

    % --- 1. CHECK & LOAD INPUTS ---
    if ~exist(ggv_file, 'file')
        potential_files = {'GGV_Final_Data.mat', 'GGV_Sensor_Data.mat', 'GGV_Safe_Data.mat', 'GGV_Map.mat','GGV_Final_Data_9000rpm.mat'};
        found = false;
        for k = 1:length(potential_files)
            if exist(potential_files{k}, 'file')
                ggv_file = potential_files{k};
                found = true;
                break;
            end
        end
        if ~found
            error('No GGV .mat files found. Please run your GGV generation script first.');
        end
    end
    
    loaded_data = load(ggv_file, 'results'); 
    if ~isfield(loaded_data, 'results')
        error('File "%s" loaded, but it does not contain the "results" structure.', ggv_file);
    end
    ggv_raw = loaded_data.results;

    if ~exist(track_file, 'file')
         track_file = 'FB Track.csv'; 
         if ~exist(track_file, 'file')
            error('Track file not found. Please ensure the CSV file is in the folder.');
         end
    end

    % --- 2. OPTIMIZE RACING LINE ---
    [path_s, path_k, path_X, path_Y] = Optimize_Racing_Line(track_file);

    % Calculate Raw Track Edges for Plotting
    try
        data = readmatrix(track_file);
        if size(data, 2) < 4
             opts = detectImportOptions(track_file);
             opts.DataLines = [2 Inf];
             data = readmatrix(track_file, opts);
        end
    catch
        error('Could not read track file. Make sure it is the 4-column CSV format.');
    end
    xc = data(:,1); yc = data(:,2); wl = data(:,3); wr = data(:,4);
    dx_t = gradient(xc); dy_t = gradient(yc); angle_t = atan2(dy_t, dx_t);
    xl_edge = xc + wl.*cos(angle_t+pi/2); yl_edge = yc + wl.*sin(angle_t+pi/2);
    xr_edge = xc - wr.*cos(angle_t+pi/2); yr_edge = yc - wr.*sin(angle_t+pi/2);

    % --- 3. PRE-PROCESS GGV MAP ---
    GGV_Env = Process_GGV_Envelope(ggv_raw);

    % --- 4. RUN ENDURANCE SIMULATION (22km) ---
    Track_Length_m = path_s(end);
    Num_Laps       = ceil(TARGET_DISTANCE_M / Track_Length_m);

    % Run Standing Start Lap (Lap 1)
    Results_SS = Solve_3Pass(path_s, path_k, GGV_Env, 0); 

    % Run Flying Lap (Laps 2 to N)
    v_flying_start = Results_SS.v(end); 
    Results_FL     = Solve_3Pass(path_s, path_k, GGV_Env, v_flying_start);

    % Calculate Total Time
    Endurance_Time = Results_SS.laptime + (Num_Laps - 1) * Results_FL.laptime;

    % --- 5. CALCULATE ENERGY CONSUMPTION ---
    [E_SS_kWh, ~]       = Calculate_Energy(Results_SS, MASS_KG, CdA, RHO, ROLLING_RES);
    [E_FL_kWh, P_FL_kW] = Calculate_Energy(Results_FL, MASS_KG, CdA, RHO, ROLLING_RES);
    Total_Energy_kWh    = E_SS_kWh + (Num_Laps - 1) * E_FL_kWh;

    % Pack primary visual results (using Flying Lap as most representative)
    Results = Results_FL;
    Results.Power_kW  = P_FL_kW; 
    Results.steer_deg = rad2deg(atan(Par.TW * path_k)); % Using Par.TW instead of hardcoded 1.53

    % --- 6. PACK ALL RESULTS INTO OUTPUT STRUCT ---
    SimResults.Endurance_Time_s = Endurance_Time;
    SimResults.Total_Energy_kWh = Total_Energy_kWh;
    SimResults.Num_Laps         = Num_Laps;
    SimResults.LapTime_FL       = Results_FL.laptime;
    SimResults.LapTime_SS       = Results_SS.laptime;
    SimResults.Avg_Power_kW     = mean(Results.Power_kW);
    
    SimResults.s         = Results.s;
    SimResults.v         = Results.v;
    SimResults.Power_kW  = Results.Power_kW;
    SimResults.ay_used   = Results.ay_used;
    SimResults.ax_used   = Results.ax_used;
    SimResults.steer_deg = Results.steer_deg;
    
    SimResults.Path.X = path_X;
    SimResults.Path.Y = path_Y;
    SimResults.Track.xl_edge = xl_edge;
    SimResults.Track.yl_edge = yl_edge;
    SimResults.Track.xr_edge = xr_edge;
    SimResults.Track.yr_edge = yr_edge;

end

% =========================================================================
% HELPER FUNCTION 1: RACING LINE OPTIMIZER
% =========================================================================
function [s_final, k_final, x_final, y_final] = Optimize_Racing_Line(filename)
    try
        data = readmatrix(filename);
        if size(data, 2) < 4
             opts = detectImportOptions(filename);
             opts.DataLines = [2 Inf];
             data = readmatrix(filename, opts);
        end
    catch
        error('Could not read track file. Make sure it is the 4-column CSV format.');
    end
    
    xc = data(:,1); yc = data(:,2); wl = data(:,3); wr = data(:,4);
    
    if norm([xc(1)-xc(end), yc(1)-yc(end)]) > 0.1
        xc(end+1)=xc(1); yc(end+1)=yc(1); wl(end+1)=wl(1); wr(end+1)=wr(1);
    end
    
    N = length(xc); x = xc; y = yc; 
    Iterations = 200; Safety_Margin = 0.5;
    
    for iter = 1:Iterations
        xt = 0.5 * ([x(end-1); x(1:end-1)] + [x(2:end); x(2)]);
        yt = 0.5 * ([y(end-1); y(1:end-1)] + [y(2:end); y(2)]);
        for i = 1:N
            dx = xt(i) - xc(i); dy = yt(i) - yc(i); dist = sqrt(dx^2 + dy^2);
            if i < N, tx = xc(i+1)-xc(i); ty = yc(i+1)-yc(i); else, tx = xc(2)-xc(1); ty = yc(2)-yc(1); end
            if (tx*dy - ty*dx) > 0, max_w = wl(i)-Safety_Margin; else, max_w = wr(i)-Safety_Margin; end
            if max_w < 0.1, max_w = 0.1; end
            if dist > max_w, scale = max_w / dist; x(i) = xc(i) + dx * scale; y(i) = yc(i) + dy * scale;
            else, x(i) = xt(i); y(i) = yt(i); end
        end
    end
    dx = gradient(x); dy = gradient(y); ddx = gradient(dx); ddy = gradient(dy);
    k_raw = (dx .* ddy - dy .* ddx) ./ ((dx.^2 + dy.^2).^1.5);
    k_final = smoothdata(k_raw, 'gaussian', 15);
    ds = sqrt(dx.^2 + dy.^2); s_final = [0; cumsum(ds(2:end))];
    x_final = x; y_final = y;
end

% =========================================================================
% HELPER FUNCTION 2: PROCESS GGV STRUCT
% =========================================================================
function Env = Process_GGV_Envelope(results)
    Env.v = results.v(:); 
    Env.ay_max = abs(results.ay_max(:)) * 9.81;
    Env.ax_acc = results.ax_max(:) * 9.81;
    Env.ax_brk = abs(results.ax_brk(:)) * 9.81;
    [Env.v, idx] = sort(Env.v);
    Env.ay_max = Env.ay_max(idx); Env.ax_acc = Env.ax_acc(idx); Env.ax_brk = Env.ax_brk(idx);
end

% =========================================================================
% HELPER FUNCTION 3: 3-PASS SOLVER
% =========================================================================
function R = Solve_3Pass(s, k, Env, v_start)
    if nargin < 4, v_start = 0; end
    N = length(s);
    v_apex = zeros(N, 1); v_acc = zeros(N, 1); ax_acc_log = zeros(N, 1);
    v_brk = zeros(N, 1); ax_brk_log = zeros(N, 1);
    
    for i = 1:N
        curv = abs(k(i));
        if curv < 1e-4, v_apex(i) = max(Env.v);
        else
            v_g = 20; 
            for iter=1:5
                ay_lim = interp1(Env.v, Env.ay_max, v_g, 'linear', 'extrap');
                v_new = sqrt(ay_lim / curv);
                v_g = 0.6*v_g + 0.4*v_new;
            end
            v_apex(i) = v_g;
        end
    end
    
    v_acc(1) = v_start;
    for i = 1:N-1
        ds = s(i+1) - s(i); v_curr = v_acc(i);
        ay_req = v_curr^2 * abs(k(i));
        ay_max_v = interp1(Env.v, Env.ay_max, v_curr, 'linear', 'extrap');
        ax_max_v = interp1(Env.v, Env.ax_acc, v_curr, 'linear', 'extrap');
        if ay_req >= ay_max_v, ax_avail = 0; else, ax_avail = ax_max_v * sqrt(1 - (ay_req/ay_max_v)^2); end
        ax_acc_log(i) = ax_avail; 
        v_next = sqrt(v_curr^2 + 2 * ax_avail * ds);
        v_acc(i+1) = min(v_next, v_apex(i+1));
    end
    ax_acc_log(N) = 0; 
    
    v_brk(N) = v_apex(N);
    for i = N:-1:2
        ds = s(i) - s(i-1); v_curr = v_brk(i);
        ay_req = v_curr^2 * abs(k(i));
        ay_max_v = interp1(Env.v, Env.ay_max, v_curr, 'linear', 'extrap');
        ax_brk_v = abs(interp1(Env.v, Env.ax_brk, v_curr, 'linear', 'extrap'));
        if ay_req >= ay_max_v, ax_avail = 0; else, ax_avail = ax_brk_v * sqrt(1 - (ay_req/ay_max_v)^2); end
        ax_brk_log(i) = -ax_avail;
        v_prev = sqrt(v_curr^2 + 2 * ax_avail * ds);
        v_brk(i-1) = min(v_prev, v_apex(i-1));
    end
    ax_brk_log(1) = -abs(interp1(Env.v, Env.ax_brk, v_brk(1), 'linear', 'extrap'));
    
    v_final = zeros(N, 1); ax_final = zeros(N, 1);
    for i = 1:N
        [v_lim, idx] = min([v_acc(i), v_brk(i), v_apex(i)]);
        v_final(i) = v_lim;
        if idx == 1, ax_final(i) = ax_acc_log(i);
        elseif idx == 2, ax_final(i) = ax_brk_log(i);
        else, ax_final(i) = 0; end
    end
    
    R.s = s; R.v = v_final; R.ay_used = v_final.^2 .*k; R.ax_used = ax_final;
    R.laptime = sum(diff(s) ./ ((v_final(1:end-1) + v_final(2:end))/2));
end

% =========================================================================
% HELPER FUNCTION 4: CALCULATE ENERGY
% =========================================================================
function [E_kWh, P_kW] = Calculate_Energy(Results, m, CdA, rho, Cr)
    v = Results.v;      
    a = Results.ax_used; 
    
    F_aero = 0.5 * rho * CdA * v.^2;
    F_roll = Cr * m * 9.81;
    F_acc  = m * a;
    
    F_total = F_aero + F_roll + F_acc;
    P_mech_W = F_total .* v;
    
    eff_motor = 0.90; 
    P_elec_W = zeros(size(P_mech_W));
    
    for i = 1:length(P_mech_W)
        if P_mech_W(i) > 0
            P_elec_W(i) = P_mech_W(i) / eff_motor; 
        else
            P_elec_W(i) = 0; 
        end
    end
    
    P_kW = P_elec_W / 1000;
    
    ds = gradient(Results.s);
    dt = ds ./ max(v, 0.1); 
    
    E_joules = sum(P_elec_W .* dt);
    E_kWh    = E_joules / (3.6e6);
end
% % =========================================================================
% % STANDARD ENDURANCE DASHBOARD PLOTTING SCRIPT 
% % Ensure 'SimResults' is in your workspace before running this.
% % =========================================================================
% SimResults = Run_Endurance_Simulation("C:\Disk_E\TOR_controls\Latest\Kari_FB.csv", "GGV_Final_Data.mat", Par, aero_par);
% % 0. Extract Data and Set Limits
% v_kph = SimResults.v * 3.6;
% v_min_kph = min(v_kph);
% v_max_kph = max(v_kph);
% c_limits  = [v_min_kph, v_max_kph]; 
% 
% % Initialize Dark Mode Figure & Tiled Layout
% figure('Name', 'Endurance Analysis', 'Color', 'k', 'Position', [50 50 1400 900]);
% t = tiledlayout(2, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
% 
% % =========================================================================
% % --- PLOT 1: TRACK MAP (LEFT LARGE) ---
% nexttile([2 1]); % Spans 2 rows, 1 column
% hold on; axis equal; grid on;
% set(gca, 'Color', 'k', 'XColor', 'w', 'YColor', 'w', 'GridColor', 'w', 'GridAlpha', 0.2);
% 
% % Plot raw track edges
% plot(SimResults.Track.xl_edge, SimResults.Track.yl_edge, 'w-', 'LineWidth', 0.5);
% plot(SimResults.Track.xr_edge, SimResults.Track.yr_edge, 'w-', 'LineWidth', 0.5);
% 
% % Plot Colored Racing Line
% z = zeros(size(SimResults.Path.X));
% surface([SimResults.Path.X'; SimResults.Path.X'], ...
%         [SimResults.Path.Y'; SimResults.Path.Y'], ...
%         [z'; z'], [v_kph'; v_kph'], ...
%         'FaceColor', 'no', 'EdgeColor', 'interp', 'LineWidth', 3);
% 
% colormap(gca, 'turbo'); 
% clim(c_limits);         
% c = colorbar('southoutside'); 
% c.Color = 'w'; c.Label.Color = 'w'; c.Label.String = 'Speed [km/h]';
% title(sprintf('Endurance: %d Laps', SimResults.Num_Laps), 'Color', 'w', 'FontSize', 12);
% xlabel('X [m]', 'Color', 'w'); ylabel('Y [m]', 'Color', 'w');
% 
% % =========================================================================
% % --- PLOT 2: SPEED TRACE (TOP MIDDLE) ---
% nexttile;
% plot(SimResults.s, v_kph, 'LineWidth', 2, 'Color', '#4DBEEE'); 
% grid on; xlim([0 max(SimResults.s)]);
% set(gca, 'Color', 'k', 'XColor', 'w', 'YColor', 'w', 'GridColor', 'w', 'GridAlpha', 0.2);
% ylabel('Speed [km/h]', 'Color', 'w'); 
% title(sprintf('Flying Lap: %.2fs', SimResults.LapTime_FL), 'Color', 'w');
% 
% % =========================================================================
% % --- PLOT 3: POWER TRACE (TOP RIGHT) ---
% nexttile;
% area(SimResults.s, SimResults.Power_kW, 'FaceColor', '#77AC30', 'FaceAlpha', 0.5, 'EdgeColor', 'w');
% grid on; xlim([0 max(SimResults.s)]);
% set(gca, 'Color', 'k', 'XColor', 'w', 'YColor', 'w', 'GridColor', 'w', 'GridAlpha', 0.2);
% ylabel('Power [kW]', 'Color', 'w'); 
% title(sprintf('Total Energy: %.2f kWh', SimResults.Total_Energy_kWh), 'Color', 'w');
% xlabel('Distance [m]', 'Color', 'w');
% 
% % =========================================================================
% % --- PLOT 4: STEERING TRACE (BOTTOM MIDDLE) ---
% nexttile;
% plot(SimResults.s, SimResults.steer_deg, 'LineWidth', 1.5, 'Color', '#D95319');
% grid on; xlim([0 max(SimResults.s)]);
% set(gca, 'Color', 'k', 'XColor', 'w', 'YColor', 'w', 'GridColor', 'w', 'GridAlpha', 0.2);
% ylabel('Steer [deg]', 'Color', 'w');
% xlabel('Distance [m]', 'Color', 'w'); 
% 
% % =========================================================================
% % --- PLOT 5: G-G USAGE DIAGRAM (BOTTOM RIGHT) ---
% nexttile;
% hold on; grid on; axis equal;
% set(gca, 'Color', 'k', 'XColor', 'w', 'YColor', 'w', 'GridColor', 'w', 'GridAlpha', 0.2);
% 
% scatter(SimResults.ay_used/9.81, SimResults.ax_used/9.81, 15, v_kph, 'filled');
% 
% xlabel('Lat G', 'Color', 'w'); ylabel('Long G', 'Color', 'w');
% title('G-G Diagram', 'Color', 'w');
% xlim([-3 3]); ylim([-3 3]); 
% colormap(gca, 'turbo'); clim(c_limits);