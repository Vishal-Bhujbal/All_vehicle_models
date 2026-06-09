function SimResults = Lapsim_final_for_app(track_file, ggv_file, Par, TireData)
% RUN_LAP_SIMULATION Master Lap Time Simulation 3D
% Designed for background execution (e.g., MATLAB App Designer)
%
% INPUTS:
%   track_file - String path to the track .csv file
%   ggv_file   - String path to the GGV .mat file
%   Par        - Struct containing vehicle parameters (requires Par.TW)
%   TireData   - (Optional) Struct containing tire scaling data.
%
% OUTPUT:
%   SimResults - Struct containing all data needed for visualization.

    % --- 0. HANDLE OPTIONAL INPUTS ---
    if nargin < 4
        TireData = []; 
    end

    % --- 1. CHECK & LOAD INPUTS ---
    if ~exist(track_file, 'file')
        error('Track file "%s" not found.', track_file);
    end
    
    if ~exist(ggv_file, 'file')
        potential_files = {'GGV_Final_Data.mat', 'GGV_Sensor_Data.mat', 'GGV_Safe_Data.mat', 'GGV_Map.mat'};
        found = false;
        for k = 1:length(potential_files)
            if exist(potential_files{k}, 'file')
                ggv_file = potential_files{k};
                found = true;
                break;
            end
        end
        if ~found
            error('No GGV files found. Please ensure the 3D GGV Script has been run.');
        end
    end
    
    loaded_data = load(ggv_file, 'results'); 
    if ~isfield(loaded_data, 'results')
        error('File "%s" loaded, but it lacks the "results" structure.', ggv_file);
    end
    ggv_raw = loaded_data.results;

    % --- 2. OPTIMIZE RACING LINE ---
    [path_s, path_k, path_X, path_Y] = Optimize_Racing_Line(track_file);

    % Calculate Raw Track Edges for Plotting
    data = readmatrix(track_file);
    xc = data(:,1); yc = data(:,2); wr = data(:,3); wl = data(:,4);
    dx_t = gradient(xc); dy_t = gradient(yc); angle_t = atan2(dy_t, dx_t);
    xl_edge = xc + wl.*cos(angle_t+pi/2); yl_edge = yc + wl.*sin(angle_t+pi/2);
    xr_edge = xc - wr.*cos(angle_t+pi/2); yr_edge = yc - wr.*sin(angle_t+pi/2);

    % --- 3. PRE-PROCESS GGV MAP ---
    GGV_Env = Process_GGV_Envelope(ggv_raw);

    % --- 4. RUN 3-PASS SOLVER ---
    Results = Solve_3Pass(path_s, path_k, GGV_Env);
    
    % Calculate Steering Angle using Par struct
    Results.steer_deg = rad2deg(atan(Par.TW * path_k));

    % --- 5. EXTRACT TIRE LIMITS FOR PLOTTING ---
    try
        if isempty(TireData)
            mu_x_peak = 1.6; mu_y_peak = 1.6;
        else
            mu_x_peak = TireData.PDX1 * TireData.Scales.LMUX;
            mu_y_peak = TireData.PDY1 * TireData.Scales.LMUY;
        end
    catch
        mu_x_peak = 1.6; mu_y_peak = 1.6;
    end

    % --- 6. PACK ALL RESULTS INTO OUTPUT STRUCT ---
    SimResults.LapTime   = Results.laptime;
    SimResults.s         = Results.s;
    SimResults.v         = Results.v;
    SimResults.ay_used   = Results.ay_used;
    SimResults.ax_used   = Results.ax_used;
    SimResults.steer_deg = Results.steer_deg;
    
    SimResults.Path.X = path_X;
    SimResults.Path.Y = path_Y;
    SimResults.Path.k = path_k;
    SimResults.Track.xl_edge = xl_edge;
    SimResults.Track.yl_edge = yl_edge;
    SimResults.Track.xr_edge = xr_edge;
    SimResults.Track.yr_edge = yr_edge;
    
    SimResults.GGV_Env    = GGV_Env;
    SimResults.TireLimits = struct('mu_x_peak', mu_x_peak, 'mu_y_peak', mu_y_peak);

end

% =========================================================================
% HELPER FUNCTION 1: RACING LINE OPTIMIZER 
% =========================================================================
function [s_final, k_final, x_final, y_final] = Optimize_Racing_Line(filename)
    data = readmatrix(filename);
    xc = data(:,1); yc = data(:,2); wr = data(:,3); wl = data(:,4);
    
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
            
            if dist > max_w
                scale = max_w / dist; 
                x(i) = xc(i) + dx * scale; y(i) = yc(i) + dy * scale;
            else
                x(i) = xt(i); y(i) = yt(i); 
            end
        end
    end
    
    dx = gradient(x); dy = gradient(y); ddx = gradient(dx); ddy = gradient(dy);
    k_raw = (dx .* ddy - dy .* ddx) ./ ((dx.^2 + dy.^2).^1.5);
    k_final = smoothdata(k_raw, 'gaussian', 15); 
    
    ds = sqrt(dx.^2 + dy.^2); 
    s_final = [0; cumsum(ds(2:end))];
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
    Env.ay_max = Env.ay_max(idx);
    Env.ax_acc = Env.ax_acc(idx);
    Env.ax_brk = Env.ax_brk(idx);
end

% =========================================================================
% HELPER FUNCTION 3: 3-PASS SOLVER 
% =========================================================================
function R = Solve_3Pass(s, k, Env)
    N = length(s);
    v_apex = zeros(N, 1);
    v_acc = zeros(N, 1); ax_acc_log = zeros(N, 1);
    v_brk = zeros(N, 1); ax_brk_log = zeros(N, 1);
    
    % --- PASS 1: APEX SPEED ---
    for i = 1:N
        curv = abs(k(i));
        if curv < 1e-4
            v_apex(i) = max(Env.v); 
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
    
    % --- PASS 2: ACCELERATION (Forward) ---
    v_acc(1) = 0;      
    for i = 1:N-1
        ds = s(i+1) - s(i); 
        v_curr = v_acc(i);
        ay_req = v_curr^2 * abs(k(i));
        
        ay_max_v = interp1(Env.v, Env.ay_max, v_curr, 'linear', 'extrap');
        ax_max_v = interp1(Env.v, Env.ax_acc, v_curr, 'linear', 'extrap');
        
        if ay_req >= ay_max_v
            ax_avail = 0; 
        else
            ax_avail = ax_max_v * sqrt(1 - (ay_req/ay_max_v)^2);
        end
        ax_acc_log(i) = ax_avail; 
        
        v_next = sqrt(v_curr^2 + 2 * ax_avail * ds);
        v_acc(i+1) = min(v_next, v_apex(i+1));
    end
    ax_acc_log(N) = 0; 
    
    % --- PASS 3: BRAKING (Backward) ---
    v_brk(N) = v_apex(N);
    for i = N:-1:2
        ds = s(i) - s(i-1); 
        v_curr = v_brk(i);
        ay_req = v_curr^2 * abs(k(i));
        
        ay_max_v = interp1(Env.v, Env.ay_max, v_curr, 'linear', 'extrap');
        ax_brk_v = abs(interp1(Env.v, Env.ax_brk, v_curr, 'linear', 'extrap'));
        
        if ay_req >= ay_max_v
            ax_avail = 0;
        else
            ax_avail = ax_brk_v * sqrt(1 - (ay_req/ay_max_v)^2);
        end
        ax_brk_log(i) = -ax_avail; 
        
        v_prev = sqrt(v_curr^2 + 2 * ax_avail * ds);
        v_brk(i-1) = min(v_prev, v_apex(i-1));
    end
    ax_brk_log(1) = -abs(interp1(Env.v, Env.ax_brk, v_brk(1), 'linear', 'extrap'));
    
    % --- FINAL COMBINATION ---
    v_final = zeros(N, 1);
    ax_final = zeros(N, 1);
    
    for i = 1:N
        [v_lim, idx] = min([v_acc(i), v_brk(i), v_apex(i)]);
        v_final(i) = v_lim;
        
        if idx == 1 
            ax_final(i) = ax_acc_log(i);
        elseif idx == 2
            ax_final(i) = ax_brk_log(i);
        else
            ax_final(i) = 0; 
        end
    end
    
    R.s = s; 
    R.v = v_final;
    R.ay_used = v_final.^2 .* k;
    R.ax_used = ax_final;
    R.laptime = sum(diff(s) ./ ((v_final(1:end-1) + v_final(2:end))/2));
end

% % =========================================================================
% % DASHBOARD PLOTTING SCRIPT
% % Ensure 'SimResults' is in your workspace before running this.
% % =========================================================================
% SimResults = Run_Lap_Simulation("C:\Disk_E\TOR_controls\Latest\Kari_FB.csv", "GGV_Final_Data.mat", Par, TireData);
% % 0. Define Global Color Limits (CRITICAL FOR CONSISTENCY)
% v_kph = SimResults.v * 3.6;
% v_min_kph = min(v_kph);
% v_max_kph = max(v_kph);
% c_limits  = [v_min_kph, v_max_kph]; 
% 
% % Initialize Dark Mode Figure & Tiled Layout
% figure('Name', 'Race Analysis', 'Color', 'k', 'Position', [100 100 1400 900]);
% t = tiledlayout(2, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
% 
% % =========================================================================
% % --- PLOT 1: TRACK MAP (LEFT LARGE) ---
% nexttile(1, [2 1]); % Spans 2 rows, 1 column
% hold on; axis equal; grid on;
% set(gca, 'Color', 'k', 'XColor', 'w', 'YColor', 'w', 'GridColor', 'w', 'GridAlpha', 0.2);
% 
% % Plot raw track edges 
% plot(SimResults.Track.xl_edge, SimResults.Track.yl_edge, 'w-', 'LineWidth', 0.5);
% plot(SimResults.Track.xr_edge, SimResults.Track.yr_edge, 'w-', 'LineWidth', 0.5);
% 
% % Plot Colored Racing Line using the 3D surface trick
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
% title('Optimal Racing Line', 'Color', 'w', 'FontSize', 14);
% xlabel('X [m]'); ylabel('Y [m]');
% 
% % =========================================================================
% % --- PLOT 2: SPEED TRACE (TOP MIDDLE) ---
% nexttile(2);
% plot(SimResults.s, v_kph, 'LineWidth', 2, 'Color', '#4DBEEE'); 
% grid on; xlim([0 max(SimResults.s)]);
% set(gca, 'Color', 'k', 'XColor', 'w', 'YColor', 'w', 'GridColor', 'w', 'GridAlpha', 0.2);
% ylabel('Speed [km/h]', 'Color', 'w', 'FontWeight', 'bold'); 
% xlabel('Distance [m]', 'Color', 'w'); 
% title(sprintf('Lap Time: %.2fs', SimResults.LapTime), 'Color', 'w');
% 
% % =========================================================================
% % --- PLOT 3: GGV USAGE vs TIRE LIMITS (TOP RIGHT) ---
% nexttile(3);
% hold on; grid on; axis equal;
% set(gca, 'Color', 'k', 'XColor', 'w', 'YColor', 'w', 'GridColor', 'w', 'GridAlpha', 0.2);
% 
% % Generate Theoretical Tire Limit Ellipse
% theta = linspace(0, 2*pi, 100);
% tire_lim_lat  = SimResults.TireLimits.mu_y_peak * cos(theta); 
% tire_lim_long = SimResults.TireLimits.mu_x_peak * sin(theta); 
% 
% % Plot Elements
% plot(tire_lim_lat, tire_lim_long, 'm--', 'LineWidth', 2, 'DisplayName', 'Tire Potential');
% scatter(SimResults.ay_used/9.81, SimResults.ax_used/9.81, 20, v_kph, 'filled', 'DisplayName', 'Lap Data');
% 
% xlabel('Lat G', 'Color', 'w'); ylabel('Long G', 'Color', 'w');
% title('Transient Usage vs Tire Limit', 'Color', 'w');
% legend('Location', 'best', 'TextColor', 'w', 'Color', 'none', 'EdgeColor', 'none');
% xlim([-3.0 3.0]); ylim([-3.0 3.0]); 
% colormap(gca, 'turbo'); clim(c_limits);
% 
% % =========================================================================
% % --- PLOT 4: STEERING TRACE (BOTTOM MIDDLE) ---
% nexttile(5);
% plot(SimResults.s, SimResults.steer_deg, 'LineWidth', 1.5, 'Color', '#D95319');
% grid on; xlim([0 max(SimResults.s)]);
% set(gca, 'Color', 'k', 'XColor', 'w', 'YColor', 'w', 'GridColor', 'w', 'GridAlpha', 0.2);
% ylabel('Steer [deg]', 'Color', 'w', 'FontWeight', 'bold');
% xlabel('Distance [m]', 'Color', 'w'); 
% title('Steering Input', 'Color', 'w');
% 
% % =========================================================================
% % --- PLOT 5: 3D PERFORMANCE TUBE (BOTTOM RIGHT) ---
% nexttile(6);
% hold on; grid on;
% 
% % Create Mesh from Envelope
% Env = SimResults.GGV_Env; 
% [V_mesh, Theta_mesh] = meshgrid(linspace(min(Env.v), max(Env.v), 40), linspace(0, 2*pi, 40));
% Ax_surf = zeros(size(V_mesh));
% Ay_surf = zeros(size(V_mesh));
% 
% % Build the "Tube"
% for r = 1:size(V_mesh, 1)
%     for c = 1:size(V_mesh, 2)
%         v_val = V_mesh(r,c);
%         th = Theta_mesh(r,c);
% 
%         ay_lim = interp1(Env.v, Env.ay_max, v_val, 'linear', 'extrap') / 9.81;
%         ax_acc = interp1(Env.v, Env.ax_acc, v_val, 'linear', 'extrap') / 9.81;
%         ax_brk = abs(interp1(Env.v, Env.ax_brk, v_val, 'linear', 'extrap') / 9.81);
% 
%         if sin(th) >= 0, r_long = ax_acc; else, r_long = ax_brk; end
%         rad = (r_long * ay_lim) / sqrt((ay_lim * sin(th))^2 + (r_long * cos(th))^2);
% 
%         Ay_surf(r,c) = rad * cos(th); % Lateral
%         Ax_surf(r,c) = rad * sin(th); % Longitudinal
%     end
% end
% 
% % Plot Surface and Actual Lap Data
% surf(Ay_surf, Ax_surf, V_mesh * 3.6, 'FaceAlpha', 0.1, 'EdgeColor', 'none', 'FaceColor', 'w');
% scatter3(SimResults.ay_used/9.81, SimResults.ax_used/9.81, v_kph, 25, v_kph, 'filled');
% view(135, 30);
% set(gca, 'Color', 'k', 'XColor', 'w', 'YColor', 'w', 'ZColor', 'w', 'GridColor', 'w', 'GridAlpha', 0.2);
% xlabel('Lat G', 'Color', 'w'); ylabel('Long G', 'Color', 'w'); zlabel('Speed [km/h]', 'Color', 'w');
% title('3D Performance Envelope', 'Color', 'w');
% colormap(gca, 'turbo'); clim(c_limits);