%% Skidpad Error Computation & Visualization Script
% Assumes the following variables are already in your workspace:
% OPTIMAL TRACK: ctrl_s, path_X, path_Y, path_Theta (1538 points)
% ACTUAL PATH: out.x, out.y, out.psi (Timeseries data)

%% 1. INITIALIZE ERROR ARRAYS
N_act = length(out.x);
e_ct = zeros(N_act, 1);
theta_e_rad = zeros(N_act, 1);

%% 2. COMPUTE ERRORS
fprintf('Computing errors for %d telemetry points against skidpad centerline...\n', N_act);

for i = 1:N_act
    % Get current actual vehicle state from timeseries
    xa = out.x(i);
    ya = out.y(i);
    psia = out.psi(i);
    
    % Find the closest index on the optimal skidpad path
    % Using squared distance to avoid slow sqrt() in the loop
    dist_sq = (path_X - xa).^2 + (path_Y - ya).^2;
    [~, idx_closest] = min(dist_sq);
    
    % Retrieve optimal states at that closest point
    xo = path_X(idx_closest);
    yo = path_Y(idx_closest);
    psio = path_Theta(idx_closest);
    
    % Calculate Cross-Track Error (e_ct)
    % Positive means car is to the left of the path, negative means right
    e_ct(i) = -(xa - xo)*sin(psio) + (ya - yo)*cos(psio);
    
    % Calculate Heading Error (theta_e)
    % Difference between optimal and actual heading, wrapped to [-pi, pi]
    theta_e_rad(i) = atan2(sin(psio - psia), cos(psio - psia));
end

% Convert heading error to degrees for plotting
theta_e_deg = rad2deg(theta_e_rad);

%% 3. VISUALIZATION
fig = figure('Name', 'Skidpad Tracking Error Visualization', 'Color', 'k', 'Units', 'normalized', 'Position', [0.1 0.1 0.8 0.8]);

% --- Subplot 1: Track Map colored by Cross-Track Error ---
ax1 = subplot(2, 2, [1, 3]);
hold(ax1, 'on'); grid(ax1, 'on'); axis(ax1, 'equal');
set(ax1, 'Color', [0.1 0.1 0.1], 'XColor', 'w', 'YColor', 'w');

% Plot Skidpad Centerline
plot(ax1, path_X, path_Y, 'w--', 'LineWidth', 1.5, 'DisplayName', 'Skidpad Centerline');

% Plot Actual Path colored by Cross Track Error
scatter(ax1, out.x, out.y, 15, e_ct, 'filled', 'DisplayName', 'Actual Path (Color = CTE)');
colormap(ax1, 'jet');
cb1 = colorbar(ax1); cb1.Color = 'w';
title(cb1, 'CTE [m]', 'Color', 'w');
max_cte = max(abs(e_ct));
if max_cte == 0; max_cte = 1; end % Prevent caxis error if perfectly zero
caxis(ax1, [-max_cte max_cte]);

xlabel(ax1, 'X [m]'); ylabel(ax1, 'Y [m]');
title(ax1, 'Track Map: Actual vs Skidpad Centerline', 'Color', 'w');
legend(ax1, 'TextColor', 'w', 'Color', 'none', 'Location', 'best');

% --- Subplot 2: Cross Track Error vs Data Points ---
ax2 = subplot(2, 2, 2);
hold(ax2, 'on'); grid(ax2, 'on');
set(ax2, 'Color', [0.1 0.1 0.1], 'XColor', 'w', 'YColor', 'w');
plot(ax2, 1:N_act, e_ct, 'y', 'LineWidth', 1.5);
plot(ax2, [1 N_act], [0 0], 'w--', 'LineWidth', 1); % Zero reference line
ylabel(ax2, 'Cross Track Error [m]');
title(ax2, 'Cross Track Error vs Samples', 'Color', 'w');

% --- Subplot 3: Heading Error vs Data Points ---
ax3 = subplot(2, 2, 4);
hold(ax3, 'on'); grid(ax3, 'on');
set(ax3, 'Color', [0.1 0.1 0.1], 'XColor', 'w', 'YColor', 'w');
plot(ax3, 1:N_act, theta_e_deg, 'm', 'LineWidth', 1.5);
plot(ax3, [1 N_act], [0 0], 'w--', 'LineWidth', 1); % Zero reference line
xlabel(ax3, 'Sample Index');
ylabel(ax3, 'Heading Error [deg]');
title(ax3, 'Heading Error vs Samples', 'Color', 'w');

fprintf('Computation and plotting complete.\n');