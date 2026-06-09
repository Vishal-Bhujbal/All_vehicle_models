%% 1. DATA EXTRACTION
% Extract arrays from Simulink 'out' objects for easier reading
x_path     = out.path_X_ts.Data;
y_path     = out.path_Y_ts.Data;
theta_path = out.path_Theta_ts.Data; % Centerline heading

x_act      = out.x.Data;
y_act      = out.y.Data;
psi_act    = out.psi.Data;           % Actual vehicle heading

%% 2. ERROR COMPUTATION
% N_act is just the total number of recorded samples
N_act = length(x_act); 

% --- Compute Heading Error ---
% Difference between actual and desired heading
theta_e_rad = psi_act - theta_path;
% Wrap angles to [-pi, pi] to prevent large jumps (e.g., 359 deg to 1 deg)
theta_e_rad = atan2(sin(theta_e_rad), cos(theta_e_rad)); 
theta_e_deg = theta_e_rad * (180/pi); % Convert to degrees for plotting

% --- Compute Cross-Track Error (e_ct) ---
% Lateral distance from the actual position to the path centerline
dx = x_act - x_path;
dy = y_act - y_path;
% Project position error onto the path's normal vector
e_ct = -dx .* sin(theta_path) + dy .* cos(theta_path);

%% 3. VISUALIZATION
fig = figure('Name', 'Skidpad Tracking Error Visualization', 'Color', 'k', 'Units', 'normalized', 'Position', [0.1 0.1 0.8 0.8]);

% --- Subplot 1: Track Map colored by Cross-Track Error ---
ax1 = subplot(2, 2, [1, 3]);
hold(ax1, 'on'); grid(ax1, 'on'); axis(ax1, 'equal');
set(ax1, 'Color', [0.1 0.1 0.1], 'XColor', 'w', 'YColor', 'w');

% Plot Skidpad Centerline
scatter(ax1, x_path, y_path, 20, 'MarkerFaceColor', [0 1 1], 'MarkerEdgeColor', [0 0.4 1], 'MarkerFaceAlpha', 0.8, 'LineWidth', 1.2, 'DisplayName', 'Centerline');

% Plot Actual Path colored by Cross Track Error (e_ct)
scatter(ax1, x_act, y_act, 20, e_ct, 'filled', 'MarkerFaceAlpha', 0.8, 'DisplayName', 'Actual Path');

colormap(ax1, 'jet');
cb1 = colorbar(ax1); 
cb1.Color = 'w';
title(cb1, 'CTE [m]', 'Color', 'w');

% Define color limits based on max error
max_cte = max(abs(e_ct));
if max_cte == 0; max_cte = 1; end % Prevent limits error if perfectly zero
clim(ax1, [-max_cte max_cte]); % Note: Use caxis(ax1, [-max_cte max_cte]) if on older MATLAB versions

xlabel(ax1, 'X [m]'); ylabel(ax1, 'Y [m]');
title(ax1, 'Track Map: Actual vs Skidpad Centerline', 'Color', 'w');
legend(ax1,  'TextColor', 'w', 'Color', 'none', 'Location', 'best');

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