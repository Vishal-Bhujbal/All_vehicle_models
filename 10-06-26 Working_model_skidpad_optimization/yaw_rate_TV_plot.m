% ---------------------------------------------------------
% Yaw Rate Comparison: Torque Vectoring vs Baseline
% ---------------------------------------------------------

% Create a clean figure with a white background
figure('Name', 'Yaw Rate Comparison', 'Color', 'w');
hold on;
grid on;
box on;

% Plot 1: Target Yaw Rate (Black, Dash-Dot)
% Makes the reference ideal clear and distinct from the physical responses
plot(out.tout, out.Target_yaw_rate.Data, ...
    'Color', 'k', 'LineStyle', '-.', 'LineWidth', 2.0, ...
    'DisplayName', 'Target Yaw Rate');

% Plot 2: With Torque Vectoring (High-Contrast Blue, Solid)
plot(yaw_rate_tv.Time, yaw_rate_tv.Data, ...
    'Color', '#0072BD', 'LineStyle', '-', 'LineWidth', 2.5, ...
    'DisplayName', 'Active Torque Vectoring');

% Plot 3: Without Torque Vectoring (Vermillion/Dark Orange, Dashed)
% Changed hex code to #D55E00 for better contrast against the blue
plot(yaw_rate_no_tv.Time, yaw_rate_no_tv.Data, ...
    'Color', '#D55E00', 'LineStyle', '--', 'LineWidth', 2.5, ...
    'DisplayName', 'Baseline (No TV)');

% ---------------------------------------------------------
% Plot Formatting
% ---------------------------------------------------------
title('Vehicle Yaw Rate Response Comparison', 'FontSize', 12, 'FontWeight', 'bold');
xlabel('Time (s)', 'FontSize', 11);
ylabel('Yaw Rate (rad/s)', 'FontSize', 11); 

% Add a legend in the best available position
legend('Location', 'best', 'FontSize', 10);

hold off;