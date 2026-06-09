function interactive_vehicle_viewer_neon
    clc;
    close all;

    % =========================================================================
    % LOAD DATA
    % =========================================================================
    out = evalin('base','out');

    % =========================================================================
    % TIME EXTRACTION
    % =========================================================================
    try
        time = squeeze(out.tout);
    catch
        try
            time = squeeze(out.time);
        catch
            time = squeeze(out.x.Time);
        end
    end

    % =========================================================================
    % DATA EXTRACTION
    % =========================================================================
    x_data = squeeze(-out.y.Data);
    y_data = squeeze(out.x.Data);
    yaw_rate  = squeeze(out.Yaw_rate.Data);
    
    Fx_fl = squeeze(out.Longitudinal_force_fl.Data);
    Fy_fl = squeeze(out.Lateral_force_fl.Data);
    Fx_fr = squeeze(out.Longitudinal_force_fr.Data);
    Fy_fr = squeeze(out.Lateral_force_fr.Data);
    Fx_rl = squeeze(out.Longitudinal_force_rl.Data);
    Fy_rl = squeeze(out.Lateral_force_rl.Data);
    Fx_rr = squeeze(out.Longitudinal_force_rr.Data);
    Fy_rr = squeeze(out.Lateral_force_rr.Data);
    
    N = length(time);

    % =========================================================================
    % COLOR PALETTE (RGB 0-1 SCALE FOR MATLAB STABILITY)
    % =========================================================================
    bg_color     = [0.00, 0.00, 0.00];  % Pure Pitch Black
    ax_color     = [0.00, 0.00, 0.00];  % Pure Pitch Black
    grid_col     = [0.15, 0.15, 0.15];  % Dark grey grid lines
    text_col     = [0.90, 0.90, 0.90];  % Crisp light grey text
    
    vibrant_blue = [0.00, 1.00, 1.00];  % Neon Sky Blue (Trajectory & Accents)
    vibrant_red  = [1.00, 0.10, 0.10];  % Neon Red (Force Vectors)
    car_fill     = [1.00, 1.00, 1.00];  % Pure White Car

    % =========================================================================
    % MAIN FIGURE
    % =========================================================================
    fig = figure( ...
        'Name', 'FSAE Telemetry Viewer', ...
        'Color', bg_color, ...
        'Renderer', 'opengl', ...
        'Position', [100 100 1500 750], ...
        'MenuBar', 'none', ...
        'ToolBar', 'none');
    set(fig, 'GraphicsSmoothing', 'on'); 

    % =========================================================================
    % AXES 1: TRAJECTORY
    % =========================================================================
    ax1 = subplot(1,2,1);
    hold(ax1, 'on'); grid(ax1, 'on'); axis(ax1, 'equal');
    set(ax1, 'Color', ax_color, 'XColor', text_col, 'YColor', text_col, ...
             'GridColor', grid_col, 'GridAlpha', 1, 'LineWidth', 1);
    
    % Trajectory Line
    plot(ax1, x_data, y_data, 'Color', vibrant_blue, 'LineWidth', 2);
    
    marker = plot(ax1, x_data(1), y_data(1), 'o', ...
        'MarkerEdgeColor', vibrant_blue, 'MarkerFaceColor', [1 1 1], ...
        'MarkerSize', 8, 'LineWidth', 2);
    
    title(ax1, 'GLOBAL TRAJECTORY', 'Color', text_col, 'FontSize', 12, 'FontWeight', 'bold');
    xlabel(ax1, 'X Position (m)'); ylabel(ax1, 'Y Position (m)');

    % =========================================================================
    % AXES 2: VEHICLE DYNAMICS VIEW
    % =========================================================================
    ax2 = subplot(1,2,2);
    hold(ax2, 'on'); grid(ax2, 'on'); axis(ax2, 'equal');
    set(ax2, 'Color', ax_color, 'XColor', text_col, 'YColor', text_col, ...
             'GridColor', grid_col, 'GridAlpha', 1, 'XDir', 'reverse', 'LineWidth', 1);
    
    % Vehicle Dimensions
    L = 1.53;
    W = 1.20;
    
    % --- Draw Formula Style Car ---
    % Chassis / Cockpit
    chassis_x = [-0.25, -0.25, -0.1, 0.1, 0.25, 0.25];
    chassis_y = [-L/2, L/2, L/2+0.4, L/2+0.4, L/2, -L/2];
    fill(ax2, chassis_x, chassis_y, car_fill, 'EdgeColor', vibrant_blue, 'LineWidth', 2);
    
    % Front Wing
    fw_x = [-0.55, -0.55, 0.55, 0.55];
    fw_y = [L/2+0.3, L/2+0.5, L/2+0.5, L/2+0.3];
    fill(ax2, fw_x, fw_y, car_fill, 'EdgeColor', vibrant_blue, 'LineWidth', 2);
    
    % Rear Wing
    rw_x = [-0.45, -0.45, 0.45, 0.45];
    rw_y = [-L/2-0.25, -L/2, -L/2, -L/2-0.25];
    fill(ax2, rw_x, rw_y, car_fill, 'EdgeColor', vibrant_blue, 'LineWidth', 2);

    % Tires (Open Wheel)
    wheel_x = [ W/2 -W/2  W/2 -W/2];
    wheel_y = [ L/2  L/2 -L/2 -L/2];
    t_width = 0.15; t_length = 0.4;
    
    for i = 1:4
        tx = [wheel_x(i)-t_width, wheel_x(i)-t_width, wheel_x(i)+t_width, wheel_x(i)+t_width];
        ty = [wheel_y(i)-t_length, wheel_y(i)+t_length, wheel_y(i)+t_length, wheel_y(i)-t_length];
        fill(ax2, tx, ty, [0.1 0.1 0.1], 'EdgeColor', [0.4 0.4 0.4], 'LineWidth', 1.5);
    end

    % =========================================================================
    % FORCE VECTORS (Neon Red)
    % =========================================================================
    force_scale = 0.0015;
    force_lines = gobjects(4,1);
    for i = 1:4
        force_lines(i) = line(ax2, [wheel_x(i) wheel_x(i)], [wheel_y(i) wheel_y(i)], ...
            'Color', vibrant_red, 'LineWidth', 3);
    end

    % =========================================================================
    % TELEMETRY HUD TEXT (Cleanly docked in the top corner)
    % =========================================================================
    % Using XDir reverse means visually left is positive X.
    % Anchoring at X=1.8 (left side), Y=1.8 (top side)
    yaw_text = text(ax2, 1.8, 1.8, '', ...
        'Color', vibrant_blue, ...
        'BackgroundColor', [0 0 0], ... % Match axes background
        'EdgeColor', vibrant_blue, ...
        'Margin', 8, ...
        'FontName', 'Courier', ...
        'FontWeight', 'bold', ...
        'FontSize', 12);
    
    xlim(ax2, [-2 2]); ylim(ax2, [-2.2 2.2]);
    title(ax2, 'WHEEL FORCES & DYNAMICS', 'Color', text_col, 'FontSize', 12, 'FontWeight', 'bold');
    xlabel(ax2, 'Lateral Axis (m)'); ylabel(ax2, 'Longitudinal Axis (m)');

    % =========================================================================
    % TIMELINE AXES & UI
    % =========================================================================
    timeline_ax = axes('Position', [0.13 0.08 0.60 0.05]);
    hold(timeline_ax, 'on');
    xlim(timeline_ax, [time(1) time(end)]); ylim(timeline_ax, [0 1]);
    timeline_ax.YTick = [];
    set(timeline_ax, 'Color', ax_color, 'XColor', text_col, 'YColor', 'none', ...
        'GridColor', grid_col, 'GridAlpha', 1, 'LineWidth', 1);
    
    plot(timeline_ax, time, 0.5*ones(size(time)), 'Color', grid_col, 'LineWidth', 4);
    timeline_marker = plot(timeline_ax, time(1), 0.5, 'o', ...
        'MarkerFaceColor', vibrant_blue, 'MarkerEdgeColor', [1 1 1], 'MarkerSize', 10, 'LineWidth', 2);
    xlabel(timeline_ax, 'Simulation Time (s)', 'Color', text_col, 'FontWeight', 'bold');

    % =========================================================================
    % BUTTONS
    % =========================================================================
    play_button = uicontrol('Style', 'togglebutton', 'String', '▶ PLAY', ...
        'Units', 'normalized', 'Position', [0.77 0.07 0.08 0.06], ...
        'FontWeight', 'bold', 'FontSize', 11, 'ForegroundColor', [1 1 1], ...
        'BackgroundColor', [0.2 0.5 0.9], 'Callback', @play_callback);
        
    speed_menu = uicontrol('Style', 'popupmenu', 'String', {'1x','2x','5x','10x','20x','50x'}, ...
        'Value', 4, 'Units', 'normalized', 'Position', [0.86 0.07 0.06 0.06], ...
        'ForegroundColor', [1 1 1], 'BackgroundColor', [0.15 0.15 0.15], 'FontWeight', 'bold');

    current_idx = 1;
    update_frame(1);

    % =========================================================================
    % KEYBOARD SUPPORT
    % =========================================================================
    set(fig, 'WindowKeyPressFcn', @key_press);
    function key_press(~, event)
        switch event.Key
            case 'rightarrow'
                current_idx = min(current_idx+10, N);
                update_frame(current_idx);
            case 'leftarrow'
                current_idx = max(current_idx-10, 1);
                update_frame(current_idx);
        end
    end

    % =========================================================================
    % PLAYBACK FUNCTION
    % =========================================================================
    function play_callback(src, ~)
        speeds = [1 2 5 10 20 50];
        while get(src, 'Value')
            speed = speeds(get(speed_menu, 'Value'));
            current_idx = current_idx + speed;
            if current_idx > N
                current_idx = N;
                set(src, 'Value', 0);
                src.String = '▶ PLAY';
                break;
            end
            src.String = '⏸ PAUSE';
            update_frame(current_idx);
        end
        if ~get(src, 'Value')
            src.String = '▶ PLAY';
        end
    end

    % =========================================================================
    % ULTRA FAST UPDATE FUNCTION
    % =========================================================================
    function update_frame(idx)
        current_idx = idx;
        
        marker.XData = x_data(idx);
        marker.YData = y_data(idx);
        timeline_marker.XData = time(idx);
        
        Fx = [Fx_fl(idx) Fx_fr(idx) Fx_rl(idx) Fx_rr(idx)];
        Fy = [Fy_fl(idx) Fy_fr(idx) Fy_rl(idx) Fy_rr(idx)];
        
        for k = 1:4
            x0 = wheel_x(k);
            y0 = wheel_y(k);
            force_lines(k).XData = [x0 x0 + Fy(k)*force_scale];
            force_lines(k).YData = [y0 y0 + Fx(k)*force_scale];
        end
        
        yaw_text.String = sprintf(' TIME : %05.2f s \n YAW  : %05.2f rad/s', ...
             time(idx), yaw_rate(idx));
         
        drawnow nocallbacks
    end
end