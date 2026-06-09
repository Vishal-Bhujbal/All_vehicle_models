function interactive_vehicle_viewer_pro
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
    
    Fx_fl = squeeze(out.Longitudinal_force_body_fl.Data);
    Fy_fl = squeeze(out.Lateral_force_body_fl.Data);
    Fx_fr = squeeze(out.Longitudinal_force_body_fr.Data);
    Fy_fr = squeeze(out.Lateral_force_body_fr.Data);
    Fx_rl = squeeze(out.Longitudinal_force_body_rl.Data);
    Fy_rl = squeeze(out.Lateral_force_body_rl.Data);
    Fx_rr = squeeze(out.Longitudinal_force_body_rr.Data);
    Fy_rr = squeeze(out.Lateral_force_body_rr.Data);
    
    N = length(time);

    % Calculate Total Forces for the G-G Diagram
    Total_Fx = Fx_fl + Fx_fr + Fx_rl + Fx_rr;
    Total_Fy = Fy_fl + Fy_fr + Fy_rl + Fy_rr;
    
    % Find max force to scale the G-G diagram circles dynamically
    max_F = max(sqrt(Total_Fx.^2 + Total_Fy.^2));
    gg_lim = ceil(max_F / 1000) * 1000; % Round up to nearest 1000 N
    if gg_lim == 0; gg_lim = 1000; end  % Fallback

    % =========================================================================
    % COLOR PALETTE (PRO WIREFRAME MODE)
    % =========================================================================
    bg_color  = [0.00, 0.00, 0.00];  % Pure Black
    grid_col  = [0.15, 0.15, 0.15];  % Dark grey grid lines
    text_col  = [1.00, 1.00, 1.00];  % Pure White
    cyan_col  = [0.00, 1.00, 1.00];  % Cyan for markers
    red_col   = [1.00, 0.00, 0.00];  % Red for force vectors

    % =========================================================================
    % MAIN FIGURE
    % =========================================================================
    fig = figure( ...
        'Name', 'FSAE Pro Telemetry Viewer', ...
        'Color', bg_color, ...
        'Renderer', 'opengl', ...
        'Position', [50 100 1600 700], ...
        'MenuBar', 'none', ...
        'ToolBar', 'none');
    set(fig, 'GraphicsSmoothing', 'on'); 

    % =========================================================================
    % AXES 1: TRAJECTORY
    % =========================================================================
    ax1 = subplot(1,3,1);
    hold(ax1, 'on'); grid(ax1, 'on'); axis(ax1, 'equal');
    set(ax1, 'Color', bg_color, 'XColor', text_col, 'YColor', text_col, ...
             'GridColor', grid_col, 'GridAlpha', 1, 'LineWidth', 1);
    
    % Trajectory Line
    plot(ax1, x_data, y_data, 'Color', text_col, 'LineWidth', 1);
    
    marker = plot(ax1, x_data(1), y_data(1), 'o', ...
        'MarkerEdgeColor', cyan_col, 'MarkerFaceColor', cyan_col, ...
        'MarkerSize', 6);
    
    title(ax1, 'GLOBAL TRAJECTORY', 'Color', text_col, 'FontSize', 11);
    xlabel(ax1, 'X Position (m)'); ylabel(ax1, 'Y Position (m)');

    % =========================================================================
    % AXES 2: VEHICLE DYNAMICS VIEW (Wireframe)
    % =========================================================================
    ax2 = subplot(1,3,2);
    hold(ax2, 'on'); grid(ax2, 'on'); axis(ax2, 'equal');
    set(ax2, 'Color', bg_color, 'XColor', text_col, 'YColor', text_col, ...
             'GridColor', grid_col, 'GridAlpha', 1, 'XDir', 'reverse', 'LineWidth', 1);
    
    % Vehicle Dimensions
    L = 1.53;
    W = 1.20;
    
    % --- Draw Wireframe Car (Using pure plot with closed loops to prevent errors) ---
    chassis_x = [-0.25, -0.25, -0.1, 0.1, 0.25, 0.25, -0.25];
    chassis_y = [-L/2, L/2, L/2+0.4, L/2+0.4, L/2, -L/2, -L/2];
    plot(ax2, chassis_x, chassis_y, 'Color', text_col, 'LineWidth', 1.5);
    
    fw_x = [-0.55, -0.55, 0.55, 0.55, -0.55];
    fw_y = [L/2+0.3, L/2+0.5, L/2+0.5, L/2+0.3, L/2+0.3];
    plot(ax2, fw_x, fw_y, 'Color', text_col, 'LineWidth', 1.5);
    
    rw_x = [-0.45, -0.45, 0.45, 0.45, -0.45];
    rw_y = [-L/2-0.25, -L/2, -L/2, -L/2-0.25, -L/2-0.25];
    plot(ax2, rw_x, rw_y, 'Color', text_col, 'LineWidth', 1.5);

    % Black Tires with White Borders (fill works fine because we give it a color array)
    wheel_x = [ W/2 -W/2  W/2 -W/2];
    wheel_y = [ L/2  L/2 -L/2 -L/2];
    t_width = 0.15; t_length = 0.4;
    
    for i = 1:4
        tx = [wheel_x(i)-t_width, wheel_x(i)-t_width, wheel_x(i)+t_width, wheel_x(i)+t_width];
        ty = [wheel_y(i)-t_length, wheel_y(i)+t_length, wheel_y(i)+t_length, wheel_y(i)-t_length];
        fill(ax2, tx, ty, [0 0 0], 'EdgeColor', text_col, 'LineWidth', 1.5);
    end

    % --- Force Vectors (Red, Length scales with magnitude) ---
    force_scale = 0.0025; % Multiplier to make forces visibly scale
    force_lines = gobjects(4,1);
    for i = 1:4
        force_lines(i) = line(ax2, [wheel_x(i) wheel_x(i)], [wheel_y(i) wheel_y(i)], ...
            'Color', red_col, 'LineWidth', 2);
    end

    % --- Standard Text HUD ---
    yaw_text = text(ax2, 1.8, 1.8, '', 'Color', text_col, 'FontSize', 10);
    
    xlim(ax2, [-2.5 2.5]); ylim(ax2, [-2.5 2.5]);
    title(ax2, 'WHEEL FORCES', 'Color', text_col, 'FontSize', 11);
    xlabel(ax2, 'Lateral Axis (m)'); ylabel(ax2, 'Longitudinal Axis (m)');

    % =========================================================================
    % AXES 3: G-G DIAGRAM (Friction Circle)
    % =========================================================================
    ax3 = subplot(1,3,3);
    hold(ax3, 'on'); grid(ax3, 'on'); axis(ax3, 'equal');
    set(ax3, 'Color', bg_color, 'XColor', text_col, 'YColor', text_col, ...
             'GridColor', grid_col, 'GridAlpha', 1, 'XDir', 'reverse', 'LineWidth', 1);
    
    % Draw friction circle references (Every 1000 N or 2000 N)
    theta_circ = linspace(0, 2*pi, 100);
    for r = 1000:1000:gg_lim
        plot(ax3, r*cos(theta_circ), r*sin(theta_circ), '--', 'Color', [0.3 0.3 0.3]);
    end
    
    % Plot the entire force history in faint grey
    plot(ax3, Total_Fy, Total_Fx, '.', 'Color', [0.2 0.2 0.2], 'MarkerSize', 2);
    
    % Current force marker
    gg_marker = plot(ax3, Total_Fy(1), Total_Fx(1), 'o', ...
        'MarkerEdgeColor', cyan_col, 'MarkerFaceColor', cyan_col, 'MarkerSize', 8);
    
    % Crosshairs
    plot(ax3, [-gg_lim gg_lim], [0 0], '-', 'Color', [0.3 0.3 0.3]);
    plot(ax3, [0 0], [-gg_lim gg_lim], '-', 'Color', [0.3 0.3 0.3]);
    
    xlim(ax3, [-gg_lim gg_lim]); ylim(ax3, [-gg_lim gg_lim]);
    title(ax3, 'G-G DIAGRAM (TOTAL TIRE FORCE)', 'Color', text_col, 'FontSize', 11);
    xlabel(ax3, 'Lat Force (N)'); ylabel(ax3, 'Long Force (N)');

    % =========================================================================
    % TIMELINE AXES & UI
    % =========================================================================
    timeline_ax = axes('Position', [0.05 0.08 0.70 0.04]);
    hold(timeline_ax, 'on');
    xlim(timeline_ax, [time(1) time(end)]); ylim(timeline_ax, [0 1]);
    timeline_ax.YTick = [];
    set(timeline_ax, 'Color', bg_color, 'XColor', text_col, 'YColor', 'none', ...
        'GridColor', grid_col, 'GridAlpha', 1, 'LineWidth', 1);
    
    plot(timeline_ax, time, 0.5*ones(size(time)), 'Color', grid_col, 'LineWidth', 2);
    timeline_marker = plot(timeline_ax, time(1), 0.5, 'o', ...
        'MarkerFaceColor', cyan_col, 'MarkerEdgeColor', cyan_col, 'MarkerSize', 8);
    xlabel(timeline_ax, 'Simulation Time (s)', 'Color', text_col);

    % =========================================================================
    % BUTTONS
    % =========================================================================
    play_button = uicontrol('Style', 'togglebutton', 'String', 'PLAY', ...
        'Units', 'normalized', 'Position', [0.78 0.07 0.06 0.05], ...
        'ForegroundColor', [0 0 0], 'BackgroundColor', [0.9 0.9 0.9], 'Callback', @play_callback);
        
    speed_menu = uicontrol('Style', 'popupmenu', 'String', {'1x','2x','5x','10x','20x','50x'}, ...
        'Value', 4, 'Units', 'normalized', 'Position', [0.85 0.07 0.05 0.05], ...
        'ForegroundColor', [0 0 0], 'BackgroundColor', [1 1 1]);

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
                src.String = 'PLAY';
                break;
            end
            src.String = 'PAUSE';
            update_frame(current_idx);
        end
        if ~get(src, 'Value')
            src.String = 'PLAY';
        end
    end

    % =========================================================================
    % ULTRA FAST UPDATE FUNCTION
    % =========================================================================
    function update_frame(idx)
        current_idx = idx;
        
        % Update Trajectory and Timeline
        marker.XData = x_data(idx);
        marker.YData = y_data(idx);
        timeline_marker.XData = time(idx);
        
        % Update Force Vectors
        Fx = [Fx_fl(idx) Fx_fr(idx) Fx_rl(idx) Fx_rr(idx)];
        Fy = [Fy_fl(idx) Fy_fr(idx) Fy_rl(idx) Fy_rr(idx)];
        
        for k = 1:4
            x0 = wheel_x(k);
            y0 = wheel_y(k);
            force_lines(k).XData = [x0 x0 + Fy(k)*force_scale];
            force_lines(k).YData = [y0 y0 + Fx(k)*force_scale];
        end
        
        % Update G-G Diagram Marker
        gg_marker.XData = Total_Fy(idx);
        gg_marker.YData = Total_Fx(idx);
        
        % Update Basic Text HUD
        yaw_text.String = sprintf('Time: %.2f s\nYaw: %.2f rad/s', ...
             time(idx), yaw_rate(idx));
         
        drawnow nocallbacks
    end
end