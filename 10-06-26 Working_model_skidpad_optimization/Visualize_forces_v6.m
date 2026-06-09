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
    
    % Tire Frame Forces
    Fx_fl = squeeze(out.Longitudinal_force_fl.Data);
    Fy_fl = squeeze(out.Lateral_force_fl.Data);
    Fx_fr = squeeze(out.Longitudinal_force_fr.Data);
    Fy_fr = squeeze(out.Lateral_force_fr.Data);
    Fx_rl = squeeze(out.Longitudinal_force_rl.Data);
    Fy_rl = squeeze(out.Lateral_force_rl.Data);
    Fx_rr = squeeze(out.Longitudinal_force_rr.Data);
    Fy_rr = squeeze(out.Lateral_force_rr.Data);
    
    % Extract Dynamic Peak Limits (Dx and Dy)
    Dx_raw = squeeze(out.Dx.Data);
    Dy_raw = squeeze(out.Dy.Data);
    
    if size(Dx_raw, 1) == 4 && size(Dx_raw, 2) > 4
        Dx_raw = Dx_raw';
        Dy_raw = Dy_raw';
    elseif size(Dx_raw, 2) == 1 
        Dx_raw = repmat(Dx_raw, 1, 4); 
        Dy_raw = repmat(Dy_raw, 1, 4);
    end
    
    N = length(time);

    % =========================================================================
    % PHYSICS CONVERSIONS 
    % =========================================================================
    mass = 270; % kg 
    g = 9.81;   % m/s^2
    
    Total_Fx = Fx_fl + Fx_fr + Fx_rl + Fx_rr;
    Total_Fy = Fy_fl + Fy_fr + Fy_rl + Fy_rr;
    
    Total_Gx = -(Total_Fx) / (mass * g);
    Total_Gy = -(Total_Fy) / (mass * g);
    
    max_G = max(sqrt(Total_Gx.^2 + Total_Gy.^2));
    gg_lim = ceil(max_G); 
    if gg_lim < 2; gg_lim = 2; end 

    max_wheel_F = max(abs([Fx_fl; Fx_fr; Fx_rl; Fx_rr; Fy_fl; Fy_fr; Fy_rl; Fy_rr]));
    if max_wheel_F == 0; max_wheel_F = 1000; end
    force_scale = 0.5 / max_wheel_F; 

    % =========================================================================
    % COLOR PALETTE 
    % =========================================================================
    bg_color    = [0.05, 0.05, 0.06];  % Deep slate/charcoal background
    grid_col    = [0.15, 0.15, 0.18];  
    text_col    = [0.90, 0.90, 0.90];  
    cyan_col    = [0.00, 1.00, 1.00];  
    
    % Car Materials
    body_fill   = [0.15, 0.15, 0.17];  % Monocoque color
    aero_fill   = [0.10, 0.10, 0.12];  % Wings color
    line_col    = [0.40, 0.40, 0.50];  % Suspension lines

    % =========================================================================
    % MAIN FIGURE
    % =========================================================================
    fig = figure( ...
        'Name', 'FSAE Telemetry Viewer (Real-Time Grip Tracking)', ...
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
    hold(ax1, 'on'); grid(ax1, 'on'); 
    axis(ax1, 'equal'); 
    set(ax1, 'Color', bg_color, 'XColor', text_col, 'YColor', text_col, ...
             'GridColor', grid_col, 'GridAlpha', 1, 'LineWidth', 1);
    
    plot(ax1, x_data, y_data, 'Color', text_col, 'LineWidth', 1);
    marker = plot(ax1, x_data(1), y_data(1), 'o', ...
        'MarkerEdgeColor', cyan_col, 'MarkerFaceColor', cyan_col, 'MarkerSize', 6);
    
    title(ax1, 'GLOBAL TRAJECTORY', 'Color', text_col, 'FontSize', 11);
    xlabel(ax1, 'X Position (m)'); ylabel(ax1, 'Y Position (m)');

    % =========================================================================
    % AXES 2: VEHICLE DYNAMICS VIEW (High-Fidelity Blueprint)
    % =========================================================================
    ax2 = subplot(1,3,2);
    hold(ax2, 'on'); grid(ax2, 'on'); axis(ax2, 'equal');
    set(ax2, 'Color', bg_color, 'XColor', text_col, 'YColor', text_col, ...
             'GridColor', grid_col, 'GridAlpha', 1, 'XDir', 'reverse', 'LineWidth', 1);
    
    L = 1.53; W = 1.20;
    
    % --- 1. SUSPENSION A-ARMS ---
    % Front Left & Right
    plot(ax2, [0.2, 0.54], [0.85, 0.765], 'Color', line_col, 'LineWidth', 1.5);
    plot(ax2, [0.2, 0.54], [0.65, 0.765], 'Color', line_col, 'LineWidth', 1.5);
    plot(ax2, [-0.2, -0.54], [0.85, 0.765], 'Color', line_col, 'LineWidth', 1.5);
    plot(ax2, [-0.2, -0.54], [0.65, 0.765], 'Color', line_col, 'LineWidth', 1.5);
    % Rear Left & Right
    plot(ax2, [0.2, 0.54], [-0.65, -0.765], 'Color', line_col, 'LineWidth', 1.5);
    plot(ax2, [0.2, 0.54], [-0.85, -0.765], 'Color', line_col, 'LineWidth', 1.5);
    plot(ax2, [-0.2, -0.54], [-0.65, -0.765], 'Color', line_col, 'LineWidth', 1.5);
    plot(ax2, [-0.2, -0.54], [-0.85, -0.765], 'Color', line_col, 'LineWidth', 1.5);

    % --- 2. AERODYNAMICS ---
    % Front Wing Mainplane
    fw_x = [-0.65, 0.65, 0.65, -0.65];
    fw_y = [1.15, 1.15, 1.35, 1.35];
    fill(ax2, fw_x, fw_y, aero_fill, 'EdgeColor', cyan_col, 'LineWidth', 1);
    % Front Wing Endplates
    fill(ax2, [0.63, 0.67, 0.67, 0.63], [1.10, 1.10, 1.40, 1.40], aero_fill, 'EdgeColor', cyan_col);
    fill(ax2, [-0.63, -0.67, -0.67, -0.63], [1.10, 1.10, 1.40, 1.40], aero_fill, 'EdgeColor', cyan_col);

    % Rear Wing Mainplane
    rw_x = [-0.5, 0.5, 0.5, -0.5];
    rw_y = [-1.0, -1.0, -1.2, -1.2];
    fill(ax2, rw_x, rw_y, aero_fill, 'EdgeColor', cyan_col, 'LineWidth', 1);
    % Rear Wing Endplates
    fill(ax2, [0.48, 0.52, 0.52, 0.48], [-0.95, -0.95, -1.25, -1.25], aero_fill, 'EdgeColor', cyan_col);
    fill(ax2, [-0.48, -0.52, -0.52, -0.48], [-0.95, -0.95, -1.25, -1.25], aero_fill, 'EdgeColor', cyan_col);

    % --- 3. CHASSIS & BODYWORK ---
    % Sidepods
    sp_x = [0.25, 0.55, 0.55, 0.25, -0.25, -0.55, -0.55, -0.25];
    sp_y = [0.30, 0.30, -0.40, -0.70, -0.70, -0.40, 0.30, 0.30];
    fill(ax2, sp_x, sp_y, body_fill, 'EdgeColor', line_col, 'LineWidth', 1.5);
    
    % Main Monocoque
    mono_x = [0, 0.12, 0.2, 0.28, 0.28, 0.2, -0.2, -0.28, -0.28, -0.2, -0.12];
    mono_y = [1.35, 1.15, 0.765, 0.3, -0.2, -0.8, -0.8, -0.2, 0.3, 0.765, 1.15];
    fill(ax2, mono_x, mono_y, body_fill, 'EdgeColor', text_col, 'LineWidth', 1.5);

    % --- 4. COCKPIT & DRIVER ---
    % Cockpit Opening
    cockpit_x = [0, 0.18, 0.18, -0.18, -0.18];
    cockpit_y = [0.25, 0.1, -0.3, -0.3, 0.1];
    fill(ax2, cockpit_x, cockpit_y, bg_color, 'EdgeColor', text_col, 'LineWidth', 1);
    
    % Steering Wheel
    plot(ax2, [-0.1, 0.1], [0.15, 0.15], 'Color', text_col, 'LineWidth', 2);
    
    % Driver Helmet
    theta = linspace(0, 2*pi, 30);
    helmet_x = 0.08 * cos(theta);
    helmet_y = 0.0 + 0.09 * sin(theta); % Slightly oval
    fill(ax2, helmet_x, helmet_y, [0.8 0.8 0.8], 'EdgeColor', cyan_col, 'LineWidth', 1.5);

    % --- 5. WHEELS & HUD ---
    wheel_x = [ W/2 -W/2  W/2 -W/2]; % FL, FR, RL, RR
    wheel_y = [ L/2  L/2 -L/2 -L/2];
    t_width = 0.12; t_length = 0.28;
    
    wheel_fills = gobjects(4,1);
    wheel_texts = gobjects(4,1);
    force_lines = gobjects(4,1);
    
    for i = 1:4
        tx = [wheel_x(i)-t_width, wheel_x(i)-t_width, wheel_x(i)+t_width, wheel_x(i)+t_width];
        ty = [wheel_y(i)-t_length, wheel_y(i)+t_length, wheel_y(i)+t_length, wheel_y(i)-t_length];
        
        % Static hollow tire outline (Closed loop fix applied)
        plot(ax2, [tx tx(1)], [ty ty(1)], 'Color', text_col, 'LineWidth', 1.5);
        
        % Dynamic inner fill gauge (Starts flat at bottom of tire)
        wheel_fills(i) = fill(ax2, tx, [ty(1) ty(1) ty(1) ty(1)], [0 1 0], ...
            'EdgeColor', 'none', 'FaceAlpha', 0.6);
        
        % Force Vectors (Drawn on top)
        force_lines(i) = line(ax2, [wheel_x(i) wheel_x(i)], [wheel_y(i) wheel_y(i)], ...
            'Color', cyan_col, 'LineWidth', 2);
            
        % HUD Text - Generous Offset to prevent clipping
        if mod(i, 2) ~= 0 
            txt_x = wheel_x(i) + 0.40; % Left wheels offset
        else
            txt_x = wheel_x(i) - 0.40; % Right wheels offset
        end
        
        wheel_texts(i) = text(ax2, txt_x, wheel_y(i), '0%', ...
            'Color', [0 1 0], 'FontSize', 10, 'FontWeight', 'bold', ...
            'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle');
    end

    yaw_text = text(ax2, 1.2, 1.5, '', 'Color', text_col, 'FontSize', 10, 'FontWeight', 'bold');
    
    % Expanded Limits to show everything clearly
    xlim(ax2, [-1.7 1.7]); ylim(ax2, [-1.8 1.8]);
    title(ax2, 'DYNAMIC GRIP SATURATION', 'Color', text_col, 'FontSize', 11);
    xlabel(ax2, 'Lateral Axis (m)'); ylabel(ax2, 'Longitudinal Axis (m)');

    % =========================================================================
    % AXES 3: G-G DIAGRAM
    % =========================================================================
    ax3 = subplot(1,3,3);
    hold(ax3, 'on'); grid(ax3, 'on'); 
    axis(ax3, 'equal'); 
    set(ax3, 'Color', bg_color, 'XColor', text_col, 'YColor', text_col, ...
             'GridColor', grid_col, 'GridAlpha', 1, 'XDir', 'reverse', 'LineWidth', 1);
    
    theta_circ = linspace(0, 2*pi, 100);
    for r = 0.5:0.5:gg_lim
        plot(ax3, r*cos(theta_circ), r*sin(theta_circ), '--', 'Color', [0.3 0.3 0.3]);
    end
    
    plot(ax3, Total_Gy, Total_Gx, '.', 'Color', [0.2 0.2 0.2], 'MarkerSize', 2);
    gg_marker = plot(ax3, Total_Gy(1), Total_Gx(1), 'o', ...
        'MarkerEdgeColor', cyan_col, 'MarkerFaceColor', cyan_col, 'MarkerSize', 8);
    
    plot(ax3, [-gg_lim gg_lim], [0 0], '-', 'Color', [0.3 0.3 0.3]);
    plot(ax3, [0 0], [-gg_lim gg_lim], '-', 'Color', [0.3 0.3 0.3]);
    
    xlim(ax3, [-gg_lim gg_lim]); ylim(ax3, [-gg_lim gg_lim]);
    title(ax3, 'G-G DIAGRAM (INVERTED)', 'Color', text_col, 'FontSize', 11);
    xlabel(ax3, 'Lateral (g)'); ylabel(ax3, 'Longitudinal (g)');

    % =========================================================================
    % TIMELINE & PLAYBACK UI
    % =========================================================================
    timeline_ax = axes('Position', [0.05 0.08 0.80 0.04]);
    hold(timeline_ax, 'on');
    xlim(timeline_ax, [time(1) time(end)]); ylim(timeline_ax, [0 1]);
    timeline_ax.YTick = [];
    set(timeline_ax, 'Color', bg_color, 'XColor', text_col, 'YColor', 'none', ...
        'GridColor', grid_col, 'GridAlpha', 1, 'LineWidth', 1);
    
    plot(timeline_ax, time, 0.5*ones(size(time)), 'Color', grid_col, 'LineWidth', 2);
    timeline_marker = plot(timeline_ax, time(1), 0.5, 'o', ...
        'MarkerFaceColor', cyan_col, 'MarkerEdgeColor', cyan_col, 'MarkerSize', 8);
    xlabel(timeline_ax, 'Simulation Time (s)', 'Color', text_col);

    play_button = uicontrol('Style', 'togglebutton', 'String', 'PLAY (1:1 Real-Time)', ...
        'Units', 'normalized', 'Position', [0.86 0.07 0.12 0.06], ...
        'ForegroundColor', [0 0 0], 'BackgroundColor', [0.9 0.9 0.9], ...
        'FontWeight', 'bold', 'Callback', @play_callback);

    current_idx = 1;
    update_frame(1);

    % =========================================================================
    % KEYBOARD & PLAYBACK FUNCTIONS
    % =========================================================================
    set(fig, 'WindowKeyPressFcn', @key_press);
    function key_press(~, event)
        if get(play_button, 'Value'); return; end
        switch event.Key
            case 'rightarrow'
                current_idx = min(current_idx+10, N);
                update_frame(current_idx);
            case 'leftarrow'
                current_idx = max(current_idx-10, 1);
                update_frame(current_idx);
        end
    end

    function play_callback(src, ~)
        if get(src, 'Value')
            src.String = 'PAUSE';
            sys_start_time = tic;
            sim_start_time = time(current_idx);
            
            while get(src, 'Value')
                elapsed_sys_time = toc(sys_start_time);
                target_sim_time = sim_start_time + elapsed_sys_time;
                temp_idx = current_idx;
                
                while temp_idx < N && time(temp_idx) < target_sim_time
                    temp_idx = temp_idx + 1;
                end
                
                if temp_idx >= N
                    current_idx = N; update_frame(N);
                    set(src, 'Value', 0); src.String = 'REPLAY';
                    src.Callback = @replay_wrapper;
                    break;
                end
                
                if temp_idx > current_idx
                    update_frame(temp_idx);
                else
                    pause(0.005);
                end
            end
        else
            src.String = 'PLAY (1:1 Real-Time)';
        end
    end

    function replay_wrapper(src, event)
        if current_idx == N
            current_idx = 1; update_frame(1);
        end
        src.Callback = @play_callback; play_callback(src, event);
    end

    % =========================================================================
    % RENDER FRAME FUNCTION
    % =========================================================================
    function update_frame(idx)
        current_idx = idx;
        
        marker.XData = x_data(idx);
        marker.YData = y_data(idx);
        timeline_marker.XData = time(idx);
        
        Fx = [Fx_fl(idx) Fx_fr(idx) Fx_rl(idx) Fx_rr(idx)];
        Fy = [Fy_fl(idx) Fy_fr(idx) Fy_rl(idx) Fy_rr(idx)];
        Dx = [Dx_raw(idx, 1) Dx_raw(idx, 2) Dx_raw(idx, 3) Dx_raw(idx, 4)];
        Dy = [Dy_raw(idx, 1) Dy_raw(idx, 2) Dy_raw(idx, 3) Dy_raw(idx, 4)];
        
        for k = 1:4
            % 1. Update Force Vectors
            x0 = wheel_x(k); y0 = wheel_y(k);
            force_lines(k).XData = [x0 x0 + Fy(k)*force_scale];
            force_lines(k).YData = [y0 y0 + Fx(k)*force_scale];
            
            % 2. Calculate Grip Utilization Factor
            dx_val = max(abs(Dx(k)), 1e-3);
            dy_val = max(abs(Dy(k)), 1e-3);
            eta = sqrt((Fx(k)/dx_val)^2 + (Fy(k)/dy_val)^2);
            
            % 3. Map eta to RGB
            eta_clamped = max(0, min(1, eta));
            if eta_clamped < 0.5
                c = [2*eta_clamped, 1, 0];
            else
                c = [1, 2*(1-eta_clamped), 0];
            end
            
            % 4. Update Internal Fill Gauge
            gauge_height = (2 * t_length) * eta_clamped;
            y_bottom = wheel_y(k) - t_length;
            y_top = y_bottom + gauge_height;
            
            wheel_fills(k).YData = [y_bottom, y_top, y_top, y_bottom];
            wheel_fills(k).FaceColor = c;
            
            % 5. Apply Updates to Text HUD
            if eta >= 1.0
                wheel_texts(k).String = sprintf('SLIP\n%.0f%%', eta*100);
                wheel_texts(k).Color = [1 0.2 0.2]; 
            else
                wheel_texts(k).String = sprintf('%.0f%%', eta*100);
                wheel_texts(k).Color = c; 
            end
        end
        
        % Update G-G Diagram Marker
        gg_marker.XData = Total_Gy(idx);
        gg_marker.YData = Total_Gx(idx);
        
        yaw_text.String = sprintf('Time: %.2f s\nYaw Rate: %.2f rad/s', ...
             time(idx), yaw_rate(idx));
         
        drawnow nocallbacks
    end
end