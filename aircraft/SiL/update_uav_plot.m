function update_uav_plot(pn_pe_pd, phi_theta_psi, path_cmd)
    % persistent variables keep the figure open and update it every step
    persistent fig_handle patches traj_actual p_history last_update_time
    
    % fps limiter (caps graphics to ~25 fps to prevent lag)
    if isempty(last_update_time)
        last_update_time = tic;
    elseif toc(last_update_time) < 0.04 
        return;
    else
        last_update_time = tic;
    end
    
    % extract states
    pn  = pn_pe_pd(1);
    pe  = pn_pe_pd(2);
    alt = -pn_pe_pd(3); 
    phi   = phi_theta_psi(1);
    theta = phi_theta_psi(2);
    psi   = phi_theta_psi(3);
    
    % initialize figure
    if isempty(fig_handle) || ~isvalid(fig_handle)
        fig_handle = figure('Name', '3d uav visualization', 'Color', ...
            'w', 'NumberTitle', 'off');
        ax = axes('Parent', fig_handle, 'Color', [0.98 0.98 0.98]);
        hold(ax, 'on'); grid(ax, 'on'); view(ax, 3);
        
        % scale to fit the stadium pattern
        axis(ax, [100 900 100 900 -100 500]); 
        xlabel(ax, 'east (m)', 'FontWeight', 'bold');
        ylabel(ax, 'north (m)', 'FontWeight', 'bold');
        zlabel(ax, 'altitude (m)', 'FontWeight', 'bold');
        
        % generate and plot the full desired 3d path
        if length(path_cmd) >= 4
            % left straight climb
            n_straight_left = linspace(200, 800, 50);
            e_straight_left = 300 * ones(1, 50);
            alt_straight_left = linspace(100, 150, 50);
            
            % top right arc
            theta_top = linspace(-pi/2, pi/2, 50);
            n_top_arc = 800 + 150 * cos(theta_top);
            e_top_arc = 450 + 150 * sin(theta_top);
            alt_top_arc = 150 * ones(1, 50);
            
            % right straight descend
            n_straight_right = linspace(800, 200, 50);
            e_straight_right = 600 * ones(1, 50);
            alt_straight_right = linspace(150, 100, 50);
            
            % bottom right arc
            theta_bottom = linspace(pi/2, 3*pi/2, 50);
            n_bottom_arc = 200 + 150 * cos(theta_bottom);
            e_bottom_arc = 450 + 150 * sin(theta_bottom);
            alt_bottom_arc = 100 * ones(1, 50);
            
            % concatenate path arrays
            n_full = [n_straight_left, n_top_arc, n_straight_right, n_bottom_arc];
            e_full = [e_straight_left, e_top_arc, e_straight_right, e_bottom_arc];
            alt_full = [alt_straight_left, alt_top_arc, alt_straight_right, alt_bottom_arc];
            
            % draw the full static path (red dashed line)
            plot3(ax, e_full, n_full, alt_full, 'r--', 'LineWidth', ...
                1.5, 'DisplayName', 'desired path');
        end
        
        % plot actual path (blue)
        traj_actual = plot3(ax, pe, pn, alt, 'b-', 'LineWidth', 1.5, ...
            'DisplayName', 'actual path');
        p_history = [pe; pn; alt];
        
        % enforce only the trajectory lines in the legend
        legend(ax, 'Location', 'northeast');
        
        % visualize the uav
        s_scale = 3.0; % size scale
        uav_color = [0.6 0.6 0.6]; % grey color
        
        % vertices (body frame: x-forward, y-right, z-down)
        v_fuse = [1    0    0;
                  0  0.1  0.1;
                  0 -0.1  0.1;
                  0  0.1 -0.1;
                  0 -0.1 -0.1;
                 -3    0    0] * s_scale;
        f_fuse = [1 2 4;
                  1 4 3; 
                  1 3 5; 
                  1 5 2; 
                  6 2 4; 
                  6 4 3; 
                  6 3 5; 
                  6 5 2];
        
        v_wing = [  0  2 0; 
                 -0.5  2 0; 
                 -0.5 -2 0; 
                    0 -2 0] * s_scale;
        f_wing = [1 2 3 4];
        
        v_htail = [-2.5  0.7 0;
                     -3  0.7 0;
                     -3 -0.7 0;
                   -2.5 -0.7 0] * s_scale;
        f_htail = [1 2 3 4];
        
        v_vtail = [-2.5 0  0;
                     -3 0  0;
                     -3 0 -1;
                   -2.5 0 -1] * s_scale;
        f_vtail = [1 2 3 4];
        
        % save geometry
        patches.v_fuse = v_fuse; patches.v_wing = v_wing; 
        patches.v_htail = v_htail; patches.v_vtail = v_vtail;
        
        % render (handlevisibility = 'off' keeps them out of the legend)
        patches.h_fuse = patch(ax, 'Vertices', v_fuse, 'Faces', f_fuse, ...
            'FaceColor', uav_color, 'EdgeColor', 'k', ...
            'HandleVisibility', 'off');
        patches.h_wing = patch(ax, 'Vertices', v_wing, 'Faces', f_wing, ...
            'FaceColor', uav_color, 'EdgeColor', 'k', ...
            'HandleVisibility', 'off');
        patches.h_htail = patch(ax, 'Vertices', v_htail, 'Faces', f_htail, ...
            'FaceColor', uav_color, 'EdgeColor', 'k', ...
            'HandleVisibility', 'off');
        patches.h_vtail = patch(ax, 'Vertices', v_vtail, 'Faces', f_vtail, ...
            'FaceColor', uav_color, 'EdgeColor', 'k', ...
            'HandleVisibility', 'off');
            
    else
        % update actual trajectory path line
        p_history(:, end+1) = [pe; pn; alt];
        set(traj_actual, 'XData', p_history(1,:), ...
                         'YData', p_history(2,:), ...
                         'ZData', p_history(3,:));
    end
    
    % update uav 3d orientation
    r_roll  = [1 0 0; 0 cos(phi) -sin(phi); 0 sin(phi) cos(phi)];
    r_pitch = [cos(theta) 0 sin(theta); 0 1 0; -sin(theta) 0 cos(theta)];
    r_yaw   = [cos(psi) -sin(psi) 0; sin(psi) cos(psi) 0; 0 0 1];
    r_body2ned = r_yaw * r_pitch * r_roll;
    
    % map ned body coordinates to plot axes (x=east, y=north, z=up)
    c_ned2plot = [0 1 0; 1 0 0; 0 0 -1];
    r_total = c_ned2plot * r_body2ned;
    tf = @(v) (r_total * v')' + [pe, pn, alt];
    
    % apply math to graphics objects
    set(patches.h_fuse, 'Vertices', tf(patches.v_fuse));
    set(patches.h_wing, 'Vertices', tf(patches.v_wing));
    set(patches.h_htail, 'Vertices', tf(patches.v_htail));
    set(patches.h_vtail, 'Vertices', tf(patches.v_vtail));
    drawnow limitrate;
end