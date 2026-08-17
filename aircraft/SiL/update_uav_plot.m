function update_uav_plot(pn_pe_pd, phi_theta_psi, path_cmd)
    % persistent variables keep the figure open and update it every step
    persistent fig_handle patches traj_actual ...
               traj_desired p_history last_update_time

    % fps limiter (caps graphics to ~25 FPS to prevent lag)
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
        fig_handle = figure('Name', '3D UAV Visualization', 'Color', ...
            'w', 'NumberTitle', 'off');
        ax = axes('Parent', fig_handle, 'Color', [0.98 0.98 0.98]);
        hold(ax, 'on'); grid(ax, 'on'); view(ax, 3);
        
        % scale to fit the figure-8 and helix
        axis(ax, [100 900 100 900 -100 500]); 
        xlabel(ax, 'East (m)', 'FontWeight', 'bold');
        ylabel(ax, 'North (m)', 'FontWeight', 'bold');
        zlabel(ax, 'Altitude (m)', 'FontWeight', 'bold');

        % generate and plot the full desired path
        if length(path_cmd) >= 4
            traj_type = path_cmd(4);
            A = 300;
            N_center = 500;
            E_center = 500;
            
            if traj_type == 1 % figure-8
                theta_full = linspace(0, 2*pi, 200); % full geometric loop
                N_full = N_center + A * sin(theta_full) .* cos(theta_full);
                E_full = E_center + A * sin(theta_full);
                Alt_full = 100 * ones(size(theta_full));
            else % helix
                theta_full = linspace(0, 8*pi, 400); % 4 spiral loops
                N_full = N_center + A * cos(theta_full);
                E_full = E_center + A * sin(theta_full);
                Alt_full = 100 + (13.0 * theta_full); 
            end
            
            % draw the full static path (red dashed line)
            plot3(ax, E_full, N_full, Alt_full, 'r--', 'LineWidth', ...
                1.5, 'DisplayName', 'Desired Path');
            
            % initialize the moving target "carrot" (red dot)
            traj_desired = plot3(ax, path_cmd(2), path_cmd(1), path_cmd(3), ...
                                 'ro', 'MarkerSize', 6, ...
                                 'MarkerFaceColor', 'r', ...
                                 'DisplayName', 'Target');
        end

        % plot actual path (blue)
        traj_actual = plot3(ax, pe, pn, alt, 'b-', 'LineWidth', 1.5, ...
            'DisplayName', 'Actual Path');
        p_history = [pe; pn; alt];

        % enforce only the trajectory lines in the legend
        legend(ax, 'Location', 'northeast');

        % visualize the uav
        S = 3.0; % size scale
        uav_color = [0.6 0.6 0.6]; % grey color
        
        % vertices (body frame: x-forward, y-right, z-down)
        V_fuse = [1    0    0;
                  0  0.1  0.1;
                  0 -0.1  0.1;
                  0  0.1 -0.1;
                  0 -0.1 -0.1;
                 -3    0    0] * S;
        F_fuse = [1 2 4;
                  1 4 3; 
                  1 3 5; 
                  1 5 2; 
                  6 2 4; 
                  6 4 3; 
                  6 3 5; 
                  6 5 2];
        
        V_wing = [  0  2 0; 
                 -0.5  2 0; 
                 -0.5 -2 0; 
                    0 -2 0] * S;
        F_wing = [1 2 3 4];
        
        V_htail = [-2.5  0.7 0;
                     -3  0.7 0;
                     -3 -0.7 0;
                   -2.5 -0.7 0] * S;
        F_htail = [1 2 3 4];
        
        V_vtail = [-2.5 0  0;
                     -3 0  0;
                     -3 0 -1;
                   -2.5 0 -1] * S;
        F_vtail = [1 2 3 4];
        
        % save geometry
        patches.v_fuse = V_fuse; patches.v_wing = V_wing; 
        patches.v_htail = V_htail; patches.v_vtail = V_vtail;

        % render (HandleVisibility = 'off' keeps them out of the legend)
        patches.h_fuse = patch(ax, 'Vertices', V_fuse, 'Faces', F_fuse, ...
            'FaceColor', uav_color, 'EdgeColor', 'k', ...
            'HandleVisibility', 'off');
        patches.h_wing = patch(ax, 'Vertices', V_wing, 'Faces', F_wing, ...
            'FaceColor', uav_color, 'EdgeColor', 'k', ...
            'HandleVisibility', 'off');
        patches.h_htail = patch(ax, 'Vertices', V_htail, 'Faces', F_htail, ...
            'FaceColor', uav_color, 'EdgeColor', 'k', ...
            'HandleVisibility', 'off');
        patches.h_vtail = patch(ax, 'Vertices', V_vtail, 'Faces', F_vtail, ...
            'FaceColor', uav_color, 'EdgeColor', 'k', ...
            'HandleVisibility', 'off');
        
    else
        % update actual trajectory path line
        p_history(:, end+1) = [pe; pn; alt];
        set(traj_actual, 'XData', p_history(1,:), ...
                         'YData', p_history(2,:), ...
                         'ZData', p_history(3,:));
        
        % update target carrot position
        if length(path_cmd) >= 3
            set(traj_desired, 'XData', path_cmd(2), ...
                              'YData', path_cmd(1), ...
                              'ZData', path_cmd(3));
        end
    end

    % update uav 3d orientation
    R_roll  = [1 0 0; 0 cos(phi) -sin(phi); 0 sin(phi) cos(phi)];
    R_pitch = [cos(theta) 0 sin(theta); 0 1 0; -sin(theta) 0 cos(theta)];
    R_yaw   = [cos(psi) -sin(psi) 0; sin(psi) cos(psi) 0; 0 0 1];
    R_body2ned = R_yaw * R_pitch * R_roll;

    % map NED body coordinates to plot axes (x=east, y=north, z=up)
    C_ned2plot = [0 1 0; 1 0 0; 0 0 -1];
    R_total = C_ned2plot * R_body2ned;

    tf = @(v) (R_total * v')' + [pe, pn, alt];

    % apply math to graphics objects
    set(patches.h_fuse, 'Vertices', tf(patches.v_fuse));
    set(patches.h_wing, 'Vertices', tf(patches.v_wing));
    set(patches.h_htail, 'Vertices', tf(patches.v_htail));
    set(patches.h_vtail, 'Vertices', tf(patches.v_vtail));

    drawnow limitrate;
end