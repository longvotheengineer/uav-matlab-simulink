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
        
        % expand scale to fit the massive 8-state rollercoaster
        axis(ax, [-200 1400 -600 1200 0 400]); 
        xlabel(ax, 'east (m)', 'FontWeight', 'bold');
        ylabel(ax, 'north (m)', 'FontWeight', 'bold');
        zlabel(ax, 'altitude (m)', 'FontWeight', 'bold');
        
        % generate and plot the full desired 3d geometry
        if length(path_cmd) >= 4
            pts = 50;
            
            % s1: climb north
            n_s1 = linspace(200, 800, pts); e_s1 = zeros(1, pts); alt_s1 = linspace(100, 150, pts);
            
            % s2: orbit right (starts at -90 deg, sweeps up to +90 deg)
            th2 = linspace(-pi/2, pi/2, pts); 
            n_s2 = 800 + 200*cos(th2); e_s2 = 200 + 200*sin(th2); alt_s2 = 150*ones(1,pts);
            
            % s3: dive south
            n_s3 = linspace(800, 200, pts); e_s3 = 400*ones(1, pts); alt_s3 = linspace(150, 50, pts);
            
            % s4: orbit left (starts at -90 deg, sweeps down through south to -270 deg)
            th4 = linspace(-pi/2, -3*pi/2, pts); 
            n_s4 = 200 + 200*cos(th4); e_s4 = 600 + 200*sin(th4); alt_s4 = 50*ones(1,pts);
            
            % s5: steep climb north
            n_s5 = linspace(200, 800, pts); e_s5 = 800*ones(1, pts); alt_s5 = linspace(50, 300, pts);
            
            % s6: orbit right (starts at -90 deg, sweeps up to +90 deg)
            th6 = linspace(-pi/2, pi/2, pts); 
            n_s6 = 800 + 200*cos(th6); e_s6 = 1000 + 200*sin(th6); alt_s6 = 300*ones(1,pts);
            
            % s7: descend south
            n_s7 = linspace(800, 200, pts); e_s7 = 1200*ones(1, pts); alt_s7 = linspace(300, 100, pts);
            
            % s8: giant return orbit right (starts at +90 deg, sweeps right/west to +270 deg)
            th8 = linspace(pi/2, 3*pi/2, pts); 
            n_s8 = 200 + 600*cos(th8); e_s8 = 600 + 600*sin(th8); alt_s8 = 100*ones(1,pts);
            
            % concatenate path arrays sequentially
            n_full = [n_s1, n_s2, n_s3, n_s4, n_s5, n_s6, n_s7, n_s8];
            e_full = [e_s1, e_s2, e_s3, e_s4, e_s5, e_s6, e_s7, e_s8];
            alt_full = [alt_s1, alt_s2, alt_s3, alt_s4, alt_s5, alt_s6, alt_s7, alt_s8];
            
            % draw the pure static path (red dashed line)
            plot3(ax, e_full, n_full, alt_full, 'r--', 'LineWidth', 1.5, 'DisplayName', 'desired geometry');
        end
        
        % plot actual path (blue)
        traj_actual = plot3(ax, pe, pn, alt, 'b-', 'LineWidth', 1.5, 'DisplayName', 'actual path');
        p_history = [pe; pn; alt];
        
        % enforce only the trajectory lines in the legend
        legend(ax, 'Location', 'northeast');
        
        % visualize the uav
        s_scale = 3.0; 
        uav_color = [0.6 0.6 0.6]; 
        
        v_fuse = [1 0 0; 0 0.1 0.1; 0 -0.1 0.1; 0 0.1 -0.1; 0 -0.1 -0.1; -3 0 0] * s_scale;
        f_fuse = [1 2 4; 1 4 3; 1 3 5; 1 5 2; 6 2 4; 6 4 3; 6 3 5; 6 5 2];
        
        v_wing = [0 2 0; -0.5 2 0; -0.5 -2 0; 0 -2 0] * s_scale;
        f_wing = [1 2 3 4];
        
        v_htail = [-2.5 0.7 0; -3 0.7 0; -3 -0.7 0; -2.5 -0.7 0] * s_scale;
        f_htail = [1 2 3 4];
        
        v_vtail = [-2.5 0 0; -3 0 0; -3 0 -1; -2.5 0 -1] * s_scale;
        f_vtail = [1 2 3 4];
        
        patches.v_fuse = v_fuse; patches.v_wing = v_wing; 
        patches.v_htail = v_htail; patches.v_vtail = v_vtail;
        
        patches.h_fuse = patch(ax, 'Vertices', v_fuse, 'Faces', f_fuse, 'FaceColor', uav_color, 'EdgeColor', 'k', 'HandleVisibility', 'off');
        patches.h_wing = patch(ax, 'Vertices', v_wing, 'Faces', f_wing, 'FaceColor', uav_color, 'EdgeColor', 'k', 'HandleVisibility', 'off');
        patches.h_htail = patch(ax, 'Vertices', v_htail, 'Faces', f_htail, 'FaceColor', uav_color, 'EdgeColor', 'k', 'HandleVisibility', 'off');
        patches.h_vtail = patch(ax, 'Vertices', v_vtail, 'Faces', f_vtail, 'FaceColor', uav_color, 'EdgeColor', 'k', 'HandleVisibility', 'off');
            
    else
        % update actual trajectory path line
        p_history(:, end+1) = [pe; pn; alt];
        set(traj_actual, 'XData', p_history(1,:), 'YData', p_history(2,:), 'ZData', p_history(3,:));
    end
    
    % update uav 3d orientation
    r_roll  = [1 0 0; 0 cos(phi) -sin(phi); 0 sin(phi) cos(phi)];
    r_pitch = [cos(theta) 0 sin(theta); 0 1 0; -sin(theta) 0 cos(theta)];
    r_yaw   = [cos(psi) -sin(psi) 0; sin(psi) cos(psi) 0; 0 0 1];
    r_body2ned = r_yaw * r_pitch * r_roll;
    
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