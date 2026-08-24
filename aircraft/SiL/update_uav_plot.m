function update_uav_plot(pn_pe_pd, phi_theta_psi, ...
                         waypoints, num_waypoints, path_data)
                     
    persistent fig_handle patches traj_actual p_history ...
               last_update_time traj_desired d_history
    
    if isempty(last_update_time)
        last_update_time = tic;
    elseif toc(last_update_time) < 0.04 
        return;
    else
        last_update_time = tic;
    end
    
    % Unpack state variables
    pn  = pn_pe_pd(1); 
    pe  = pn_pe_pd(2); 
    alt = -pn_pe_pd(3); 
    
    phi   = phi_theta_psi(1); 
    theta = phi_theta_psi(2); 
    psi   = phi_theta_psi(3);
    
    if isempty(fig_handle) || ~isvalid(fig_handle)
        % Initialize figure and axes
        fig_handle = figure('Name', '3d uav visualization', ...
                            'Color', 'w', 'NumberTitle', 'off');
        ax = axes('Parent', fig_handle, 'Color', [0.98 0.98 0.98]);
        hold(ax, 'on'); 
        grid(ax, 'on'); 
        view(ax, 3);
        
        % Massive scale to fit the star
        axis(ax, [-1200 1200 -1200 1200 0 800]); 
        xlabel(ax, 'east (m)', 'FontWeight', 'bold');
        ylabel(ax, 'north (m)', 'FontWeight', 'bold');
        zlabel(ax, 'altitude (m)', 'FontWeight', 'bold');
        
        if num_waypoints > 1
            % Draw the mission planner reference lines and waypoints
            w_n = [waypoints(1:num_waypoints, 1); waypoints(1, 1)];
            w_e = [waypoints(1:num_waypoints, 2); waypoints(1, 2)];
            w_d = [waypoints(1:num_waypoints, 3); waypoints(1, 3)];
            
            plot3(ax, w_e, w_n, -w_d, 'r--o', ...
                  'LineWidth', 1.5, ...
                  'MarkerFaceColor', 'r', ...
                  'DisplayName', 'waypoint');
        end
        
        traj_desired = plot3(ax, pe, pn, alt, 'g--', ...
                             'LineWidth', 2.5, ...
                             'DisplayName', 'desired path');
                         
        d_history = [];
        
        traj_actual = plot3(ax, pe, pn, alt, 'b-', ...
                            'LineWidth', 1.5, ...
                            'DisplayName', 'actual path');
                        
        p_history = [pe; pn; alt];
        legend(ax, 'Location', 'northeast');
        
        % Draw UAV
        s = 15.0; 
        uav_c = [0.6 0.6 0.6]; 
        
        v_fuse = [ 1  0    0; 
                   0  0.1  0.1; 
                   0 -0.1  0.1; 
                   0  0.1 -0.1; 
                   0 -0.1 -0.1; 
                  -3  0    0] * s; 
              
        f_fuse = [1 2 4; 1 4 3; 1 3 5; 1 5 2; ...
                  6 2 4; 6 4 3; 6 3 5; 6 5 2];
              
        v_wing = [ 0   2 0; 
                  -0.5  2 0; 
                  -0.5 -2 0; 
                   0  -2 0] * s;
               
        f_wing = [1 2 3 4];
        
        v_htail = [-2.5  0.7 0; 
                   -3    0.7 0; 
                   -3   -0.7 0; 
                   -2.5 -0.7 0] * s;
               
        f_htail = [1 2 3 4];
        
        v_vtail = [-2.5 0  0; 
                   -3   0  0; 
                   -3   0 -1; 
                   -2.5 0 -1] * s;
               
        f_vtail = [1 2 3 4];
        
        % Create patch objects
        patches.h_fuse = patch(ax, 'Vertices', v_fuse, ...
                               'Faces', f_fuse, 'FaceColor', uav_c, ...
                               'EdgeColor', 'k', ...
                               'HandleVisibility', 'off');
                           
        patches.h_wing = patch(ax, 'Vertices', v_wing, ...
                               'Faces', f_wing, 'FaceColor', uav_c, ...
                               'EdgeColor', 'k', ...
                               'HandleVisibility', 'off');
                           
        patches.h_htail = patch(ax, 'Vertices', v_htail, ...
                                'Faces', f_htail, 'FaceColor', uav_c, ...
                                'EdgeColor', 'k', ...
                                'HandleVisibility', 'off');
                            
        patches.h_vtail = patch(ax, 'Vertices', v_vtail, ...
                                'Faces', f_vtail, 'FaceColor', uav_c, ...
                                'EdgeColor', 'k', ...
                                'HandleVisibility', 'off');
                            
        % Store base vertices
        patches.v_fuse  = v_fuse; 
        patches.v_wing  = v_wing; 
        patches.v_htail = v_htail; 
        patches.v_vtail = v_vtail;
        
    else
        % 1. Append and update actual UAV path
        p_history(:, end+1) = [pe; pn; alt];
        set(traj_actual, ...
            'XData', p_history(1,:), ...
            'YData', p_history(2,:), ...
            'ZData', p_history(3,:));
        
        % 2. Project UAV onto the active FSM field
        flag = path_data(1);
        p_vec = [pn; pe; -alt]; % Current NED position
        
        if flag == 1 % Orbit FSM State
            c_vec = [path_data(2); path_data(3); path_data(4)];
            R = path_data(5);
            d_vec = p_vec(1:2) - c_vec(1:2);
            pt_h = c_vec(1:2) + R * (d_vec / norm(d_vec));
            pt_desired = [pt_h(1); pt_h(2); c_vec(3)];
            
        else % Straight Line FSM State
            r_vec = [path_data(7); path_data(8); path_data(9)];
            q_vec = [path_data(10); path_data(11); path_data(12)];
            pt_desired = r_vec + dot(p_vec - r_vec, q_vec) * q_vec;
        end
        
        % 3. Hold and append the desired trajectory trail
        % [East; North; Alt]
        d_history(:, end+1) = [pt_desired(2); 
                               pt_desired(1); 
                              -pt_desired(3)]; 
                          
        set(traj_desired, ...
            'XData', d_history(1,:), ...
            'YData', d_history(2,:), ...
            'ZData', d_history(3,:));
    end
    
    % Rotation matrices
    r_body2ned = ...
        [cos(psi) -sin(psi) 0; sin(psi) cos(psi) 0; 0 0 1] * ...
        [cos(theta) 0 sin(theta); 0 1 0; -sin(theta) 0 cos(theta)] * ...
        [1 0 0; 0 cos(phi) -sin(phi); 0 sin(phi) cos(phi)];
    
    % Transformation function handle
    tf = @(v) ([0 1 0; 1 0 0; 0 0 -1] * r_body2ned * v')' ...
              + [pe, pn, alt];
    
    % Apply transformations
    set(patches.h_fuse,  'Vertices', tf(patches.v_fuse)); 
    set(patches.h_wing,  'Vertices', tf(patches.v_wing));
    set(patches.h_htail, 'Vertices', tf(patches.v_htail)); 
    set(patches.h_vtail, 'Vertices', tf(patches.v_vtail));
    
    drawnow limitrate;
end