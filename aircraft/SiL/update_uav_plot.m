function update_uav_plot(pn_pe_pd, phi_theta_psi, ...
                         waypoints, num_waypoints, path_data, ...
                         obstacles, tree, num_nodes)

    persistent fig_handle ax patches traj_actual p_history ...
               last_update_time traj_desired d_history

%% update rate
    if isempty(last_update_time)
        last_update_time = tic;
    elseif toc(last_update_time) < 0.04
        return;
    else
        last_update_time = tic;
    end

%% unpack state variables
    pn  = pn_pe_pd(1);
    pe  = pn_pe_pd(2);
    alt = -pn_pe_pd(3);

    phi   = phi_theta_psi(1);
    theta = phi_theta_psi(2);
    psi   = phi_theta_psi(3);

%% initialize figure and UAV model
    if isempty(fig_handle) || ~isvalid(fig_handle)

        fig_handle = figure( ...
            'Name', '3d UAV visualization', ...
            'Color', 'w', ...
            'NumberTitle', 'off');

        ax = axes( ...
            'Parent', fig_handle, ...
            'Color', [0.98 0.98 0.98]);

        hold(ax, 'on');
        grid(ax, 'on');
        view(ax, 3);

        axis(ax, [-400 1200 -100 1300 0 500]);

        xlabel(ax, 'east (m)', 'FontWeight', 'bold');
        ylabel(ax, 'north (m)', 'FontWeight', 'bold');
        zlabel(ax, 'altitude (m)', 'FontWeight', 'bold');

%% draw obstacles
        for k = 1:size(obstacles, 1)
            if obstacles(k, 3) > 0

                [X, Y, Z] = cylinder(obstacles(k, 3), 20);

                surf(ax, ...
                    X + obstacles(k, 2), ...
                    Y + obstacles(k, 1), ...
                    Z * -obstacles(k, 4), ...
                    'FaceColor', 'r', ...
                    'FaceAlpha', 0.2, ...
                    'EdgeColor', 'none', ...
                    'HandleVisibility', 'off');

                theta_cyl = linspace(0, 2 * pi, 20);

                x_cap = obstacles(k, 2) + ...
                    obstacles(k, 3) * cos(theta_cyl);

                y_cap = obstacles(k, 1) + ...
                    obstacles(k, 3) * sin(theta_cyl);

                z_cap = -obstacles(k, 4) * ones(1, 20);

                fill3(ax, x_cap, y_cap, z_cap, 'r', ...
                    'FaceAlpha', 0.2, ...
                    'EdgeColor', 'r', ...
                    'HandleVisibility', 'off');
            end
        end

%% draw RRT tree
        for i = 2:num_nodes
            p_idx = tree(i, 4);

            if p_idx > 0
                plot3(ax, ...
                    [tree(i, 2), tree(p_idx, 2)], ...
                    [tree(i, 1), tree(p_idx, 1)], ...
                    [-tree(i, 3), -tree(p_idx, 3)], ...
                    'Color', [0.8 0.8 0.8], ...
                    'LineWidth', 0.5, ...
                    'HandleVisibility', 'off');
            end
        end

%% draw waypoints
        if num_waypoints > 1
            w_n = waypoints(1:num_waypoints, 1);
            w_e = waypoints(1:num_waypoints, 2);
            w_d = waypoints(1:num_waypoints, 3);

            plot3(ax, w_e, w_n, -w_d, 'r--o', ...
                'LineWidth', 1.5, ...
                'MarkerFaceColor', 'r', ...
                'DisplayName', 'waypoint');
        end

%% initialize trajectory plots
        traj_desired = plot3(ax, pe, pn, alt, 'g--', ...
            'LineWidth', 2.5, ...
            'DisplayName', 'desired path');

        traj_actual = plot3(ax, pe, pn, alt, 'b-', ...
            'LineWidth', 1.5, ...
            'DisplayName', 'actual path');

        p_history = [pe; pn; alt];
        d_history = [];

        legend(ax, 'Location', 'northeast');

%% define UAV geometry
        s     = 15.0;
        uav_c = [0.6 0.6 0.6];

        v_fuse = s * [ ...
             1    0    0;
             0    0.1  0.1;
             0   -0.1  0.1;
             0    0.1 -0.1;
             0   -0.1 -0.1;
            -3    0    0];

        f_fuse = [ ...
            1 2 4;
            1 4 3;
            1 3 5;
            1 5 2;
            6 2 4;
            6 4 3;
            6 3 5;
            6 5 2];

        v_wing = s * [ ...
             0    2  0;
            -0.5  2  0;
            -0.5 -2  0;
             0   -2  0];

        f_wing = [1 2 3 4];

        v_htail = s * [ ...
            -2.5   0.7  0;
            -3     0.7  0;
            -3    -0.7  0;
            -2.5  -0.7  0];

        f_htail = [1 2 3 4];

        v_vtail = s * [ ...
            -2.5  0  0;
            -3     0  0;
            -3     0 -1;
            -2.5  0 -1];

        f_vtail = [1 2 3 4];

%% create UAV patches
        patches.h_fuse = patch(ax, ...
            'Vertices', v_fuse, ...
            'Faces', f_fuse, ...
            'FaceColor', uav_c, ...
            'EdgeColor', 'k', ...
            'HandleVisibility', 'off');

        patches.h_wing = patch(ax, ...
            'Vertices', v_wing, ...
            'Faces', f_wing, ...
            'FaceColor', uav_c, ...
            'EdgeColor', 'k', ...
            'HandleVisibility', 'off');

        patches.h_htail = patch(ax, ...
            'Vertices', v_htail, ...
            'Faces', f_htail, ...
            'FaceColor', uav_c, ...
            'EdgeColor', 'k', ...
            'HandleVisibility', 'off');

        patches.h_vtail = patch(ax, ...
            'Vertices', v_vtail, ...
            'Faces', f_vtail, ...
            'FaceColor', uav_c, ...
            'EdgeColor', 'k', ...
            'HandleVisibility', 'off');

        patches.v_fuse  = v_fuse;
        patches.v_wing  = v_wing;
        patches.v_htail = v_htail;
        patches.v_vtail = v_vtail;

    else

%% detect simulation restart
        current_pos = [pe; pn; alt];

        if norm(p_history(:, end) - current_pos) > 100

            p_history(:, end + 1) = [NaN; NaN; NaN];

            if ~isempty(d_history)
                d_history(:, end + 1) = [NaN; NaN; NaN];
            end

%% draw new RRT tree
            for i = 2:num_nodes
                p_idx = tree(i, 4);

                if p_idx > 0
                    plot3(ax, ...
                        [tree(i, 2), tree(p_idx, 2)], ...
                        [tree(i, 1), tree(p_idx, 1)], ...
                        [-tree(i, 3), -tree(p_idx, 3)], ...
                        'Color', [0.8 0.8 0.8], ...
                        'LineWidth', 0.5, ...
                        'HandleVisibility', 'off');
                end
            end

%% draw new waypoints
            if num_waypoints > 1
                w_n = waypoints(1:num_waypoints, 1);
                w_e = waypoints(1:num_waypoints, 2);
                w_d = waypoints(1:num_waypoints, 3);

                plot3(ax, w_e, w_n, -w_d, 'r--o', ...
                    'LineWidth', 1.5, ...
                    'MarkerFaceColor', 'r', ...
                    'HandleVisibility', 'off');
            end
        end

%% update actual trajectory
        p_history(:, end + 1) = current_pos;

        set(traj_actual, ...
            'XData', p_history(1, :), ...
            'YData', p_history(2, :), ...
            'ZData', p_history(3, :));

%% calculate desired trajectory point
        flag  = path_data(1);
        p_vec = [pn; pe; -alt];

        if flag == 1
            c_vec = path_data(2:4);
            R     = path_data(5);

            d_vec = p_vec(1:2) - c_vec(1:2);
            pt_h  = c_vec(1:2) + R * d_vec / norm(d_vec);

            pt_desired = [pt_h; c_vec(3)];

        else
            r_vec = path_data(7:9);
            q_vec = path_data(10:12);

            pt_desired = r_vec + ...
                dot(p_vec - r_vec, q_vec) * q_vec;
        end

%% update desired trajectory
        d_history(:, end + 1) = [ ...
            pt_desired(2);
            pt_desired(1);
           -pt_desired(3)];

        set(traj_desired, ...
            'XData', d_history(1, :), ...
            'YData', d_history(2, :), ...
            'ZData', d_history(3, :));
    end

%% calculate body-to-NED rotation
    r_body2ned = ...
        [cos(psi)   -sin(psi)    0;
         sin(psi)    cos(psi)    0;
         0           0           1] * ...
        [cos(theta)  0           sin(theta);
         0           1           0;
        -sin(theta)  0           cos(theta)] * ...
        [1           0           0;
         0           cos(phi)   -sin(phi);
         0           sin(phi)    cos(phi)];

%% transform UAV geometry
    tf = @(v) ...
        ([0 1 0;
          1 0 0;
          0 0 -1] * r_body2ned * v')' + ...
        [pe, pn, alt];

    set(patches.h_fuse,  'Vertices', tf(patches.v_fuse));
    set(patches.h_wing,  'Vertices', tf(patches.v_wing));
    set(patches.h_htail, 'Vertices', tf(patches.v_htail));
    set(patches.h_vtail, 'Vertices', tf(patches.v_vtail));

%% update visualization
    drawnow limitrate;

end