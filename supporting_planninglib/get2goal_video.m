function[achieved_goal, achieved_goalDQ, achieved_jointvec, theta_SCLERP] = get2goal_video(theta0, A, B, myVideo)
    % Define global variables
    global g;              global q_o;         global v_ts2js;
    global dof;            global dista;       global count;
    global contact_normal; global jcon_array;  global g_intermediate; 
    global J_st;           global Tau;         global beta;
    global box_vertices_array; global box_normals_array; global hfig;
    
    count = 1; reach_dist = 0.001; rot_dist = 0.001;
    prev_position_err = 100; prev_rotation_err = 100;
    
% % %     % Plot local start and goal
% % %     g_init = DQ2Mat(A); g_finl = DQ2Mat(B);
% % %     pt_ls = scatter3(g_init(1, 4), g_init(2, 4), g_init(3, 4), 'k', 'filled');
% % %     hold on;
% % %     pt_lg = scatter3(g_finl(1, 4), g_finl(2, 4), g_finl(3, 4), 'k', 'filled');
    
    % Set the bounds of unknowns for PATH Solver
    for j = 1 : 14
        l(j) = -Inf; u(j) = Inf;
    end
    l(1, 8:14) = 0;   % sice complementarity velocity is always >= 0 
    z = zeros(14, 1); % first 7 are joint_q and last 7 are compensating velocities
    q_o = theta0;     % initial config

    % Main loop of iterative motion plan using PATH solver
    currentDQ = A;
    finalDQ = B;
    [g, ~, ~, ~, g_intermediate] = FK_RealBaxter(q_o);
    not_reached_flag = true;
    theta_SCLERP = [];
    theta_SCLERP(:, count) = q_o;
    
    while not_reached_flag
        % Compute g_tau and DQ tau    
        [~, ResDQ] = Screw_Lin(currentDQ, finalDQ, Tau);
        % Move end effector to g_tau by approximating joint angles   
        J_st = JS_RealBaxter(q_o);
        [thetha_next, v_ts2js] = theta_next_step2(J_st, g, ResDQ, q_o, beta);
%         [contact_normal, dista, jcon_array] = get_collision_info4(g_intermediate, box_vertices_array, box_normals_array);
        [contact_normal, dista, jcon_array] = get_collision_info4_graspobj(g_intermediate, box_vertices_array, box_normals_array);
        [z, ~, ~] = pathmcp(z, l, u, 'mcpfuncjacEval');
        q_o = z(1:dof);
        [g, ~, ~, ~, g_intermediate] = FK_RealBaxter(q_o);
        currentDQ = Mat2DQ(g);
        count = count+1;
        theta_SCLERP(:, count) = q_o;

        % Check how far from goal
        [rotation_err, position_err] = distDQ(ResDQ, currentDQ);
        
        if (prev_position_err - position_err) > 1e-4 || (prev_rotation_err - rotation_err) > 1e-4
            prev_position_err = position_err;
            prev_rotation_err = rotation_err;
        else
            not_reached_flag = false;
            fprintf("position or rotation errors do not improve. \n");
        end

        if position_err < reach_dist && rotation_err < rot_dist
            not_reached_flag = false;
            fprintf("goal reached according to given tolerance. \n");
        end

        %/////////////// Plot the motion ///////////////    
        % Plot motion
        plot_motion_video(hfig, myVideo);
%         plot_motion(hfig);
    end
    
% % %     delete(pt_ls); delete(pt_lg);
    
    achieved_goal = g;
    achieved_goalDQ = currentDQ;
    achieved_jointvec = theta_SCLERP(:, count);
end