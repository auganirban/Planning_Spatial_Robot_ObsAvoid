clc;
clear;
close all;

% To do: modify the get_collision_info4.m such that "get_obstacle_wall"
% menthod is called only once. (Done).

% Add path to PATH solver
addpath("../supporting_quatlib"); addpath("../supporting_colli_funcs");
addpath("../supporting_kinlib");  addpath("../pathmexa64"); addpath("../supporting_colli_models");

% Define global variables
global g;              global q_o;         global h;              global v_ts2js;
global dof;            global dista;       global safe_dist;      global count;
global contact_normal; global jcon_array;  global g_intermediate; global task_dimen;
global J_st;

hfig  = figure(1); hold on; view(164, 47);
title("Baxter left arm transferring object through the window"); xlabel("x [m]"); ylabel("y [m]"); zlabel("z [m]");

%////////////////////// Inputs/////////////////////////
theta1 = [0.307, -1.130, -0.503, 2.458, 0.8130, -1.216, -0.122]';
window_position = [0.75; 0.20; 0.30]; % Joint positions of the left arm of Baxter robot
[~, box_vertices_array, box_normals_array] = get_obstacle_wall(window_position);

% Planning algorithm parameters
beta = 0.02; Tau = 0.5; h = 0.01; reach_dist = 0.010;
dof = 7; safe_dist = 0.009; num_of_contacts = 7; count = 1; task_dimen = 6;

% Initial loaction in task space
g = FK_URDF(theta1); A = Mat2DQ(g);

% Goal location in task space
% g2 = [-0.2995,-0.7249, 0.6203, 0.8718;
%       -0.2933, 0.6886, 0.6631, 0.4725;
%       -0.9079, 0.0167,-0.4189, 0.1847;
%             0,      0,      0, 1.0000];
g2 = [g(1:3, 1:3), [window_position(1)+0.050; window_position(2); window_position(3)]; zeros(1, 3), 1];
plot3(g2(1, 4), g2(2, 4), g2(3, 4), 'b*');
B = Mat2DQ(g2);

% Set the bounds of unknowns for PATH Solver
for j = 1 : 14
    l(j) = -Inf; u(j) = Inf;
end
l(1, 8:14) = 0;   % sice complementarity velocity is always >= 0 
z = zeros(14, 1); % first 7 are joint_q and last 7 are compensating velocities
q_o = theta1;     % initial config

% Main loop of iterative motion plan using PATH solver
currentDQ = A;
finalDQ = B;
[g, ~, ~, ~, g_intermediate] = FK_URDF(q_o);
not_reached_flag = true;

while not_reached_flag
    % Compute g_tau and DQ tau    
    [~, ResDQ] = Screw_Lin(currentDQ, finalDQ, Tau);
    % Move end effector to g_tau by approximating joint angles   
    J_st = JS_URDF(q_o);
    [thetha_next, v_ts2js] = theta_next_step2(J_st, g, ResDQ, q_o, beta);
    [contact_normal, dista, jcon_array] = get_collision_info4(g_intermediate, box_vertices_array, box_normals_array);
    [z, f, J] = pathmcp(z, l, u, 'mcpfuncjacEval');
    q_o = z(1:dof);
    [g, ~, ~, ~, g_intermediate] = FK_URDF(q_o);
    currentDQ = Mat2DQ(g);
    count = count+1;
    theta_SCLERP(:, count) = q_o;

    % Check how far from goal
    [m, position_err] = distDQ(ResDQ, currentDQ);

    if position_err < reach_dist
        not_reached_flag = false;
    end
    
    %/////////////// Plot the motion ///////////////    
    % Plot motion
    plot_motion(hfig);
    
end
fprintf("Reached\n");
close(myVideo);

function[] =  plot_motion(hfig)
    global link1_cylinder1; global link2_cylinder2; global link3_cylinder3;
    global link4_cylinder4; global link5_cylinder5; global link6_cylinder6;
    global link7_cylinder7; global link8_cylinder8; global link9_cylinder9;
    global link10_cylinder10; global n_pts; global g; global contact_pts_array;
    global count;
    %/////////////////////////////////////////////////////////////
    %///////////////////// Plotting //////////////////////////////
    %/////////////////////////////////////////////////////////////
    % Plot the links as cylinders
    link1 = surf(reshape(link1_cylinder1(1, :), [2, n_pts]), reshape(link1_cylinder1(2, :), [2, n_pts]), reshape(link1_cylinder1(3, :), [2, n_pts]), 'FaceColor', [0.9, 0.2, 0.2], 'Facealpha', 0.8, 'EdgeColor', 'none');
    hold on;
    offset1 = surf(reshape(link2_cylinder2(1, :), [2, n_pts]), reshape(link2_cylinder2(2, :), [2, n_pts]), reshape(link2_cylinder2(3, :), [2, n_pts]), 'FaceColor', [0.9, 0.2, 0.2], 'Facealpha', 0.8, 'EdgeColor', 'none');
    link2 = surf(reshape(link3_cylinder3(1, :), [2, n_pts]), reshape(link3_cylinder3(2, :), [2, n_pts]), reshape(link3_cylinder3(3, :), [2, n_pts]), 'FaceColor', [0.2, 0.2, 0.9], 'Facealpha', 0.8, 'EdgeColor', 'none');
    link3 = surf(reshape(link4_cylinder4(1, :), [2, n_pts]), reshape(link4_cylinder4(2, :), [2, n_pts]), reshape(link4_cylinder4(3, :), [2, n_pts]), 'FaceColor', [0.9, 0.2, 0.2], 'Facealpha', 0.8, 'EdgeColor', 'none');
    offset2 = surf(reshape(link5_cylinder5(1, :), [2, n_pts]), reshape(link5_cylinder5(2, :), [2, n_pts]), reshape(link5_cylinder5(3, :), [2, n_pts]), 'FaceColor', [0.9, 0.2, 0.2], 'Facealpha', 0.8, 'EdgeColor', 'none');
    link4 = surf(reshape(link6_cylinder6(1, :), [2, n_pts]), reshape(link6_cylinder6(2, :), [2, n_pts]), reshape(link6_cylinder6(3, :), [2, n_pts]), 'FaceColor', [0.2, 0.2, 0.9], 'Facealpha', 0.8, 'EdgeColor', 'none');
    link5 = surf(reshape(link7_cylinder7(1, :), [2, n_pts]), reshape(link7_cylinder7(2, :), [2, n_pts]), reshape(link7_cylinder7(3, :), [2, n_pts]), 'FaceColor', [0.9, 0.2, 0.2], 'Facealpha', 0.8, 'EdgeColor', 'none');
    offset3 = surf(reshape(link8_cylinder8(1, :), [2, n_pts]), reshape(link8_cylinder8(2, :), [2, n_pts]), reshape(link8_cylinder8(3, :), [2, n_pts]), 'FaceColor', [0.9, 0.2, 0.2], 'Facealpha', 0.8, 'EdgeColor', 'none');
    link6 = surf(reshape(link9_cylinder9(1, :), [2, n_pts]), reshape(link9_cylinder9(2, :), [2, n_pts]), reshape(link9_cylinder9(3, :), [2, n_pts]), 'FaceColor', [0.2, 0.2, 0.9], 'Facealpha', 0.8, 'EdgeColor', 'none');
    link7 = surf(reshape(link10_cylinder10(1, :), [2, n_pts]), reshape(link10_cylinder10(2, :), [2, n_pts]), reshape(link10_cylinder10(3, :), [2, n_pts]), 'FaceColor', [0.9, 0.2, 0.2], 'Facealpha', 0.8, 'EdgeColor', 'none');

%     % Draw the frames
%     frame_obj_arr = [];
%     for i = 1:8
%         T = g_intermediate(:, :, i);
%         frame_obj_arr(i) = draw_frame(T(1:3, 1:3), T(1:3, 4));
%     end
    
%     title("Baxter left arm model");
%     xlabel("x [m]");
%     ylabel("y [m]");
%     zlabel("z [m]");
%     view(164, 47);
    
    % Comment this part if you do not need to visualize the closest points.
    line_obj_arr = []; obj_count = 1;
    for i = 1:7
        cnt_pts = contact_pts_array(:, :, i);

        % Plot the closest point pair
        line_obj_arr(obj_count) = plot3([cnt_pts(1, 1), cnt_pts(1, 2)], [cnt_pts(2, 1), cnt_pts(2, 2)], [cnt_pts(3, 1), cnt_pts(3, 2)], 'ko');
        line_obj_arr(obj_count+1) = plot3([cnt_pts(1, 1), cnt_pts(1, 2)], [cnt_pts(2, 1), cnt_pts(2, 2)], [cnt_pts(3, 1), cnt_pts(3, 2)], 'g');
        obj_count = obj_count + 2;
    end
    
%     % Plot end effector poses
%     if mod(count, 10) == 0
%         quiver3(g(1, 4), g(2, 4), g(3, 4), g(1, 1), g(2, 1), g(3, 1), 0.05, 'r');
%         quiver3(g(1, 4), g(2, 4), g(3, 4), g(1, 2), g(2, 2), g(3, 2), 0.05, 'g');
%         quiver3(g(1, 4), g(2, 4), g(3, 4), g(1, 3), g(2, 3), g(3, 3), 0.05, 'b');
%     end
    
    drawnow;
    
    % delete drawn objects
    delete(link1); delete(link2); delete(link3); delete(link4); delete(link5);
    delete(link6); delete(link7); delete(offset1); delete(offset2); delete(offset3);
    
    for i = 1:obj_count-1
        delete(line_obj_arr(i));
    end
    
%     for i = 1:8
%         delete(frame_obj_arr(i));
%     end
%     drawnow;
end
