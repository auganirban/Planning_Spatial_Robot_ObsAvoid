clc;
clear;
close all;

% Input: Theta_initial, Goal_pose
% Output: Sequence of joint angle vectors.
% Date: Novemver 27th., 2019.
% Brief description: Manipulator holding a rectangular box should move from
% its given start to a desired goal pose while always trying to move
% towards goal but avoiding obstacle in tis path if any. The initial and
% goal poses are at the two opposite sides of a wall and the robot has to
% move through the window opening in the wall.

% Add path to PATH solver
addpath("../supporting_quatlib"); addpath("../supporting_colli_funcs");
addpath("../supporting_kinlib");  addpath("../pathmexa64"); addpath("../supporting_colli_models");

% Define global variables
global g;              global q_o;         global h;              global v_ts2js;
global dof;            global dista;       global safe_dist;      global count;
global contact_normal; global jcon_array;  global g_intermediate; global task_dimen;
global J_st;           global grasped_box;

grasped_box = load("box_grasped29.mat");

hfig  = figure(1); hold on; view(164, 47);
title("Baxter left arm transferring object through the window"); xlabel("x [m]"); ylabel("y [m]"); zlabel("z [m]");

% Initialize video (uncomment if you need to record the video)
myVideo = VideoWriter('myVideoFile2'); %open video file
myVideo.FrameRate = 30;  %can adjust this, 5 - 10 works well for me
open(myVideo)

%////////////////////// Inputs/////////////////////////
% works
% theta1 = [-0.2353, -0.9611, -0.6287, 2.4577, 0.1458, -1.2846, 0.3167]';
% theta1 = [-0.2438, -0.9710, -0.6289, 2.3474, 0.1774, -1.1418, 0.2963]';
% theta1 = [-0.4195, -1.0417, -0.4153, 2.3650, 0.1965, -1.1688, 0.9043]';
theta1 = [0.8505, -1.0595, -0.6603, 2.3772, 0.6860, -1.1455, -0.0778]';
window_position = [0.850; 0.20; 0.30]; % Joint positions of the left arm of Baxter robot
[~, box_vertices_array, box_normals_array] = get_obstacle_wall(window_position);

% Planning algorithm parameters
beta = 0.02; Tau = 0.5; h = 0.01; reach_dist = 0.010;
dof = 7; safe_dist = 0.009; num_of_contacts = 7; count = 1; task_dimen = 6;

% Initial loaction in task space
g = FK_URDF(theta1); A = Mat2DQ(g);

% Goal location in task space
% g2 = [g(1:3, 1:3)*rotm_z(pi/4), [0.7986; 0.2022; 0.3005]; zeros(1, 3), 1];
% g2 = [-0.2995,-0.7249, 0.6203, 0.8718;
%       -0.2933, 0.6886, 0.6631, 0.4725;
%       -0.9079, 0.0167,-0.4189, 0.1847;
%             0,      0,      0, 1.0000];
theta2 = [-0.7512, -0.2385, -0.3903, 0.7972, 0.0548, -0.6028, 0.2899];
g2 = FK_URDF(theta2);
% g2 = [g(1:3, 1:3), [window_position(1)+0.050; window_position(2); window_position(3)]; zeros(1, 3), 1];
% temp_rot = [-0.0123, 0.1381, 0.9903; -0.0578, 0.9887, -0.1386; -0.9983, -0.0589, -0.0042];
% g2 = [temp_rot, [window_position(1)+0.050; window_position(2); window_position(3)]; zeros(1, 3), 1];
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
temp_goal1_reached = true;
temp_goal2_reached = true;
pos_err_imprv = [];
rot_err_imprv = [];
tilt = pi/8;
mov_back = 0.05;
incr_tilt = 0.05;
while not_reached_flag || ~temp_goal1_reached || ~temp_goal2_reached
    % Compute g_tau and DQ tau
    if temp_goal1_reached && temp_goal2_reached
        goalDQ = finalDQ;
        [~, ResDQ] = Screw_Lin(currentDQ, finalDQ, Tau);
    elseif temp_goal2_reached
        goalDQ = temp1DQ;
        [~, ResDQ] = Screw_Lin(currentDQ, temp1DQ, Tau);
    else
        goalDQ = temp2DQ;
        [~, ResDQ] = Screw_Lin(currentDQ, temp2DQ, Tau);
    end
    
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
    [rotation_err, position_err] = distDQ(currentDQ, goalDQ);
    
    for chk = 1:length(pos_err_imprv)
        if position_err > pos_err_imprv(chk) && temp_goal1_reached
            temp_goal1 = [g(1:3, 1:3)*rotm_z(tilt), g(1:3, 4); zeros(1, 3), 1];
            temp1DQ = Mat2DQ(temp_goal1);
            temp_goal1_reached = false;
            fprintf("temp_goal1 is created. \n");
            pos_err_imprv = [];
            break;
        end
    end
    
    for chk2 = 1:length(rot_err_imprv)
        if rotation_err > rot_err_imprv(chk2) && temp_goal2_reached && ~temp_goal1_reached
            temp_goal2 = [g(1:3, 1:3), [g(1, 4)-mov_back; g(2, 4); g(3, 4)]; zeros(1, 3), 1];
            temp2DQ = Mat2DQ(temp_goal2);
            % temp_goal1_reached = true;
            temp_goal2_reached = false;
            fprintf("temp_goal2 is created. \n");
            rot_err_imprv = [];
            break;
        end
    end
    
    if length(pos_err_imprv) < 10
        pos_err_imprv = [pos_err_imprv, position_err];
    else
        pos_err_imprv = [pos_err_imprv(2:end), position_err];
    end
    
    if length(rot_err_imprv) < 10
        rot_err_imprv = [rot_err_imprv, rotation_err];
    else
        rot_err_imprv = [rot_err_imprv(2:end), rotation_err];
    end

    if position_err < reach_dist && rotation_err < 0.02
        if temp_goal1_reached && temp_goal2_reached
            not_reached_flag = false;
        else
            if ~temp_goal2_reached
                temp_goal2_reached = true;
                fprintf("Temp goal2 achieved\n");
                rot_err_imprv = [];
                % Set temp_goal2 from this pose
                temp_goal1 = [g(1:3, 1:3)*rotm_z(tilt), g(1:3, 4); zeros(1, 3), 1];
                temp1DQ = Mat2DQ(temp_goal1);
            else
                temp_goal1_reached = true;
                fprintf("Temp goal1 achieved\n");
                pos_err_imprv = [];
            end
        end
    end
    
    %/////////////// Plot the motion ///////////////    
    % Plot motion
    plot_motion(hfig, myVideo);
%     plot_motion(hfig);
    
end
fprintf("Reached\n");
close(myVideo);

% function[] =  plot_motion(hfig)
function[] =  plot_motion(hfig, myVideo)
    global link1_cylinder1; global link2_cylinder2; global link3_cylinder3;
    global link4_cylinder4; global link5_cylinder5; global link6_cylinder6;
    global link7_cylinder7; global link8_cylinder8; global link9_cylinder9;
    global link10_cylinder10; global n_pts; global g; global contact_pts_array;
    global count; global grasped_obj_vertx;
    
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

    if ~isempty(grasped_obj_vertx)
        grasp_plane1 = poly_rectangle2(grasped_obj_vertx(:, 1), grasped_obj_vertx(:, 2), grasped_obj_vertx(:, 3), grasped_obj_vertx(:, 4), 1);
        grasp_plane2 = poly_rectangle2(grasped_obj_vertx(:, 1), grasped_obj_vertx(:, 2), grasped_obj_vertx(:, 5), grasped_obj_vertx(:, 8), 1);
        grasp_plane3 = poly_rectangle2(grasped_obj_vertx(:, 5), grasped_obj_vertx(:, 6), grasped_obj_vertx(:, 7), grasped_obj_vertx(:, 8), 1);
        grasp_plane4 = poly_rectangle2(grasped_obj_vertx(:, 3), grasped_obj_vertx(:, 4), grasped_obj_vertx(:, 7), grasped_obj_vertx(:, 6), 1);
        grasp_plane5 = poly_rectangle2(grasped_obj_vertx(:, 2), grasped_obj_vertx(:, 3), grasped_obj_vertx(:, 6), grasped_obj_vertx(:, 5), 1);
        grasp_plane6 = poly_rectangle2(grasped_obj_vertx(:, 1), grasped_obj_vertx(:, 4), grasped_obj_vertx(:, 7), grasped_obj_vertx(:, 8), 1);
    end
    
    drawnow;
    
    frame = getframe(gcf); %get frame
    writeVideo(myVideo, frame);
    
    % delete drawn objects
    delete(link1); delete(link2); delete(link3); delete(link4); delete(link5);
    delete(link6); delete(link7); delete(offset1); delete(offset2); delete(offset3);
    
    for i = 1:obj_count-1
        delete(line_obj_arr(i));
    end
    
    if ~isempty(grasped_obj_vertx)
        delete(grasp_plane1); delete(grasp_plane2); delete(grasp_plane3);
        delete(grasp_plane4); delete(grasp_plane5); delete(grasp_plane6);
    end
    
%     for i = 1:8
%         delete(frame_obj_arr(i));
%     end
%     drawnow;
end

function [rotm] = rotm_z(ang)
    rotm = [cos(ang), -sin(ang), 0; sin(ang), cos(ang), 0; 0, 0, 1];
end