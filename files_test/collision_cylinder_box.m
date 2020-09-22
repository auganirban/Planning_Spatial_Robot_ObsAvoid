clc;
clear;
close all;

rc = 0.020; hc = 0.050;

% Load the cylinder and box data
box_data = load("box_vrtx_nrml.mat");
cyn_data = load("cylinder_data.mat");

% Transform of the cylinder wrt base frame
Rcyn = [1, 0, 0; 0, cos(pi/3), -sin(pi/3); 0, sin(pi/3), cos(pi/3)];
pcyn = [0.05; 0.20; 0.05];
% baseTcyn = [eye(3, 3), [0.05; 0.05; 0]; zeros(1, 3), 1];
baseTcyn = [Rcyn, pcyn; zeros(1, 3), 1];
cyn_pts = baseTcyn(1:3, 1:3) * [reshape(cyn_data.X, [1, 42]); reshape(cyn_data.Y, [1, 42]);  reshape(cyn_data.Z, [1, 42])] + baseTcyn(1:3, 4);

cyn_info.T = baseTcyn;
cyn_info.r = rc;
cyn_info.h = hc;

% Transform of the box frame wrt base frame
baseTbox = [eye(3, 3), [0.2; 0.2; 0]; zeros(1, 3), 1];
box_pts = baseTbox(1:3, 1:3)*box_data.blk_vrtx' + baseTbox(1:3, 4);

% //////////////// Find the closest point pairs ////////////////////
box_normals = baseTbox(1:3, 1:3)*box_data.blk_nrm';
box_bvec = cmp_bvec(box_normals, box_pts);

% Initial guess to solve nonlinear optimization problem
% to find the closest distance between the box and the cylinder
x0 = [box_pts(:, 1); cyn_pts(:, 1)];

% Call collision check method
[x1, x2, min_d] = check_colli(box_normals, box_bvec, x0, baseTcyn, rc, hc);

% ////////////// Visualize /////////////////////////
% Plot the box and the cylinder
box3d(box_pts, 2);
surf(reshape(cyn_pts(1, :), [2, 21]), reshape(cyn_pts(2, :), [2, 21]), reshape(cyn_pts(3, :), [2, 21]), 'FaceColor', [0.5, 0.5, 0.5], 'Facealpha', 0.7);

% Plot the closest point pair
plot3([x1(1), x2(1)], [x1(2), x2(2)], [x1(3), x2(3)], 'ro', [x1(1), x2(1)], [x1(2), x2(2)], [x1(3), x2(3)], 'b');

grid on;
draw_frame(baseTbox(1:3, 1:3), baseTbox(1:3, 4));
draw_frame(baseTcyn(1:3, 1:3), baseTcyn(1:3, 4));
draw_frame(eye(3, 3), [0; 0; 0]);
xlabel("x [m]");
ylabel("y [m]");
zlabel("z [m]");
view(-113, 71);

%//////////////////////////////////////////////////////
%                  Helper Functions                  //
%//////////////////////////////////////////////////////
%% computes b values for all the planes
function [bvec] = cmp_bvec(w, vrtx)
    bvec = [dot(w(:, 1), vrtx(:, 2));
        dot(w(:, 2), vrtx(:, 2));
        dot(w(:, 3), vrtx(:, 2));
        dot(w(:, 4), vrtx(:, 7));
        dot(w(:, 5), vrtx(:, 7));
        dot(w(:, 6), vrtx(:, 7))];
end

%% check collision between box and cylinder 
function [x1, x2, min_d] = check_colli(blk1_N, blk1_b, x0, cynT, cynr, cynh)
%     global baseTcyn; global hc; global rc;
    Rbox = cynT(1:3, 1:3); pbox = cynT(1:3, 4);
    
    % Objective function to minimize
    obj = @(x)norm(x(1:3) - x(4:6))^2;
    
    % Build-up the constraints
    A = [blk1_N', zeros(6, 3); zeros(2, 3), [Rbox(:, 3)'; -Rbox(:, 3)']];
    b = [blk1_b; cynh + Rbox(:, 3)'*pbox; -Rbox(:, 3)'*pbox];
    % nonlcon = @disk_constraint;
    opts = optimoptions('fmincon','Algorithm','interior-point', 'Display', 'off');
    
    % Call MATLAB's built-in fmincon function
%     x = fmincon(obj, x0, A, b, [], [], [], [], nonlcon, opts);
    x = fmincon(obj, x0, A, b, [], [], [], [], @(x)disk_constraint(x, Rbox, pbox, cynr), opts);
    
    % Separate the points on the gripper plate and the peg
    x1 = x(1:3);
    x2 = x(4:6);
    
    % Find the closest distance between the two objects
    min_d = norm(x1 - x2);
end

%% circular cross section constraint of the peg to be used with fmincon
function [c,ceq] = disk_constraint(x, Rbox, pbox, rc)
    % global baseTcyn; global rc;
%     Rbox = baseTcyn(1:3, 1:3); pbox = baseTcyn(1:3, 4);
    temp1 = Rbox(:, 1)'*(x(4:6) - pbox);
    temp2 = Rbox(:, 2)'*(x(4:6) - pbox);
    c = temp1^2 + temp2^2 - (rc)^2;
    ceq = [];
end

%% Draw 3D Box
function box3d(pt, ele)
    poly_rectangle(pt(:, 1), pt(:, 2), pt(:, 3), pt(:, 4), ele);
    poly_rectangle(pt(:, 1), pt(:, 2), pt(:, 5), pt(:, 8), ele);
    poly_rectangle(pt(:, 5), pt(:, 6), pt(:, 7), pt(:, 8), ele);
    poly_rectangle(pt(:, 3), pt(:, 4), pt(:, 7), pt(:, 6), ele);
    poly_rectangle(pt(:, 2), pt(:, 3), pt(:, 6), pt(:, 5), ele);
    poly_rectangle(pt(:, 1), pt(:, 4), pt(:, 7), pt(:, 8), ele);
end

%% Function that plots the gripper plates
function poly_rectangle(p1, p2, p3, p4, ele)
    % The points must be in the correct sequence.
    % The coordinates must consider x, y and z-axes.
    x = [p1(1) p2(1) p3(1) p4(1)];
    y = [p1(2) p2(2) p3(2) p4(2)];
    z = [p1(3) p2(3) p3(3) p4(3)];
    if ele == 1
        colr = [0.9, 0, 0];
    else
        colr = [0.5, 0.5, 0.5];
    end
    fill3(x, y, z, colr, 'Facealpha', 0.7);
    hold on
end

%% Draw frame
function draw_frame(R, p)
    quiver3(p(1), p(2), p(3), R(1, 1), R(2, 1), R(3, 1), 0.025, 'r', 'LineWidth', 2.0);
    quiver3(p(1), p(2), p(3), R(1, 2), R(2, 2), R(3, 2), 0.025, 'g', 'LineWidth', 2.0);
    quiver3(p(1), p(2), p(3), R(1, 3), R(2, 3), R(3, 3), 0.025, 'b', 'LineWidth', 2.0);
end
