clc;
clear;
close all;

% To do: Convert this into a function with th as input argument.

% Add custom libraries to the Matlab path
addpath("supporting_kinlib");

% Joint angle vector
% th = [0.3071796527740541, -1.13054384067155, -0.5039126888203584, 2.4585877077833467, 0.8130098175792693, -1.2160632695961617, -0.12233496783386175];
th = zeros(1, 7);

%////////////////////////////////////
%      Solve forward kinematics    //
%////////////////////////////////////
[g_current, ~, ~, ~, g_intermediate] = FK_URDF(th);

% /////////// Link 1 ////////////
rc = 0.010;
[X, Y, Z] = cylinder(rc);
hc1 = g_intermediate(3, 4, 2) - g_intermediate(3, 4, 1);
Z1 = hc1 * Z;
cylinder_pts1 = [reshape(X, [1, 42]); reshape(Y, [1, 42]); reshape(Z1, [1, 42])];
link1_cylinder1 = g_intermediate(1:3, 1:3, 1) * cylinder_pts1 + g_intermediate(1:3, 4, 1);
surf(reshape(link1_cylinder1(1, :), [2, 21]), reshape(link1_cylinder1(2, :), [2, 21]), reshape(link1_cylinder1(3, :), [2, 21]), 'FaceColor', [0.9, 0.2, 0.2], 'Facealpha', 0.8, 'EdgeColor', 'none');
hold on;

% /////////////// Link 2 ////////////
Z2_dir = g_intermediate(1:3, 4, 2) - (g_intermediate(1:3, 4, 1) + [0; 0; hc1]);
hc2 = norm(Z2_dir);
Z_ax = Z2_dir/hc2;
X_ax = -g_intermediate(1:3, 3);
Y_ax = cross(Z_ax, X_ax);
cylinder_pts2 = [reshape(X, [1, 42]); reshape(Y, [1, 42]); reshape(hc2 * Z, [1, 42])];
link2_cylinder2 = [X_ax, Y_ax, Z_ax] * cylinder_pts2 + (g_intermediate(1:3, 4, 1) + [0; 0; hc1]);
surf(reshape(link2_cylinder2(1, :), [2, 21]), reshape(link2_cylinder2(2, :), [2, 21]), reshape(link2_cylinder2(3, :), [2, 21]), 'FaceColor', [0.9, 0.2, 0.2], 'Facealpha', 0.8, 'EdgeColor', 'none');

% //////////////// Link 3 ///////////////
hc3 = 0.005;
[X, Y, Z] = cylinder(norm(g_intermediate(1:3, 4, 3) - g_intermediate(1:3, 4, 2)));
cylinder_pts3 = [reshape(X, [1, 42]); reshape(Y, [1, 42]); reshape(hc3 * Z, [1, 42])];
link3_cylinder3 = g_intermediate(1:3, 1:3, 2) * cylinder_pts3 + g_intermediate(1:3, 4, 2);
surf(reshape(link3_cylinder3(1, :), [2, 21]), reshape(link3_cylinder3(2, :), [2, 21]), reshape(link3_cylinder3(3, :), [2, 21]), 'FaceColor', [0.2, 0.2, 0.9], 'Facealpha', 0.8, 'EdgeColor', 'none');

% /////////////// Link 4 /////////////////
hc4 = sqrt((norm(g_intermediate(1:3, 4, 4) - g_intermediate(1:3, 4, 3)))^2 - (0.069)^2);
rc4 = 0.010;
[X, Y, Z] = cylinder(rc4);
cylinder_pts4 = [reshape(X, [1, 42]); reshape(Y, [1, 42]); reshape(hc4 * Z, [1, 42])];
link4_cylinder4 = g_intermediate(1:3, 1:3, 3) * cylinder_pts4 + g_intermediate(1:3, 4, 3);
surf(reshape(link4_cylinder4(1, :), [2, 21]), reshape(link4_cylinder4(2, :), [2, 21]), reshape(link4_cylinder4(3, :), [2, 21]), 'FaceColor', [0.9, 0.2, 0.2], 'Facealpha', 0.8, 'EdgeColor', 'none');

% //////////////// Link 5 //////////////////
hc5 = 0.069;
rc5 = 0.010;
[X, Y, Z] = cylinder(rc5);
cylinder_pts5 = [reshape(X, [1, 42]); reshape(Y, [1, 42]); reshape(hc5 * Z, [1, 42])];
temp_pt = g_intermediate(1:3, 4, 3) + hc4 * g_intermediate(1:3, 3, 3);
Z_ax = temp_pt - g_intermediate(1:3, 4, 4); Z_ax = Z_ax / norm(Z_ax);
X_ax = g_intermediate(1:3, 3, 4);
Y_ax = cross(Z_ax, X_ax);
link5_cylinder5 = [X_ax, Y_ax, Z_ax] * cylinder_pts5 + g_intermediate(1:3, 4, 4);
surf(reshape(link5_cylinder5(1, :), [2, 21]), reshape(link5_cylinder5(2, :), [2, 21]), reshape(link5_cylinder5(3, :), [2, 21]), 'FaceColor', [0.9, 0.2, 0.2], 'Facealpha', 0.8, 'EdgeColor', 'none');

% //////////////////// Link 6 ///////////////////
hc6 = 0.010;
[X, Y, Z] = cylinder(norm(g_intermediate(1:3, 4, 4) - g_intermediate(1:3, 4, 5)));
cylinder_pts6 = [reshape(X, [1, 42]); reshape(Y, [1, 42]); reshape(hc6 * Z, [1, 42])];
link6_cylinder6 = g_intermediate(1:3, 1:3, 4) * cylinder_pts6 + g_intermediate(1:3, 4, 4);
surf(reshape(link6_cylinder6(1, :), [2, 21]), reshape(link6_cylinder6(2, :), [2, 21]), reshape(link6_cylinder6(3, :), [2, 21]), 'FaceColor', [0.2, 0.2, 0.9], 'Facealpha', 0.8, 'EdgeColor', 'none');

% //////////////////// Link 7 ////////////////////
hc7 = sqrt((norm(g_intermediate(1:3, 4, 6) - g_intermediate(1:3, 4, 5)))^2 - (0.010)^2);
rc7 = 0.010;
[X, Y, Z] = cylinder(rc7);
cylinder_pts7 = [reshape(X, [1, 42]); reshape(Y, [1, 42]); reshape(hc7 * Z, [1, 42])];
link7_cylinder7 = g_intermediate(1:3, 1:3, 5) * cylinder_pts7 + g_intermediate(1:3, 4, 5);
surf(reshape(link7_cylinder7(1, :), [2, 21]), reshape(link7_cylinder7(2, :), [2, 21]), reshape(link7_cylinder7(3, :), [2, 21]), 'FaceColor', [0.9, 0.2, 0.2], 'Facealpha', 0.8, 'EdgeColor', 'none');

% ////////////////////// Link 8 ///////////////////
hc8 = 0.010;
rc8 = 0.010;
[X, Y, Z] = cylinder(rc8);
cylinder_pts8 = [reshape(X, [1, 42]); reshape(Y, [1, 42]); reshape(hc8 * Z, [1, 42])];
temp_pt = g_intermediate(1:3, 4, 5) + hc7 * g_intermediate(1:3, 3, 5);
Z_ax = temp_pt - g_intermediate(1:3, 4, 6); Z_ax = Z_ax / norm(Z_ax);
X_ax = g_intermediate(1:3, 3, 6);
Y_ax = cross(Z_ax, X_ax); Y_ax = Y_ax/norm(Y_ax);
link8_cylinder8 = [X_ax, Y_ax, Z_ax] * cylinder_pts8 + g_intermediate(1:3, 4, 6);
surf(reshape(link8_cylinder8(1, :), [2, 21]), reshape(link8_cylinder8(2, :), [2, 21]), reshape(link8_cylinder8(3, :), [2, 21]), 'FaceColor', [0.9, 0.2, 0.2], 'Facealpha', 0.8, 'EdgeColor', 'none');

% //////////////////////// Link 9 //////////////////////
hc9 = 0.010;
[X, Y, Z] = cylinder(norm(g_intermediate(1:3, 4, 7) - g_intermediate(1:3, 4, 6)));
cylinder_pts9 = [reshape(X, [1, 42]); reshape(Y, [1, 42]); reshape(hc9 * Z, [1, 42])];
link9_cylinder9 = g_intermediate(1:3, 1:3, 6) * cylinder_pts9 + g_intermediate(1:3, 4, 6);
surf(reshape(link9_cylinder9(1, :), [2, 21]), reshape(link9_cylinder9(2, :), [2, 21]), reshape(link9_cylinder9(3, :), [2, 21]), 'FaceColor', [0.2, 0.2, 0.9], 'Facealpha', 0.8, 'EdgeColor', 'none');

% /////////////////////// Link 10 ///////////////////////
rc10 = 0.010;
hc10 = norm(g_intermediate(1:3, 4, 8) - g_intermediate(1:3, 4, 7));
[X, Y, Z] = cylinder(rc10);
cylinder_pts10 = [reshape(X, [1, 42]); reshape(Y, [1, 42]); reshape(hc10 * Z, [1, 42])];
link10_cylinder10 = g_intermediate(1:3, 1:3, 7) * cylinder_pts10 + g_intermediate(1:3, 4, 7);
surf(reshape(link10_cylinder10(1, :), [2, 21]), reshape(link10_cylinder10(2, :), [2, 21]), reshape(link10_cylinder10(3, :), [2, 21]), 'FaceColor', [0.9, 0.2, 0.2], 'Facealpha', 0.8, 'EdgeColor', 'none');

xlabel("x [m]");
ylabel("y [m]");
zlabel("z [m]");
view(20, 12);


% % Plot robot arm
%     line_obj_arr = []; bubble_obj_arr = [];
% for i = 1:7
%     line_obj_arr(i) = line([g_intermediate(1, 4, i), g_intermediate(1, 4, i+1)], [g_intermediate(2, 4, i), g_intermediate(2, 4, i+1)],[g_intermediate(3, 4, i), g_intermediate(3, 4, i+1)], 'Color', 'blue', 'LineWidth', 4);
%     bubble_obj_arr(i) = bubbleplot3(g_intermediate(1, 4, i), g_intermediate(2, 4, i), g_intermediate(3, 4, i), 0.01, [0 1 0], 1, 150, 150);
% end
% 
% xlim([-0.05, 0.80]);
% ylim([0.20, 0.60]);
% zlim([0.00, 1.00]);
% xlabel("x [m]");
% ylabel("y [m]");
% zlabel("z [m]");

