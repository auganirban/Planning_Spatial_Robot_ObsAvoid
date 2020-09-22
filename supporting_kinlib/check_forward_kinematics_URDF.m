clc;
clear;
close all;

% Joint angle vector of the lef tarm
% theta_l = [-0.54456,-0.33364,-0.01917,1.19688,0.27841,-0.8870243,-0.158767];
% theta_l = [0.1, -0.1, 0.1, 0.1, 0.1, 0.1, 0.1];
% theta_l = [0.1, -0.2, 0.3, 0.1, 0.2, 0.2, 0.1];
theta_l = [0.25349032519806464, -0.6841554313968945, -0.388480634531981, 1.861869181295921, 0.943014689352558, -1.045024411746938, -0.2124563391221298]';

% Solve forward kinematics
[g_st, p, g_st1, g_st2, g_st3, g_st4, g_st5, g_st6, g_st7, g_st8] = forward_kinematics_URDF(theta_l);
fprintf("g_st: \n");
disp(g_st);

[g_st, ~] = FK_URDF(theta_l);
fprintf("g_st: \n");
disp(g_st);

[J_st, w, q, g_st, R, g0] = jacobian_baxter_URDF(theta_l);
fprintf("J_st: \n");
disp(J_st);

[J_st, J_ana] = JS_URDF(theta_l);
fprintf("J_st: \n");
disp(J_st);
