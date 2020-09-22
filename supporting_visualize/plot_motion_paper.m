function[] =  plot_motion_paper(hfig)
    global link1_cylinder1; global link2_cylinder2; global link3_cylinder3;
    global link4_cylinder4; global link5_cylinder5; global link6_cylinder6;
    global link7_cylinder7; global link8_cylinder8; global link9_cylinder9;
    global link10_cylinder10; global n_pts;         global grasped_obj_vertx;
    global grasped_box;     global g_intermediate;
    
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

    if ~isempty(grasped_box)
        grasped_obj_vertx = g_intermediate(1:3, 1:3, 8)*grasped_box.blk_vrtx' + g_intermediate(1:3, 4, 8);
        grasped_obj_nrml = g_intermediate(1:3, 1:3, 8)*grasped_box.blk_nrm';
        grasped_obj_bvec = cmp_bvec(grasped_obj_nrml, grasped_obj_vertx);
        grasped_obj_con = 1;
    end

    if ~isempty(grasped_obj_vertx)
        grasp_plane1 = poly_rectangle2(grasped_obj_vertx(:, 1), grasped_obj_vertx(:, 2), grasped_obj_vertx(:, 3), grasped_obj_vertx(:, 4), 1);
        grasp_plane2 = poly_rectangle2(grasped_obj_vertx(:, 1), grasped_obj_vertx(:, 2), grasped_obj_vertx(:, 5), grasped_obj_vertx(:, 8), 1);
        grasp_plane3 = poly_rectangle2(grasped_obj_vertx(:, 5), grasped_obj_vertx(:, 6), grasped_obj_vertx(:, 7), grasped_obj_vertx(:, 8), 1);
        grasp_plane4 = poly_rectangle2(grasped_obj_vertx(:, 3), grasped_obj_vertx(:, 4), grasped_obj_vertx(:, 7), grasped_obj_vertx(:, 6), 1);
        grasp_plane5 = poly_rectangle2(grasped_obj_vertx(:, 2), grasped_obj_vertx(:, 3), grasped_obj_vertx(:, 6), grasped_obj_vertx(:, 5), 1);
        grasp_plane6 = poly_rectangle2(grasped_obj_vertx(:, 1), grasped_obj_vertx(:, 4), grasped_obj_vertx(:, 7), grasped_obj_vertx(:, 8), 1);
    end

end