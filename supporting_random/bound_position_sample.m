function [v1_min, v1_max, v2_min, v2_max] = bound_position_sample(window_position, box_h, box_w, box_d, wall_thickness)
    % Front sampling box
    vertex_A1 = [window_position(1)-0.5*wall_thickness; window_position(2)+box_w*0.5; window_position(3)+box_h*0.5];
    vertex_B1 = [window_position(1)-0.5*wall_thickness-box_d; window_position(2)+box_w*0.5; window_position(3)+box_h*0.5];
    vertex_C1 = [window_position(1)-0.5*wall_thickness-box_d; window_position(2)+box_w*0.5; window_position(3)-box_h*0.5];
    vertex_D1 = [window_position(1)-0.5*wall_thickness; window_position(2)+box_w*0.5; window_position(3)-box_h*0.5];
    vertex_E1 = [window_position(1)-0.5*wall_thickness; window_position(2)-box_w*0.5;window_position(3)+box_h*0.5];
    vertex_F1 = [window_position(1)-0.5*wall_thickness-box_d; window_position(2)-box_w*0.5; window_position(3)+box_h*0.5];
    vertex_G1 = [window_position(1)-0.5*wall_thickness-box_d; window_position(2)-box_w*0.5; window_position(3)-box_h*0.5];
    vertex_H1 = [window_position(1)-0.5*wall_thickness; window_position(2)-box_w*0.5; window_position(3)-box_h*0.5];
    box1_vertices = [vertex_A1, vertex_B1, vertex_C1, vertex_D1, vertex_E1, vertex_F1];
    
    v1_min = min(box1_vertices, [], 2) - 0.005;
    v1_max = max(box1_vertices, [], 2) + 0.005;
    
    % Draw the bounding box in fornt of the wall
% % %     poly_rectangle(vertex_A1, vertex_B1, vertex_C1, vertex_D1, 1);
% % %     poly_rectangle(vertex_A1, vertex_B1, vertex_F1, vertex_E1, 1);
% % %     poly_rectangle(vertex_A1, vertex_B1, vertex_C1, vertex_D1, 1);
% % %     poly_rectangle(vertex_D1, vertex_C1, vertex_G1, vertex_H1, 1);
% % %     poly_rectangle(vertex_E1, vertex_F1, vertex_G1, vertex_H1, 1);
% % %     poly_rectangle(vertex_A1, vertex_D1, vertex_H1, vertex_E1, 1);
% % %     poly_rectangle(vertex_B1, vertex_C1, vertex_G1, vertex_F1, 1);
    
    % Back sampling box
    vertex_A2 = [window_position(1)+0.5*wall_thickness; window_position(2)+box_w*0.5; window_position(3)+box_h*0.5];
    vertex_B2 = [window_position(1)+0.5*wall_thickness+box_d; window_position(2)+box_w*0.5; window_position(3)+box_h*0.5];
    vertex_C2 = [window_position(1)+0.5*wall_thickness+box_d; window_position(2)+box_w*0.5; window_position(3)-box_h*0.5];
    vertex_D2 = [window_position(1)+0.5*wall_thickness; window_position(2)+box_w*0.5; window_position(3)-box_h*0.5];
    vertex_E2 = [window_position(1)+0.5*wall_thickness; window_position(2)-box_w*0.5;window_position(3)+box_h*0.5];
    vertex_F2 = [window_position(1)+0.5*wall_thickness+box_d; window_position(2)-box_w*0.5; window_position(3)+box_h*0.5];
    vertex_G2 = [window_position(1)+0.5*wall_thickness+box_d; window_position(2)-box_w*0.5; window_position(3)-box_h*0.5];
    vertex_H2 = [window_position(1)+0.5*wall_thickness; window_position(2)-box_w*0.5; window_position(3)-box_h*0.5];
    box2_vertices = [vertex_A2, vertex_B2, vertex_C2, vertex_D2, vertex_E2, vertex_F2];

    v2_min = min(box2_vertices, [], 2);
    v2_max = max(box2_vertices, [], 2);

     % Draw the bounding box at the back of the wall
% % %     poly_rectangle(vertex_A2, vertex_B2, vertex_C2, vertex_D2, 1);
% % %     poly_rectangle(vertex_A2, vertex_B2, vertex_F2, vertex_E2, 1);
% % %     poly_rectangle(vertex_A2, vertex_B2, vertex_C2, vertex_D2, 1);
% % %     poly_rectangle(vertex_D2, vertex_C2, vertex_G2, vertex_H2, 1);
% % %     poly_rectangle(vertex_E2, vertex_F2, vertex_G2, vertex_H2, 1);
% % %     poly_rectangle(vertex_A2, vertex_D2, vertex_H2, vertex_E2, 1);
% % %     poly_rectangle(vertex_B2, vertex_C2, vertex_G2, vertex_F2, 1);

end

function poly_rectangle(p1, p2, p3, p4, ele)
    % Function to plot the plane
    % The points must be in the correct sequence.
    % The coordinates must consider x, y and z-axes.
    x = [p1(1) p2(1) p3(1) p4(1)];
    y = [p1(2) p2(2) p3(2) p4(2)];
    z = [p1(3) p2(3) p3(3) p4(3)];
    if ele == 1
        colr = [0.9, 0.4, 0.3];
    else
        colr = [0.5, 0.5, 0.5];
    end
    fill3(x, y, z, colr, 'Facealpha', 0.15);
    hold on
end