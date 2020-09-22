function [] = extract_rrt_path(closest_node_indx)
    global tree;
    
    % Extract the path from the tree
    path_node_indx = [tree.node_count];
    current_parent = closest_node_indx;
    while current_parent > 0
         current_parent = tree.nodes(current_parent, end);
         path_node_indx = [path_node_indx, current_parent];
    end
    disp(path_node_indx);
    path_node_indx_reversed = flip(path_node_indx);
    fprintf("Path written in file named my_data.mat \n");
    save('my_data.mat', 'path_node_indx_reversed', 'tree');
end