function [not_reached_flag, old_small] = addNodes2TreeStep(g_collision_free, closest_node_indx, q_o, old_small, B)
    % Function that adds a nodeto the tree
    global tree; global count; global not_reached_flag; global closest_sofar_index;
    
    tree.nodes = [tree.nodes; g_collision_free(1:3, 4)', rotm2quat(g_collision_free(1:3, 1:3)), closest_node_indx];
    tree.nodesDQ = [tree.nodesDQ; Mat2DQ(g_collision_free)];
    tree.nodesAng = [tree.nodesAng; q_o'];
    tree.node_count = tree.node_count + 1;
          
   %%%%%%%%%%%%%%%%%%%% Compute error %%%%%%%%%%%%%%%%%%%%
   [rotational_err, translation_err] = distDQ(tree.nodesDQ(end, :), B);
   count = count + 1;

   new_small = translation_err;
   if new_small < old_small
       old_small = new_small;
       closest_sofar_index = tree.node_count;
   end

   if translation_err < 0.030
       not_reached_flag = false;
   end
end