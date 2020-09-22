function [dist_indx, old_dist] = find_closest_node(g)
    global tree;
    old_dist = 1000;
    qc = rotm2quat(g(1:3, 1:3));
    for i = 1:tree.node_count
        pos_dist = norm(g(1:3, 4)' - tree.nodes(i, 1:3));
        rot_dist = min(norm(qc + tree.nodes(i, 4:7)), norm(qc - tree.nodes(i, 4:7)));
        new_dist = pos_dist + rot_dist;
        if new_dist < old_dist
            old_dist = new_dist;
            dist_indx = i;
        end
    end
end