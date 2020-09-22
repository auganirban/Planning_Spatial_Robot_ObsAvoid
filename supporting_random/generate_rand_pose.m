%%%%%%%%%%%%%%% Helper functions %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [data] = generate_rand_pose(r_min, r_max)
    data = zeros(3, 1);
    for i = 1:3
        data(i) = (r_max(i)-r_min(i))*rand(1, 1) + r_min(i);
    end
end

