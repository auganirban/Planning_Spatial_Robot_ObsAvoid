function [data] = generate_rand_se3(r_min, r_max)
    % First 3 are position and last 3 are orientation
    data = zeros(7, 1);
    for i = 1:3
        data(i) = (r_max(i)-r_min(i))*rand(1, 1) + r_min(i);
    end
    data(4:end) = generate_rand_so3();
end