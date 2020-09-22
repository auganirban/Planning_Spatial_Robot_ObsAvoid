function[q_rand] = generate_rand_so3()
    u_vec = rand(1, 3);
    q_rand = [sqrt(1-u_vec(1))*sin(2*pi*u_vec(2)), sqrt(1-u_vec(1))*cos(2*pi*u_vec(2)), sqrt(u_vec(1))*sin(2*pi*u_vec(3)), sqrt(u_vec(1))*cos(2*pi*u_vec(3))];
end