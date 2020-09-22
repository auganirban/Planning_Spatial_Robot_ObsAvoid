function [J_st,w,q,g_st,R,g0] = jacobian_baxter_URDF(theta)
    %//////////////////////////////////////////////////////////
    %////////////  Joint information for left arm /////////////
    %//////////////////////////////////////////////////////////
    % Joint origins wrt base frame
    q = [0.064, 0.259, 0.130;
        0.113, 0.308, 0.400;
        0.185, 0.380, 0.400;
        0.371, 0.566, 0.331;
        0.444, 0.639, 0.331;
        0.635, 0.830, 0.321;
        0.717, 0.912, 0.321;
        0.882, 1.077, 0.321]'; % s0, s1, e0, e1. w0, w1, w2, gripper

%     % Joint orientations wrt base frame
%     Orientation = [0.924, 0.000, 0.000, 0.383;
%                    0.653, -0.653, -0.271, 0.271;
%                    0.653, -0.271, 0.653, 0.271;
%                    0.653, -0.653, -0.271, 0.271;
%                    0.653, -0.271, 0.653, 0.271;
%                    0.653, -0.653, -0.271, 0.271;
%                    0.653, -0.271, 0.653, 0.271;
%                    0.653, -0.271, 0.653, 0.271]; % s0, s1, e0, e1. w0, w1, w2, gripper
               
    % Joint orientations wrt base frame
    Orientation = [0.000, 0.000, 0.383, 0.924;
                   -0.653, -0.271, 0.271, 0.653;
                   -0.271, 0.653, 0.271, 0.653;
                   -0.653, -0.271, 0.271, 0.653;
                   -0.271, 0.653, 0.271, 0.653;
                   -0.653, -0.271, 0.271, 0.653;
                   -0.271, 0.653, 0.271, 0.653;
                   -0.271, 0.653, 0.271, 0.653]; % s0, s1, e0, e1. w0, w1, w2, gripper
    
%     R = quat2rotm(Orientation);
    R = QuaternionToMatrix(Orientation);
    for i = 1:8
        w(:, i) = R(:, 3, i);
    end

    p_ab = q(:, 8);
    g0 = [R(:, :, 8), p_ab; 0 0 0 1];
    g_ab = eye(4, 4);
    
    for i = 1:7
        [e(:,:,i),xi(:,:,i),xih(:,:,i)] = expon(w(:,i), q(:,i), theta(i));
        g_ab = g_ab*e(:,:,i); % Forward Kinematics
    end
    
    g_st = g_ab*g0;
    p = g_st(1:3,4);
    % Compute columns of Jacobian matrix (2nd column and afterwards)
    A = eye(4,4);
    for i = 2:7
        Ad_g(:,:,i) = A*e(:,:,i-1);
        p = Ad_g(1:3,4,i);
        ph = hat(p);
        R = Ad_g(1:3,1:3,i);
        pR = ph*R;
        g(:,:,i) = [R pR ; zeros(3,3) R];
        xi_dash(:,:,i) = g(:,:,i)*xi(:,:,i);
        A = Ad_g(:,:,i);
    end

    %Jacobian
    J_st = [xi(:,:,1) xi_dash(:,:,2) xi_dash(:,:,3) xi_dash(:,:,4) xi_dash(:,:,5) xi_dash(:,:,6) xi_dash(:,:,7)];
    
    
end