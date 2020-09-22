function [jcon] = contact_jacobian_grasped(conpt)
    global J_st;
    
    P_hat=[0,-conpt(3),conpt(2); 
           conpt(3),0,-conpt(1); 
          -conpt(2),conpt(1),0];
    Js = J_st;
    temp = [eye(3,3) -P_hat]*Js;
    jcon = [temp; zeros(3, size(temp, 2))];
end