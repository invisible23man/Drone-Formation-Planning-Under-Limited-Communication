function J_eta_i = get_jaco(eta_i, c_i)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

phi = eta_i(1);
sx = eta_i(2);
sy = eta_i(3);
% tx = eta_i(4);
% ty = eta_i(5);
cx = c_i(1);
cy = c_i(2);
cz = c_i(3);

J_eta_i = [-sin(phi)*sx*cx - cos(phi)*sy*cy , cos(phi)*cx , -sin(phi)*cy , 1 , 0;
            cos(phi)*sx*cx - sin(phi)*sy*cy ,  sin(phi)*cy , cos(phi)*cy , 0 , 1;
            0, 0,  0, 0, 0];

        
end

