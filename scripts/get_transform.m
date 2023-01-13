function p = get_transform(eta_i, c_i)
% get_transform: The transformation of a swarm consisting of N robots
% is parameterised through the following scaling, rotation and
% translation of a base configuration.

phi = eta_i(1);
sx = eta_i(2);
sy = eta_i(3);
tx = eta_i(4);
ty = eta_i(5);

R = [cos(phi) -sin(phi) 0;
     sin(phi) cos(phi) 0;
     0 0 1];
    
S = [sx 0 0;
     0 sy 0;
     0 0 1];
t = [tx;
     ty;
     0];

p = R*S*c_i + t;

end

