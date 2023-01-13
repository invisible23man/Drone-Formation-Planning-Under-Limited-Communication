function [phi_proj, s_proj, t_proj] = get_proj_eta(eta_i, eps, r_max)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
if ~exist('eps','var')
    eps = 0.1;
end
if ~exist('r_max','var')
    r_max = 0.5;
end

phi = eta_i(1);
s = eta_i(2:3);
t = eta_i(4:5);
phi_proj = phi; t_proj = t;
delta = sqrt(r_max^2-eps^2);

if s(1)<eps && s(2)<eps
    s_proj = [eps, eps];
elseif s(1)>=eps && s(1)<delta && s(1)<eps
    s_proj = [s(1), eps]; 
elseif s(1)<eps && s(2)>=eps && s(2)<delta
    s_proj = [eps, s(2)];
elseif s(1)>=delta && s(2)<eps
    s_proj = [delta, eps];
elseif s(1)<eps && s(2)>=delta
    s_proj = [eps, delta];
elseif s(1)>=eps && s(2)>=eps && norm(s) > r_max
    s_proj = r_max*s/norm(s);
else
    s_proj = s;
end


