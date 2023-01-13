function [constraint_i, proj_i] = constrain_satisfaction(eta_i, eps_i,...
    r_max_i, mu_i)
%constraint_i To ensure that there is no inbounded scaling
%   resulting in communication loss, a cinstrain is apploed


[phi_proj, s_proj, t_proj] = get_proj_eta(eta_i, eps_i, r_max_i);
proj_i = [phi_proj, s_proj, t_proj];
constraint_i = (mu_i*(eta_i-proj_i))';

