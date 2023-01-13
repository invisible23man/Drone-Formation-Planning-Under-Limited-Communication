function d_mui = penalty_multiplier(eta_i, gamma_i, eps_i, r_max_i)

[phi_proj, s_proj, t_proj] = get_proj_eta(eta_i, eps_i, r_max_i);
proj_i = [phi_proj, s_proj, t_proj];
d_mui = gamma_i*norm(eta_i-proj_i);
end

