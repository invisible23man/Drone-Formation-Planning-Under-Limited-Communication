function [tracking_i, J_eta_i, J_eta_inv_i, p_i] = tracking2d(eta_i, c_i, v_des_i, p_i)
% tracking: The transformation of a swarm consisting of N robots
% is parameterised through the following scaling, rotation and
% translation of a base configuration.
% Using the chain-rule, the time-derivative of the position of
% robot i in the swarm, with regard to the time-derivative of the
% parameters, can be expressed as
% d/dt(p_i) = J_ηi*d/dt(η_i)
% The time derivative of the parameters for robot i can be found as 
% d/dt(η_i) = J+_ηi*v_des_i

J_eta_i = get_jaco2d(eta_i, c_i);
J_eta_inv_i = pinv(J_eta_i); %Moore-Penrose pseudoinverse
tracking_i = J_eta_inv_i*v_des_i;
end

