function v_des_i = local_planner2d(p, p_goal)
% Local Planner: Provides the desired velocity to be tracked
% 
% Description
% o To be written  
%
% Parameters
%
%   p   : Current Position of the Drone (transformed)

% Simulation Parameters for APF Planner
nu = 1.5; % repulsor distance
zeta = 0.25; % obstacle clearance
k_att = 5; % attractor velocity
k_rep = 5; % repulsor velocity
rho = 0.1; % attractor switch distance

p_o_1 = [6; -2]; % Obstacle 1 , rad 2
p_o_2 = [8.5; 3.5]; % Obstacle 2 , rad 2
p_obstacles =[p_o_1'; p_o_2'];

e = p_goal - p;

% Attraction
if norm(e) < rho
    v_att_i = k_att*e/rho;
else
    v_att_i = k_att*e/norm(e);
end

% Repulsion
v_rep_i = 0;
for j=1:size(p_obstacles,1)
    p_o = p_obstacles(j,:);
    if norm(p_o-p) < nu
        v_rep_i = v_rep_i + k_rep*1/((norm(p_o-p)-zeta))-1/(nu-zeta)*...
                    (p_o-p)/norm(p_o-p);
    else
        v_rep_i = 0;
    end

end

v_des_i = (v_att_i + v_rep_i)';


