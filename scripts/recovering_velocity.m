function [v_i, unused] = recovering_velocity(v_des_i, lambda_i, J_eta_i, ...
    tracking_i, consensus_i, constraint_i, disturbance_rej_i)
%recovering_velocity Transforms the parameter derivatives to 
%the velocity space for the local controllers to operate
if ~exist('disturbance_rej','var')
    disturbance_rej_i = 0;
end

unused = v_des_i + lambda_i;

v_i = J_eta_i*tracking_i - J_eta_i*consensus_i - ...
    J_eta_i*constraint_i - disturbance_rej_i;

end

