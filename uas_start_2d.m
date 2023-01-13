%% INITIALIZATION

clear
close all
clc

project_dir = '/Users/invisible_man/Documents/DTU/Courses/Drones/Simulations/uas-31390';
cd(project_dir);
%% SIMULATION PARAMETERS
load("uas_parameters.mat")
load("./scripts/sim_params.mat")

global c_trfmd p_goals lambda eps r_max K_p gamma

num_of_cfs = 9;

ts = 1e-3; tf = 9; k=0; max_flight_time = 450;
% lambda = 4; gamma = 32; K_p = 2; eps = 0.75; r_max = 2.5;
% lambda = 32; gamma = 64; K_p = 2; eps = 0.75; r_max = 2.5;
lambda = 0; gamma = 0; K_p = 0; eps = 0; r_max = inf;

lambda = lambda*ones(num_of_cfs,1);
gamma = gamma*ones(num_of_cfs,1);
eps = eps*ones(num_of_cfs,1);
r_max = r_max*ones(num_of_cfs,1);
K_p = K_p*ones(num_of_cfs,1);
mu = zeros(num_of_cfs,1);

goal_reached = false;
c = get_base_config(ci_1, ci_2);
eta = eta_0.*ones(num_of_cfs,size(eta_0,1));
eta_goals = eta_goal.*ones(num_of_cfs,size(eta_0,1));
c_trfmd = zeros(num_of_cfs,2);
p_goals = zeros(num_of_cfs,2);

for i=1:num_of_cfs
    c_i = c(i,:);
    c_i_trfmd = get_transform2d(eta_0, c_i');
    c_trfmd(i,:) = c_i_trfmd';
    p_i_goal = get_transform2d(eta_goal, c_i');
    p_goals(i,:) = p_i_goal';
end

p = c_trfmd;
route = zeros(size(c,1),size(c,2),length(max_flight_time));
flight_time_instant = 1;

%% PLOTS Initialization

% [X,Y,Z] = sphere;% Plot Flight Trajectory
% f1 = figure("Name", "Flight Trajectory"); 
% r = rotate3d(f1);

%% EXECUTION OF ALGORITHM

f = waitbar(0, 'Starting');
while ~goal_reached    
    
    % Run algorithm for each drone at k_th timestep
    for i=1:num_of_cfs
        % get position in base configuration for the drone
        c_i = c_trfmd(i,:)';
        % get current position for the drone
        p_i = p(i,:);
        % get current parameters for the drone
        eta_i = eta(i,:);

        % get v_des from local planner
        v_des_i = local_planner2d(p_i, p_goals(i,:));
        % perform tracking
        [tracking_i, J_eta_i, J_eta_inv_i, p_i] = tracking2d(eta_i, c_i, v_des_i, p_i);
        % perform consensus
        eta_Nis = get_neigbour_etas(i, eta);
        consensus_i = consensus(eta_i, eta_Nis, lambda(i));        
        % perform constraint satisfaction
        d_mui = penalty_multiplier(eta_i, gamma(i), eps(i), r_max(i));
        [constraint_i, proj_i] = constrain_satisfaction(eta_i, eps(i), r_max(i), mu(i));
        % perform recovering velocity
        [v_i, ~] = recovering_velocity(v_des_i, lambda(i), J_eta_i, ...
                tracking_i, consensus_i, constraint_i); %, disturbance_rej_i)
        
        % perform integration
        p(i,:) = (p(i,:)' + ts.*v_i)';
        % perform integration
        mu(i) = mu(i) + ts.*d_mui;
    
        % update parameters
        d_eta_i = tracking_i - consensus_i - constraint_i;
        eta(i,:) = eta_i + ts.*d_eta_i';
    end
   
    % save positions to route
    route(:,:,flight_time_instant) = p;  
%     make_plot(route, num_of_cfs, c_trfmd, p_goals, flight_time_instant,X,Y,Z,max_flight_time)
    flight_time_instant = flight_time_instant + 1;
    k = k+ts;   
    goal_reached = all(eta == eta_goals) | k>tf |flight_time_instant > max_flight_time;
    waitbar(flight_time_instant/max_flight_time, f, ...
        sprintf('Progress: %d %%', floor(flight_time_instant/max_flight_time*100)));
    pause(0.0001);
%     if k~= max_flight_time
%     clf
%     end

end

%% PLOTS

% [X,Y,Z] = sphere;% Plot Flight Trajectory
f1 = figure("Name", "Flight Trajectory"); 
% r = rotate3d(f1);

for k=1:length(route)

    % Drone Plots
    c = route(:,:,k);
%     plot3(c(1:4,1), c(1:4,2), c(1:4,3), '*',...
    plot(c(:,1), c(:,2), '*',...
        'Color','b','MarkerSize',3,'MarkerFaceColor','#000000',...
        'Displayname', 'Current Config')
    hold on
    plot(c_trfmd(:,1), c_trfmd(:,2), 'o',...
        'Color','b','MarkerSize',3,'MarkerFaceColor','#FF0000',...
        'Displayname', 'Initial Config')
    plot(p_goals(:,1), p_goals(:,2), 'o',...
        'Color','b','MarkerSize',3,'MarkerFaceColor','#00FF00',...
        'Displayname', 'Goal Config')

    % Line Plot
    if k>1        
        for i=1:num_of_cfs
            cf_path = squeeze(route(i,:,1:k));
            plot(cf_path(1,:), cf_path(2,:), '-')
            hold on
        end
    end
    
    % Obstacle Plot
%     r = 2;
%     X1 = X *r; Y1 = Y *r; Z1 = Y *r;
%     X2 = X *r; Y2 = Y *r; Z2 = Y *r;
% 
%     surf(X1+6, Y1-2, Z1+1, 'DisplayName', 'Obstacle1')
%     surf(X2+8.5, Y2+3.5, Z1+1, 'DisplayName', 'Obstacle2')

    axis([-10 25 -8 8]) 
    grid on
    legend()
    xlabel(["Sample Time: ",k])
    pause(0.0005)
  
    if k~= max_flight_time
        clf
    end
end

% direction = [1 0 0];
% rotate(f1.Children,direction,25)

close(f)

