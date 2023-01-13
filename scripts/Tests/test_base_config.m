%% INITIALIZATION

clear
close all
clc

project_dir = '/Users/invisible_man/Documents/DTU/Courses/Drones/Simulations/uas-31390';
cd(project_dir);
run("./uas_start.m")

eta = [pi/4, 1, 2, 3, 1];
%eta = [0, 1, 1, 0, 0];

c_trfmd = zeros(size(c,1),3);
c_goal = zeros(size(c,1),3);
[X,Y,Z] = sphere;

%% TRANSFORMATIONS

% Generate Transformed Configurations for all drones
for i=1:size(c,1)
    c_i = c(i,:);
    c_i_trfmd = get_transform(eta, c_i');
    c_trfmd(i,:,:) = c_i_trfmd';
    c_i_goal = get_transform(eta_goal, c_i');
    c_goal(i,:,:) = c_i_goal';
end

%%
% Calculate Route for all drones

flight_time = 1:1:50;
delta_init = 0.05;
delta = 0.05;
route = zeros(size(c,1),size(c,2),length(flight_time));
for i=1:length(flight_time)
   route(:,:,i) = c + delta;  
   delta = delta + delta_init; 
end

%% BASIC PLOTS

% Plot Base and Transformed Configurations
f1 = figure("Name", "Transformation and Base Configuration"); 
plot3(c(:,1), c(:,2), c(:,3), 'o',...
    'Color','b','MarkerSize',10,'MarkerFaceColor','#000000',...
    'Displayname', 'Base Config')
hold on
plot3(c_trfmd(:,1), c_trfmd(:,2), c_trfmd(:,3), 'o',...
    'Color','b','MarkerSize',10,'MarkerFaceColor','#FF0000',...
    'Displayname', 'Transformed Config')
plot3(c_goal(:,1), c_goal(:,2), c_goal(:,3), 'o',...
    'Color','b','MarkerSize',10,'MarkerFaceColor','#00FF00',...
    'Displayname', 'Goal Config')

% Plot Obstacles
r = 2;
X1 = X *r; Y1 = Y *r; Z1 = Y *r;
X2 = X *r; Y2 = Y *r; Z2 = Y *r;

surf(X1+6, Y1-2, Z1+1, 'DisplayName', 'Obstacle1')
surf(X2+8.5, Y2+3.5, Z1+1, 'DisplayName', 'Obstacle2')

axis([-30 30 -30 30 -10 10]) 
grid on
legend()

%% ANIMATED PLOTS

% Plot Flight Trajectory
f2 = figure("Name", "Flight Trajectory"); 
for k=1:length(flight_time)

    % Drone Plots
    c = route(:,:,k);
    plot3(c(:,1), c(:,2), c(:,3), 'o',...
        'Color','b','MarkerSize',10,'MarkerFaceColor','#000000',...
        'Displayname', 'Base Config')
    hold on
%     plot3(c_trfmd(:,1), c_trfmd(:,2), c_trfmd(:,3), 'o',...
%         'Color','b','MarkerSize',10,'MarkerFaceColor','#FF0000',...
%         'Displayname', 'Transformed Config')
%     plot3(c_goal(:,1), c_goal(:,2), c_goal(:,3), 'o',...
%         'Color','b','MarkerSize',10,'MarkerFaceColor','#00FF00',...
%         'Displayname', 'Goal Config')

    % Line Plot
    if k>1        
        for i=1:num_of_cfs
            cf_path = squeeze(route(i,:,1:k));
            plot3(cf_path(1,:), cf_path(2,:), cf_path(3,:), '-')
            hold on
        end
    end
    
    % Obstacle Plot
    r = 2;
    X1 = X *r; Y1 = Y *r; Z1 = Y *r;
    X2 = X *r; Y2 = Y *r; Z2 = Y *r;

    surf(X1+6, Y1-2, Z1+1, 'DisplayName', 'Obstacle1')
    surf(X2+8.5, Y2+3.5, Z1+1, 'DisplayName', 'Obstacle2')

    axis([-30 30 -30 30 -10 10]) 
    grid on
    legend()
    pause(0.05)
    
    if k~= length(flight_time)
        clf
    end

end