function [plot_done] = make_animation_plot(route, c_trfmd, p_goals, ...
   num_of_cfs, max_flight_time)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

global lambda eps r_max K_p gamma

[X,Y,Z] = sphere;% Plot Flight Trajectory
f1 = figure("Name", "Flight Trajectory"); 
% r = rotate3d(f1);

% Obstacle Plot Calculations
r = 2;
X1 = X *r; Y1 = Y *r; Z1 = Z *r;
X2 = X *r; Y2 = Y *r; Z2 = Z *r;

% Obstacle Plots
surf(X1+6, Y1-2, Z1+1, 'DisplayName', 'Obstacle1')
hold on
surf(X2+8.5, Y2+3.5, Z2+1, 'DisplayName', 'Obstacle2')
hold on

% Initial Positions
plot3(c_trfmd(:,1), c_trfmd(:,2), c_trfmd(:,3), 'o',...
    'Color','b','MarkerSize',10,'MarkerFaceColor','#FF0000',...
    'Displayname', 'Initial Config')
hold on

% Goal Positions
plot3(p_goals(:,1), p_goals(:,2), p_goals(:,3), 'o',...
    'Color','b','MarkerSize',10,'MarkerFaceColor','#00FF00',...
    'Displayname', 'Goal Config')
hold on

xlabel('X');
ylabel('Y');
zlabel('Z');
title(['\lambda=', num2str(lambda(1)), '  \gamma=', num2str(gamma(1)), ...
'  K_p=',num2str(K_p(1)), '  eps=', num2str(eps(1)), '  r_{max}=',num2str(r_max(1))]);
axis([-5 30 -10 10 -2 4]) 
grid on  

for k=1:length(route)

    % Drone Trajectory Plot
    c = route(:,:,k);
    traj = plot3(c(:,1), c(:,2), c(:,3), '*',...
            'Color','b','MarkerSize',10,'MarkerFaceColor','#000000',...
            'Displayname', 'Current Config');
    hold on
    % Line Plot
    if k>1        
        for i=1:num_of_cfs
            cf_path = squeeze(route(i,:,1:k));
            plot3(cf_path(1,:), cf_path(2,:), cf_path(3,:), '-')
            hold on
        end
    end
    
 
    legend()

    xlabel(["Sample Time: ",k])
    pause(0.000005)
  
    if k~= max_flight_time
%         clf
%         set(traj,'Visible','off')
        delete(traj)
        legend delete h(5:);
    end
end

% direction = [1 0 0];
% rotate(f1.Children,direction,25)
plot_done = true

end

