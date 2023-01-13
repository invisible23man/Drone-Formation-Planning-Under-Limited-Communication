function plot_done = make_plot(route, num_of_cfs, c_trfmd, p_goals, ...
                                    k, X, Y, Z, max_flight_time)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
    c = route(:,:,k);
%     plot3(c(1:4,1), c(1:4,2), c(1:4,3), '*',...
    plot3(c(:,1), c(:,2), c(:,3), '*',...
        'Color','b','MarkerSize',10,'MarkerFaceColor','#000000',...
        'Displayname', 'Cuurent Config')
    hold on
    plot3(c_trfmd(:,1), c_trfmd(:,2), c_trfmd(:,3), 'o',...
        'Color','b','MarkerSize',10,'MarkerFaceColor','#FF0000',...
        'Displayname', 'Initial Config')
    plot3(p_goals(:,1), p_goals(:,2), p_goals(:,3), 'o',...
        'Color','b','MarkerSize',10,'MarkerFaceColor','#00FF00',...
        'Displayname', 'Goal Config')

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
    X1 = X *r; Y1 = Y *r; Z1 = Z *r;
    X2 = X *r; Y2 = Y *r; Z2 = Z *r;

    surf(X1+6, Y1-2, Z1+1, 'DisplayName', 'Obstacle1')
    surf(X2+8.5, Y2+3.5, Z2+1, 'DisplayName', 'Obstacle2')

    axis([-50 50 -100 100 -10 10]) 
    grid on
    legend()
    xlabel(["Sample Time: ",k])
    pause(0.05)
  
    if k~= max_flight_time
        clf
    end
    
    plot_done = true;
end