%% INITIALIZATION

clear
close all
clc

% run("./test_base_config.m")

%% Example 2-D animation plot

t = 0:0.1:2*pi;
y = sin(t);
y2 = cos(t);

for k= 1:length(t)

    %% marker plots
    plot(t(k), y(k), 'x')
    hold on
    plot(t(k), y2(k), 'o')
    hold on

    %% line plots
    plot(t(1:k), y(1:k))
    hold on
    plot(t(1:k), y2(1:k))
     
    %% graph properties
    axis([0 6*pi -1 1]); %limits for x and y
    grid on
    xlabel('t')
    ylabel('y')
    legend('cos(t) marker', 'sin(t) marker', 'cos(t)', 'sin(t)')
    pause(0.05)
    
    if k~= length(t)
        clf
    end
end 