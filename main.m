clc; 
clear; 
close all;

%% Constants
V = 21; % airspeed (m/s)
h = 1609.34; % height (m)
% Case 1
x0_1 = [     0,      0, -1800;     % m
             0, .02780,     0;     % 2 deg 1 rad
         20.99,      0, .5837;     % m/s
             0,      0,     0   ]; % deg/s
u0_1 = [.1079; 0; 0; .3182];       % rad
% Case 2
x0_2 = [     0,      0, -1800;     % m
             15,   -12,   270;     % deg
             19,     3,    -2;     % m/s
            .08,  -0.2,     0   ]; % deg/s
u0_2 = [5; 2; -13; .3];            % 3 deg 1 rad