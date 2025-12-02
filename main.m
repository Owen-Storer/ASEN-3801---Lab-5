%{ 
clc; 
clear; 
close all;

%% Constants
V = 21; % airspeed (m/s)
h = 1609.34; % height (m)
% Case 1
x0_1 = [     0;      0; -1800;     % m
             0; .02780;     0;     % 2 deg 1 rad
         20.99;      0; .5837;     % m/s
             0;      0;     0   ]; % deg/s
u0_1 = [.1079; 0; 0; .3182];       % rad

attitude_1 = x0_1(4:6,1);
DCM_1 = RotationMatrix321(attitude_1);

% Case 2
x0_2 = [      0;     0; -1800;     % m
             15;   -12;   270;     % deg
             19;     3;    -2;     % m/s
            .08;  -0.2;     0   ]; % deg/s
u0_2 = [5; 2; -13; .3];            % 3 deg 1 rad

attitude_2_deg = x0_2(4:6,1);
attitude_2 = deg2rad(attitude_2_deg);

DCM_2 = RotationMatrix321(attitude_2);


%} 

%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%% UPDATED BY MATTIAS 

clc; 
clear; 
close all;

%% Problem 2 

ttwistor; 

%% Case 1 
V = [21;0;0]; % airspeed (m/s)
Vwind_inertial = [0;0;0];
h = 1609.34; % height (m)

%t1 = linspace(0,1800,1800); % define time span 
x0_1 = [0; 0; -h; 0; 0; 0; 0; 0; 0; 0; 0; 0]; % initial conditions are all zero 
u0_1 = zeros(1,4)'; % control surfaces are all zero 
x0_1(7) = V(1); % inertial velocity in body x is set to airspeed 

% call ode45 to simulate the equations of motion 
[t1,aircraftStateArray] = ode45(@(t, var) AircraftEOM(t, var, u0_1, Vwind_inertial, aircraft_parameters), [0,1800], x0_1);

control_input_array = zeros(4,length(t1));

% plot output from ode45 
PlotAircraftSim(t1, aircraftStateArray, control_input_array, 'r');


%% Case 2
x0_2 = [     0,      0, -1800;     % m
             0, .02780,     0;     % 2 deg 1 rad
         20.99,      0, .5837;     % m/s
             0,      0,     0   ]; % deg/s
u0_2 = [.1079; 0; 0; .3182];       % rad

% call ode45 to simulate the equations of motion 
[t2,aircraftStateArray2] = ode45(@(t, var) AircraftEOM(t, var, u0_2, Vwind_inertial, aircraft_parameters), [0,1800], x0_2);

control_input_array2 = repmat(u0_2, 1, length(t2));

% plot output from ode45 
PlotAircraftSim(t2, aircraftStateArray2, control_input_array2, 'r');



%% Case 3
x0_3 = [     0,      0, -1800;     % m
             15,   -12,   270;     % deg
             19,     3,    -2;     % m/s
            .08,  -0.2,     0   ]; % deg/s
u0_3 = [5; 2; -13; .3];            % 3 deg 1 rad

% call ode45 to simulate the equations of motion 
[t3,aircraftStateArray3] = ode45(@(t, var) AircraftEOM(t, var, u0_3, Vwind_inertial, aircraft_parameters), [0,1800], x0_3);

control_input_array3 = repmat(u0_3, 1, length(t3));

% plot output from ode45 
PlotAircraftSim(t3, aircraftStateArray3, control_input_array3, 'r');