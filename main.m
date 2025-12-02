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
%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%% UPDATED BY MATTIAS 

clc; 
clear; 
close all;

%% Problem 2 

ttwistor; 

% Case 1 
V = [21;0;0]; % airspeed (m/s)
h = 1609.34; % height (m)

t1 = linspace(0,1800,1800); % define time span 
x0_1 = zeros(1, 12)'; % initial conditions are all zero 
u0_1 = zeros(1,4)'; % control surfaces are all zero 
x0_1(7) = V(1); % inertial velocity in body x is set to airspeed 

% find inertial wind
windAngles1 = WindAnglesFromVelocityBody(V); 
windInertial1 = wind_inertial(windAngles1);

% call AircraftEOM to find derivative of the statevector 
xdot = AircraftEOM(t1, x0_1, u0_1, windInertial1, aircraft_parameters); 

% call ode45 to simulate the equations of motion 
[t1,aircraftStateArray] = ode45(xdot, [0,1800], x0_1);

% plot output from ode45 
PlotAircraftSim(t1, aircraftStateArray, control_input_array, 1, 'r');


% Case 2
x0_2 = [     0,      0, -1800;     % m
             0, .02780,     0;     % 2 deg 1 rad
         20.99,      0, .5837;     % m/s
             0,      0,     0   ]; % deg/s
u0_2 = [.1079; 0; 0; .3182];       % rad





% Case 3
x0_3 = [     0,      0, -1800;     % m
             15,   -12,   270;     % deg
             19,     3,    -2;     % m/s
            .08,  -0.2,     0   ]; % deg/s
u0_3 = [5; 2; -13; .3];            % 3 deg 1 rad

