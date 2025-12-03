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




%% PROBLEM 3 

% givens 
doublet_time = 0.25; % seconds 
doublet_size = 15; % degrees 


[t1,aircraftStateArrayDoublet2] = ode45(@(t, var) AircraftEOMDoublet(t, var, u0_2, doublet_size, doublet_time, windInertial2, aircraft_parameters), [0,1800], x0_2);
aircraftStateArrayDoublet2 = aircraftStateArrayDoublet2'; % transpose for plotting 


% plot output from ode45 
PlotAircraftSim(t1, aircraftStateArrayDoublet2, u0_2, 1, 'r');

% find the two peaks used for finding natural frequency and damping ratio
[peaks, locations] = findpeaks(aircraftStateArrayDoublet(5)); % short period mode is charactarized by fast change in pitch angle 
locationOfFirstPeak = 0; 

for i = 1 : length(locations)

    if locations(i) > 0.5 % assuming short period starts after 0.5 seconds (according to lab doc and from the AircraftEOMDoublet function) 
        locationOfFirstPeak = i; 
    break;
    end

end

% define the two peaks used for finding natural frequency and damping ratio
peak1 = peaks(locationOfFirstPeak);
peak2 = peaks(locationOfFirstPeak + 1);

time1 = locations(locationOfFirstPeak);
time2 = locations(locationOfFirstPeak + 1); 

logDecrement = log(peak1 / peak2);
dampingRatio = logDecrement / sqrt(4 * (pi^2) + (logDecrement^2));

dampedNatFreq = (2 * pi) / (time2 - time1);
naturalFrequency = dampedNatFreq / sqrt(1 - dampingRatio^2);



% repeat for phugoid
% FINDING NATURAL FREQUENCY AND DAMPING RATIO OF THE PHUGOID RESPONSE 

% find the two peaks used for finding natural frequency and damping ratio
[peaks2, locations2] = findpeaks(aircraftStateArrayDoublet2(7)); % phugoid mode is charactarized by change in inertial velocity
locationOfFirstPeak2 = 0; 

for i = 1 : length(locations2)

    if locations2(i) > 20 % assuming short period dies out by 20 seconds
        locationOfFirstPeak2 = i; 
    break;
    end

end

% define the two peaks used for finding natural frequency and damping ratio
peak1Phugoid = peaks(locationOfFirstPeak2);
peak2Phugoid = peaks(locationOfFirstPeak2 + 1);

time1Phugoid = locations2(locationOfFirstPeak2);
time2Phugoid = locations2(locationOfFirstPeak2 + 1); 

logDecrement2 = log(peak1Phugoid / peak2Phugoid);
dampingRatioPhugoid = logDecrement2 / sqrt(4 * (pi^2) + (logDecrement2^2));

dampedNatFreqPhugoid = (2 * pi) / (time2Phugoid - time1Phugoid);
naturalFrequencyPhugoid = dampedNatFreqPhugoid / sqrt(1 - dampingRatioPhugoid^2);




dampedNatFreq = (2 * pi) / (time2 - time1);
naturalFrequency = dampedNatFreq / sqrt(1 - dampingRatio^2);

