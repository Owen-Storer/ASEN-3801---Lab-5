function PlotAircraftSim(time, aircraft_state_array, control_input_array, fig, col)



%{
% Purpose: Plots 4 figures with subplots indicating inertial position, euler angles, inertial
% velocity in body frame, and angular velocity.  Also plots a 3D path of the aircraft with (+)
% height upward.

% Inputs: 
% 1.) time = time vector in seconds -- self explanatory
% 2.) aircrat_state_array = 12xn array of aircraft states at different instances
% 3.) control inputs = 4xn array of control inputs [elevator, aileron, rudder, Thrust_fraction]^T
for the aircraft
% 4.) fig = number of figures to plot over
% 5.) col = string indicating plotting options for each figure 
% 6.) graph_AC_sim == boolearn variable which toggles graphing outputs on and off. If == 1 then
graphs are on, if == 0 then graphs are off
% 7.) Fc == control forces of form [Xc, Yc, Zc] in [N]
% 8.) Gc == control moments of form [Lc, Mc, Nc] in [Nm]

% Outputs: 12 Graphs, one graph for each state vector element changing w.r.t time as defined by
ode45
%}

if graph_AC_sim == 1

%% Devectorize necessary vectors

% Inertial Position
X_E = aircraft_state_array(1,:);
Y_E = aircraft_state_array(2,:);
Z_E = aircraft_state_array(3,:);

% Euler Angle -- Attitude Position
phi = aircraft_state_array(4,:);
theta = aircraft_state_array(5,:);
psi = aircraft_state_array(6,:);


% Inertial Velocity in the Body Frame
u_E = aircraft_state_array(7,:);
v_E = aircraft_state_array(8,:);
w_E = aircraft_state_array(9,:);


% Angular Velocities
p = aircraft_state_array(10,:);
q = aircraft_state_array(11,:);
r = aircraft_state_array(12,:);

% Controls
elevator_deg = control_input_array(1,:);
aileron_deg = control_input_array(2,:);
rudder_deg = control_input_array(3,:);
thrustFraction = control_input_array(4,:);

%% Plot necessary graphs

% Plotting Inertial Position
figure();
subplot(3,1,1)
plot(time, X_E, col);
title("Inertial X Position w.r.t time")
xlabel("Time in [s]")
ylabel("X Position in [m]")

subplot(3,1,2)
plot(time, Y_E, col);
title("Inertial Y Position w.r.t time")
xlabel("Time in [s]")
ylabel("Y Position in [m]")

subplot(3,1,3)
plot(time, Z_E, col);
title("Inertial Z Position w.r.t time")
xlabel("Time in [s]")
ylabel("Z Position in [m]")

%----------------------------------------

% Plotting Euler Angles
figure();
subplot(3,1,1)
plot(time, phi, col);
title("Roll (phi) w.r.t time")
xlabel("Time in [s]")
ylabel("Phi in [radians]")

subplot(3,1,2)
plot(time, theta, col);
title("Pitch (Theta) w.r.t time")
xlabel("Time in [s]")
ylabel("Theta in [radians]")

subplot(3,1,3)
plot(time, psi, col);
title("Yaw (Psi) w.r.t time")
xlabel("Time in [s]")
ylabel("Psi in [radians]")

%----------------------------------------

% Plotting Inertial Velocity in the body frame
figure();
subplot(3,1,1)
plot(time, u_E, col);
title("U Inertial Velocity in Body Frame w.r.t Time")
xlabel("Time in [s]")
ylabel("U_E in [m/s]")

subplot(3,1,2)
plot(time, v_E, col);
title("V Inertial Velocity in Body Frame w.r.t Time")
xlabel("Time in [s]")
ylabel("V_E in [m/s]")

subplot(3,1,3)
plot(time, w_E, col);
yline(0, col)
title("W Inertial Velocity in Body Frame w.r.t Time")
xlabel("Time in [s]")
ylabel("W_E in [m/s]")

%----------------------------------------

% Plot Angular velocities
figure();
subplot(3,1,1)
plot(time, p, col);
title("Angular Velocity in I_B hat direction w.r.t Time")
xlabel("Time in [s]")
ylabel("Angular Velocity in [rad/s]")

subplot(3,1,2)
plot(time, q, col);
title("Angular Velocity in J_B hat direction w.r.t Time")
xlabel("Time in [s]")
ylabel("Angular Velocity in [rad/s]")

subplot(3,1,3)
plot(time, r, col);
title("Angular Velocity in K_B hat direction w.r.t Time")
xlabel("Time in [s]")
ylabel("Angular Velocity in [rad/s]")

%----------------------------------------

% Plotting controls (elevator, aileron, rudder, and thrust)

figure();
subplot(4,1,1)
plot(time, elevator_deg, col)
title("Elevator Control Angle vs. Time")
xlabel("Time in [s]")
ylabel("Elevator Angle [deg]")

subplot(4,1,2)
plot(time, aileron_deg, col)
title("Aileron Control Angle vs. Time")
xlabel("Time in [s]")
ylabel("Aileron Angle [deg]")

subplot(4,1,3)
plot(time, rudder_deg, col)
title("Rudder Control Angle vs. Time")
xlabel("Time in [s]")
ylabel("Rudder Angle [deg]")

subplot(4,1,4)
plot(time, thrustFraction, col)
title("Thrust Fraction vs. Time")
xlabel("Time in [s]")
ylabel("Thrust Ratio [0->1]")

%----------------------------------------

% Plot 3D Graph of aircraft position w.r.t inertial frame
figure();
plot3(X_E, Y_E, Z_E, col);
plot3(X_E(1), Y_E(1), Z_E(1), "g*") % plot start of path with green marker
plot3(X_E(end), Y_E(end), Z_E(end), "r*") % plot end of path with red marker
title("3D aircraft path expressed in body frame coordinates")
xlabel("X_E position in [m]")
ylabel("Y_E position in [m]")
legend("Z_E position in [m]")
zlim([0 5])

elseif graph_AC_sim == 0
end
end