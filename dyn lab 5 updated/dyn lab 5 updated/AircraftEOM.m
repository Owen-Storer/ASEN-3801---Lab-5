function  xdot = AircraftEOM(time, aircraft_state, aircraft_surfaces, wind_inertial, aircraft_parameters)
g = 9.81;
h=1609.34;
m = aircraft_parameters.m;

x = aircraft_state(1);
y = aircraft_state(2);
z = aircraft_state(3);
phi = aircraft_state(4);
theta = aircraft_state(5);
psi = aircraft_state(6);
u = aircraft_state(7);
v = aircraft_state(8);
w = aircraft_state(9);
p = aircraft_state(10);
q = aircraft_state(11);
r = aircraft_state(12);

rho = stdatmo(-z);

velocity = [u;v;w];
rates = [p;q;r];
[aero_forces, aero_moments] = AeroForcesAndMoments(aircraft_state, aircraft_surfaces, wind_inertial, rho, aircraft_parameters); 
L = aero_moments(1);
M = aero_moments(2);
N = aero_moments(3);

R_b2i = [ cos(theta)*cos(psi),  sin(phi)*sin(theta)*cos(psi) - cos(phi)*sin(psi),  cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi);
          cos(theta)*sin(psi),  sin(phi)*sin(theta)*sin(psi) + cos(phi)*cos(psi),  cos(phi)*sin(theta)*sin(psi) - sin(phi)*cos(psi);
          -sin(theta),          sin(phi)*cos(theta),                               cos(phi)*cos(theta) ];

position_dot = R_b2i * velocity;
x_dot = position_dot(1);
y_dot = position_dot(2);
z_dot = position_dot(3);

TROC_angles = [1, sin(phi)*tan(theta), cos(phi)*tan(theta);
               0, cos(phi),            -sin(phi);
               0, sin(phi)/cos(theta), cos(phi)/cos(theta)] * rates;
phi_dot = TROC_angles(1);
theta_dot = TROC_angles(2);
psi_dot = TROC_angles(3);

u_dot = (r*v - q*w) - g*sin(theta) + (aero_forces(1))/m; 
v_dot = (p*w - r*u) - g*(cos(theta)*sin(phi)) + (aero_forces(2))/m;
w_dot = (q*u - p*v) + g*(cos(theta)*cos(phi)) + (aero_forces(3))/m;

% define moment of ineritas from aircraft_parameters
I_x = aircraft_parameters.Ix; 
I_y = aircraft_parameters.Iy; 
I_z = aircraft_parameters.Iz; 
I_xz = aircraft_parameters.Ixz;

% define Gamma using moi
Gamma = I_x * I_z - (I_xz^2);
Gamma_1 = (I_xz * (I_x - I_y + I_z)) / Gamma;
Gamma_2 = (I_z * (I_z - I_y) + (I_xz ^2)) / Gamma;
Gamma_3 = I_z / Gamma; 
Gamma_4 = I_xz / Gamma; 
Gamma_5 = (I_x- I_z) / I_y;
Gamma_6 = I_xz / I_y; 
Gamma_7 = ((I_x * (I_x - I_y)) + (I_xz ^ 2)) / Gamma;
Gamma_8 = I_x / Gamma;


p_dot = (Gamma_1*p*q - Gamma_2*q*r) + (Gamma_3*L + Gamma_4*N);
q_dot = (Gamma_5*p*r - Gamma_6*(p^2 - r^2)) + (M/I_y);
r_dot = (Gamma_7*p*q - Gamma_1*q*r) + (Gamma_4*L + Gamma_8*N);



xdot = [x_dot; y_dot; z_dot; phi_dot; theta_dot; psi_dot; u_dot; v_dot; w_dot; p_dot; q_dot; r_dot];
end



