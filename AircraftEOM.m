function  xdot = AircraftEOM(time, aircraft_state, aircraft_surfaces, wind_inertial, aircraft_parameters)
g = 9.81;


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

rho = stdatmo(z);

velocity = [u;v;w];
rates = [p;q;r];
[aero_forces, aero_moments] = AeroForcesAndMoments(aircraft_state, aircraft_surfaces, wind_inertial, rho, aircraft_parameters); 

R_b2i = [ cos(theta)*cos(psi),  sin(phi)*sin(theta)*cos(psi) - cos(phi)*sin(psi),  cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi);
          cos(theta)*sin(psi),  sin(phi)*sin(theta)*sin(psi) + cos(phi)*cos(psi),  cos(phi)*sin(theta)*sin(psi) - sin(phi)*cos(psi);
          -sin(theta),          sin(phi)*cos(theta),                               cos(phi)*cos(theta) ];

position_dot = R_b2i * velocity;
x_dot = position_dot(1);
y_dot = position_dot(2);
z_dot = position_dot(3);

TROC_angles = [1, sin(phi)*tan(theta), cos(phi)*tan(theta);
               0, cos(phi),            -sin(phi);
               0, sin(phi)*sec(theta), cos(phi)*sec(theta)] * rates;
phi_dot = TROC_angles(1);
theta_dot = TROC_angles(2);
psi_dot = TROC_angles(3);

u_dot = (r*v - q*w) - g*sin(theta) + X/m; %define X (aerodyn force)
v_dot = (p*w - r*u) - g*(cos(theta)*sin(phi) + Y/m;
w_dot = (q*u - p*v) + g*(cos(theta)*cos(phi)) + Z/m;

Gamma_1 = 
Gamma_2 = 
Gamma_3 = 
Gamma_4 = 
Gamma_5 = 
Gamma_6 = 
Gamma_7 = 
Gamma_8 = 

p_dot = ()



xdot = [x_dot; y_dot; z_dot; phi_dot; theta_dot; psi_dot; u_dot; v_dot; w_dot; p_dot; q_dot; r_dot];
end