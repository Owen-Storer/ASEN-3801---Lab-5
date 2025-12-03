function wind_velocities = wind_inertial(wind_angles)
V = wind_angles(1);
beta = wind_angles(2);
alpha = wind_angles(3);

u = V * (cos(alpha)*cos(beta));
v = V * (sin(beta));
w = V *(sin(alpha)*cos(beta));

wind_velocities = [u; v; w];

end