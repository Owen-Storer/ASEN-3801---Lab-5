function wind_body = TransformFromInertialToBody(wind_inertial, aircraft_state)

phi = aircraft_state(1);
theta = aircraft_state(2);
psi = aircraft_state(3);

% DCM matrix 
R_b2i = [ cos(theta)*cos(psi),  sin(phi)*sin(theta)*cos(psi) - cos(phi)*sin(psi),  cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi);
          cos(theta)*sin(psi),  sin(phi)*sin(theta)*sin(psi) + cos(phi)*cos(psi),  cos(phi)*sin(theta)*sin(psi) - sin(phi)*cos(psi);
          -sin(theta),          sin(phi)*cos(theta),                               cos(phi)*cos(theta) ]; 

% transpose 
R_i2b = transpose(R_b2i);

wind_body = R_b2i*wind_inertial; 

end 