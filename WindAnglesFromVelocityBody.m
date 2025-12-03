function wind_angles = WindAnglesFromVelocityBody(velocity_body)

V = sqrt(velocity_body(1)^2+velocity_body(2)^2+velocity_body(3)^2);
alpha = atan2(velocity_body(3),velocity_body(1));
beta = asin(velocity_body(2)/V);

wind_angles = [V; beta; alpha];
end