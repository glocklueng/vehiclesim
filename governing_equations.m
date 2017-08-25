%a bunch of function handles for functions which define how physics work
%stuff that relate to the car's dynamics

%force when going around a corner
centripetal_force = @(car, velocity, radius) car.m * velocity^2 / radius;

static_weight = @(mass, gravity, swd) mass * gravity * swd;
static_weight_rear = @(car, param) static_weight(car.m, param.g, car.swd);
static_weight_front = @(car, param) static_weight(car.m, param.g, 1-car.swd);

down_force = @(rho, Af, Cdf, vel) 0.5 * rho * Af * Cdf * vel^2;
down_force_rear  = @(car, param, vel) down_force(param.air_p, car.a_f, car.c_df, vel) * (car.cp);
down_force_front = @(car, param, vel) down_force(param.air_p, car.a_f, car.c_df, vel) * (1-car.cp);

drag_force = @(car, param, vel) 0.5 * param.air_p * car.a_f * car.c_d * vel^2;

%transfer of weight to the rear tires from the force
weight_transfer = @(car, F_long) F_long * car.cg / car.wb;

%an ellipse equation for combining available traction on the tire
traction_ellipse = @(car, F_lat, F_long, F_norm, scale) (F_lat/(car.u_lat * F_norm))^2 + (F_long/(car.u_long * F_norm)) == (1/scale)^2;