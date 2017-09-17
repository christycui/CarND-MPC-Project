# CarND-Controls-MPC

---

## The model

- Vehicle State: there are 6 variables captured in vehicle state: x and y positions (according to vehicle coordinate), psi (vehicle orientation), v (speed), cte (cross track error) and epsi (orientation error).

- Actuators are steering angle and throttle. Negative throttle indicates braking.

- Update Equations are given as below: 
```c++
fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
fg[1 + v_start + t] = v1 - (v0 + a0 * dt);
fg[1 + psi_start + t] = psi1 - (psi0 + v0/Lf * delta0 * dt);
AD<double> f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * CppAD::pow(x0, 2) + coeffs[3] * CppAD::pow(x0, 3);
fg[1 + cte_start + t] = cte1 - ((f0-y0) - v0*CppAD::sin(epsi0)*dt);
AD<double> psides0 = CppAD::atan(coeffs[1] + 2 * coeffs[2] * x0 + 3 * coeffs[3] * CppAD::pow(x0, 2));
fg[1 + epsi_start + t] = epsi1 - (psi0 - psides0 + v0/Lf*delta0*dt);
```

## Timestep length (N) and duration (dt)

The N and dt values are large chosen by trial and error. I've found that dt smaller or equal than 0.05 makes the vehicle swirl more when N is kept constant. In choosing N, I try to keep it small (decrease latency) while making sure enough distance is being predicted. 

My final result of N = 15 and dt = 0.08 work pretty well together. The vehicle is able to drive smoothly at a speed around 35 - 40 mph.

## Cost Function

The cost function I designed focuses on minimizing the cross track error and heading orientation error. It also penalzes large steering and inconsistent steering. The velocity and orientation errors are present in the cost function but they are not the main drivers. This design ensures that the first priority of the vehicle is to drive smoothly and correctly along the waypoints.

## Latency

I incorporated the latency into the MPC controller by considering a 0.1 second lapse into the initial state. As such, my initial state is described as:
```c++
double x_ = v*latency;
double y_ = 0;
double psi_ = - v/Lf * steering * latency;
double v_ = v + throttle * latency;
auto coeffs = polyfit(ptsx_transform, ptsy_transform, 3);

double cte =  polyeval(coeffs, 0);
double epsi = - atan(coeffs[1]);

double cte_ = cte + (v * sin(epsi) * latency);
double epsi_ = epsi - (v/Lf * steering * latency);

Eigen::VectorXd state(6);
state << x_, y_, psi_, v_, cte_, epsi_;
```