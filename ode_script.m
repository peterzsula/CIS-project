%% Parameters (same values as pendulum_script.m)
m = 1.0;  l = 0.5;  b = 0.05;  g = 9.81;
J = m*l^2;
u = 0;                                   % no torque in this experiment

%% Dynamics  f(x,u)   with x(1) = theta, x(2) = thetadot
f = @(t, x) [ x(2);
    (m*g*l*sin(x(1)) - b*x(2) + u) / J ];

%% Simulate
tspan = [0 20];
x0    = [0.01; 0];                       % small perturbation from upright
[t, x] = ode45(f, tspan, x0);

%% Plot
plot(t, x(:,1)); xlabel('t [s]'); ylabel('\theta [rad]');
yline(pi, '--'); yline(-pi, '--');       % hanging-down positions