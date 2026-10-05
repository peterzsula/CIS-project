%% Open-loop simulation of the pendulum equation with ode45 (report 1.1)
pendulum_setup                           % model parameters m.*, g
u = 0;                                   % no torque in this experiment

%% Dynamics  f(x,u)   with x(1) = theta, x(2) = thetadot
f = @(t, x) [ x(2);
    (m.m*g*m.l*sin(x(1)) - m.b*x(2) + u) / m.J ];

%% Simulate
tspan = [0 20];
% columns: exactly upright, +-0.01 rad angle perturbation, 0.01 rad/s velocity perturbation
X0 = [0  0.01  -0.01  0;
      0  0      0     0.01];

% Tight tolerances: errors grow like the unstable mode, and the default
% RelTol = 1e-3 shifts the fall by ~0.04 s compared with the Simscape plant.
opts = odeset('RelTol', 1e-8, 'AbsTol', 1e-10);

%% Plot
figure; hold on
for x0 = X0
    [t, x] = ode45(f, tspan, x0, opts);
    plot(t, rad2deg(x(:,1)));
end
% Linear approximation xdot = Ac*x for the +0.01 rad case (report 1.2)
[tl, xl] = ode45(@(t, x) Ac*x, tspan, [0.01; 0], opts);
plot(tl, rad2deg(xl(:,1)), 'k--');
ylim([-300 300]);
xlabel('t [s]'); ylabel('\theta [deg]'); grid on
yline(180, '--'); yline(-180, '--');     % hanging-down positions
legend('\theta_0 = 0', '\theta_0 = +0.01 rad', '\theta_0 = -0.01 rad', ...
    '\omega_0 = 0.01 rad/s', 'linear model, \theta_0 = +0.01 rad', 'Location', 'east');
exportgraphics(gcf, 'open_loop.png', 'Resolution', 200);

%% Linear vs. nonlinear model: time to reach a given angle from theta0 = +0.01 rad
[t, x] = ode45(f, tspan, [0.01; 0], opts);
fprintf('\n%10s %12s %12s\n', 'angle', 'nonlinear', 'linear');
for a = [5 10 20 30 45 90]
    fprintf('%7d deg %10.3f s %10.3f s\n', a, t(find(rad2deg(x(:,1)) >= a, 1)), tl(find(rad2deg(xl(:,1)) >= a, 1)));
end
