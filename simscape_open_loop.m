%% Open-loop check: Simscape plant vs. ode45 of the pendulum equation (report 1.1)
% Runs the Simscape plant with zero torque from a 0.01 rad perturbation and
% compares it with ode45 of the pendulum equation using the model parameters m.*

pendulum_setup

%% Conditions of this experiment
K = [0 0];                                   % zero torque
p.theta0 = rad2deg(0.01);  p.omega0 = 0;     % 0.01 rad perturbation from upright
use_observer = false;  xhat0 = [0; 0];       % irrelevant with K = 0
u_noise = 0;  y_noise = 0;                   % no noise

%% Simscape plant
out = sim('actuated_pendulum', 'StopTime', '20');

ts = out.theta_sim.Time;  ths = squeeze(out.theta_sim.Data);
assert(max(abs(squeeze(out.u_sim.Data))) == 0, 'torque was not zero');

t90 = @(t, th) t(find(abs(th) >= pi/2, 1));  % first time past horizontal
fprintf('Simscape     : t90 = %.3f s, peak = %.1f deg, final = %.2f deg\n', ...
    t90(ts, ths), max(abs(rad2deg(ths))), rad2deg(ths(end)));

%% ode45 with the model parameters
opts = odeset('RelTol', 1e-8, 'AbsTol', 1e-10);
f = @(t, x) [x(2); (m.m*g*m.l*sin(x(1)) - m.b*x(2)) / m.J];
[t, x] = ode45(f, ts, [0.01; 0], opts);
fprintf('ode45 l=%.2f : t90 = %.3f s, peak = %.1f deg, final = %.2f deg, max diff to Simscape = %.1f deg\n', ...
    m.l, t90(t, x(:,1)), max(abs(rad2deg(x(:,1)))), rad2deg(x(end,1)), max(abs(rad2deg(x(:,1) - ths))));

figure; plot(ts, rad2deg(ths), 'k', 'LineWidth', 1.5); hold on
plot(t, rad2deg(x(:,1)), '--');
legend('Simscape plant', 'ode45');
xlabel('t [s]'); ylabel('\theta [deg]'); grid on
exportgraphics(gcf, 'simscape_vs_ode.png', 'Resolution', 200);
