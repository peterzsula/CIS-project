%% Disturbance on the control input (Part 1, "Disturbances")
% The plant receives u[k] = -K*x[k] + v[k], with v[k] uniform in [-a, a] and a new
% value every Ts. The controller does not know v.
pendulum_setup
mdl = 'actuated_pendulum';

%% Conditions common to all runs
use_observer = false;          % controller uses the measured state
y_noise = 0;                   % no measurement noise
xhat0 = [0; 0];                % observer runs in parallel, not used for control
% torque limits: nominal umin, umax from pendulum_setup

%% Balancing at upright for 30 s: size of the fluctuations vs. noise amplitude
% "predicted" is the standard deviation of theta from the linear closed-loop model,
% x[k+1] = (Ad - Bd*K) x[k] + Bd v[k], with var(v) = a^2/3.
p.theta0 = 0;  p.omega0 = 0;
fprintf('\n%8s | %10s %10s %10s | %9s %8s %6s\n', 'a [N m]', 'std th', 'predicted', 'max|th|', 'std u', 'max|u|', 'sat %');
for a = [0.005 0.05 0.5 1 2]
    u_noise = a;
    out = sim(mdl, 'StopTime', '30');
    t = out.theta_sim.Time;  th = rad2deg(squeeze(out.theta_sim.Data));  u = squeeze(out.u_sim.Data);
    P = dlyap(Ad - Bd*K, Bd*Bd' * a^2/3);
    fprintf('%8g | %10.4f %10.4f %10.4f | %9.4f %8.3f %6.1f\n', a, std(th(t > 3)), rad2deg(sqrt(P(1,1))), ...
            max(abs(th)), std(u(out.u_sim.Time > 3)), max(abs(u)), 100*mean(abs(u) >= 0.999*umax));
end

%% Release from 35 deg with and without disturbance
p.theta0 = 35;
figure('Position', [100 100 700 500]);
ax1 = subplot(2,1,1); hold on; grid on; ylabel('\theta [deg]');
ax2 = subplot(2,1,2); hold on; grid on; ylabel('u [N m]'); xlabel('t [s]');
fprintf('\nRelease from 35 deg\n%8s | %14s %12s\n', 'a [N m]', 'max|th|, t>3 s', 'final th');
for a = [0 0.05 0.5]
    u_noise = a;
    out = sim(mdl, 'StopTime', '6');
    t = out.theta_sim.Time;  th = rad2deg(squeeze(out.theta_sim.Data));
    fprintf('%8g | %14.3f %12.3f\n', a, max(abs(th(t > 3))), th(end));
    plot(ax1, t, th, 'DisplayName', sprintf('a = %g N m', a));
    stairs(ax2, out.u_sim.Time, squeeze(out.u_sim.Data));
end
legend(ax1, 'Location', 'northeast');
exportgraphics(gcf, 'input_disturbance.png', 'Resolution', 200);
