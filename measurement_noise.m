%% Measurement noise (Part 1, "Measurement disturbances")
% The observer receives y[k] = C*x[k] + w[k], with w[k] uniform in [-a, a] and a new
% value every Ts. The controller acts on the observer estimate.
% "pred" values are standard deviations predicted with the linear model of plant,
% controller and observer, with state [x; e], e = x - xhat, and var(w) = a^2/3.
pendulum_setup
mdl = 'actuated_pendulum';
L_nom = L;

%% Conditions common to all runs
use_observer = true;           % controller uses the observer estimate
u_noise = 0;                   % no input disturbance
p.theta0 = 0;  p.omega0 = 0;   % balancing at upright
xhat0 = [0; 0];                % correct initial estimate
% torque limits: nominal umin, umax from pendulum_setup

%% Nominal observer, noise amplitude varied
fprintf('\nNominal observer (k = %g)\n%8s %8s | %8s %8s %8s | %8s %8s %8s %6s\n', k_obs, 'a [rad]', 'a [deg]', ...
        'std th', 'pred', 'max|th|', 'std u', 'pred', 'max|u|', 'sat %');
for a = [0.005 0.02 0.05 0.1 0.2]
    y_noise = a;
    r = run_case(mdl, Ad, Bd, K, L, a);
    fprintf('%8g %8.2f | %8.3f %8.3f %8.2f | %8.3f %8.3f %8.2f %6.1f\n', a, rad2deg(a), ...
            r.th_std, r.th_pred, r.thmax, r.u_std, r.u_pred, r.umax, r.sat);
end

%% Figure: chosen observer and a very fast one (k as in observer_poles.m), a = 0.005 rad
y_noise = 0.005;
figure('Position', [100 100 700 500]);
ax1 = subplot(2,1,1); hold on; grid on; ylabel('\theta [deg]');
ax2 = subplot(2,1,2); hold on; grid on; ylabel('u [N m]'); xlabel('t [s]');
for k = [100 k_obs]                      % 100 first, so it is drawn underneath
    L = place(Ad', C', p_slow.^[k, 1.5*k])';
    r = run_case(mdl, Ad, Bd, K, L, y_noise);
    plot(ax1, r.t, r.th, 'DisplayName', sprintf('k = %g', k));
    stairs(ax2, r.tu, r.u);
end
legend(ax1, 'Location', 'northeast');  set([ax1 ax2], 'XLim', [0 5]);
exportgraphics(gcf, 'measurement_noise.png', 'Resolution', 200);
L = L_nom;

function r = run_case(mdl, Ad, Bd, K, L, a)
% Simulate 30 s and compute the metrics for t > 3 s. Angles in deg.
    out = sim(mdl, 'StopTime', '30');
    r.t  = out.theta_sim.Time;  r.th = rad2deg(squeeze(out.theta_sim.Data));
    r.tu = out.u_sim.Time;      r.u  = squeeze(out.u_sim.Data);
    r.th_std = std(r.th(r.t > 3));   r.thmax = max(abs(r.th));
    r.u_std  = std(r.u(r.tu > 3));   r.umax  = max(abs(r.u));
    r.sat    = 100 * mean(abs(r.u) >= 0.999*evalin('base', 'umax'));
    Pn = dlyap([Ad-Bd*K, Bd*K; zeros(2), Ad-L*[1 0]], [0; 0; L]*[0; 0; L]' * a^2/3);
    r.th_pred = rad2deg(sqrt(Pn(1,1)));
    r.u_pred  = sqrt([-K K]*Pn*[-K K]');
end
