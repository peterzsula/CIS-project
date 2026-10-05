%% Q/R sweep for the LQR controller (Part 1, "Choosing Q and R")
% The weights are parametrized with Bryson's rule,
%   Q = diag(1/th_max^2, 1/w_max^2),  R = 1/u_max^2,
% and each case changes one of the three limits of the nominal design.
% Two tests per case:
%   release   start at rest at 35 deg        -> settling time, control effort, max control
%   kick      start upright with 100 deg/s   -> max angular displacement, max control
pendulum_setup
mdl = 'actuated_pendulum';

%% Cases: [th_max (deg), w_max (rad/s), u_max (N m)], first row is the nominal design
nom = [rad2deg(th_max), w_max, 1/sqrt(R)];
cases = [nom
         60      nom(2)  nom(3)      % larger angle weight
         20      nom(2)  nom(3)
         nom(1)  8       nom(3)      % smaller velocity weight
         nom(1)  0.5     nom(3)      % larger velocity weight
         nom(1)  nom(2)  0.5         % expensive control
         nom(1)  nom(2)  12.5];      % cheap control

%% Conditions common to all runs
use_observer = false;          % controller uses the measured state
u_noise = 0;  y_noise = 0;     % no noise
umin = -inf;  umax = inf;      % no saturation
xhat0 = [0; 0];                % observer runs in parallel, not used for control

%% Run
fprintf('\n%29s | %25s | %16s\n', '', 'release from 35 deg', 'kick 100 deg/s');
fprintf('%6s %5s %5s %12s | %8s %7s %7s | %8s %7s\n', 'th_max', 'w_max', 'u_max', 'K', ...
        't_settle', 'effort', 'max|u|', 'max|th|', 'max|u|');
figure('Position', [100 100 900 600]);
ax = gobjects(2, 2);  for i = 1:4, ax(i) = subplot(2, 2, i);  hold(ax(i), 'on');  grid(ax(i), 'on');  end
ax = ax';                                    % ax(row, col): rows theta/u, cols release/kick
for c = cases'
    K = dlqr(Ad, Bd, diag([1/deg2rad(c(1))^2, 1/c(2)^2]), 1/c(3)^2);
    p.theta0 = 35;  p.omega0 = 0;    rel  = run_case(mdl);
    p.theta0 = 0;   p.omega0 = 100;  kick = run_case(mdl);
    fprintf('%6g %5g %5g [%4.2f %4.2f] | %8.2f %7.2f %7.2f | %8.1f %7.2f\n', c, K, ...
            rel.t_settle, rel.effort, rel.umax, kick.thmax, kick.umax);

    lw = 0.75 + 1.25*isequal(c', nom);       % nominal design drawn thicker
    name = sprintf('%g deg, %g rad/s, %g N m', c);
    plot(ax(1,1), rel.t, rel.th, 'LineWidth', lw, 'DisplayName', name);
    stairs(ax(2,1), rel.tu, rel.u, 'LineWidth', lw);
    plot(ax(1,2), kick.t, kick.th, 'LineWidth', lw);
    stairs(ax(2,2), kick.tu, kick.u, 'LineWidth', lw);
end
title(ax(1,1), 'Release from 35 deg');  title(ax(1,2), 'Kick 100 deg/s');
ylabel(ax(1,1), '\theta [deg]');  ylabel(ax(2,1), 'u [N m]');
xlabel(ax(2,1), 't [s]');  xlabel(ax(2,2), 't [s]');
set(ax(:,1), 'XLim', [0 3]);  set(ax(:,2), 'XLim', [0 1.5]);
legend(ax(1,1), 'Location', 'northeast');
exportgraphics(gcf, 'qr_sweep.png', 'Resolution', 200);

function r = run_case(mdl)
% Simulate 10 s and compute the metrics. Angles in deg.
%   t_settle   last time |theta| > 1 deg (NaN if it has not settled by the end)
%   effort     integral of u^2
    out = sim(mdl, 'StopTime', '10');
    t = out.theta_sim.Time;  th = rad2deg(squeeze(out.theta_sim.Data));
    u = squeeze(out.u_sim.Data);
    r.t_settle = max([0; t(abs(th) > 1)]);  if abs(th(end)) > 1, r.t_settle = NaN; end
    r.thmax  = max(abs(th));
    r.effort = trapz(out.u_sim.Time, u.^2);
    r.umax   = max(abs(u));
    r.t = t;  r.th = th;  r.tu = out.u_sim.Time;  r.u = u;
end
