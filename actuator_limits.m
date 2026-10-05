%% Effect of torque saturation on the LQR (Part 1, "Actuator limits")
% The controller output is clipped to [umin, umax] in the MATLAB Function block.
% Holding the pendulum at rest at angle theta needs the torque m*g*l*sin(theta),
% at most m*g*l (2.45 N m) at horizontal.
pendulum_setup
mdl = 'actuated_pendulum';
u_nom = umax;                  % nominal limit from pendulum_setup

%% Conditions common to all runs
use_observer = false;          % controller uses the measured state
u_noise = 0;  y_noise = 0;     % no noise
xhat0 = [0; 0];                % observer runs in parallel, not used for control

%% Start at rest: start angle x torque limit
starts = [20 35 60 90 180];            % deg
limits = [inf u_nom 2 1.8 1];          % N m
p.omega0 = 0;
fprintf('\nmax holding torque m*g*l = %.2f N m\n', m.m*g*m.l);
fprintf('largest angle that can be held: %s deg for limits %s N m\n', ...
        mat2str(round(asind(min(1, limits/(m.m*g*m.l))), 1)), mat2str(limits));
fprintf('\nSettling time [s] (|theta| < 1 deg), or the angle where it comes to rest\n%8s', 'theta0');
hdr = compose('u<=%g', limits);  fprintf('%12s', hdr{:});  fprintf('\n');
for th0 = starts
    fprintf('%8d', th0);
    for ulim = limits
        p.theta0 = th0;  umin = -ulim;  umax = ulim;
        r = run_case(mdl, ulim);
        if r.ok, fprintf('%12.2f', r.t_settle); else, fprintf('%12s', sprintf('(%.0f deg)', r.final)); end
    end
    fprintf('\n');
end

%% Check of the holding angle: start 1.5 deg inside and outside asin(ulim/(m*g*l))
fprintf('\n%6s %10s | %8s %4s | %8s %4s\n', 'u<=', 'hold angle', 'theta0', 'ok', 'theta0', 'ok');
for ulim = [2 1.8 1]
    hold_angle = asind(ulim/(m.m*g*m.l));
    umin = -ulim;  umax = ulim;
    p.theta0 = hold_angle - 1.5;  in  = run_case(mdl, ulim, 40);
    p.theta0 = hold_angle + 1.5;  out_ = run_case(mdl, ulim, 40);
    fprintf('%6g %10.1f | %8.1f %4d | %8.1f %4d\n', ulim, hold_angle, hold_angle - 1.5, in.ok, hold_angle + 1.5, out_.ok);
end

%% Nominal limit with initial velocity: [theta0 (deg), omega0 (deg/s)]
umin = -u_nom;  umax = u_nom;
fprintf('\nNominal limit %.1f N m, with initial velocity\n%8s %8s %4s %9s %9s %6s\n', u_nom, 'theta0', 'omega0', 'ok', 'max|th|', 't_settle', 'sat %');
for x0 = [0 100; 0 300; 0 600; 35 300; 35 -300; 90 -300]'
    p.theta0 = x0(1);  p.omega0 = x0(2);
    r = run_case(mdl, u_nom);
    fprintf('%8d %8d %4d %9.1f %9.2f %6.1f\n', x0(1), x0(2), r.ok, r.thmax, r.t_settle, r.sat);
end

%% Figure: release from 90 deg without limit, with the nominal limit and with 2 N m
figure('Position', [100 100 700 500]);
ax1 = subplot(2,1,1); hold on; grid on; ylabel('\theta [deg]');
ax2 = subplot(2,1,2); hold on; grid on; ylabel('u [N m]'); xlabel('t [s]');
p.theta0 = 90;  p.omega0 = 0;
for ulim = [inf u_nom 2]
    umin = -ulim;  umax = ulim;
    r = run_case(mdl, ulim);
    name = sprintf('|u| \\leq %g N m', ulim);  if isinf(ulim), name = 'no limit'; end
    plot(ax1, r.t, r.th, 'DisplayName', name);
    stairs(ax2, r.tu, r.u);
end
legend(ax1, 'Location', 'east');  set([ax1 ax2], 'XLim', [0 6]);
exportgraphics(gcf, 'actuator_limits.png', 'Resolution', 200);

function r = run_case(mdl, ulim, tstop)
% Simulate (15 s by default) and compute the metrics. Angles in deg.
%   ok         within 1 deg of upright for the last 2 s
%   t_settle   last time |theta| > 1 deg (NaN if not ok)
%   sat        share of time the torque is at its limit [%]
    if nargin < 3, tstop = 15; end
    out = sim(mdl, 'StopTime', num2str(tstop));
    r.t  = out.theta_sim.Time;  r.th = rad2deg(squeeze(out.theta_sim.Data));
    r.tu = out.u_sim.Time;      r.u  = squeeze(out.u_sim.Data);
    r.ok     = all(abs(r.th(r.t > tstop - 2)) < 1);
    r.thmax  = max(abs(r.th));
    r.final  = r.th(end);
    r.t_settle = NaN;  if r.ok, r.t_settle = max([0; r.t(abs(r.th) > 1)]); end
    r.sat    = 100 * mean(abs(r.u) >= 0.999*ulim);
end
