%% LQR on the nonlinear plant from different initial states (Part 1, "Evaluation")
% The measured angle is not wrapped, so theta = 0 is the only target.
pendulum_setup
mdl = 'actuated_pendulum';

%% Conditions common to all runs
use_observer = false;          % controller uses the measured state
u_noise = 0;  y_noise = 0;     % no noise
xhat0 = [0; 0];                % observer runs in parallel, not used for control
umin = -inf;  umax = inf;      % no saturation

%% Different initial states
% rows: [theta0 (deg), omega0 (deg/s)]
X0 = [5 0; 35 0; 90 0; 135 0; 180 0; 35 300; 35 -300; 0 600; 180 600];
fprintf('\n%8s %8s %4s %8s %9s %9s\n', 'theta0', 'omega0', 'ok', 'max|u|', 'max|th|', 't_settle');
figure('Position', [100 100 700 500]);
ax1 = subplot(2,1,1); hold on; grid on; ylabel('\theta [deg]');
ax2 = subplot(2,1,2); hold on; grid on; ylabel('u [N m]'); xlabel('t [s]');
for x0 = X0'
    p.theta0 = x0(1);  p.omega0 = x0(2);
    r = run_case(mdl);
    fprintf('%8d %8d %4d %8.2f %9.1f %9.2f\n', x0(1), x0(2), r.ok, r.umax, r.thmax, r.t_settle);
    if x0(2) == 0                            % figure: the releases from rest
        plot(ax1, r.t, r.th, 'DisplayName', sprintf('\\theta(0) = %d deg', x0(1)));
        stairs(ax2, r.tu, r.u);
    end
end
legend(ax1, 'Location', 'northeast');  set([ax1 ax2], 'XLim', [0 4]);
exportgraphics(gcf, 'lqr_evaluation.png', 'Resolution', 200);

function r = run_case(mdl)
% Simulate 10 s and compute the metrics. Angles in deg.
%   ok         within 1 deg of upright for the last 2 s
%   t_settle   last time |theta| > 1 deg (NaN if not ok)
    out = sim(mdl, 'StopTime', '10');
    t  = out.theta_sim.Time;  th = rad2deg(squeeze(out.theta_sim.Data));
    r.ok     = all(abs(th(t > 8)) < 1);
    r.umax   = max(abs(squeeze(out.u_sim.Data)));
    r.thmax  = max(abs(th));
    r.t_settle = NaN;  if r.ok, r.t_settle = max([0; t(abs(th) > 1)]); end
    r.t = t;  r.th = th;  r.tu = out.u_sim.Time;  r.u = squeeze(out.u_sim.Data);
end
