%% Effect of the sampling period (Part 1, "Sampling time")
% For each Ts the model is discretized again and the controller and the observer are
% redesigned with the same rules as in pendulum_setup: the same Q and R, and observer
% eigenvalues k_obs and 1.5*k_obs times faster than the dominant closed-loop eigenvalue.
% Three tests per Ts, all a release at rest from 20 deg:
%   controller   controller on the measured state
%   observer     observer in parallel with that run, estimate starting at zero
%   both         controller on the observer estimate, estimate starting at zero
pendulum_setup
mdl = 'actuated_pendulum';

%% Conditions common to all runs
u_noise = 0;  y_noise = 0;     % no noise
p.theta0 = 20;  p.omega0 = 0;
xhat0 = [0; 0];
% torque limits: nominal umin, umax from pendulum_setup

%% Sweep
fprintf('\n%6s %12s %7s | %8s %8s %7s | %7s | %8s %8s %7s\n', 'Ts', 'K', 'growth', ...
        't_settle', 'oversh.', 'max|u|', 't_conv', 't_settle', 'oversh.', 'max|u|');
fprintf('%27s | %25s | %7s | %25s\n', '', 'controller', 'obs.', 'both');
figure('Position', [100 100 700 500]);
ax1 = subplot(2,1,1); hold on; grid on; ylabel('\theta [deg]');
ax2 = subplot(2,1,2); hold on; grid on; ylabel('u [N m]'); xlabel('t [s]');
for Ts = [0.005 0.01 0.02 0.05 0.1 0.15 0.2 0.3 0.4 0.5]
    sysd = c2d(ss(Ac, Bc, C, 0), Ts, 'zoh');  Ad = sysd.A;  Bd = sysd.B;
    K = dlqr(Ad, Bd, Q, R);
    p_slow = max(abs(eig(Ad - Bd*K)));
    L = place(Ad', C', p_slow.^[k_obs, 1.5*k_obs])';

    use_observer = false;  ctrl = run_case(mdl);
    use_observer = true;   both = run_case(mdl);

    % growth: factor by which a deviation grows in open loop during one sampling period
    fprintf('%6g [%4.2f %5.2f] %7.2f | %8.2f %8.1f %7.2f | %7.2f | %8.2f %8.1f %7.2f\n', Ts, K, max(abs(eig(Ad))), ...
            ctrl.t_settle, ctrl.overshoot, ctrl.umax, ctrl.t_conv, both.t_settle, both.overshoot, both.umax);
    if ismember(Ts, [0.02 0.1 0.2 0.3])
        plot(ax1, both.t, both.th, '.-', 'DisplayName', sprintf('T_s = %g s', Ts));
        stairs(ax2, both.tu, both.u);
    end
end
legend(ax1, 'Location', 'northeast');  set([ax1 ax2], 'XLim', [0 5]);  ylim(ax1, [-40 40]);
exportgraphics(gcf, 'sampling_time.png', 'Resolution', 200);

function r = run_case(mdl)
% Simulate 10 s and compute the metrics from the sampled signals. Angles in deg.
%   t_settle    last time |theta| > 1 deg (NaN if not within 1 deg at the end)
%   overshoot   largest angle on the other side of upright
%   t_conv      last time the angle estimation error exceeds 0.1 deg (NaN if at the end)
    out = sim(mdl, 'StopTime', '10');
    r.t  = out.theta_sim.Time;  r.th = rad2deg(squeeze(out.theta_sim.Data));
    r.tu = out.u_sim.Time;      r.u  = squeeze(out.u_sim.Data);
    thh  = rad2deg(squeeze(out.theta_hat.Data));
    e    = mod(r.th - thh + 180, 360) - 180;
    r.t_conv = max([0; r.t(abs(e) > 0.1)]);    if abs(e(end)) > 0.1,  r.t_conv = NaN;   end
    r.t_settle = max([0; r.t(abs(r.th) > 1)]); if abs(r.th(end)) > 1, r.t_settle = NaN; end
    r.overshoot = max(0, -min(r.th));
    r.umax = max(abs(r.u));
end
