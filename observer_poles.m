%% Observer eigenvalues and incorrect initial estimate (Part 1, "Observer poles")
% The observer eigenvalues are set relative to the dominant (slowest) closed-loop
% eigenvalue of the controller, p_slow: eigenvalues p_slow^k and p_slow^(1.5k) decay
% k and 1.5k times faster. The design in pendulum_setup uses k = k_obs.
% The pendulum is released from 20 deg.
pendulum_setup
mdl = 'actuated_pendulum';
L_nom = L;

%% Conditions common to all runs
u_noise = 0;  y_noise = 0;     % no noise
p.theta0 = 20;  p.omega0 = 0;  % release at rest from 20 deg
x0 = [deg2rad(p.theta0); 0];
% torque limits: nominal umin, umax from pendulum_setup

%% Sweep
% "noise" columns: standard deviation of theta and u while balancing at upright for
% 30 s with a uniform measurement noise of +-a_y, observer in the loop. Simulated;
% "pred" is the same quantity predicted with the linear model, as a cross-check.
a_y = 0.005;                   % rad
fprintf('\n%5s %15s %15s | %7s %8s | %8s %8s %7s %6s | %8s | %8s %8s %8s\n', 'k', 'eigenvalues', 'L', ...
        't_conv', 'max e_w', 't_settle', 'min th', 'max|u|', 'sat %', 't_settle', 'std th', 'std u', 'pred u');
fprintf('%37s | %16s | %32s | %8s | %26s\n', '', 'parallel, xhat0=0', 'in the loop, xhat0=0', 'xhat0=x0', 'measurement noise');
figure('Position', [100 100 700 500]);
ax1 = subplot(2,1,1); hold on; grid on; ylabel('\theta [deg]');
ax2 = subplot(2,1,2); hold on; grid on; ylabel('u [N m]'); xlabel('t [s]');
for k = [0.5 2 5 10 30 100]
    obs_poles = p_slow.^[k, 1.5*k];
    L = place(Ad', C', obs_poles)';

    % observer running in parallel: controller uses the measured state
    use_observer = false;  xhat0 = [0; 0];
    par = run_case(mdl, 10);
    % observer in the loop, estimate starts at zero (20 deg wrong)
    use_observer = true;   xhat0 = [0; 0];
    wrong = run_case(mdl, 10);
    % observer in the loop, estimate starts at the true state
    use_observer = true;   xhat0 = x0;
    exact = run_case(mdl, 10);

    % balancing at upright with measurement noise, observer in the loop
    p.theta0 = 0;  xhat0 = [0; 0];  y_noise = a_y;
    noise = run_case(mdl, 30);
    p.theta0 = 20;  y_noise = 0;
    Pn = dlyap([Ad-Bd*K, Bd*K; zeros(2), Ad-L*C], [0; 0; L]*[0; 0; L]' * a_y^2/3);

    fprintf('%5g [%5.3f %5.3f] [%5.2f %7.2f] | %7.2f %8.2f | %8.2f %8.1f %7.2f %6.1f | %8.2f | %8.3f %8.3f %8.3f\n', k, obs_poles, L, ...
            par.t_conv, par.ewmax, wrong.t_settle, wrong.thmin, wrong.umax, wrong.sat, exact.t_settle, ...
            noise.th_std, noise.u_std, sqrt([-K K]*Pn*[-K K]'));
    if ismember(k, [0.5 2 10 100])
        plot(ax1, wrong.t, wrong.th, 'DisplayName', sprintf('k = %g', k));
        stairs(ax2, wrong.tu, wrong.u);
    end
end
legend(ax1, 'Location', 'northeast');  set([ax1 ax2], 'XLim', [0 4]);
exportgraphics(gcf, 'observer_poles.png', 'Resolution', 200);
L = L_nom;

function r = run_case(mdl, tstop)
% Simulate and compute the metrics. Angles in deg.
%   t_conv     last time the angle estimation error exceeds 0.1 deg
%   ewmax      largest velocity estimation error [rad/s]
%   t_settle   last time |theta| > 1 deg (NaN if not within 1 deg at the end)
    out = sim(mdl, 'StopTime', num2str(tstop));
    r.t  = out.theta_sim.Time;  r.th = rad2deg(squeeze(out.theta_sim.Data));
    r.tu = out.u_sim.Time;      r.u  = squeeze(out.u_sim.Data);
    w   = squeeze(out.omega_sim.Data);
    thh = rad2deg(squeeze(out.theta_hat.Data));  wh = squeeze(out.omega_hat.Data);
    e   = mod(r.th - thh + 180, 360) - 180;
    r.t_conv = max([0; r.t(abs(e) > 0.1)]);
    r.ewmax  = max(abs(w - wh));
    r.thmax  = max(abs(r.th));
    r.thmin  = min(r.th);
    r.umax   = max(abs(r.u));
    r.sat    = 100 * mean(abs(r.u) >= 0.999*evalin('base', 'umax'));
    r.t_settle = max([0; r.t(abs(r.th) > 1)]);  if abs(r.th(end)) > 1, r.t_settle = NaN; end
    r.th_std = std(r.th(r.t > 3));  r.u_std = std(r.u(r.tu > 3));   % used for the noise run
end
