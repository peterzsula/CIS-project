%% Other ways to estimate the angular velocity (Part 1, "Alternative estimation methods")
% The observer block computes  xhat[k+1] = Ae*xhat[k] + Be*u[k] + Le*(y[k] - C*xhat[k])
% with the workspace variables Ad, Bd, L in the places of Ae, Be, Le. Other choices of
% these three matrices turn the same block into a model-free estimator:
%   backward difference   xhat1[k+1] = y[k],  xhat2[k+1] = (y[k] - y[k-1])/Ts
%   filtered difference   as above, with a first-order low-pass on xhat2 (factor alpha)
% The controller always uses K from pendulum_setup and acts on the estimate.
pendulum_setup
mdl = 'actuated_pendulum';
alpha = 0.8;
est = {'observer (k = 5)',     Ad,           Bd,     L
       'backward difference',  [1 0; 0 0],   [0; 0], [1; 1/Ts]
       'filtered difference',  [1 0; 0 alpha], [0; 0], [1; (1-alpha)/Ts]};

%% Conditions common to all runs
use_observer = true;           % controller uses the estimate
u_noise = 0;                   % no input disturbance
p.omega0 = 0;
% torque limits: nominal umin, umax from pendulum_setup

%% Run
fprintf('\n%-22s | %8s %8s | %8s %8s %8s %8s\n', '', 't_settle', 'max e_w', 'std th', 'std u', 'rms e_w', 'max|u|');
fprintf('%-22s | %17s | %35s\n', 'estimator', 'release 20 deg', 'balancing, noise +-0.005 rad');
figure('Position', [100 100 700 350]); hold on; grid on;
for i = 1:size(est, 1)
    [name, Ad, Bd, L] = est{i,:};

    % release from 20 deg, no noise, correct initial estimate
    p.theta0 = 20;  xhat0 = [deg2rad(20); 0];  y_noise = 0;
    out = sim(mdl, 'StopTime', '10');
    t = out.theta_sim.Time;  th = rad2deg(squeeze(out.theta_sim.Data));
    ew = squeeze(out.omega_sim.Data) - squeeze(out.omega_hat.Data);
    t_settle = max([0; t(abs(th) > 1)]);  if abs(th(end)) > 1, t_settle = NaN; end
    ew_max = max(abs(ew));

    % balancing at upright with measurement noise
    p.theta0 = 0;  xhat0 = [0; 0];  y_noise = 0.005;
    out = sim(mdl, 'StopTime', '30');
    t = out.theta_sim.Time;  th = rad2deg(squeeze(out.theta_sim.Data));  u = squeeze(out.u_sim.Data);
    w = squeeze(out.omega_sim.Data);  wh = squeeze(out.omega_hat.Data);
    s = t > 3;
    fprintf('%-22s | %8.2f %8.2f | %8.3f %8.3f %8.3f %8.2f\n', name, t_settle, ew_max, ...
            std(th(s)), std(u(out.u_sim.Time > 3)), rms(w(s) - wh(s)), max(abs(u)));
    if i == 1, plot(t, w, 'k', 'LineWidth', 1.5, 'DisplayName', 'true (observer run)'); end
    if i < 3,  plot(t, wh, 'DisplayName', name); end
end
xlim([5 7]); xlabel('t [s]'); ylabel('\omega [rad/s]'); legend('Location', 'northeast');
exportgraphics(gcf, 'alternative_estimation.png', 'Resolution', 200);
