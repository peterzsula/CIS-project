%% Model uncertainty (Part 1, "Model uncertainties")
% The controller gain K is designed on the nominal model m.* (pendulum_setup).
% The plant parameters p.* are then scaled one at a time, so the model no longer
% matches the system being controlled.
pendulum_setup
mdl = 'actuated_pendulum';
p_nom = p;                     % nominal plant

%% Conditions common to all runs
use_observer = false;          % controller uses the measured state
u_noise = 0;  y_noise = 0;     % no noise
xhat0 = [0; 0];                % observer runs in parallel, not used for control
p.theta0 = 20;  p.omega0 = 0;  % release at rest from 20 deg
% torque limits: nominal umin, umax from pendulum_setup

%% Scale one plant parameter at a time
% rho: largest closed-loop eigenvalue magnitude of the linear model of the actual
%      plant with the nominal K (stable if rho < 1)
sweeps = {'m', [0.1 0.2 0.5 0.8 1 1.2 1.5 1.8 2]
          'l', [0.1 0.2 0.5 0.8 1 1.2 1.5 1.8 2]
          'b', [0 1 10 100]};
figure('Position', [100 100 700 300]); hold on; grid on;   % responses for the mass sweep
for s = 1:size(sweeps, 1)
    name = sweeps{s,1};
    fprintf('\nplant p.%s scaled\n%7s %7s %4s %9s %8s %6s\n', name, 'factor', 'rho', 'ok', 't_settle', 'max|u|', 'sat %');
    for f = sweeps{s,2}
        p = p_nom;  p.theta0 = 20;  p.omega0 = 0;
        p.(name) = f * p_nom.(name);

        lt = p.l/2;  Jt = p.m*lt^2;                         % actual plant, linearized
        st = c2d(ss([0 1; g/lt, -p.b/Jt], [0; 1/Jt], C, 0), Ts, 'zoh');
        rho = max(abs(eig(st.A - st.B*K)));

        out = sim(mdl, 'StopTime', '10');
        t = out.theta_sim.Time;  th = rad2deg(squeeze(out.theta_sim.Data));  u = squeeze(out.u_sim.Data);
        ok = all(abs(th(t > 8)) < 1);
        ts = NaN;  if ok, ts = max([0; t(abs(th) > 1)]); end
        fprintf('%7g %7.3f %4d %9.2f %8.2f %6.1f\n', f, rho, ok, ts, max(abs(u)), 100*mean(abs(u) >= 0.999*umax));
        if name == 'm' && ismember(f, [0.5 1 1.5 2]), plot(t, th, 'DisplayName', sprintf('mass x %g', f)); end
    end
end
xlabel('t [s]'); ylabel('\theta [deg]'); xlim([0 6]); ylim([-5 60]); legend('Location', 'northeast');
exportgraphics(gcf, 'model_uncertainty.png', 'Resolution', 200);

%% Stability threshold for a heavier / longer plant
% Predicted: the factor at which rho = 1. Simulated: 300 s runs around it; near the
% threshold the dynamics are very slow, so the angle after 150 s and 300 s is compared.
for s = 1:2
    name = sweeps{s,1};
    f_pred = fzero(@(f) rho_of(f, name, p_nom, g, C, Ts, K) - 1, [1.5 2]);
    fprintf('\nplant p.%s scaled, predicted threshold: factor %.3f\n%7s %7s %12s %12s\n', name, f_pred, 'factor', 'rho', 'th(150 s)', 'th(300 s)');
    for f = [1.80 1.82 1.84 1.86 1.90]
        p = p_nom;  p.theta0 = 20;  p.omega0 = 0;
        p.(name) = f * p_nom.(name);
        out = sim(mdl, 'StopTime', '300');
        t = out.theta_sim.Time;  th = rad2deg(squeeze(out.theta_sim.Data));
        fprintf('%7.2f %7.4f %12.3f %12.3f\n', f, rho_of(f, name, p_nom, g, C, Ts, K), interp1(t, th, 150), th(end));
    end
end
p = p_nom;

function rho = rho_of(f, name, p, g, C, Ts, K)
% Largest closed-loop eigenvalue magnitude of the linearized plant with p.(name) scaled by f
    p.(name) = f * p.(name);
    lt = p.l/2;  Jt = p.m*lt^2;
    st = c2d(ss([0 1; g/lt, -p.b/Jt], [0; 1/Jt], C, 0), Ts, 'zoh');
    rho = max(abs(eig(st.A - st.B*K)));
end
