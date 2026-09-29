%% Q/R sweep for the LQR controller, starting from hanging down (180 deg)
% Run pendulum_script.m once first (it defines p, m, g, Ts, Ad, Bd, C, umin, umax).
% The torque limits from pendulum_script are kept, because the swing-up depends on them.
% The observer block must contain the angle wrapping.
%
% Metrics (on the wrapped angle, so 360 deg counts as upright):
%   success      ends within 2 deg of upright and stays there for the last 2 s
%   t_reach      first time |theta| < 10 deg (arrives near the top)
%   t_settle     last time |theta| > 2 deg
%   turns        full revolutions made before settling (spinning)
%   final        final angle (shows the tilted saturated equilibrium if it gets stuck)
%   effort       integral of u^2
%   sat %        share of time the torque was at its limit

%% Fixed test conditions, so only Q and R differ between runs
p.theta0 = 180;                       % deg, hanging down
xhat0 = [0; 0];       % perfect initial estimate: no lucky observer kick
L = place(Ad', C', [0.6 0.65])';      % fixed observer, so it does not change with K

% Switch off both noise sources (restored at the end).
% The block requires Minimum < Maximum, so use a negligibly small range instead of 0.
load_system('actuated_pendulum');     % find_system only searches loaded models
rnd = find_system('actuated_pendulum', 'BlockType', 'UniformRandomNumber');
oldMin = get_param(rnd, 'Minimum');  oldMax = get_param(rnd, 'Maximum');
for i = 1:numel(rnd), set_param(rnd{i}, 'Minimum', '-1e-12', 'Maximum', '1e-12'); end

%% Cases: {Q, R, label}
th_max = deg2rad(180);  w_max = 2;  u_max = umax;         % Bryson's rule (notes eq. 43-44)
cases = {
    eye(2),                          1,           'baseline Q=I, R=1'
    eye(2),                          0.1,         'cheap control R=0.1'
    eye(2),                          10,          'expensive control R=10'
    diag([10 1]),                    1,           'angle weight q1=10'
    diag([1 10]),                    1,           'velocity weight q2=10'
    diag([1/th_max^2, 1/w_max^2]),   1/u_max^2,   'Bryson (10 deg, 2 rad/s, umax)'
};

%% Run
wrap = @(a) mod(a + pi, 2*pi) - pi;
res = struct();
figure;  ax1 = subplot(2,1,1); hold on;  ax2 = subplot(2,1,2); hold on;
for c = 1:size(cases,1)
    [K, ~, P] = dlqr(Ad, Bd, cases{c,1}, cases{c,2});
    out = sim("actuated_pendulum");

    t  = out.theta_sim.Time;  th = squeeze(out.theta_sim.Data);  thw = wrap(th);
    tu = out.u_sim.Time;      u  = squeeze(out.u_sim.Data);

    ok = abs(thw) < deg2rad(2);
    res(c).label   = cases{c,3};
    res(c).K       = K;
    res(c).success = all(ok(t > t(end) - 2));
    i_reach = find(abs(thw) < deg2rad(10), 1, 'first');
    res(c).t_reach = NaN;  if ~isempty(i_reach), res(c).t_reach = t(i_reach); end
    i_last  = find(~ok, 1, 'last');
    res(c).t_settle = NaN; if res(c).success, res(c).t_settle = t(min(i_last+1, numel(t))); end
    res(c).turns   = round((th(end) - th(1)) / (2*pi));
    res(c).final   = rad2deg(thw(end));
    res(c).effort  = trapz(tu, u.^2);
    res(c).umax    = max(abs(u));
    res(c).sat     = 100 * mean(abs(u) >= 0.999*max(abs([umin umax])));

    plot(ax1, t, rad2deg(thw), 'DisplayName', cases{c,3});
    stairs(ax2, tu, u, 'DisplayName', cases{c,3});
end

% Restore noise settings
for i = 1:numel(rnd), set_param(rnd{i}, 'Minimum', oldMin{i}, 'Maximum', oldMax{i}); end

ylabel(ax1, '\theta wrapped [deg]'); ylim(ax1, [-190 190]); yticks(ax1, -180:90:180);
legend(ax1, 'Location', 'best'); grid(ax1, 'on');
ylabel(ax2, 'u [N m]'); xlabel(ax2, 't [s]'); grid(ax2, 'on');
title(ax1, sprintf('Start at %d deg, |u| <= %.2f N m', p.theta0, umax));

%% Table
fprintf('\nStart %d deg, torque limit [%.2f, %.2f] N m\n', p.theta0, umin, umax);
fprintf('%-32s %16s %4s %8s %9s %6s %8s %8s %7s\n', 'case', 'K', 'ok', ...
        't_reach', 't_settle', 'turns', 'final', 'effort', 'sat %');
for c = 1:numel(res)
    fprintf('%-32s [%6.2f %6.2f] %4d %8.2f %9.2f %6d %8.1f %8.2f %7.1f\n', res(c).label, ...
        res(c).K, res(c).success, res(c).t_reach, res(c).t_settle, res(c).turns, ...
        res(c).final, res(c).effort, res(c).sat);
end
