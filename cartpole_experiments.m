%% Cart-pole experiments (Part 2)
% Runs setup_cartpole.m first (nominal design, iLQR and Simscape run) and then the
% experiments of the assignment. iLQR and the capture-zone runs use the nonlinear
% model in discrete_step.m; the runs marked "Simscape" use the Simscape plant.
setup_cartpole
close all
wrap = @(a) atan2(sin(a), cos(a));
nominal = struct('Xnom', Xnom, 'Unom', Unom, 'Knom', Knom, 'N', N, 'T', T);

fprintf('\n===== Nominal design (setup_cartpole.m), Simscape =====\n');
report_sim(out, capture, T, xmax);
fprintf('nominal terminal state [p v theta w] = [%.2f %.2f %.1f deg %.2f], iLQR iterations %d, cost %.5g -> %.5g\n', ...
        Xnom(1,end), Xnom(2,end), rad2deg(wrap(Xnom(3,end))), Xnom(4,end), numel(costHistory)-1, costHistory(1), costHistory(end));

%% Upright LQR on the Simscape plant: start at rest 15 deg from upright
fprintf('\n===== Upright LQR, Simscape, start at theta = 15 deg =====\n');
param.theta0 = 15;
outLqr = sim(modelName, 'StopTime', '8');
report_sim(outLqr, capture, 0, xmax);
param.theta0 = 180;

%% Capture zone of the saturated LQR (nonlinear model, cart must stay within xmax)
fprintf('\n===== Capture zone of the saturated LQR =====\n');
names = {'p [m]', 'v [m/s]', 'theta [deg]', 'omega [rad/s]'};
grids = {0:0.05:2, 0:0.25:12, 0:1:90, 0:0.25:15};  unit = [1 1 pi/180 1];
for j = 1:4
    largest = NaN;
    for v = grids{j}
        xi = zeros(4,1);  xi(j) = v*unit(j);
        if lqr_recovers(xi, K_lqr, model, Ts, Fmax, xmax), largest = v; else, break; end
    end
    fprintf('largest %-14s alone that is recovered: %g\n', names{j}, largest);
end
boxes = [capture; 0.5 1.0 deg2rad(20) 0.75; 0.4 1.0 deg2rad(20) 0.75; 0.3 1.0 deg2rad(20) 0.75; 0.3 0.75 deg2rad(15) 0.75];
for b = boxes'
    fails = 0;
    for c = 0:15
        sgn = 2*(dec2bin(c,4) == '1') - 1;
        fails = fails + ~lqr_recovers((b'.*sgn)', K_lqr, model, Ts, Fmax, xmax);
    end
    fprintf('box |p|<=%.2f |v|<=%.2f |theta|<=%.0f deg |w|<=%.2f: %2d of 16 corners not recovered\n', b(1), b(2), rad2deg(b(3)), b(4), fails);
end

%% iLQR experiments
% success = terminal state inside the capture box and cart travel within xmax
guessNames = {'sine 1.3 Hz', 'zeros', 'sine 0.5 Hz, 5 N'};
guess = @(i, N, F) pick({F*sin(2*pi*1.30*(0:N-1)*Ts + 3*pi/2), zeros(1,N), 5*sin(2*pi*0.5*(0:N-1)*Ts)}, i);

fprintf('\n===== iLQR: initial guess (nominal weights, T = %.2f s) =====\n', T);
for i = 1:3, run_ilqr(guessNames{i}, x0, guess(i, N, Fmax), model, Ts, Q, R, Qf, Fmax, capture, xmax); end

fprintf('\n===== iLQR: horizon (nominal weights, zero guess) =====\n');
for Th = [2 3 4.18 5 6]
    Nh = round(Th/Ts);
    run_ilqr(sprintf('T = %.2f s', Th), x0, guess(2, Nh, Fmax), model, Ts, Q, R, Qf, Fmax, capture, xmax);
end

fprintf('\n===== iLQR: maximum force (nominal weights, zero guess) =====\n');
for F = [5 7.5 10 15 20 30]
    run_ilqr(sprintf('Fmax = %g N', F), x0, guess(2, N, F), model, Ts, Q, R, Qf, F, capture, xmax);
end

fprintf('\n===== iLQR: weights. Running cost qs*Q_lqr, R varied; successes out of 9 =====\n');
fprintf('(3 initial guesses x 3 horizons: 3, 4.18 and 5 s)\n%8s %8s %10s\n', 'qs', 'R', 'successes');
for qs = [1 0.1 0.01 0.001]
    for Rr = [R_lqr 0.04]
        count = 0;
        for Th = [3 4.18 5]
            Nh = round(Th/Ts);
            for i = 1:3
                [Xg, ~] = ilqr(x0, guess(i, Nh, Fmax), model, Ts, qs*Q_lqr, Rr, Qf, Fmax);
                count = count + is_success(Xg, capture, xmax);
            end
        end
        fprintf('%8g %8g %10d\n', qs, Rr, count);
    end
end

%% Upright LQR weights as running cost: Q = Q_lqr, R = R_lqr
fprintf('\n===== iLQR with Q = Q_lqr, R = R_lqr: initial guess (T = %.2f s) =====\n', T);
for i = 1:3, run_ilqr(guessNames{i}, x0, guess(i, N, Fmax), model, Ts, Q_lqr, R_lqr, Qf, Fmax, capture, xmax); end
fprintf('\n===== iLQR with Q = Q_lqr, R = R_lqr: horizon (sine guess) =====\n');
for Th = [2 3 4.18 5 6]
    Nh = round(Th/Ts);
    run_ilqr(sprintf('T = %.2f s', Th), x0, guess(1, Nh, Fmax), model, Ts, Q_lqr, R_lqr, Qf, Fmax, capture, xmax);
end
fprintf('\n===== iLQR with Q = Q_lqr, R = R_lqr: maximum force (sine guess with amplitude Fmax) =====\n');
for F = [10 15 19 20 21 25 30]
    run_ilqr(sprintf('Fmax = %g N', F), x0, guess(1, N, F), model, Ts, Q_lqr, R_lqr, Qf, F, capture, xmax);
end

fprintf('\n===== Upright LQR weights, sine guess, Simscape =====\n');
[Xnom, Unom, Knom, ch2] = ilqr(x0, guess(1, N, Fmax), model, Ts, Q_lqr, R_lqr, Qf, Fmax);
fprintf('iLQR iterations %d, cost %.5g -> %.5g, terminal [%.2f %.2f %.1f deg %.2f]\n', numel(ch2)-1, ch2(1), ch2(end), ...
        Xnom(1,end), Xnom(2,end), rad2deg(wrap(Xnom(3,end))), Xnom(4,end));
out2 = sim(modelName, 'StopTime', num2str(T+8));
report_sim(out2, capture, T, xmax);

%% Figure: nominal design and upright LQR weights on the Simscape plant
figure('Position', [100 100 700 520]);
runs = {out, 'nominal: small running cost, zero guess'; out2, 'LQR weights as running cost, tuned guess'};
for i = 1:2
    xs = squeeze(double(runs{i,1}.x.Data));  if size(xs,1) == 4, xs = xs.'; end
    ts = double(runs{i,1}.x.Time(:));
    subplot(3,1,1); hold on; plot(ts, rad2deg(wrap(xs(:,3))), 'DisplayName', runs{i,2});
    subplot(3,1,2); hold on; plot(ts, xs(:,1));
    subplot(3,1,3); hold on; stairs(double(runs{i,1}.u.Time(:)), squeeze(double(runs{i,1}.u.Data)));
end
subplot(3,1,1); grid on; ylabel('\theta [deg]'); xlim([0 9]); legend('Location', 'southeast');
subplot(3,1,2); grid on; ylabel('p [m]'); xlim([0 9]); yline(xmax, '--'); yline(-xmax, '--');
subplot(3,1,3); grid on; ylabel('F [N]'); xlabel('t [s]'); xlim([0 9]);
exportgraphics(gcf, 'cartpole_swingup.png', 'Resolution', 200);

% leave the workspace with the nominal trajectory of setup_cartpole.m
Xnom = nominal.Xnom;  Unom = nominal.Unom;  Knom = nominal.Knom;

%% Local functions
function v = pick(c, i), v = c{i}; end

function ok = is_success(X, capture, xmax)
    xe = X(:,end);  xe(3) = atan2(sin(xe(3)), cos(xe(3)));
    ok = all(abs(xe') <= capture) && max(abs(X(1,:))) <= xmax;
end

function run_ilqr(label, x0, U0, model, Ts, Q, R, Qf, Fmax, capture, xmax)
    [X, U, ~, ch] = ilqr(x0, U0, model, Ts, Q, R, Qf, Fmax);
    fprintf('%-18s success %d | iterations %2d, cost %9.5g -> %9.5g | terminal [%5.2f %5.2f %6.1f deg %5.2f] | peak |p| %.2f m, saturated %2.0f %%\n', ...
            label, is_success(X, capture, xmax), numel(ch)-1, ch(1), ch(end), X(1,end), X(2,end), ...
            rad2deg(atan2(sin(X(3,end)), cos(X(3,end)))), X(4,end), max(abs(X(1,:))), 100*mean(abs(U) >= Fmax - 1e-6));
end

function ok = lqr_recovers(x, K, model, Ts, Fmax, xmax)
% Saturated LQR on the nonlinear model for 10 s; the cart must stay within xmax.
    ok = false;
    for k = 1:1000
        xw = x;  xw(3) = atan2(sin(x(3)), cos(x(3)));
        x = discrete_step(x, min(Fmax, max(-Fmax, -K*xw)), model, Ts);
        if abs(x(1)) > xmax || any(~isfinite(x)), return; end
    end
    ok = abs(x(1)) < 0.05 && abs(x(2)) < 0.1 && abs(atan2(sin(x(3)), cos(x(3)))) < deg2rad(2) && abs(x(4)) < 0.1;
end

function report_sim(out, capture, T, xmax)
% Capture time, cart travel and settling of a Simscape run.
    xs = squeeze(double(out.x.Data));  if size(xs,1) == 4, xs = xs.'; end
    ts = double(out.x.Time(:));
    th = atan2(sin(xs(:,3)), cos(xs(:,3)));
    inCap = abs(xs(:,1)) <= capture(1) & abs(xs(:,2)) <= capture(2) & abs(th) <= capture(3) & abs(xs(:,4)) <= capture(4);
    settled = abs(xs(:,1)) <= 0.05 & abs(xs(:,2)) <= 0.10 & abs(th) <= deg2rad(2) & abs(xs(:,4)) <= 0.10;
    tCap = ts(find(inCap, 1));  if isempty(tCap), tCap = NaN; end
    tSet = NaN;  if all(settled(ts >= ts(end)-1)), tSet = ts(find(~settled, 1, 'last')); end
    fprintf('capture at %.2f s (trajectory ends at %.2f s), settled after %.2f s, max |p| = %.2f m (limit %.2f), max |F| = %.1f N\n', ...
            tCap, T, tSet, max(abs(xs(:,1))), xmax, max(abs(squeeze(double(out.u.Data)))));
end
