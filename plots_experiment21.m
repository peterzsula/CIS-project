%% ============================================================
% EXPERIMENT 1 - CAPTURE ZONE RESULT ANALYSIS
%
% Loads the previously generated 4-D sweep results.
% Does NOT run any Simulink simulations.
%
% Required file:
%   capture_zone_4D_results.mat
%
% Results columns:
%   1 = theta0 [deg]
%   2 = omega0 [rad/s]
%   3 = p0 [m]
%   4 = v0 [m/s]
%   5 = success
%   6 = settling time [s]
%
% ============================================================

clear;
close all;
clc;

%% ============================================================
% Load results
% ============================================================

fprintf('\n============================================\n');
fprintf('CAPTURE-ZONE RESULT ANALYSIS\n');
fprintf('============================================\n\n');

load('capture_zone_4D_results.mat');

fprintf('Loaded %d simulation results.\n', size(results,1));

%% ============================================================
% Extract columns
% ============================================================

theta0 = results(:,1);
omega0 = results(:,2);
p0     = results(:,3);
v0     = results(:,4);

success = results(:,5);
settlingTime = results(:,6);

%% ============================================================
% Basic statistics
% ============================================================

totalSimulations = length(success);

numSuccess = sum(success == 1);
numFailure = sum(success == 0);

successRate = 100*numSuccess/totalSimulations;

successfulTimes = settlingTime(success == 1);

fprintf('\n--------------------------------------------\n');
fprintf('OVERALL RESULTS\n');
fprintf('--------------------------------------------\n');

fprintf('Total simulations : %d\n', totalSimulations);
fprintf('Successful        : %d\n', numSuccess);
fprintf('Failed            : %d\n', numFailure);
fprintf('Success rate      : %.2f %%\n', successRate);

if ~isempty(successfulTimes)

    fprintf('Mean settling time: %.3f s\n', ...
        mean(successfulTimes));

    fprintf('Minimum settling  : %.3f s\n', ...
        min(successfulTimes));

    fprintf('Maximum settling  : %.3f s\n', ...
        max(successfulTimes));

    fprintf('Median settling   : %.3f s\n', ...
        median(successfulTimes));

end

fprintf('--------------------------------------------\n\n');


%% ============================================================
% 1. SUCCESS / FAILURE PIE CHART
% ============================================================

figure('Name','Overall Success Rate');

pie( ...
    [numSuccess numFailure], ...
    {'Success','Failure'});

title(sprintf( ...
    'Overall Capture-Zone Success Rate = %.1f%%', ...
    successRate));


%% ============================================================
% 2. THETA vs OMEGA
%    p0 = 0, v0 = 0
% ============================================================

theta_vals = unique(theta0);
omega_vals = unique(omega0);

p_fixed = 0;
v_fixed = 0;

success_map = nan( ...
    length(theta_vals), ...
    length(omega_vals));

for i = 1:length(theta_vals)

    for j = 1:length(omega_vals)

        idx = ...
            theta0 == theta_vals(i) & ...
            omega0 == omega_vals(j) & ...
            p0 == p_fixed & ...
            v0 == v_fixed;

        if any(idx)
            success_map(i,j) = ...
                results(find(idx,1),5);
        end

    end

end

figure('Name','Capture Zone - Theta vs Omega');

imagesc( ...
    omega_vals, ...
    theta_vals, ...
    success_map);

set(gca,'YDir','normal');

xlabel('Initial Angular Velocity \omega_0 [rad/s]');
ylabel('Initial Angle \theta_0 [deg]');

title(sprintf( ...
    '\\theta_0 vs \\omega_0  (p_0 = %.1f m, v_0 = %.1f m/s)', ...
    p_fixed, ...
    v_fixed));

colormap([ ...
    1 0.3 0.3;
    0.3 0.8 0.3]);

cb = colorbar;
cb.Ticks = [0 1];
cb.TickLabels = {'Failure','Success'};

clim([0 1]);

xticks(omega_vals);
yticks(theta_vals);

grid on;


%% ============================================================
% 3. THETA vs CART POSITION
%    omega0 = 0, v0 = 0
% ============================================================

p_vals = unique(p0);

omega_fixed = 0;
v_fixed = 0;

success_map = nan( ...
    length(theta_vals), ...
    length(p_vals));

for i = 1:length(theta_vals)

    for j = 1:length(p_vals)

        idx = ...
            theta0 == theta_vals(i) & ...
            omega0 == omega_fixed & ...
            p0 == p_vals(j) & ...
            v0 == v_fixed;

        if any(idx)
            success_map(i,j) = ...
                results(find(idx,1),5);
        end

    end

end

figure('Name','Capture Zone - Theta vs Position');

imagesc( ...
    p_vals, ...
    theta_vals, ...
    success_map);

set(gca,'YDir','normal');

xlabel('Initial Cart Position p_0 [m]');
ylabel('Initial Angle \theta_0 [deg]');

title(sprintf( ...
    '\\theta_0 vs p_0  (\\omega_0 = %.1f rad/s, v_0 = %.1f m/s)', ...
    omega_fixed, ...
    v_fixed));

cb = colorbar;
cb.Ticks = [0 1];
cb.TickLabels = {'Failure','Success'};

clim([0 1]);

xticks(p_vals);
yticks(theta_vals);

grid on;


%% ============================================================
% 4. THETA vs CART VELOCITY
%    omega0 = 0, p0 = 0
% ============================================================

v_vals = unique(v0);

omega_fixed = 0;
p_fixed = 0;

success_map = nan( ...
    length(theta_vals), ...
    length(v_vals));

for i = 1:length(theta_vals)

    for j = 1:length(v_vals)

        idx = ...
            theta0 == theta_vals(i) & ...
            omega0 == omega_fixed & ...
            p0 == p_fixed & ...
            v0 == v_vals(j);

        if any(idx)
            success_map(i,j) = ...
                results(find(idx,1),5);
        end

    end

end

figure('Name','Capture Zone - Theta vs Velocity');

imagesc( ...
    v_vals, ...
    theta_vals, ...
    success_map);

set(gca,'YDir','normal');

xlabel('Initial Cart Velocity v_0 [m/s]');
ylabel('Initial Angle \theta_0 [deg]');

title(sprintf( ...
    '\\theta_0 vs v_0  (\\omega_0 = %.1f rad/s, p_0 = %.1f m)', ...
    omega_fixed, ...
    p_fixed));

cb = colorbar;
cb.Ticks = [0 1];
cb.TickLabels = {'Failure','Success'};

clim([0 1]);

xticks(v_vals);
yticks(theta_vals);

grid on;


%% ============================================================
% 5. OMEGA vs CART POSITION
%    theta0 = 0, v0 = 0
% ============================================================

theta_fixed = 0;
v_fixed = 0;

success_map = nan( ...
    length(omega_vals), ...
    length(p_vals));

for i = 1:length(omega_vals)

    for j = 1:length(p_vals)

        idx = ...
            theta0 == theta_fixed & ...
            omega0 == omega_vals(i) & ...
            p0 == p_vals(j) & ...
            v0 == v_fixed;

        if any(idx)
            success_map(i,j) = ...
                results(find(idx,1),5);
        end

    end

end

figure('Name','Capture Zone - Omega vs Position');

imagesc( ...
    p_vals, ...
    omega_vals, ...
    success_map);

set(gca,'YDir','normal');

xlabel('Initial Cart Position p_0 [m]');
ylabel('Initial Angular Velocity \omega_0 [rad/s]');

title(sprintf( ...
    '\\omega_0 vs p_0  (\\theta_0 = %.1f deg, v_0 = %.1f m/s)', ...
    theta_fixed, ...
    v_fixed));

cb = colorbar;
cb.Ticks = [0 1];
cb.TickLabels = {'Failure','Success'};

clim([0 1]);

xticks(p_vals);
yticks(omega_vals);

grid on;


%% ============================================================
% 6. OMEGA vs CART VELOCITY
%    theta0 = 0, p0 = 0
% ============================================================

theta_fixed = 0;
p_fixed = 0;

success_map = nan( ...
    length(omega_vals), ...
    length(v_vals));

for i = 1:length(omega_vals)

    for j = 1:length(v_vals)

        idx = ...
            theta0 == theta_fixed & ...
            omega0 == omega_vals(i) & ...
            p0 == p_fixed & ...
            v0 == v_vals(j);

        if any(idx)
            success_map(i,j) = ...
                results(find(idx,1),5);
        end

    end

end

figure('Name','Capture Zone - Omega vs Velocity');

imagesc( ...
    v_vals, ...
    omega_vals, ...
    success_map);

set(gca,'YDir','normal');

xlabel('Initial Cart Velocity v_0 [m/s]');
ylabel('Initial Angular Velocity \omega_0 [rad/s]');

title(sprintf( ...
    '\\omega_0 vs v_0  (\\theta_0 = %.1f deg, p_0 = %.1f m)', ...
    theta_fixed, ...
    p_fixed));

cb = colorbar;
cb.Ticks = [0 1];
cb.TickLabels = {'Failure','Success'};

clim([0 1]);

xticks(v_vals);
yticks(omega_vals);

grid on;


%% ============================================================
% 7. CART POSITION vs CART VELOCITY
%    theta0 = 0, omega0 = 0
% ============================================================

theta_fixed = 0;
omega_fixed = 0;

success_map = nan( ...
    length(p_vals), ...
    length(v_vals));

for i = 1:length(p_vals)

    for j = 1:length(v_vals)

        idx = ...
            theta0 == theta_fixed & ...
            omega0 == omega_fixed & ...
            p0 == p_vals(i) & ...
            v0 == v_vals(j);

        if any(idx)
            success_map(i,j) = ...
                results(find(idx,1),5);
        end

    end

end

figure('Name','Capture Zone - Position vs Velocity');

imagesc( ...
    v_vals, ...
    p_vals, ...
    success_map);

set(gca,'YDir','normal');

xlabel('Initial Cart Velocity v_0 [m/s]');
ylabel('Initial Cart Position p_0 [m]');

title(sprintf( ...
    'p_0 vs v_0  (\\theta_0 = %.1f deg, \\omega_0 = %.1f rad/s)', ...
    theta_fixed, ...
    omega_fixed));

cb = colorbar;
cb.Ticks = [0 1];
cb.TickLabels = {'Failure','Success'};

clim([0 1]);

xticks(v_vals);
yticks(p_vals);

grid on;


%% ============================================================
% 8. SETTLING-TIME HEATMAP
%    THETA vs OMEGA
%    p0 = 0, v0 = 0
% ============================================================

settling_map = nan( ...
    length(theta_vals), ...
    length(omega_vals));

for i = 1:length(theta_vals)

    for j = 1:length(omega_vals)

        idx = ...
            theta0 == theta_vals(i) & ...
            omega0 == omega_vals(j) & ...
            p0 == 0 & ...
            v0 == 0;

        if any(idx)

            row = results(find(idx,1),:);

            if row(5) == 1
                settling_map(i,j) = row(6);
            end

        end

    end

end

figure('Name','Settling Time - Theta vs Omega');

imagesc( ...
    omega_vals, ...
    theta_vals, ...
    settling_map);

set(gca,'YDir','normal');

xlabel('Initial Angular Velocity \omega_0 [rad/s]');
ylabel('Initial Angle \theta_0 [deg]');

title('\bfSettling Time for Successful Cases');

cb = colorbar;
cb.Label.String = 'Settling Time [s]';

xticks(omega_vals);
yticks(theta_vals);

grid on;


%% ============================================================
% 9. SUCCESS RATE VS INITIAL ANGLE
% ============================================================

angle_success_rate = zeros(size(theta_vals));

for i = 1:length(theta_vals)

    idx = theta0 == theta_vals(i);

    angle_success_rate(i) = ...
        100*mean(success(idx));

end

figure('Name','Success Rate vs Angle');

plot( ...
    theta_vals, ...
    angle_success_rate, ...
    '-o', ...
    'LineWidth',1.5);

xlabel('Initial Angle \theta_0 [deg]');
ylabel('Success Rate [%]');

title('Capture Probability vs Initial Angle');

ylim([0 100]);
grid on;


%% ============================================================
% 10. SUCCESS RATE VS INITIAL ANGULAR VELOCITY
% ============================================================

omega_success_rate = zeros(size(omega_vals));

for i = 1:length(omega_vals)

    idx = omega0 == omega_vals(i);

    omega_success_rate(i) = ...
        100*mean(success(idx));

end

figure('Name','Success Rate vs Angular Velocity');

plot( ...
    omega_vals, ...
    omega_success_rate, ...
    '-o', ...
    'LineWidth',1.5);

xlabel('Initial Angular Velocity \omega_0 [rad/s]');
ylabel('Success Rate [%]');

title('Capture Probability vs Initial Angular Velocity');

ylim([0 100]);
grid on;


%% ============================================================
% 11. SUCCESS RATE VS CART POSITION
% ============================================================

p_success_rate = zeros(size(p_vals));

for i = 1:length(p_vals)

    idx = p0 == p_vals(i);

    p_success_rate(i) = ...
        100*mean(success(idx));

end

figure('Name','Success Rate vs Position');

plot( ...
    p_vals, ...
    p_success_rate, ...
    '-o', ...
    'LineWidth',1.5);

xlabel('Initial Cart Position p_0 [m]');
ylabel('Success Rate [%]');

title('Capture Probability vs Initial Cart Position');

ylim([0 100]);
grid on;


%% ============================================================
% 12. SUCCESS RATE VS CART VELOCITY
% ============================================================

v_success_rate = zeros(size(v_vals));

for i = 1:length(v_vals)

    idx = v0 == v_vals(i);

    v_success_rate(i) = ...
        100*mean(success(idx));

end

figure('Name','Success Rate vs Cart Velocity');

plot( ...
    v_vals, ...
    v_success_rate, ...
    '-o', ...
    'LineWidth',1.5);

xlabel('Initial Cart Velocity v_0 [m/s]');
ylabel('Success Rate [%]');

title('Capture Probability vs Initial Cart Velocity');

ylim([0 100]);
grid on;


%% ============================================================
% 13. 3-D SCATTER OF SUCCESSFUL INITIAL CONDITIONS
% ============================================================

successful_idx = success == 1;

figure('Name','Successful Initial Conditions - 3D');

scatter3( ...
    theta0(successful_idx), ...
    omega0(successful_idx), ...
    p0(successful_idx), ...
    40, ...
    v0(successful_idx), ...
    'filled');

xlabel('Initial Angle \theta_0 [deg]');
ylabel('Initial Angular Velocity \omega_0 [rad/s]');
zlabel('Initial Position p_0 [m]');

title('Successful Initial Conditions');
cb = colorbar;
cb.Label.String = 'Initial Cart Velocity v_0 [m/s]';

grid on;
view(45,30);


%% ============================================================
% 14. 3-D SCATTER OF FAILED INITIAL CONDITIONS
% ============================================================

failed_idx = success == 0;

figure('Name','Failed Initial Conditions - 3D');

scatter3( ...
    theta0(failed_idx), ...
    omega0(failed_idx), ...
    p0(failed_idx), ...
    40, ...
    v0(failed_idx), ...
    'filled');

xlabel('Initial Angle \theta_0 [deg]');
ylabel('Initial Angular Velocity \omega_0 [rad/s]');
zlabel('Initial Position p_0 [m]');

title('Failed Initial Conditions');
cb = colorbar;
cb.Label.String = 'Initial Cart Velocity v_0 [m/s]';

grid on;
view(45,30);


%% ============================================================
% 15. SETTLING TIME DISTRIBUTION
% ============================================================

figure('Name','Settling Time Distribution');

histogram(successfulTimes,20);

xlabel('Settling Time [s]');
ylabel('Number of Successful Simulations');

title('Distribution of Settling Times');

grid on;


%% ============================================================
% 16. SAVE ANALYSIS DATA
% ============================================================

save( ...
    'capture_zone_analysis.mat', ...
    'successRate', ...
    'numSuccess', ...
    'numFailure', ...
    'successfulTimes', ...
    'angle_success_rate', ...
    'omega_success_rate', ...
    'p_success_rate', ...
    'v_success_rate');

fprintf('\n============================================\n');
fprintf('ANALYSIS COMPLETE\n');
fprintf('============================================\n');

fprintf('Generated plots:\n');
fprintf('  1. Overall success rate\n');
fprintf('  2. Theta vs Omega\n');
fprintf('  3. Theta vs Position\n');
fprintf('  4. Theta vs Velocity\n');
fprintf('  5. Omega vs Position\n');
fprintf('  6. Omega vs Velocity\n');
fprintf('  7. Position vs Velocity\n');
fprintf('  8. Settling-time heatmap\n');
fprintf('  9. Success rate vs Angle\n');
fprintf(' 10. Success rate vs Angular Velocity\n');
fprintf(' 11. Success rate vs Position\n');
fprintf(' 12. Success rate vs Cart Velocity\n');
fprintf(' 13. Successful initial conditions (3-D)\n');
fprintf(' 14. Failed initial conditions (3-D)\n');
fprintf(' 15. Settling-time distribution\n');

fprintf('\nAnalysis data saved to:\n');
fprintf('capture_zone_analysis.mat\n');

fprintf('============================================\n');

%% %% ============================================================
% 13. AUTOMATICALLY FIND 100% SUCCESS CAPTURE BOX
%
% Find the largest symmetric box around the upright equilibrium
% for which ALL tested initial conditions are successful:
%
% |theta0| <= thetaMax
% |omega0| <= omegaMax
% |p0|     <= pMax
% |v0|     <= vMax
%
% The search is performed only over the experimentally tested grid.
% ============================================================

fprintf('\n--------------------------------------------\n');
fprintf('AUTOMATIC CAPTURE-BOX SEARCH\n');
fprintf('--------------------------------------------\n');

% Possible symmetric limits from the tested grid
thetaLimits = unique(abs(theta0));
omegaLimits = unique(abs(omega0));
pLimits     = unique(abs(p0));
vLimits     = unique(abs(v0));

% Maximum investigated ranges
thetaRange = max(thetaLimits);
omegaRange = max(omegaLimits);
pRange     = max(pLimits);
vRange     = max(vLimits);

bestScore = -inf;
bestLimits = [0 0 0 0];
bestNumPoints = 0;

for i = 1:length(thetaLimits)

    for j = 1:length(omegaLimits)

        for k = 1:length(pLimits)

            for l = 1:length(vLimits)

                thetaMax = thetaLimits(i);
                omegaMax = omegaLimits(j);
                pMax     = pLimits(k);
                vMax     = vLimits(l);

                % Select all tested initial conditions inside
                % this candidate capture box
                idx = ...
                    abs(theta0) <= thetaMax & ...
                    abs(omega0) <= omegaMax & ...
                    abs(p0)     <= pMax & ...
                    abs(v0)     <= vMax;

                % Skip empty regions
                if ~any(idx)
                    continue;
                end

                % Require 100% success inside the box
                if all(success(idx) == 1)

                    % Normalised 4-D volume.
                    % This avoids comparing quantities with
                    % different physical units directly.
                    score = ...
                        (thetaMax/thetaRange) * ...
                        (omegaMax/omegaRange) * ...
                        (pMax/pRange) * ...
                        (vMax/vRange);

                    % Keep the largest successful box
                    if score > bestScore

                        bestScore = score;

                        bestLimits = [ ...
                            thetaMax, ...
                            omegaMax, ...
                            pMax, ...
                            vMax];

                        bestNumPoints = sum(idx);

                    end
                end

            end
        end
    end
end


% Extract final result
thetaCapture = bestLimits(1);
omegaCapture = bestLimits(2);
pCapture     = bestLimits(3);
vCapture     = bestLimits(4);


fprintf('\nLargest experimentally verified 100%%-success box:\n');
fprintf('--------------------------------------------\n');
fprintf('|theta0| <= %.1f deg\n', thetaCapture);
fprintf('|omega0| <= %.2f rad/s\n', omegaCapture);
fprintf('|p0|     <= %.2f m\n', pCapture);
fprintf('|v0|     <= %.2f m/s\n', vCapture);
fprintf('Successful points inside box: %d\n', bestNumPoints);
fprintf('Normalised 4-D box volume: %.4f\n', bestScore);
fprintf('--------------------------------------------\n');


%% ============================================================
% 13b. VERIFY THE SELECTED CAPTURE BOX
% ============================================================

captureIdx = ...
    abs(theta0) <= thetaCapture & ...
    abs(omega0) <= omegaCapture & ...
    abs(p0)     <= pCapture & ...
    abs(v0)     <= vCapture;

captureSuccessRate = ...
    100*mean(success(captureIdx));

captureNumPoints = sum(captureIdx);
captureNumFailures = sum(success(captureIdx) == 0);

fprintf('\nSelected capture box verification:\n');
fprintf('--------------------------------------------\n');
fprintf('Number of tested points : %d\n', captureNumPoints);
fprintf('Successful              : %d\n', ...
    sum(success(captureIdx) == 1));
fprintf('Failed                  : %d\n', captureNumFailures);
fprintf('Success rate             : %.2f %%\n', ...
    captureSuccessRate);
fprintf('--------------------------------------------\n');


%% ============================================================
% 13c. SAVE CAPTURE-BOX RESULT
% ============================================================

save( ...
    'capture_zone_analysis.mat', ...
    'successRate', ...
    'numSuccess', ...
    'numFailure', ...
    'successfulTimes', ...
    'angle_success_rate', ...
    'omega_success_rate', ...
    'p_success_rate', ...
    'v_success_rate', ...
    'thetaCapture', ...
    'omegaCapture', ...
    'pCapture', ...
    'vCapture', ...
    'captureSuccessRate', ...
    'captureNumPoints');
%% 

%% 13d. Marginal Average Success-Rate Heatmaps
%
% These heatmaps show the average success rate over the two dimensions
% that are not displayed.
%
% theta-omega heatmap:
%   average over cart position and cart velocity
%
% position-velocity heatmap:
%   average over pole angle and angular velocity

%% 13d.1 Pole angle vs angular velocity

theta_vals = unique(theta0);
omega_vals = unique(omega0);

thetaOmega_success_rate = NaN(length(omega_vals), length(theta_vals));

for i = 1:length(omega_vals)
    for j = 1:length(theta_vals)

        % Select all simulations with this theta/omega combination
        % and average over all positions and velocities
        idx = (theta0 == theta_vals(j)) & ...
              (omega0 == omega_vals(i));

        if any(idx)
            thetaOmega_success_rate(i,j) = ...
                100 * mean(success(idx));
        end

    end
end

figure;
imagesc(theta_vals, omega_vals, thetaOmega_success_rate);

set(gca, 'YDir', 'normal');

xlabel('Initial Pole Angle [deg]');
ylabel('Initial Angular Velocity [rad/s]');
title('Average Success Rate: Pole Angle vs Angular Velocity');

cb = colorbar;
ylabel(cb, 'Average Success Rate [%]');

clim([0 100]);

hold on;

% Capture-zone boundaries
xline(-thetaCapture, '--k', 'LineWidth', 1.5);
xline( thetaCapture, '--k', 'LineWidth', 1.5);
yline(-omegaCapture, '--k', 'LineWidth', 1.5);
yline( omegaCapture, '--k', 'LineWidth', 1.5);

hold off;

grid on;


%% 13d.2 Initial pole angle vs initial cart velocity

theta_vals = unique(theta0);
v_vals = unique(v0);

thetaV_success_rate = NaN(length(v_vals), length(theta_vals));

for i = 1:length(v_vals)
    for j = 1:length(theta_vals)

        % Select all simulations with this angle/velocity combination
        % and average over cart position and angular velocity
        idx = (theta0 == theta_vals(j)) & ...
              (v0 == v_vals(i));

        if any(idx)
            thetaV_success_rate(i,j) = ...
                100 * mean(success(idx));
        end

    end
end

figure;
imagesc(theta_vals, v_vals, thetaV_success_rate);

set(gca, 'YDir', 'normal');

xlabel('Initial Pole Angle [deg]');
ylabel('Initial Cart Velocity [m/s]');
title('Average Success Rate: Pole Angle vs Cart Velocity');

cb = colorbar;
ylabel(cb, 'Average Success Rate [%]');

clim([0 100]);

hold on;

% Capture-zone boundaries
xline(-thetaCapture, '--k', 'LineWidth', 1.5);
xline( thetaCapture, '--k', 'LineWidth', 1.5);
yline(-vCapture, '--k', 'LineWidth', 1.5);
yline( vCapture, '--k', 'LineWidth', 1.5);

hold off;

grid on;

%% 13d.3 Save heatmap data

save('capture_zone_analysis.mat', ...
    'thetaOmega_success_rate', ...
    'thetaV_success_rate', ...
    '-append');

fprintf('\nMarginal success-rate heatmaps generated successfully.\n');