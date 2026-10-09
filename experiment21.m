%% ============================================================
% EXPERIMENT 1 - 4-D EMPIRICAL LQR CAPTURE ZONE
%
% Serial Simulink version
%
% Four initial conditions are varied:
%   theta0 = initial angle [deg]
%   omega0 = initial angular velocity [rad/s]
%   p0     = initial cart position [m]
%   v0     = initial cart velocity [m/s]
%
% SUCCESS CRITERION:
%   |p|     < 0.15 m
%   |v|     < 0.30 m/s
%   |theta| < 2 deg
%   |omega| < 0.40 rad/s
%
% All four conditions must remain satisfied continuously
% for at least 2 seconds.
%
% Each simulation runs for 10 seconds.
%
% Total simulations:
%   19 x 7 x 9 x 7 = 8379
%
% ============================================================

clear;
close all;
clc;


%% ============================================================
% Project folder
%% ============================================================

projectFolder = fileparts(mfilename('fullpath'));
addpath(projectFolder);


%% ============================================================
% Physical / model parameters
%% ============================================================

param.m  = 1.0;
param.M  = 1.0;
param.L  = 0.5;
param.bc = 0.1;
param.bp = 0.05;

g = 9.81;

model.M   = param.M;
model.m   = param.m;
model.ell = param.L/2;
model.bc  = param.bc;
model.bp  = param.bp;
model.g   = g;


%% ============================================================
% Simulation parameters
%% ============================================================

Ts   = 0.01;
Fmax = 20;
xmax = 1.0;

Tsim    = 10;
Tstable = 2.0;


%% ============================================================
% Stabilization bounds
%% ============================================================

p_limit     = 0.15;
v_limit     = 0.30;
theta_limit = deg2rad(2);
omega_limit = 0.40;


%% ============================================================
% 4-D INITIAL CONDITION SWEEP
%% ============================================================

theta0_list = -35:10:35;
omega0_list = -2:0.5:2;
p0_list     = -1:0.5:1;
v0_list     = -3:0.5:3;

nTheta = length(theta0_list);
nOmega = length(omega0_list);
nP     = length(p0_list);
nV     = length(v0_list);

total = nTheta*nOmega*nP*nV;


fprintf('\n============================================\n');
fprintf('4-D LQR CAPTURE-ZONE EXPERIMENT\n');
fprintf('============================================\n');

fprintf('Simulation time       : %.1f s\n',Tsim);
fprintf('Required stable time  : %.1f s\n',Tstable);

fprintf('\nStabilization limits:\n');
fprintf('Position              : +/- %.2f m\n',p_limit);
fprintf('Velocity              : +/- %.2f m/s\n',v_limit);
fprintf('Angle                 : +/- %.1f deg\n', ...
    rad2deg(theta_limit));
fprintf('Angular velocity      : +/- %.2f rad/s\n',omega_limit);

fprintf('\nInitial-condition ranges:\n');

fprintf('Theta range           : %.1f to %.1f deg\n', ...
    min(theta0_list),max(theta0_list));

fprintf('Omega range           : %.2f to %.2f rad/s\n', ...
    min(omega0_list),max(omega0_list));

fprintf('Position range        : %.2f to %.2f m\n', ...
    min(p0_list),max(p0_list));

fprintf('Velocity range        : %.2f to %.2f m/s\n', ...
    min(v0_list),max(v0_list));

fprintf('\nNumber of simulations : %d\n',total);
fprintf('============================================\n\n');


%% ============================================================
% Continuous-time linearization about upright theta = 0
%% ============================================================

M   = model.M;
m   = model.m;
ell = model.ell;

J = m*ell^2;

Delta = (M+m)*J - (m*ell)^2;


A = [ ...
    0, 1, 0, 0;

    0, ...
    -J*model.bc/Delta, ...
    m^2*g*ell^2/Delta, ...
    -m*ell*model.bp/Delta;

    0, 0, 0, 1;

    0, ...
    -m*ell*model.bc/Delta, ...
    (M+m)*m*g*ell/Delta, ...
    -(M+m)*model.bp/Delta];


B = [ ...
    0;
    J/Delta;
    0;
    m*ell/Delta];


%% ============================================================
% Discrete-time model
%% ============================================================

sysd = c2d( ...
    ss(A,B,eye(4),zeros(4,1)), ...
    Ts, ...
    'zoh');

Ad = sysd.A;
Bd = sysd.B;


%% ============================================================
% LQR weighting
%% ============================================================

vScale     = 2.0;
thetaScale = deg2rad(20);
omegaScale = 2.0;

Q_lqr = diag([ ...
    1/xmax^2, ...
    1/vScale^2, ...
    1/thetaScale^2, ...
    1/omegaScale^2]);

R_lqr = 1/Fmax^2;


%% ============================================================
% Discrete LQR
%% ============================================================

[K_lqr,~,lqrPoles] = dlqr( ...
    Ad, ...
    Bd, ...
    Q_lqr, ...
    R_lqr);

fprintf('Discrete LQR gain:\n');
disp(K_lqr);

fprintf('Maximum closed-loop pole magnitude: %.5f\n\n', ...
    max(abs(lqrPoles)));


%% ============================================================
% Capture parameters required by Simulink
%% ============================================================

captureP     = 0.60;
captureV     = 1.50;
captureTheta = deg2rad(20);
captureOmega = 0.75;

capture = [ ...
    captureP, ...
    captureV, ...
    captureTheta, ...
    captureOmega];


%% ============================================================
% Nominal trajectory variables required by Simulink
%% ============================================================

N = ceil(Tsim/Ts);

Knom = zeros(1,4,N);

for k = 1:N
    Knom(1,:,k) = K_lqr;
end

Unom = zeros(1,N);

% Upright equilibrium reference
Xnom = zeros(4,N+1);


%% ============================================================
% Load Simulink model
%% ============================================================

modelName = 'cartpole';

load_system(modelName);
set_param(modelName, 'Open', 'off');

%% ============================================================
% Preallocate results
%
% Columns:
%
% 1 = theta0 [deg]
% 2 = omega0 [rad/s]
% 3 = p0 [m]
% 4 = v0 [m/s]
% 5 = success
% 6 = settling time [s]
%% ============================================================

results = nan(total,6);


%% ============================================================
% SERIAL 4-D SWEEP
%% ============================================================

fprintf('============================================\n');
fprintf('STARTING SERIAL SIMULATIONS\n');
fprintf('============================================\n\n');

simulationCounter = 0;

tic;


for iTheta = 1:nTheta

    for iOmega = 1:nOmega

        for iP = 1:nP

            for iV = 1:nV

                %% ------------------------------------------------
                % Current initial conditions
                %% ------------------------------------------------

                th0 = theta0_list(iTheta);
                om0 = omega0_list(iOmega);
                p0  = p0_list(iP);
                v0  = v0_list(iV);


                %% ------------------------------------------------
                % Update simulation parameters
                %% ------------------------------------------------

                param.p0     = p0;
                param.v0     = v0;
                param.theta0 = th0;
                param.omega0 = om0;


                %% ------------------------------------------------
                % Initial state
                %
                % x = [p; v; theta; omega]
                %
                % LQR angle is in radians.
                %% ------------------------------------------------

                x0 = [ ...
                    p0;
                    v0;
                    deg2rad(th0);
                    om0];


                %% ------------------------------------------------
                % Upright nominal reference
                %% ------------------------------------------------

                Xnom = zeros(4,N+1);


                %% ------------------------------------------------
                % Run Simulink simulation
                %% ------------------------------------------------

                simulationCounter = simulationCounter + 1;


                fprintf( ...
                    'Simulation %5d / %5d  |  theta=%6.1f deg  omega=%5.2f  p=%5.2f  v=%5.2f\n', ...
                    simulationCounter, ...
                    total, ...
                    th0, ...
                    om0, ...
                    p0, ...
                    v0);

                try

                    simOut = sim( ...
                        modelName, ...
                        'StopTime',num2str(Tsim), ...
                        'SimulationMode','accelerator');


                    %% ------------------------------------------------
                    % Extract state
                    %% ------------------------------------------------

                    simState = squeeze( ...
                        double(simOut.x.Data));


                    %% ------------------------------------------------
                    % Check state dimensions
                    %% ------------------------------------------------

                    if size(simState,2) == 4

                        % Already [time x state]

                    elseif size(simState,1) == 4

                        % Convert [state x time]

                        simState = simState.';

                    else

                        error( ...
                            'Unexpected state log dimensions.');

                    end


                    %% ------------------------------------------------
                    % Extract time
                    %% ------------------------------------------------

                    if isprop(simOut.x,'Time')

                        time = double( ...
                            simOut.x.Time);

                    else

                        time = linspace( ...
                            0, ...
                            Tsim, ...
                            size(simState,1)).';

                    end

                    time = time(:);


                    %% ------------------------------------------------
                    % Extract states
                    %% ------------------------------------------------

                    p     = simState(:,1);
                    v     = simState(:,2);
                    theta = simState(:,3);
                    omega = simState(:,4);


                    %% ------------------------------------------------
                    % Wrap angle to [-pi,pi]
                    %% ------------------------------------------------

                    theta = atan2( ...
                        sin(theta), ...
                        cos(theta));


                    %% ------------------------------------------------
                    % Check stabilization bounds
                    %% ------------------------------------------------

                    insideBounds = ...
                        abs(p)     < p_limit & ...
                        abs(v)     < v_limit & ...
                        abs(theta) < theta_limit & ...
                        abs(omega) < omega_limit;


                    %% ------------------------------------------------
                    % Search for continuous stable interval
                    %% ------------------------------------------------

                    success = false;
                    settlingTime = NaN;


                    for k = 1:length(time)

                        % Need complete Tstable interval remaining

                        if Tsim - time(k) < Tstable
                            break;
                        end


                        % Find samples inside required interval

                        idx_window = ...
                            time >= time(k) & ...
                            time <= time(k) + Tstable;


                        % Check all four conditions

                        if all(insideBounds(idx_window))

                            success = true;

                            settlingTime = time(k);

                            break;

                        end

                    end


                catch ME

                    fprintf( ...
                        '\nWARNING: Simulation failed.\n');

                    fprintf('%s\n\n',ME.message);

                    success = false;
                    settlingTime = NaN;

                end


                %% ------------------------------------------------
                % Store result
                %% ------------------------------------------------

                results(simulationCounter,:) = [ ...
                    th0, ...
                    om0, ...
                    p0, ...
                    v0, ...
                    double(success), ...
                    settlingTime];


            end
        end
    end


    %% ============================================================
    % Progress after each theta slice
    %% ============================================================

    elapsedTime = toc;

    completed = simulationCounter;

    averageTime = elapsedTime/completed;

    remainingTime = ...
        averageTime*(total-completed);

    fprintf('\n--------------------------------------------\n');

    fprintf('Completed theta slice %d / %d\n', ...
        iTheta,nTheta);

    fprintf('Completed simulations : %d / %d\n', ...
        completed,total);

    fprintf('Elapsed time          : %.1f min\n', ...
        elapsedTime/60);

    fprintf('Estimated remaining   : %.1f min\n', ...
        remainingTime/60);

    fprintf('--------------------------------------------\n\n');

end


totalElapsed = toc;


%% ============================================================
% SUMMARY
%% ============================================================

success_rate = ...
    mean(results(:,5))*100;


successfulTimes = ...
    results(results(:,5)==1,6);


fprintf('\n============================================\n');
fprintf('EXPERIMENT FINISHED\n');
fprintf('============================================\n');

fprintf('Total simulations : %d\n', ...
    size(results,1));

fprintf('Successful        : %d\n', ...
    sum(results(:,5)==1));

fprintf('Failed            : %d\n', ...
    sum(results(:,5)==0));

fprintf('Success rate      : %.1f%%\n', ...
    success_rate);

fprintf('Total wall time   : %.2f min\n', ...
    totalElapsed/60);


if ~isempty(successfulTimes)

    fprintf('Mean settling time: %.3f s\n', ...
        mean(successfulTimes));

    fprintf('Maximum settling time: %.3f s\n', ...
        max(successfulTimes));

end

fprintf('============================================\n');


%% ============================================================
% SAVE COMPLETE 4-D RESULTS
%% ============================================================

save( ...
    'capture_zone_4D2_results.mat', ...
    'results', ...
    'theta0_list', ...
    'omega0_list', ...
    'p0_list', ...
    'v0_list');


