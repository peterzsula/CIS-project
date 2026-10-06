%% Part 2 initialization: physical parameters, LQR and iLQR (no simulation)
% Run setup_cartpole.m for simulation and plots.
% Model initialization callbacks also use this file after workspace clearing.
projectFolder = fileparts(mfilename('fullpath'));
addpath(projectFolder);
clear cis_active_model;

%% Physical and model parameters
% Simscape uses degrees for its Revolute Joint target; the optimizer uses
% radians and the convention theta = 0 upright, theta = +/-pi hanging down.
param = struct();
model = struct();
param.m  = 1.0;       % Pendulum mass [kg]
param.M  = 1.0;       % Cart mass [kg]
param.L  = 0.5;       % Full pole length [m]
param.bc = 0.1;       % Cart damping [N*s/m]
param.bp = 0.05;      % Pivot damping [N*m*s/rad]
g = 9.81;

param.p0     = 0;
param.v0     = 0;
param.theta0 = 180;   % down in the Simscape model
param.omega0 = 0;
x0 = [param.p0; param.v0; deg2rad(param.theta0); param.omega0];

model.M   = param.M;
model.m   = param.m;
model.ell = param.L/2;  % Pivot to point-mass distance [m]
model.bc  = param.bc;
model.bp  = param.bp;
model.g   = g;

%% Sampling, force, and cart travel limits
Ts   = 0.01;
Fmax = 20;
xmax = 1.0;

%% Continuous-time linearization about upright (theta = 0)
M   = model.M;
m   = model.m;
ell = model.ell;
J = m*ell^2;
Delta = (M+m)*J - (m*ell)^2;

A = [0, 1, 0, 0;
     0, -J*model.bc/Delta, m^2*g*ell^2/Delta, -m*ell*model.bp/Delta;
     0, 0, 0, 1;
     0, -m*ell*model.bc/Delta, (M+m)*m*g*ell/Delta, -(M+m)*model.bp/Delta];
B = [0; J/Delta; 0; m*ell/Delta];

%% Discrete-time LQR design
sysd = c2d(ss(A,B,eye(4),zeros(4,1)),Ts,'zoh');
Ad = sysd.A;
Bd = sysd.B;

% Bryson-style weights: normalize the states by representative acceptable
% magnitudes, and normalize force by the actuator limit.
vScale = 2.0;
thetaScale = deg2rad(20);
omegaScale = 2.0;
Q_lqr = diag([1/xmax^2, 1/vScale^2, 1/thetaScale^2, 1/omegaScale^2]);
R_lqr = 1/Fmax^2;
[K_lqr,~,lqrPoles] = dlqr(Ad,Bd,Q_lqr,R_lqr);
if max(abs(lqrPoles)) >= 1
    error('The discrete-time upright LQR is not stabilizing.');
end

fprintf('Discrete LQR gain K_lqr = ');
disp(K_lqr);
fprintf('Maximum closed-loop pole magnitude: %.5f\n',max(abs(lqrPoles)));

%% iLQR swing-up problem
T = 4.18;
N = round(T/Ts);
T = N*Ts;
tGuess = (0:N-1)*Ts;

% Seed with a bounded 1.3 Hz swing-up waveform whose open-loop rollout
% approaches the selected capture region for these physical parameters.
swingupFrequency = 1.30;
U0 = Fmax*sin(2*pi*swingupFrequency*tGuess + 3*pi/2);

Q = diag([1/xmax^2, 1/vScale^2, 1/thetaScale^2, 1/omegaScale^2]);
R = R_lqr;
% Give the terminal velocity and cart state stronger weight so the LQR can
% take over with the chosen force limit.
Qf = diag([200, 20, 30/deg2rad(10)^2, 100]);

[Xnom,Unom,Knom,costHistory] = ilqr(x0,U0,model,Ts,Q,R,Qf,Fmax);

%% Check the optimized terminal state and path limits
captureP = 0.60;
captureV = 1.50;
captureTheta = deg2rad(20);
captureOmega = 0.75;
capture = [captureP, captureV, captureTheta, captureOmega];  % passed to the Simulink controller
thetaNomWrapped = atan2(sin(Xnom(3,:)),cos(Xnom(3,:)));
terminalInCapture = abs(Xnom(1,end)) <= captureP && ...
    abs(Xnom(2,end)) <= captureV && ...
    abs(thetaNomWrapped(end)) <= captureTheta && ...
    abs(Xnom(4,end)) <= captureOmega;
cartWithinLimit = max(abs(Xnom(1,:))) <= xmax;
forceWithinLimit = all(abs(Unom) <= Fmax + 1e-9);

fprintf('\niLQR accepted %d iterations; final cost %.6g.\n', ...
    numel(costHistory)-1,costHistory(end));
fprintf('Nominal peak cart travel %.3f m (limit %.3f m).\n', ...
    max(abs(Xnom(1,:))),xmax);
if terminalInCapture
    fprintf('Nominal terminal capture check: passed.\n');
else
    fprintf('Nominal terminal capture check: failed.\n');
end
if ~terminalInCapture
    warning(['The iLQR terminal state is outside the selected capture zone. ' ...
        'Change T, Q/Qf/R, or the initial control guess before relying on ' ...
        'the swing-up.']);
end
if ~cartWithinLimit
    warning('The nominal trajectory exceeds the %.3f m cart-travel limit.',xmax);
end
if ~forceWithinLimit
    error('The iLQR control sequence exceeds the actuator force limit.');
end


% Mark ready only after every parameter and trajectory has been computed.
cis_active_model = 'cartpole';
