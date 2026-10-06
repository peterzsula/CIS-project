%% Plant, model and controller/observer design for the actuated pendulum (Part 1)
% Defines everything that is the same in every experiment. It sets no experiment
% conditions and runs no simulation: each experiment script calls this first and
% then states its own conditions (initial state, state source, noise, limits).

projectFolder = fileparts(mfilename('fullpath'));
addpath(projectFolder);
clear cis_active_model;
p = struct();
m = struct();  % Part 2 uses m as a scalar; restore the Part 1 structure.

%% Physical parameters 
%%%%% These are the parameters used by the Simulink model and they
%%%%% correspond to the properties of the real physical system

p.m = 1.0;        % kg
p.l = 0.5;        % m, rod length (Simscape puts the point mass at p.l/2)
p.b = 0.05;       % N*m/(rad/s)
g = 9.81;       % m/s^2

%% Model parameters 
%%%%%%% These are the parameters you can use for your model. Currently,
%%%%%%% they are set equal to the values of the physical system but you can
%%%%%%% change and observe how your controller and observer behave under model
%%%%%%% uncertainty

m.m = p.m;        % kg
m.l = p.l/2;        % m, distance pivot -> point mass
m.b = p.b;       % N*m/(rad/s)
m.J = m.m*m.l^2;


%% Sampling
%%%%%% Controller works with measurements obtained at regular intervals.
%%%%%% The control inputs are in discrete time as well. They are then
%%%%%% applied to the system in zero-order-hold manner. 
%%%%%% Sampling period: every Ts seconds
Ts = 0.02; 

%% Actuator limits (nominal)
umin = -2.5;
umax = 2.5;

%% Linear model
%%%%%% Here you should set the matrices Ac and Bc which correspond to
%%%%%% the linearized system 
Ac = [0, 1; g/m.l, -m.b/m.J];
Bc = [0;1/m.J];
C = [1 0];
%% Discrete-time model
%%%%% Disretize the linear apprroximation; you can use MATLAB functions
%%%%% from control systtem toolbox such as c2d or compute matrix
%%%%% exponentials

sysd = c2d(ss(Ac, Bc, C, 0), Ts, 'zoh');
Ad = sysd.A;
Bd = sysd.B;

%% LQR
%%%%% Design LQR 
%% Q = [1,0;0,1];
%% R = 1;
th_max = deg2rad(180);  w_max = 2;
Q = diag([1/th_max^2, 1/w_max^2]);
R = 1/umax^2;
[K, S, P] = dlqr(Ad, Bd, Q, R);    % P: closed-loop poles, used for the observer

%% Observer


% 1) Observability check
Wo = [C; C*Ad];
assert(rank(Wo) == 2)              % observable

% 2) Choose observer poles, faster than the controller
p_slow = max(abs(P));              % slowest controller pole
k_obs = 5;                         % observer decays k_obs and 1.5*k_obs times faster
obs_poles = p_slow.^[k_obs, 1.5*k_obs];   % two distinct real poles

% 3) Gain via duality: place works on A - B K, so transpose
L = place(Ad', C', obs_poles)';    % L is 2x1

% Experiment scripts add their own initial state and noise conditions next.
cis_active_model = 'actuated_pendulum';
