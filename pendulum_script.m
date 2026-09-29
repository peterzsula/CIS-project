%% Physical parameters 
%%%%% These are the parameters used by the Simulink model and they
%%%%% correspond to the properties of the real physical system

p.m = 1.0;        % kg
p.l = 0.5;        % m, distance pivot -> point mass
p.b = 0.05;       % N*m/(rad/s)
g = 9.81;       % m/s^2

p.theta0 = 180;  %degrees
p.omega0 = 0;   %rad/s

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

%% Actuator limits 
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
R = 1/u_max^2;
K = dlqr(Ad,Bd,Q,R,0);

%% Observer


% 1) Observability check
Wo = [C; C*Ad];
rank(Wo)                           % must be 2

% 2) Choose observer poles, faster than the controller
[K, S, P] = dlqr(Ad, Bd, Q, R);    % (you already have this in the LQR section)
p_slow = max(abs(P));              % slowest controller pole
obs_poles = [p_slow^2, p_slow^3];  % two distinct real poles

% 3) Gain via duality: place works on A - B K, so transpose
L = place(Ad', C', obs_poles)';    % L is 2x1

eig(Ad - L*C)                      % check: should equal obs_poles

% 4) Initial guess of the state
xhat0 = [0; 0];                    % "I don't know where it starts"
% xhat0 = [deg2rad(p.theta0); p.omega0];   % perfect initial guess, for comparison
%% Simulate the actuated pendulum
%%% The following command runs the simulation.
%%% You should see an animation of the pendulum and out is a struct that
%%% contains the outputs from the simulation (for example, measured and
%%% estimated values, control input, or any other thing you add)

out=sim("actuated_pendulum")

%%%%%% 
%%% Add relevant plots etc. 


