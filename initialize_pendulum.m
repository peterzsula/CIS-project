%% Part 1 baseline initialization only (no simulation or plots)
% Experiments call pendulum_setup and then override their own conditions.
projectFolder = fileparts(mfilename('fullpath'));
run(fullfile(projectFolder,'pendulum_setup.m'));

%% Conditions of this run
p.theta0 = 180;        % deg, initial angle
p.omega0 = 0;          % deg/s, initial angular velocity
xhat0 = [0; 0];        % initial state estimate: "I don't know where it starts"
% xhat0 = [deg2rad(p.theta0); deg2rad(p.omega0)];   % perfect initial guess, for comparison
use_observer = true;   % controller uses the observer estimate (false: the measured state)
u_noise = 0.005;       % N*m, amplitude of the uniform disturbance on the plant input
y_noise = 0.005;       % rad, amplitude of the uniform noise on the angle measurement
% torque limits: nominal umin, umax from pendulum_setup

cis_active_model = 'actuated_pendulum';
