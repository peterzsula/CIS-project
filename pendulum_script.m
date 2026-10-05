%% Full control system: LQR on the observer estimate, with disturbances
pendulum_setup                     % plant, model, discretization, LQR, observer

%% Conditions of this run
p.theta0 = 180;        % deg, initial angle
p.omega0 = 0;          % deg/s, initial angular velocity
xhat0 = [0; 0];        % initial state estimate: "I don't know where it starts"
% xhat0 = [deg2rad(p.theta0); deg2rad(p.omega0)];   % perfect initial guess, for comparison
use_observer = true;   % controller uses the observer estimate (false: the measured state)
u_noise = 0.005;       % N*m, amplitude of the uniform disturbance on the plant input
y_noise = 0.005;       % rad, amplitude of the uniform noise on the angle measurement
% torque limits: nominal umin, umax from pendulum_setup

%% Simulate the actuated pendulum
%%% The following command runs the simulation.
%%% You should see an animation of the pendulum and out is a struct that
%%% contains the outputs from the simulation (for example, measured and
%%% estimated values, control input, or any other thing you add)

out = sim("actuated_pendulum", 'StopTime', '10');

%% Result
t  = out.theta_sim.Time;  th  = rad2deg(squeeze(out.theta_sim.Data));
thh = rad2deg(squeeze(out.theta_hat.Data));
tu = out.u_sim.Time;      u   = squeeze(out.u_sim.Data);
t_settle = max([0; t(abs(th) > 1)]);
fprintf('settles (|theta| < 1 deg) after %.2f s; afterwards std theta = %.3f deg, std u = %.3f N m\n', ...
        t_settle, std(th(t > t_settle + 1)), std(u(tu > t_settle + 1)));

figure('Position', [100 100 700 500]);
subplot(2,1,1); plot(t, th, t, thh, '--'); grid on; xlim([0 5]);
ylabel('\theta [deg]'); legend('true', 'estimate');
subplot(2,1,2); stairs(tu, u); grid on; xlim([0 5]); ylabel('u [N m]'); xlabel('t [s]');
exportgraphics(gcf, 'full_system.png', 'Resolution', 200);
