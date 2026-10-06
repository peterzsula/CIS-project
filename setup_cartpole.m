%% Part 2: cart-pole swing-up and upright stabilization
% Run the whole script from any working directory.
projectFolder = fileparts(mfilename('fullpath'));
run(fullfile(projectFolder,'initialize_cartpole.m'));

%% Plot the nominal swing-up
tNom = (0:N)*Ts;
figure('Name','iLQR nominal swing-up');
tiledlayout(5,1);
nexttile; plot(tNom,Xnom(1,:)); ylabel('p [m]'); grid on;
nexttile; plot(tNom,Xnom(2,:)); ylabel('v [m/s]'); grid on;
nexttile; plot(tNom,rad2deg(thetaNomWrapped)); ylabel('\theta [deg]'); grid on;
nexttile; plot(tNom,Xnom(4,:)); ylabel('\omega [rad/s]'); grid on;
nexttile; stairs(tGuess,Unom); ylabel('F [N]'); xlabel('t [s]'); grid on;

%% Run the nonlinear Simscape simulation
% The disturbance-force input is wired to a Constant block set to zero.
modelName = 'cartpole';
load_system(fullfile(projectFolder,[modelName '.slx']));
out = sim(modelName,'StopTime',num2str(T+8));

%% Report capture and stabilization from the logged nonlinear state
simState = squeeze(double(out.x.Data));
if size(simState,2) == 4
    % Timeseries convention: one row per sample.
elseif size(simState,1) == 4
    simState = simState.';
else
    error('Unexpected state log dimensions from the cartpole model.');
end
simTime = double(out.x.Time(:));
if size(simState,1) ~= numel(simTime)
    error('State sample count does not match the logged simulation time.');
end
thetaSimWrapped = atan2(sin(simState(:,3)),cos(simState(:,3)));
actualPeakTravel = max(abs(simState(:,1)));
actualCartWithinLimit = actualPeakTravel <= xmax;
fprintf('Measured peak cart travel %.3f m (target %.3f m).\n',actualPeakTravel,xmax);
if ~actualCartWithinLimit
    warning('CIS:CartTravelTarget', ...
        'Measured cart travel exceeds the configured target; further controller tuning is needed.');
end
inCapture = abs(simState(:,1)) <= captureP & ...
    abs(simState(:,2)) <= captureV & ...
    abs(thetaSimWrapped) <= captureTheta & ...
    abs(simState(:,4)) <= captureOmega;
firstCapture = find(inCapture,1,'first');
if isempty(firstCapture)
    warning('Simulation did not enter the selected capture zone.');
else
    fprintf('Simulation entered the capture zone at t = %.3f s.\n', ...
        simTime(firstCapture));
end

settled = abs(simState(:,1)) <= 0.05 & abs(simState(:,2)) <= 0.10 & ...
    abs(thetaSimWrapped) <= deg2rad(2) & abs(simState(:,4)) <= 0.10;
if all(settled(simTime >= simTime(end)-1))
    fprintf('Simulation stabilized near upright for its final second.\n');
else
    warning('Simulation did not remain near upright for its final second.');
end

%% Compare nominal and simulated trajectories
figure('Name','Cart-pole swing-up and stabilization');
tiledlayout(5,1);
nexttile; plot(tNom,Xnom(1,:),'--',simTime,simState(:,1)); ylabel('p [m]');
    legend('nominal','Simscape'); grid on;
nexttile; plot(tNom,Xnom(2,:),'--',simTime,simState(:,2)); ylabel('v [m/s]'); grid on;
nexttile; plot(tNom,rad2deg(thetaNomWrapped),'--',simTime,rad2deg(thetaSimWrapped));
    ylabel('\theta wrapped [deg]'); grid on;
nexttile; plot(tNom,Xnom(4,:),'--',simTime,simState(:,4)); ylabel('\omega [rad/s]'); grid on;
nexttile; stairs(tGuess,Unom,'--'); hold on;
    simForce = squeeze(double(out.u.Data));
    stairs(double(out.u.Time(:)),simForce(:));
    ylabel('F [N]'); xlabel('t [s]'); legend('nominal','Simscape'); grid on;
