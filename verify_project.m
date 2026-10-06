function report = verify_project(includeExperiments)
% Verify startup recovery, default simulation behavior and optional experiments.
% Usage: verify_project or verify_project(true).
% Results and a detailed diary are written under output/validation/.
if nargin < 1
    includeExperiments = false;
end
projectFolder = fileparts(mfilename('fullpath'));
addpath(projectFolder);
outputFolder = fullfile(projectFolder,'output','validation');
if ~isfolder(outputFolder), mkdir(outputFolder); end
logPath = fullfile(outputFolder,'latest.log');
fid = fopen(logPath,'w'); fclose(fid);
diary(logPath);
diaryCleanup = onCleanup(@() diary('off'));
oldVisible = get(groot,'DefaultFigureVisible');
set(groot,'DefaultFigureVisible','off');
figureCleanup = onCleanup(@() set(groot,'DefaultFigureVisible',oldVisible));
report = struct('matlabVersion',version,'checks',struct('name',{},'passed',{},'detail',{}));
fprintf('CIS validation: MATLAB %s\n',version);

check('Saved model callbacks',@saved_callbacks);
check('Part 1 cleared workspace recovery',@pendulum_recovery);
check('Part 1 default launcher',@pendulum_launcher);
check('Experiment settings are preserved',@preserve_experiment);
check('Missing physical structure field recovery',@field_recovery);
check('Part 2 default launcher and balancing',@cartpole_launcher);
check('cartpole2 missing param recovery and balancing',@cartpole_alias);
check('Other numbered models',@other_aliases);
if includeExperiments
    experiments = {'simscape_open_loop','lqr_evaluation','qr_sweep','sampling_time', ...
        'actuator_limits','observer_poles','measurement_noise','input_disturbance', ...
        'model_uncertainty','alternative_estimation'};
    for k = 1:numel(experiments)
        experiment = experiments{k};
        check(['Experiment: ' experiment],@() run_base(experiment));
    end
end
check('Switching project parts restores the parameter types',@switch_parts);
report.passed = all([report.checks.passed]);
fid = fopen(fullfile(outputFolder,'latest.json'),'w');
fprintf(fid,'%s\n',jsonencode(report));
fclose(fid);
fprintf('\nRESULT: %d/%d checks passed.\n',sum([report.checks.passed]),numel(report.checks));
if ~report.passed
    warning('CIS:ValidationFailed','Some checks failed. Inspect output/validation/latest.log.');
end

    function check(name,action)
        fprintf('\nCHECK: %s\n',name);
        entry = struct('name',name,'passed',false,'detail','');
        try
            % Allow pending simulator/UI cleanup from the previous script to
            % finish before launching another command-line simulation.
            drawnow;
            action();
            drawnow;
            entry.passed = true;
            fprintf('PASS: %s\n',name);
        catch problem
            entry.detail = getReport(problem,'extended','hyperlinks','off');
            fprintf('FAIL: %s\n%s\n',name,entry.detail);
        end
        report.checks(end+1) = entry;
    end

    function run_base(script)
        path = strrep(fullfile(projectFolder,[script '.m']),'''','''''');
        evalin('base',sprintf('run(''%s'');',path));
    end

    function saved_callbacks
        for name = {'actuated_pendulum','cartpole','actuated_pendulum1','cartpole1','cartpole2'}
            if ~isfile(fullfile(projectFolder,[name{1} '.slx'])), continue; end
            load_system(fullfile(projectFolder,[name{1} '.slx']));
            assert(contains(get_param(name{1},'InitFcn'),'ensure_project_initialized'));
        end
    end

    function pendulum_recovery
        evalin('base','clear p m g Ts umin umax Ad Bd C K L xhat0 use_observer u_noise y_noise cis_active_model;');
        result = sim('actuated_pendulum','StopTime','1');
        finite_pendulum(result);
        assert(evalin('base','Ts == 0.02 && isstruct(m)'));
    end

    function pendulum_launcher
        run_base('pendulum_script');
        result = evalin('base','out');
        finite_pendulum(result);
        assert(abs(result.tout(end)-10) < 1e-6);
    end

    function preserve_experiment
        run_base('initialize_pendulum');
        evalin('base','p.theta0 = 20; use_observer = false; u_noise = 0; y_noise = 0; umin = -inf; umax = inf; K = 1.03*K;');
        beforeK = evalin('base','K');
        result = sim('actuated_pendulum','StopTime','0.2');
        finite_pendulum(result);
        assert(isequal(beforeK,evalin('base','K')));
        assert(evalin('base','p.theta0 == 20 && ~use_observer && isinf(umax) && u_noise == 0'));
    end

    function field_recovery
        evalin('base','p = rmfield(p,''l'');');
        result = sim('actuated_pendulum','StopTime','0.2');
        finite_pendulum(result);
        assert(evalin('base','isfield(p,''l'') && p.l == 0.5'));
    end

    function cartpole_launcher
        run_base('setup_cartpole');
        result = evalin('base','out');
        metrics = balanced_cartpole(result);
        assert(evalin('base','terminalInCapture && cartWithinLimit && forceWithinLimit'));
        report.cartpole = metrics;
    end

    function cartpole_alias
        evalin('base','clear param;');
        result = sim('cartpole2');
        metrics = balanced_cartpole(result);
        assert(evalin('base','isfield(param,''L'') && isfield(param,''p0'') && isfield(param,''bc'')'));
        report.cartpole2 = metrics;
    end

    function other_aliases
        result = sim('cartpole1','StopTime','0.2');
        assert(all(isfinite(result.x.Data(:))));
        result = sim('actuated_pendulum1','StopTime','0.2');
        finite_pendulum(result);
    end

    function switch_parts
        ensure_project_initialized('cartpole');
        assert(evalin('base','Ts == 0.01 && isstruct(param) && isscalar(m)'));
        ensure_project_initialized('actuated_pendulum');
        assert(evalin('base','Ts == 0.02 && isstruct(m) && numel(K) == 2'));
        ensure_project_initialized('cartpole');
        assert(evalin('base','Ts == 0.01 && numel(K_lqr) == 4'));
    end

    function finite_pendulum(result)
        for name = {'theta_sim','omega_sim','theta_hat','omega_hat','u_sim'}
            signal = result.get(name{1});
            assert(~isempty(signal.Data) && all(isfinite(signal.Data(:))));
        end
    end

    function metrics = balanced_cartpole(result)
        state = squeeze(double(result.x.Data));
        time = double(result.x.Time(:));
        if size(state,2) ~= 4, state = state.'; end
        assert(size(state,1) == numel(time) && all(isfinite(state(:))));
        theta = atan2(sin(state(:,3)),cos(state(:,3)));
        capture = evalin('base','capture');
        inCapture = abs(state(:,1)) <= capture(1) & abs(state(:,2)) <= capture(2) & ...
            abs(theta) <= capture(3) & abs(state(:,4)) <= capture(4);
        first = find(inCapture,1);
        assert(~isempty(first),'The cart-pole did not enter capture.');
        settled = abs(state(:,1)) <= 0.05 & abs(state(:,2)) <= 0.10 & ...
            abs(theta) <= deg2rad(2) & abs(state(:,4)) <= 0.10;
        assert(all(settled(time >= time(end)-1)),'The cart-pole did not remain balanced.');
        force = double(result.u.Data(:));
        assert(all(isfinite(force)) && max(abs(force)) <= evalin('base','Fmax')+1e-8);
        metrics = struct('captureTime',time(first),'endTime',time(end), ...
            'peakTravel',max(abs(state(:,1))),'travelTarget',evalin('base','xmax'), ...
            'finalAngleDegrees',rad2deg(theta(end)),'finalPosition',state(end,1));
        fprintf('Capture %.3f s, final angle %.6f deg, measured peak travel %.3f m.\n', ...
            metrics.captureTime,metrics.finalAngleDegrees,metrics.peakTravel);
    end
end
