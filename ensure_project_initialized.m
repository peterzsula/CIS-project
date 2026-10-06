function ensure_project_initialized(modelName)
% Model InitFcn callback: recover a cleared/incomplete base workspace.
% Complete same-part experiment settings are preserved, including custom K,
% observer matrices, noise, initial states, and saturation limits.
modelName = char(modelName);
switch modelName
    case {'actuated_pendulum','actuated_pendulum1'}
        family = 'actuated_pendulum';
        initializer = 'initialize_pendulum.m';
        required = {'p','m','g','Ts','umin','umax','Ad','Bd','C','K','L', ...
                    'xhat0','use_observer','u_noise','y_noise'};
        fields = {'p', {'m','l','b','theta0','omega0'}; ...
                  'm', {'m','l','b','J'}};
        sizes = {'Ad',[2 2]; 'Bd',[2 1]; 'C',[1 2]; ...
                 'K',[1 2]; 'L',[2 1]; 'xhat0',[2 1]};
    case {'cartpole','cartpole1','cartpole2'}
        family = 'cartpole';
        initializer = 'initialize_cartpole.m';
        required = {'param','g','Ts','Fmax','K_lqr','Xnom','Unom', ...
                    'Knom','capture','T'};
        fields = {'param', {'m','M','L','bc','bp','p0','v0','theta0','omega0'}};
        sizes = {'K_lqr',[1 4]; 'capture',[1 4]};
    otherwise
        error('CIS:UnknownModel','Unsupported project model: %s',modelName);
end

ready = evalin('base',sprintf( ...
    'exist(''cis_active_model'',''var'') && strcmp(cis_active_model,''%s'')',family));
for k = 1:numel(required)
    ready = ready && evalin('base',sprintf('exist(''%s'',''var'') == 1',required{k}));
end
for k = 1:size(fields,1)
    ready = ready && evalin('base',sprintf('isstruct(%s)',fields{k,1}));
    for j = 1:numel(fields{k,2})
        ready = ready && evalin('base',sprintf( ...
            'isfield(%s,''%s'')',fields{k,1},fields{k,2}{j}));
    end
end
for k = 1:size(sizes,1)
    ready = ready && evalin('base',sprintf( ...
        'isequal(size(%s),%s)',sizes{k,1},mat2str(sizes{k,2})));
end
if ready
    return;
end

initializerPath = fullfile(fileparts(mfilename('fullpath')),initializer);
fprintf('Initializing %s: restoring its physical and controller parameters.\n',modelName);
% The initializers do not load models, simulate, or plot: no recursion.
evalin('base',sprintf('run(''%s'');',strrep(initializerPath,'''','''''')));
end
