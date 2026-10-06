function repair_project_models
% Restore model InitFcn callbacks and retain the shared model file formats.
% Run this after replacing models with copies that have no startup callback.
projectFolder = fileparts(mfilename('fullpath'));
addpath(projectFolder);
models = {'actuated_pendulum','R2026a'; 'cartpole','R2026a'; ...
          'actuated_pendulum1','R2026a'; 'cartpole1','R2023b'; 'cartpole2','R2023b'};
callback = ['addpath(fileparts(get_param(bdroot,''FileName''))); ' ...
            'ensure_project_initialized(bdroot);'];
exportFolder = tempname;
mkdir(exportFolder);
for k = 1:size(models,1)
    name = models{k,1};
    modelPath = fullfile(projectFolder,[name '.slx']);
    if ~isfile(modelPath)
        continue;
    end
    load_system(modelPath);
    existing = get_param(name,'InitFcn');
    if ~contains(existing,'ensure_project_initialized')
        if ~isempty(existing)
            existing = sprintf('%s\n%s',callback,existing);
        else
            existing = callback;
        end
        set_param(name,'InitFcn',existing);
    end
    if startsWith(name,'cartpole')
        set_param(name,'StopTime','T+8');
    elseif strcmp(name,'actuated_pendulum')
        set_param(name,'StopTime','10');
    end
    save_system(name);
    targetRelease = models{k,2};
    if ~strcmpi(version('-release'),targetRelease(2:end))
        exported = fullfile(exportFolder,[name '.slx']);
        Simulink.exportToVersion(name,exported,targetRelease);
        close_system(name,0);
        copyfile(exported,modelPath,'f');
        load_system(modelPath);
    end
    fprintf('Repaired %s (file format %s).\n',name,targetRelease);
end
% The temporary export copies provide an additional recoverable snapshot.
end
