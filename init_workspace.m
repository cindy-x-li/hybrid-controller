% title: init_workspace.m
% Initializes the main workspace, auto-compiles Breach, and handles submodule relative paths

function init_workspace(model_name)
    % Allow running without passing a model name initially
    if nargin < 1
        model_name = ''; 
    end

    % 1. Get the root directory of the hybrid-controller
    project_root = fileparts(mfilename('fullpath'));
    
    % --- BREACH INITIALIZATION & COMPILATION ---
    breach_path = '';
    
    % Check if the user has defined a custom local override
    if exist('local_config', 'file') == 2
        breach_path = local_config();
    end
    
    % If no custom path is provided, default to the standard setup
    if isempty(breach_path)
        breach_path = fullfile(project_root, 'external', 'breach');
    end
    
    % Validate the path
    if ~exist(breach_path, 'dir')
        error('Breach not found at %s. Run setup.sh in the terminal or create local_config.m', breach_path);
    end
    
    addpath(breach_path);
    
    % Initialize Breach to load its internal paths
    InitBreach;
    
    % --- SUBMODULE OVERRIDE & CONTROL ---
    
    % Define the submodule path
    submodule_path = fullfile(project_root, 'stl-control-imitation');
    addpath(submodule_path);
    
    % Save our current directory so we can jump back later
    original_dir = pwd; 
    
    % Jump INSIDE the submodule so `addpath('shared_code')` works properly
    cd(submodule_path); 
    
    % Temporarily mute MATLAB's warning about folders not existing.
    % This suppresses the error from Alex's hardcoded path in the submodule.
    warning('off', 'MATLAB:mpath:nameNonexistentOrNotADirectory');
    
    try
        % Run the submodule's script safely. 
        if isempty(model_name)
            init_paths('');
        else
            init_paths(model_name);
        end
        
    catch ME
        % Safety Net: Restore settings before throwing the error if it crashes
        warning('on', 'MATLAB:mpath:nameNonexistentOrNotADirectory');
        cd(original_dir);
        rethrow(ME);
    end
    
    % Restore normal warnings and jump back to the parent directory
    warning('on', 'MATLAB:mpath:nameNonexistentOrNotADirectory');
    cd(original_dir);
    
    % --- PARALLEL WORKER SILENCING ---
    % If a parallel pool exists, mute the hardcoded path warnings on all background workers
    poolobj = gcp('nocreate');
    if ~isempty(poolobj)
        pctRunOnAll('warning(''off'', ''MATLAB:mpath:nameNonexistentOrNotADirectory'');');
    end
    
    % --- ADD LOCAL PROJECT FOLDERS ---
    % Recursively add the WaterTank folder and all its subfolders
    watertank_path = fullfile(project_root, 'WaterTank_hybrid');
    addpath(genpath(watertank_path));
    
    % --- WORKSPACE FOLDER SETUP ---
    logs_dir = fullfile(project_root, 'logs');
    if ~exist(logs_dir, 'dir')
        mkdir(logs_dir);
    end
    
    data_dir = fullfile(project_root, 'data');
    if ~exist(data_dir, 'dir')
        mkdir(data_dir);
    end

    disp('Hybrid Controller workspace successfully initialized.');
end