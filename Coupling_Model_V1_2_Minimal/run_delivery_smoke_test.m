%% Minimal delivery package smoke test
% Run this script from MATLAB after cloning the repository. It initializes
% the workspace, resolves the linked libraries, compiles the delivered
% model and advances it for a short interval. No model file is saved.

root_dir = fileparts(mfilename('fullpath'));
cd(root_dir);

addpath(root_dir, fullfile(root_dir, 'lib'), ...
    fullfile(root_dir, 'trim'), fullfile(root_dir, 'thrust'), ...
    fullfile(root_dir, 'Library'));

fprintf('\n=== Coupling model minimal-package smoke test ===\n');
fprintf('Repository root: %s\n', root_dir);

required_files = {
    'aerodynamic_model_V1_2_Wrench.slx'
    fullfile('Library', 'nodes.csv')
    fullfile('Library', 'elements.csv')
    fullfile('Library', 'frequencies.csv')
    fullfile('Library', 'modal_matrix_15.csv')
    fullfile('data', 'propeller', 'CFD_HOVER_2D.mat')
    fullfile('data', 'propeller', 'CFD_DATA_3D.mat')
    fullfile('polars', 'naca2412_Re6p81e6_Ncrit9.pol')
    fullfile('polars', 'flap_polars.mat')};
for i = 1:numel(required_files)
    assert(isfile(fullfile(root_dir, required_files{i})), ...
        'Missing required delivery file: %s', required_files{i});
end

initial_main_V1_1;

% initial_main_V1_1 intentionally clears its caller workspace, so variables
% needed below are recreated after initialization.
mdl = 'aerodynamic_model_V1_2_Wrench';
load_system(mdl);
model_dirty_before = get_param(mdl, 'Dirty');
cleanup_model = onCleanup(@() close_system(mdl, 0));

sim_in = Simulink.SimulationInput(mdl);
sim_in = sim_in.setExternalInput(trim_input_dataset);
sim_in = sim_in.setModelParameter( ...
    'StopTime', '0.02', ...
    'ReturnWorkspaceOutputs', 'on', ...
    'SaveOutput', 'on', ...
    'SignalLogging', 'off');
sim(sim_in);

assert(strcmp(get_param(mdl, 'Dirty'), model_dirty_before), ...
    'Smoke test unexpectedly changed the model dirty state.');
fprintf('PASS: initialization, library resolution and 0.02 s simulation completed.\n');
fprintf('The delivered SLX file was not saved or modified.\n');
