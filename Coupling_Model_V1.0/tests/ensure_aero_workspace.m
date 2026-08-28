function ensure_aero_workspace
%ENSURE_AERO_WORKSPACE  若缺 As / 总线对象则跑 initial_main（不 verify）
    root = fileparts(fileparts(mfilename('fullpath')));
    addpath(root);
    aeroelastic.setup_paths();
    cd(root);
    need = ~evalin('base', 'exist(''As'',''var'')') || ...
           ~evalin('base', 'exist(''Tq_mass'',''var'')') || ...
           ~evalin('base', 'exist(''W_mass_g'',''var'')') || ...
           ~evalin('base', 'exist(''e_thrust_cruise'',''var'')') || ...
           ~evalin('base', 'exist(''DynamicsAndFlexibleStates'',''var'')') || ...
           ~evalin('base', 'exist(''qS_strip_wing_R'',''var'')');
    if need
        assignin('base', 'do_verify', false);
        evalin('base', 'initial_main');
    end
end
