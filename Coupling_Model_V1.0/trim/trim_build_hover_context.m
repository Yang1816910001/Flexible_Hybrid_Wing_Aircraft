function ctx = trim_build_hover_context
%TRIM_BUILD_HOVER_CONTEXT  Workspace maps for hover static trim (tilt=pi/2, U=0).

    root = fileparts(fileparts(mfilename('fullpath')));
    addpath(root);
    addpath(fullfile(root, 'thrust'));
    addpath(fullfile(root, 'tests'));
    cd(root);
    ensure_aero_workspace();

    ctx = struct();
    ctx.root = root;
    ctx.U = 0;
    ctx.cant = evalin('base', 'v_tail_cant_rad');
    ctx.Altitude_m = evalin('base', 'Altitude_m');
    ctx.Tq_mass = evalin('base', 'Tq_mass');
    ctx.Tq_prop = evalin('base', 'Tq_prop');
    ctx.W_mass_g = evalin('base', 'W_mass_g');
    ctx.Qg = ctx.Tq_mass * ctx.W_mass_g;
    ctx.e_cruise = evalin('base', 'e_thrust_cruise');
    ctx.e_hover = evalin('base', 'e_thrust_hover');
    ctx.is_tilt = logical(evalin('base', 'eng_is_tilt'));
    ctx.p_kink = evalin('base', 'p_kink_m');
    ctx.p_eng0 = evalin('base', 'p_eng0_m');

    ctx.idx_lift_front = find(~ctx.is_tilt & (1:8)' <= 4);
    ctx.idx_lift_aft = find(~ctx.is_tilt & (1:8)' >= 5);
    ctx.idx_front = (1:4)';
    ctx.idx_aft = (5:8)';

    % Hover: all tilt = pi/2 (lift engines ignore tilt in pose)
    ctx.tilt = (pi / 2) * ones(8, 1);
    [ctx.e_s, ctx.p_eng, ~, ~] = engine_stub_pose( ...
        ctx.tilt, ctx.e_cruise, ctx.e_hover, ctx.p_kink, ctx.p_eng0, ctx.is_tilt);

    fz = ctx.W_mass_g(3:6:end);
    ctx.m = -sum(fz) / 9.80665;
    ctx.mg = ctx.m * 9.80665;
    ctx.mach = 0;
end
