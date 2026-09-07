function ctx = trim_build_context
%TRIM_BUILD_CONTEXT  Load workspace maps needed for static cruise trim.

    root = fileparts(fileparts(mfilename('fullpath')));
    addpath(root);
    addpath(fullfile(root, 'thrust'));
    addpath(fullfile(root, 'tests'));
    cd(root);
    ensure_aero_workspace();

    ctx = struct();
    ctx.root = root;
    ctx.U = evalin('base', 'U');
    ctx.cant = evalin('base', 'v_tail_cant_rad');
    ctx.Altitude_m = evalin('base', 'Altitude_m');
    ctx.polar_a = evalin('base', 'polar_alpha_deg');
    ctx.polar_CL = evalin('base', 'polar_CL');
    ctx.polar_CD = evalin('base', 'polar_CD');
    ctx.polar_CM = evalin('base', 'polar_CM');
    ctx.Tq_mass = evalin('base', 'Tq_mass');
    ctx.Tq_prop = evalin('base', 'Tq_prop');
    ctx.W_mass_g = evalin('base', 'W_mass_g');
    ctx.Qg = ctx.Tq_mass * ctx.W_mass_g;
    ctx.e_cruise = evalin('base', 'e_thrust_cruise');
    ctx.e_hover = evalin('base', 'e_thrust_hover');
    ctx.is_tilt = logical(evalin('base', 'eng_is_tilt'));
    ctx.p_kink = evalin('base', 'p_kink_m');
    ctx.p_eng0 = evalin('base', 'p_eng0_m');

    % E1..E8 = TL2_F, TL1_F, TR1_F, TR2_F, TL2_R, TL1_R, TR1_R, TR2_R
    ctx.idx_lift_front = find(~ctx.is_tilt & (1:8)' <= 4);
    ctx.idx_lift_aft = find(~ctx.is_tilt & (1:8)' >= 5);

    struct_dir = 'D:\Load_Estimation_Case\Aircraft_structure_redefine\results';
    strips = aeroelastic.load_strips(struct_dir, 'mac_m', 2.56, 'v_tail_cant_deg', 42);
    ctx.aero = strips.by_surface;
    ctx.surface_names = {'wing_R', 'wing_L', 'v_tail_R', 'v_tail_L'};

    % Cruise: tilt = 0 (forward for tilt engines)
    ctx.tilt = zeros(8, 1);
    [ctx.e_s, ctx.p_eng, ~, ~] = engine_stub_pose( ...
        ctx.tilt, ctx.e_cruise, ctx.e_hover, ctx.p_kink, ctx.p_eng0, ctx.is_tilt);

    fz = ctx.W_mass_g(3:6:end);
    ctx.m = -sum(fz) / 9.80665;
    ctx.mg = ctx.m * 9.80665;
    ctx.mach = ctx.U / 340.3;
end
