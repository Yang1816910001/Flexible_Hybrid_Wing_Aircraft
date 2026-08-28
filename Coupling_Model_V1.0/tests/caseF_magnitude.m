function report = caseF_magnitude
%CASEF_MAGNITUDE  Order-of-magnitude: lift vs weight, thrust vs weight, loads vs modes
%
% Compares modal heave GF and physical Fz/Lift against mg. Not a trim solve.

    ensure_aero_workspace;
    n_pass = 0; n_fail = 0; logs = {};
    fprintf('=== Case F: magnitude (lift / thrust / modes) ===\n');

    W = evalin('base', 'W_mass_g');
    Tq_mass = evalin('base', 'Tq_mass');
    Tq_prop = evalin('base', 'Tq_prop');
    Qg = Tq_mass * W;
    fz = W(3:6:end);
    g = 9.80665;
    m = -sum(fz) / g;
    mg = m * g;
    Qgh = abs(Qg(3));

    U = evalin('base', 'U');
    cant = evalin('base', 'v_tail_cant_rad');
    polar_a = evalin('base', 'polar_alpha_deg');
    polar_CL = evalin('base', 'polar_CL');
    polar_CD = evalin('base', 'polar_CD');

    struct_dir = 'D:\Load_Estimation_Case\Aircraft_structure_redefine\results';
    strips = aeroelastic.load_strips(struct_dir, 'mac_m', 2.56, 'v_tail_cant_deg', 42);
    aero = strips.by_surface;

    %% F1 模态投影自洽：Q_g(3) = Phi_z' * Fz
    Phi = Tq_mass.';
    phi_z = Phi(3:6:end, 3);
    [n_pass, n_fail, logs] = logic_check(abs(phi_z.' * fz - Qg(3)) < 1e-6 * max(1, Qgh), ...
        sprintf('F1 Phi_z''*Fz = Q_g(3)  got=%.6g exp=%.6g', phi_z.' * fz, Qg(3)), ...
        n_pass, n_fail, logs);

    %% F2 重力几乎只进刚体 heave（分布质量）
    [n_pass, n_fail, logs] = logic_check(norm(Qg(7:end)) < 1e-2 * Qgh, ...
        sprintf('F2 gravity |Qe|/|Qh|=%.3g (<<1)', norm(Qg(7:end)) / Qgh), ...
        n_pass, n_fail, logs);

    %% F3 巡航 α≈2°：气动 heave GF 与 |Q_g| 同量级（可托住重量）
    a_ref = deg2rad(2);
    V_b = [U; 0; U * tan(a_ref)];
    [awr, awl, atr, atl] = strip_local_alphas(V_b, cant);
    Qa = zeros(size(Qg));
    Qa = Qa + strip_Q_(aero.wing_R, awr, polar_a, polar_CL, polar_CD, 'qS_strip_wing_R');
    Qa = Qa + strip_Q_(aero.wing_L, awl, polar_a, polar_CL, polar_CD, 'qS_strip_wing_L');
    Qa = Qa + strip_Q_(aero.v_tail_R, atr, polar_a, polar_CL, polar_CD, 'qS_strip_v_tail_R');
    Qa = Qa + strip_Q_(aero.v_tail_L, atl, polar_a, polar_CL, polar_CD, 'qS_strip_v_tail_L');
    r_aero = Qa(3) / Qgh;
    [n_pass, n_fail, logs] = logic_check(r_aero > 0.5 && r_aero < 1.5, ...
        sprintf('F3 aero@2deg Qh/|Qg|=%.3f (expect ~1)', r_aero), ...
        n_pass, n_fail, logs);

    %% F4 物理翼升力 vs mg（片条 qS*CL，未投影模态）
    CL_w = interp1(polar_a, polar_CL, rad2deg(awr), 'linear', 'extrap');
    L_wing = (sum(evalin('base', 'qS_strip_wing_R')) + sum(evalin('base', 'qS_strip_wing_L'))) * CL_w;
    r_L = L_wing / mg;
    [n_pass, n_fail, logs] = logic_check(r_L > 0.4 && r_L < 1.2, ...
        sprintf('F4 wing L_phys@2deg / mg=%.3f  L=%.0f N mg=%.0f', r_L, L_wing, mg), ...
        n_pass, n_fail, logs);

    %% F5 气动外载进弹性模态：有量级、但不炸（相对 heave）
    r_ae = norm(Qa(7:end)) / max(abs(Qa(3)), 1);
    [n_pass, n_fail, logs] = logic_check(r_ae > 0.05 && r_ae < 5, ...
        sprintf('F5 aero |Qe|/|Qh|=%.3f (distributed load)', r_ae), ...
        n_pass, n_fail, logs);

    %% F6 悬停总推力 = mg：Q_heave 对消重力（同模态度量）
    T = (mg / 8) * ones(8, 1);
    e_c = evalin('base', 'e_thrust_cruise');
    e_h = evalin('base', 'e_thrust_hover');
    is_t = logical(evalin('base', 'eng_is_tilt'));
    [eh, ph] = engine_stub_pose((pi / 2) * ones(8, 1), e_c, e_h, ...
        evalin('base', 'p_kink_m'), evalin('base', 'p_eng0_m'), is_t);
    Wh = engine_pack_wrenches(T, eh, ph, evalin('base', 'p_eng0_m'));
    Qp = Tq_prop * Wh;
    r_p = Qp(3) / Qgh;
    [n_pass, n_fail, logs] = logic_check(r_p > 0.9 && r_p < 1.05, ...
        sprintf('F6 hover T=mg: Qp_h/|Qg|=%.3f (expect ~1)', r_p), ...
        n_pass, n_fail, logs);

    %% F7 物理铅垂推力 vs mg（短梁外倾 → Fz 略小于 mg）
    Fz_p = sum(Wh(3:6:end));
    r_Fz = Fz_p / mg;
    [n_pass, n_fail, logs] = logic_check(r_Fz > 0.9 && r_Fz <= 1.0 + 1e-6, ...
        sprintf('F7 hover Fz/mg=%.3f  Fz=%.0f', r_Fz, Fz_p), ...
        n_pass, n_fail, logs);

    %% F8 集中推进载荷：弹性 GF 相对 heave 显著但有界
    r_pe = norm(Qp(7:end)) / max(abs(Qp(3)), 1);
    [n_pass, n_fail, logs] = logic_check(r_pe > 0.1 && r_pe < 3, ...
        sprintf('F8 prop |Qe|/|Qh|=%.3f (point loads)', r_pe), ...
        n_pass, n_fail, logs);

    %% F9 同“托住重量”工况：气动与推进 heave GF 彼此同量级
    r_ap = abs(Qa(3) / Qp(3));
    [n_pass, n_fail, logs] = logic_check(r_ap > 0.5 && r_ap < 1.5, ...
        sprintf('F9 aero@2deg Qh / prop@T=mg Qh = %.3f', r_ap), ...
        n_pass, n_fail, logs);

    fprintf('\nCase F result: %d PASS, %d FAIL\n', n_pass, n_fail);
    for i = 1:numel(logs), fprintf('  %s\n', logs{i}); end

    report = struct('name', 'CaseF', 'n_pass', n_pass, 'n_fail', n_fail, ...
        'ok', n_fail == 0, 'logs', {logs}, ...
        'mg', mg, 'Qg_heave', Qg(3), 'Qa_heave', Qa(3), 'Qp_heave', Qp(3));
    if report.ok, fprintf('Case F: OK\n'); else, fprintf('Case F: FAILED\n'); end
end

function Q = strip_Q_(surf, alpha_rad, polar_a, polar_CL, polar_CD, qS_name)
    CL = interp1(polar_a, polar_CL, rad2deg(alpha_rad), 'linear', 'extrap');
    CD = interp1(polar_a, polar_CD, rad2deg(alpha_rad), 'linear', 'extrap');
    qS = evalin('base', qS_name);
    Q = surf.Tq * (qS(:) * CL) + surf.Tq_D * (qS(:) * CD);
end
