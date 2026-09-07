function report = caseD_aero_static
%CASED_AERO_STATIC  逻辑验证 D：静态片条气动力符号（正 α / 有 β）
%
% 用极曲线 + qS + Tq，η=η̇=0，当地 α 来自 strip_local_alphas。

    ensure_aero_workspace;
    n_pass = 0; n_fail = 0; logs = {};
    fprintf('=== Case D: static strip aero ===\n');

    cant = evalin('base', 'v_tail_cant_rad');
    U = evalin('base', 'U');
    polar_a = evalin('base', 'polar_alpha_deg');
    polar_CL = evalin('base', 'polar_CL');
    polar_CD = evalin('base', 'polar_CD');

    struct_dir = 'D:\Load_Estimation_Case\Aircraft_structure_redefine\results';
    strips = aeroelastic.load_strips(struct_dir, 'mac_m', 2.56, 'v_tail_cant_deg', 42);
    aero = strips.by_surface;

    %% D1 正迎角、无侧滑：主翼 Q_heave > 0
    w = 0.1 * U;
    V_b = [U; 0; w];
    [a_wr, ~, a_tr, a_tl] = strip_local_alphas(V_b, cant);
    Qw = strip_Q_(aero.wing_R, a_wr, polar_a, polar_CL, polar_CD, 'qS_strip_wing_R');
    [n_pass, n_fail, logs] = logic_check(Qw(3) > 0, ...
        sprintf('D1 +alpha wing Q_heave>0  a=%.4f Qh=%.4g', a_wr, Qw(3)), ...
        n_pass, n_fail, logs);

    %% D2 同样工况 V 尾左右 heave 同号（外倾，仍有 +z 分量）
    Qr = strip_Q_(aero.v_tail_R, a_tr, polar_a, polar_CL, polar_CD, 'qS_strip_v_tail_R');
    Ql = strip_Q_(aero.v_tail_L, a_tl, polar_a, polar_CL, polar_CD, 'qS_strip_v_tail_L');
    [n_pass, n_fail, logs] = logic_check(Qr(3) > 0 && Ql(3) > 0, ...
        sprintf('D2 +alpha vt heave>0  QR=%.3g QL=%.3g', Qr(3), Ql(3)), ...
        n_pass, n_fail, logs);

    %% D3 有 β：左右尾当地 α 异号；L=sign(α) 时两侧 sway 同号（净侧力）
    V_b2 = [U; 0.15 * U; 0];
    [~, ~, a_tr2, a_tl2] = strip_local_alphas(V_b2, cant);
    [n_pass, n_fail, logs] = logic_check(a_tr2 * a_tl2 < 0, ...
        sprintf('D3a beta: vt alpha opposite  R=%.4f L=%.4f', a_tr2, a_tl2), ...
        n_pass, n_fail, logs);
    % 等 |L|·sign(α)：相对 A7（等 +L → sway 异号、净≈0），β 使净侧力非零
    Lr = ones(aero.v_tail_R.n_strip, 1) * sign(a_tr2);
    Ll = ones(aero.v_tail_L.n_strip, 1) * sign(a_tl2);
    Qr2 = aero.v_tail_R.Tq * Lr;
    Ql2 = aero.v_tail_L.Tq * Ll;
    [n_pass, n_fail, logs] = logic_check(Qr2(2) * Ql2(2) > 0 && abs(Qr2(2) + Ql2(2)) > 0.05, ...
        sprintf('D3b beta: same-sign sway / net Qy  R=%.3g L=%.3g sum=%.3g', ...
        Qr2(2), Ql2(2), Qr2(2) + Ql2(2)), ...
        n_pass, n_fail, logs);

    %% D4 CL 查表有限
    CL = interp1(polar_a, polar_CL, rad2deg(a_wr), 'linear', 'extrap');
    [n_pass, n_fail, logs] = logic_check(isfinite(CL) && CL > 0, ...
        sprintf('D4 polar CL(alpha)>0  CL=%.4f', CL), ...
        n_pass, n_fail, logs);

    fprintf('\nCase D result: %d PASS, %d FAIL\n', n_pass, n_fail);
    for i = 1:numel(logs), fprintf('  %s\n', logs{i}); end

    report = struct('name', 'CaseD', 'n_pass', n_pass, 'n_fail', n_fail, ...
        'ok', n_fail == 0, 'logs', {logs});
    if report.ok, fprintf('Case D: OK\n'); else, fprintf('Case D: FAILED\n'); end
end

function Q = strip_Q_(surf, alpha_rad, polar_a, polar_CL, polar_CD, qS_name)
    ns = surf.n_strip;
    CL = interp1(polar_a, polar_CL, rad2deg(alpha_rad), 'linear', 'extrap');
    CD = interp1(polar_a, polar_CD, rad2deg(alpha_rad), 'linear', 'extrap');
    qS = evalin('base', qS_name);
    L = qS(:) * CL;
    D = qS(:) * CD;
    Q = surf.Tq * L + surf.Tq_D * D;
end
