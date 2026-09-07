function report = caseA_geometry_alpha
%CASEA_GEOMETRY_ALPHA  逻辑验证 A：当地迎角 + 主翼升力映射符号
%
% 用法（在仓库根或本目录）:
%   addpath(...);  report = caseA_geometry_alpha
%
% 不跑 Simulink；只依赖 aeroelastic.load_strips + strip_local_alphas。

    root = fileparts(fileparts(mfilename('fullpath')));
    addpath(root);
    addpath(fullfile(root, 'thrust'));

    tol_a = 1e-9;
    tol_rel = 1e-3;
    cant = deg2rad(42);
    U = 38.8;
    n_pass = 0;
    n_fail = 0;
    logs = {};

    fprintf('=== Case A: geometry / local alpha / lift map ===\n');

    %% A1 主翼 α = 体轴 atan2(w,u)
    w = 3.88;
    V_b = [U; 0; w];
    [a_wr, a_wl, a_tr, a_tl] = strip_local_alphas(V_b, cant);
    a_body = atan2(w, U);
    [n_pass, n_fail, logs] = check_(abs(a_wr - a_body) < tol_a && abs(a_wl - a_body) < tol_a, ...
        sprintf('A1 wing alpha = body atan2(w,u)  wing=%.6f body=%.6f', a_wr, a_body), ...
        n_pass, n_fail, logs);

    %% A2 无侧滑：V 尾 ≈ cosΓ·α，左右相等
    a_vt_exp = atan2(cos(cant) * w, U);  % 精确式（非小角 cos*α）
    [n_pass, n_fail, logs] = check_(abs(a_tr - a_tl) < tol_a, ...
        sprintf('A2a vtR=vtL (no beta)  R=%.6f L=%.6f', a_tr, a_tl), ...
        n_pass, n_fail, logs);
    [n_pass, n_fail, logs] = check_(abs(a_tr - a_vt_exp) < tol_a, ...
        sprintf('A2b vt alpha = atan2(cosΓ·w,u)  got=%.6f exp=%.6f', a_tr, a_vt_exp), ...
        n_pass, n_fail, logs);
    [n_pass, n_fail, logs] = check_(abs(a_tr - cos(cant)*a_body) < 5e-4, ...
        sprintf('A2c vt ≈ cosΓ·α_wing (small-α)  got=%.6f approx=%.6f', ...
        a_tr, cos(cant)*a_body), ...
        n_pass, n_fail, logs);

    %% A3 有侧滑：左右 V 尾异号
    V_b2 = [U; 5; 0];
    [~, ~, a_tr2, a_tl2] = strip_local_alphas(V_b2, cant);
    [n_pass, n_fail, logs] = check_(a_tr2 * a_tl2 < 0, ...
        sprintf('A3 vtR/vtL opposite with beta  R=%.6f L=%.6f', a_tr2, a_tl2), ...
        n_pass, n_fail, logs);

    %% A4 |V|≈0 → α=0
    [z1, z2, z3, z4] = strip_local_alphas([0; 0; 0], cant);
    [n_pass, n_fail, logs] = check_(all([z1, z2, z3, z4] == 0), ...
        'A4 zero velocity -> alpha=0', n_pass, n_fail, logs);

    %% A5 主翼正升力 → 刚体 heave 广义力 > 0（结构 +z）
    struct_dir = 'D:\Load_Estimation_Case\Aircraft_structure_redefine\results';
    strips = aeroelastic.load_strips(struct_dir, 'mac_m', 2.56, 'v_tail_cant_deg', 42);
    wing = strips.by_surface.wing_R;
    L = ones(wing.n_strip, 1);          % 正升力（沿 n̂）
    Q = wing.Tq * L;
    % 刚体 3 = heave（结构 z 上）
    [n_pass, n_fail, logs] = check_(Q(3) > 0, ...
        sprintf('A5 wing +L -> Q_heave>0  Q(1:6)=%s', mat2str(Q(1:6).', 3)), ...
        n_pass, n_fail, logs);

    %% A6 V 尾法向含外倾（与 load_strips 一致）
    nR = strips.by_surface.v_tail_R.n_hat(1, :).';
    nL = strips.by_surface.v_tail_L.n_hat(1, :).';
    nR_exp = [0; -sin(cant); cos(cant)];
    nL_exp = [0;  sin(cant); cos(cant)];
    [n_pass, n_fail, logs] = check_(norm(nR - nR_exp) < tol_rel && norm(nL - nL_exp) < tol_rel, ...
        sprintf('A6 v-tail n_hat matches cant=42deg'), ...
        n_pass, n_fail, logs);

    %% A7 等升力时左右 V 尾 sway 广义力异号（法向 ±y）
    vtR = strips.by_surface.v_tail_R;
    vtL = strips.by_surface.v_tail_L;
    L1 = ones(vtR.n_strip, 1);
    Qr = vtR.Tq * L1;
    Ql = vtL.Tq * L1;
    [n_pass, n_fail, logs] = check_(Qr(2) * Ql(2) < 0, ...
        sprintf('A7 equal +L: sway Q opposite  R=%.3g L=%.3g', Qr(2), Ql(2)), ...
        n_pass, n_fail, logs);

    fprintf('\nCase A result: %d PASS, %d FAIL\n', n_pass, n_fail);
    for i = 1:numel(logs)
        fprintf('  %s\n', logs{i});
    end

    report = struct();
    report.name = 'CaseA';
    report.n_pass = n_pass;
    report.n_fail = n_fail;
    report.ok = (n_fail == 0);
    report.logs = logs;
    if report.ok
        fprintf('Case A: OK\n');
    else
        fprintf('Case A: FAILED\n');
    end
end

function [n_pass, n_fail, logs] = check_(cond, msg, n_pass, n_fail, logs)
    if cond
        n_pass = n_pass + 1;
        logs{end+1} = ['PASS  ' msg]; %#ok<AGROW>
    else
        n_fail = n_fail + 1;
        logs{end+1} = ['FAIL  ' msg]; %#ok<AGROW>
    end
end
