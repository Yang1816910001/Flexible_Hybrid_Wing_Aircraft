function report = caseC_propulsion
%CASEC_PROPULSION  逻辑验证 C：冻倾转推力轴与 W_prop（不跑 slx）
%
% Tilt=0 → 内台 e ≈ [−1;0;0]；Tilt=π/2 → 全台 e ≈ e_hover（Fz>0）
% 举升台改 Tilt 时 e 不变。

    ensure_aero_workspace;
    n_pass = 0; n_fail = 0; logs = {};
    fprintf('=== Case C: propulsion axes / W_prop ===\n');

    e_c = evalin('base', 'e_thrust_cruise');
    e_h = evalin('base', 'e_thrust_hover');
    is_t = logical(evalin('base', 'eng_is_tilt'));
    p_kink = evalin('base', 'p_kink_m');
    p0 = evalin('base', 'p_eng0_m');
    Tq_prop = evalin('base', 'Tq_prop');

    T_mag = 1000 * ones(8, 1);

    %% C1 前飞 Tilt=0：倾转台沿 −x
    tilt0 = zeros(8, 1);
    [e0, p0_now, ~, ~] = engine_stub_pose(tilt0, e_c, e_h, p_kink, p0, is_t);
    ok_tilt = true;
    for i = 1:8
        if is_t(i)
            ok_tilt = ok_tilt && norm(e0(:, i) - [-1; 0; 0]) < 1e-6;
        else
            ok_tilt = ok_tilt && norm(e0(:, i) - e_h(:, i)) < 1e-6;
        end
    end
    [n_pass, n_fail, logs] = logic_check(ok_tilt, ...
        'C1 Tilt=0: tilt-engines e=[-1;0;0], lift=e_hover', ...
        n_pass, n_fail, logs);

    W0 = engine_pack_wrenches(T_mag, e0, p0_now, p0);
    % 前飞：力矩在悬停网格上 p≠p0 时可能非零；倾转台 Fx 应为 −T
    Fx_tilt = W0((find(is_t, 1) - 1) * 6 + 1);
    [n_pass, n_fail, logs] = logic_check(abs(Fx_tilt + 1000) < 1e-6, ...
        sprintf('C2 Tilt=0 first tilt-engine Fx=-T  Fx=%.4g', Fx_tilt), ...
        n_pass, n_fail, logs);

    %% C3 悬停 Tilt=π/2：e = e_hover，Fz>0，M≈0
    tilt_h = (pi/2) * ones(8, 1);
    [eh, ph, ~, ~] = engine_stub_pose(tilt_h, e_c, e_h, p_kink, p0, is_t);
    ok_h = true;
    for i = 1:8
        ok_h = ok_h && norm(eh(:, i) - e_h(:, i)) < 1e-6;
        ok_h = ok_h && eh(3, i) > 0;
    end
    [n_pass, n_fail, logs] = logic_check(ok_h, ...
        'C3 Tilt=pi/2: e=e_hover and Fz>0 all engines', ...
        n_pass, n_fail, logs);

    Wh = engine_pack_wrenches(T_mag, eh, ph, p0);
    Mnorm = 0;
    Fz_sum = 0;
    T_along = 0;
    for i = 1:8
        Fi = Wh((i-1)*6 + (1:3));
        Mnorm = Mnorm + norm(Wh((i-1)*6 + (4:6)));
        Fz_sum = Fz_sum + Fi(3);
        T_along = T_along + dot(Fi, eh(:, i));
    end
    [n_pass, n_fail, logs] = logic_check(Mnorm < 1e-6, ...
        sprintf('C4 hover M≈0 at mesh nodes  |M|_sum=%.3g', Mnorm), ...
        n_pass, n_fail, logs);
    % 短梁有外倾：Fz_sum < 8T，但 F·e = T 每台
    [n_pass, n_fail, logs] = logic_check(abs(T_along - 8000) < 1e-5 && Fz_sum > 0.9*8000, ...
        sprintf('C5 hover F·e=8T and Fz>~0.9*8T  F·e=%.4g Fz=%.4g', T_along, Fz_sum), ...
        n_pass, n_fail, logs);

    %% C6 举升台改 Tilt，e 不变
    tilt_mix = zeros(8, 1);
    tilt_mix(is_t) = pi/4;
    [e_mix, ~, ~, ~] = engine_stub_pose(tilt_mix, e_c, e_h, p_kink, p0, is_t);
    ok_lift = true;
    for i = 1:8
        if ~is_t(i)
            ok_lift = ok_lift && norm(e_mix(:, i) - e_h(:, i)) < 1e-6;
        end
    end
    [n_pass, n_fail, logs] = logic_check(ok_lift, ...
        'C6 lift-engines ignore Tilt (e fixed at e_hover)', ...
        n_pass, n_fail, logs);

    %% C7 Q_prop = Tq_prop*W 悬停应主要进 heave（结构 +z → 模态）
    Qp = Tq_prop * Wh;
    [n_pass, n_fail, logs] = logic_check(Qp(3) > 0, ...
        sprintf('C7 hover Q_heave>0  Q_rigid=%s', mat2str(Qp(1:6).', 3)), ...
        n_pass, n_fail, logs);

    fprintf('\nCase C result: %d PASS, %d FAIL\n', n_pass, n_fail);
    for i = 1:numel(logs), fprintf('  %s\n', logs{i}); end

    report = struct('name', 'CaseC', 'n_pass', n_pass, 'n_fail', n_fail, ...
        'ok', n_fail == 0, 'logs', {logs});
    if report.ok, fprintf('Case C: OK\n'); else, fprintf('Case C: FAILED\n'); end
end
