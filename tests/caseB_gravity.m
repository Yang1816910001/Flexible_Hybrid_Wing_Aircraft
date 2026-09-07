function report = caseB_gravity
%CASEB_GRAVITY  逻辑验证 B：重力扳手 → 广义力符号 / 量级
%
% 静态：Q_g = Tq_mass * W_mass_g；结构系重力沿 −z，期望 Q_heave < 0。

    ensure_aero_workspace;
    n_pass = 0; n_fail = 0; logs = {};
    fprintf('=== Case B: gravity → Q ===\n');

    W = evalin('base', 'W_mass_g');
    Tq = evalin('base', 'Tq_mass');
    Qg = Tq * W;

    % 节点力：结构系 Fz 应全为负（或 0）
    fz = W(3:6:end);
    [n_pass, n_fail, logs] = logic_check(all(fz <= 0) && any(fz < 0), ...
        sprintf('B1 nodal Fz <= 0 (gravity -z)  sumFz=%.4g N', sum(fz)), ...
        n_pass, n_fail, logs);

    % 刚体 heave（模态 3）广义力 < 0
    [n_pass, n_fail, logs] = logic_check(Qg(3) < 0, ...
        sprintf('B2 Q_heave < 0  Q_rigid=%s', mat2str(Qg(1:6).', 3)), ...
        n_pass, n_fail, logs);

    % 总铅垂力 ≈ −m g
    m = -sum(fz) / 9.80665;
    [n_pass, n_fail, logs] = logic_check(m > 2000 && m < 4000, ...
        sprintf('B3 recovered mass from Fz  m≈%.1f kg', m), ...
        n_pass, n_fail, logs);

    % 弹性广义力范数应远小于 |Q_heave|（重力主要进刚体）
    [n_pass, n_fail, logs] = logic_check(norm(Qg(7:end)) < 5 * abs(Qg(3)), ...
        sprintf('B4 |Q_elastic| < 5|Q_heave|  |Qe|=%.3g |Qh|=%.3g', ...
        norm(Qg(7:end)), abs(Qg(3))), ...
        n_pass, n_fail, logs);

    fprintf('\nCase B result: %d PASS, %d FAIL\n', n_pass, n_fail);
    for i = 1:numel(logs), fprintf('  %s\n', logs{i}); end

    report = struct('name', 'CaseB', 'n_pass', n_pass, 'n_fail', n_fail, ...
        'ok', n_fail == 0, 'logs', {logs}, 'Qg', Qg);
    if report.ok, fprintf('Case B: OK\n'); else, fprintf('Case B: FAILED\n'); end
end
