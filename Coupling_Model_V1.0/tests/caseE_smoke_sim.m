function report = caseE_smoke_sim
%CASEE_SMOKE_SIM  逻辑验证 E：整机开环冒烟（悬停配平短时）
%
% 断言输出有限；不要求长时间姿态稳定。

    ensure_aero_workspace;
    n_pass = 0; n_fail = 0; logs = {};
    fprintf('=== Case E: open-loop smoke sim ===\n');

    root = fileparts(fileparts(mfilename('fullpath')));
    cd(root);
    try
        out = verify_aerodynamic_model();
        ok_run = true;
        err_msg = '';
    catch ME
        ok_run = false;
        err_msg = ME.message;
        out = [];
    end
    [n_pass, n_fail, logs] = logic_check(ok_run, ...
        sprintf('E1 sim completes  %s', err_msg), n_pass, n_fail, logs);

    if ok_run
        yout = out.yout;
        finite_all = true;
        Qend = NaN; eta_end = NaN;
        for i = 1:yout.numElements
            Xi = squeeze(yout.getElement(i).Values.Data);
            finite_all = finite_all && all(isfinite(Xi(:)));
            nm = yout.getElement(i).Name;
            if contains(nm, 'Q') && size(Xi, 2) == 21
                Qend = norm(Xi(end, :));
            elseif contains(nm, 'elastic_displacement')
                eta_end = norm(Xi(end, :));
            end
        end
        [n_pass, n_fail, logs] = logic_check(finite_all, ...
            'E2 all yout finite', n_pass, n_fail, logs);
        [n_pass, n_fail, logs] = logic_check(isfinite(Qend) && Qend > 0, ...
            sprintf('E3 |Q(end)|_2=%.4g', Qend), n_pass, n_fail, logs);
        [n_pass, n_fail, logs] = logic_check(isfinite(eta_end), ...
            sprintf('E4 |eta_e(end)|_2=%.4g', eta_end), n_pass, n_fail, logs);
    end

    fprintf('\nCase E result: %d PASS, %d FAIL\n', n_pass, n_fail);
    for i = 1:numel(logs), fprintf('  %s\n', logs{i}); end

    report = struct('name', 'CaseE', 'n_pass', n_pass, 'n_fail', n_fail, ...
        'ok', n_fail == 0, 'logs', {logs});
    if report.ok, fprintf('Case E: OK\n'); else, fprintf('Case E: FAILED\n'); end
end
