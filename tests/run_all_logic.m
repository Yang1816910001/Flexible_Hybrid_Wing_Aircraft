function reports = run_all_logic
%RUN_ALL_LOGIC  跑逻辑验证 Case A–F（函数，避免 initial_main clearvars 清掉汇总）
%
%   reports = run_all_logic;

    root = fileparts(fileparts(mfilename('fullpath')));
    addpath(root);
    aeroelastic.setup_paths();
    cd(root);

    fprintf('\n########## LOGIC SUITE A–F ##########\n');
    t0 = tic;

    ensure_aero_workspace();

    reports = cell(6, 1);
    reports{1} = caseA_geometry_alpha();
    reports{2} = caseB_gravity();
    reports{3} = caseC_propulsion();
    reports{4} = caseD_aero_static();
    reports{5} = caseE_smoke_sim();
    reports{6} = caseF_magnitude();

    n_ok = 0;
    n_bad = 0;
    fprintf('\n########## SUMMARY ##########\n');
    for i = 1:numel(reports)
        r = reports{i};
        if r.ok
            n_ok = n_ok + 1;
            tag = 'OK  ';
        else
            n_bad = n_bad + 1;
            tag = 'FAIL';
        end
        fprintf('  %s  %s  (%d pass / %d fail)\n', tag, r.name, r.n_pass, r.n_fail);
    end
    fprintf('Elapsed %.1f s   cases OK=%d FAIL=%d\n', toc(t0), n_ok, n_bad);

    if n_bad > 0
        error('run_all_logic:Failed', '%d case(s) failed', n_bad);
    end
    fprintf('All logic cases passed.\n');
end
