function out = verify_cruise_trim(varargin)
%VERIFY_CRUISE_TRIM  Open-loop cruise smoke with trim IC + throttle/tilt=0.
%
%  out = verify_cruise_trim
%  out = verify_cruise_trim('StopTime', 2)

    p = inputParser;
    addParameter(p, 'StopTime', 2.0, @isnumeric);
    addParameter(p, 'retrim', false, @islogical);
    parse(p, varargin{:});
    Tstop = p.Results.StopTime;

    root = fileparts(fileparts(mfilename('fullpath')));
    addpath(root);
    addpath(fullfile(root, 'trim'));
    addpath(fullfile(root, 'thrust'));
    addpath(fullfile(root, 'tests'));
    cd(root);

    if p.Results.retrim || ~evalin('base', 'exist(''trim_cruise_report'',''var'')')
        f = fullfile(root, 'trim', 'last_cruise_trim.mat');
        if isfile(f) && ~p.Results.retrim
            S = load(f, 'report');
            report = S.report;
            assignin('base', 'trim_cruise_report', report);
        else
            report = trim_cruise_static();
        end
    else
        report = evalin('base', 'trim_cruise_report');
    end

    ic_info = apply_cruise_trim_ic(report);
    thr = ic_info.throttle(:).';

    mdl = 'aerodynamic_model';
    ts_thr = timeseries(repmat(thr, 2, 1), [0; Tstop]);
    ts_tilt = timeseries(zeros(2, 8), [0; Tstop]);
    ts_tdot = timeseries(zeros(2, 8), [0; Tstop]);
    ds = Simulink.SimulationData.Dataset;
    ds = ds.addElement(ts_thr, 'Throttle');
    ds = ds.addElement(ts_tilt, 'Tilt_cmd');
    ds = ds.addElement(ts_tdot, 'dotTilt_cmd');
    assignin('base', 'ds_cruise_trim', ds);

    fprintf('sim %s  T=%.3g s  cruise trim  tilt=0  mode=%s\n', ...
        mdl, Tstop, report.mode);

    load_system(mdl);
    ss = [mdl '/Flexible Aircraft Dynamics/Structure_SS'];
    tf = [mdl '/Propulsion Dynamics/Turbofan Engine System1'];
    x0_prev = get_param(ss, 'X0');
    ic_prev = get_param(tf, 'IC');
    tau_prev = get_param(tf, 'tau');
    set_param(ss, 'X0', 'x0_structure');
    set_param(tf, 'IC', num2str(ic_info.Fmax, 16));
    set_param(tf, 'tau', '0.01');
    cleanup = onCleanup(@() restore_trim_params_(ss, x0_prev, tf, ic_prev, tau_prev)); %#ok<NASGU>
    fprintf('  turbofan IC=%g (uninstalled Fmax; Nt applied inside), tau=0.01 (was %s)\n', ...
        ic_info.Fmax, tau_prev);

    out = evalin('base', sprintf([ ...
        'sim(''%s'', ''StopTime'', ''%g'', ' ...
        '''LoadExternalInput'', ''on'', ''ExternalInput'', ''ds_cruise_trim'', ' ...
        '''SaveOutput'', ''on'', ''ReturnWorkspaceOutputs'', ''on'', ' ...
        '''SaveFormat'', ''Dataset'')'], mdl, Tstop));

    yout = out.yout;
    t = out.tout(:);
    Q = []; eta_e = []; Vb = []; Alpha = [];
    fprintf('yout n=%d\n', yout.numElements);
    for i = 1:yout.numElements
        Xi = squeeze(yout.getElement(i).Values.Data);
        nm = yout.getElement(i).Name;
        fprintf('  [%2d] %-24s  %s  finite=%d\n', i, nm, mat2str(size(Xi)), all(isfinite(Xi(:))));
        if contains(nm, 'Q') && size(Xi, 2) == 21
            Q = Xi;
        elseif contains(nm, 'elastic_displacement')
            eta_e = Xi;
        elseif contains(nm, 'Velocity_b')
            Vb = Xi;
        elseif strcmp(nm, 'Alpha') || contains(nm, 'Alpha')
            Alpha = Xi(:);
        end
    end

    assert(~isempty(Q) && all(isfinite(Q(:))), 'verify_cruise: Q bad');
    assert(~isempty(eta_e) && all(isfinite(eta_e(:))), 'verify_cruise: eta bad');

    a0 = rad2deg(report.alpha_rad);
    if ~isempty(Alpha)
        a_end = rad2deg(Alpha(end));
    else
        a_end = NaN;
    end
    if ~isempty(Vb)
        V_end = Vb(end, :);
        a_from_V = rad2deg(atan2(V_end(3), V_end(1)));
    else
        V_end = [NaN NaN NaN];
        a_from_V = NaN;
    end

    fprintf('t(end)=%.3g s\n', t(end));
    fprintf('  |Q(end)|_2=%.4g  Q_rigid(end)=%s\n', norm(Q(end, :)), mat2str(Q(end, 1:6), 3));
    fprintf('  |eta_e(end)|_2=%.4g\n', norm(eta_e(end, :)));
    fprintf('  Alpha: trim=%.3fdeg  end=%.3fdeg  from V_b=%.3fdeg\n', a0, a_end, a_from_V);
    fprintf('  V_b(end)=%s m/s\n', mat2str(V_end, 4));

    figure(31); clf;
    subplot(2, 1, 1);
    plot(t, Q(:, [1 3 5]));
    legend('Q_x', 'Q_h', 'Q_{pitch}');
    xlabel('t (s)'); ylabel('Q'); grid on;
    title('Cruise trim open-loop: modal GF');
    subplot(2, 1, 2);
    if ~isempty(Alpha)
        plot(t, rad2deg(Alpha)); ylabel('\alpha (deg)');
    elseif ~isempty(Vb)
        plot(t, rad2deg(atan2(Vb(:, 3), Vb(:, 1)))); ylabel('\alpha from V_b (deg)');
    end
    xlabel('t (s)'); grid on;
    title('Angle of attack');

    figure(32); clf;
    plot(t, eta_e);
    xlabel('t (s)'); ylabel('\eta_e'); grid on;
    title('Elastic modes (cruise trim IC)');

    out.trim_report = report;
    out.ic_info = ic_info;
    assignin('base', 'out_cruise_trim', out);
    fprintf('OK: cruise trim sim done; out_cruise_trim / figures 31-32\n');
end

function restore_trim_params_(ss, x0_prev, tf, ic_prev, tau_prev)
    try, set_param(ss, 'X0', x0_prev); catch, end
    try, set_param(tf, 'IC', ic_prev); catch, end
    try, set_param(tf, 'tau', tau_prev); catch, end
end
