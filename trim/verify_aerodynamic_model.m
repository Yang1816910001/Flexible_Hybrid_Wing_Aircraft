function out = verify_aerodynamic_model(varargin)
%VERIFY_AERODYNAMIC_MODEL  开环跑 aerodynamic_model.slx
%
%  默认：悬停静态配平 IC（前后推力、V=0、tilt=π/2、qS=0）。
%  打印/作图：姿态、模态、发动机推力（不输出 Q）。
%  油门=0 的自由落体用 verify_aerodynamic_model('UseTrim', false)。

    p = inputParser;
    addParameter(p, 'StopTime', 1.5, @isnumeric);
    addParameter(p, 'UseTrim', true, @islogical);
    parse(p, varargin{:});
    Tstop = p.Results.StopTime;
    use_trim = p.Results.UseTrim;

    proj_dir = aeroelastic.proj_root;
    aeroelastic.setup_paths();
    cd(proj_dir);

    if ~evalin('base', 'exist(''As'',''var'')') || ...
            ~evalin('base', 'exist(''DynamicsAndFlexibleStates'',''var'')')
        assignin('base', 'do_verify', false);
        evalin('base', 'initial_main');
    end
    cd(proj_dir);

    mdl = 'aerodynamic_model';
    slx = fullfile(proj_dir, [mdl '.slx']);
    if ~isfile(slx)
        error('verify_aerodynamic_model:Missing', 'no %s', slx);
    end

    wire_response_outs();
    load_system(mdl);

    ss = [mdl '/Flexible Aircraft Dynamics/Structure_SS'];
    tf = [mdl '/Propulsion Dynamics/Turbofan Engine System1'];
    x0_prev = get_param(ss, 'X0');
    ic_prev = get_param(tf, 'IC');
    tau_prev = get_param(tf, 'tau');
    cleanup = onCleanup(@() restore_verify_run_(ss, x0_prev, tf, ic_prev, tau_prev)); %#ok<NASGU>

    if use_trim
        report = load_hover_trim_();
        ic_info = apply_hover_trim_ic(report);
        thr = ic_info.throttle(:).';
        tilt = ic_info.tilt_rad(:).';
        set_param(ss, 'X0', 'x0_structure');
        set_param(tf, 'IC', num2str(ic_info.Fmax, 16));
        set_param(tf, 'tau', '0.01');
        fprintf('sim %s  T=%.3g s  hover trim  tilt=pi/2  Tfront=%.0f N  Taft=%.0f N\n', ...
            mdl, Tstop, report.T_front_N, report.T_aft_N);
        ds_name = 'ds_hover_trim';
    else
        thr = zeros(1, 8);
        tilt = (pi / 2) * ones(1, 8);
        fprintf('sim %s  T=%.3g s  tilt=pi/2  throttle=0 (no trim)\n', mdl, Tstop);
        ds_name = 'ds_aero_verify';
    end

    ts_thr = timeseries(repmat(thr, 2, 1), [0; Tstop]);
    ts_tilt = timeseries(repmat(tilt, 2, 1), [0; Tstop]);
    ts_tdot = timeseries(zeros(2, 8), [0; Tstop]);
    ds = Simulink.SimulationData.Dataset;
    ds = ds.addElement(ts_thr, 'Throttle');
    ds = ds.addElement(ts_tilt, 'Tilt_cmd');
    ds = ds.addElement(ts_tdot, 'dotTilt_cmd');
    assignin('base', ds_name, ds);

    out = evalin('base', sprintf([ ...
        'sim(''%s'', ''StopTime'', ''%g'', ' ...
        '''LoadExternalInput'', ''on'', ''ExternalInput'', ''%s'', ' ...
        '''SaveOutput'', ''on'', ''ReturnWorkspaceOutputs'', ''on'', ' ...
        '''SaveFormat'', ''Dataset'')'], mdl, Tstop, ds_name));

    t = out.tout(:);
    yout = out.yout;
    data = struct();
    fprintf('yout n=%d\n', yout.numElements);
    for i = 1:yout.numElements
        nm = yout.getElement(i).Name;
        Xi = as_time_rows_(squeeze(yout.getElement(i).Values.Data), numel(t));
        key = matlab.lang.makeValidName(regexprep(nm, '[<>]', ''));
        data.(key) = Xi;
        if contains(nm, 'Q')
            continue
        end
        fprintf('  [%2d] %-24s  %s  finite=%d\n', i, nm, mat2str(size(Xi)), ...
            all(isfinite(Xi(:))));
    end

    Vb = pick_(data, {'Velocity_b', 'Out_Vb'});
    Wb = pick_(data, {'Omega_b', 'Out_Wb'});
    eta_r = pick_(data, {'rigid_m_displacement', 'Out_eta_r'});
    eta_e = pick_(data, {'elastic_displacement', 'Out_eta_e'});
    Teng = pick_(data, {'T_engine', 'Out_T_engine'});

    assert(~isempty(eta_e) && all(isfinite(eta_e(:))), 'verify: eta_e missing/NaN');
    n = numel(t);
    if isempty(eta_r)
        eta_r = zeros(n, 6);
    end
    att = attitude_from_eta_(eta_r, eta_e);

    Ve = row_end_(Vb);
    We = row_end_(Wb);
    Ae = att(end, :);
    Te = row_end_(Teng);
    spd = NaN;
    if ~isempty(Ve)
        spd = norm(Ve);
    end

    fprintf('t(end)=%.3g s\n', t(end));
    if ~isempty(Ve)
        fprintf('  V_b [u v w]     = %s m/s   |V|=%.3g\n', mat2str(Ve, 3), spd);
    end
    if ~isempty(We)
        fprintf('  Omega_b [p q r] = %s rad/s\n', mat2str(We, 3));
    end
    fprintf('  att [phi theta psi] = %s deg\n', mat2str(rad2deg(Ae), 3));
    if ~isempty(Ve) && spd >= 0.05
        a_flow = atan2d(Ve(3), Ve(1));
        fprintf('  flow atan2(w,u)    = %.1f deg  (not pitch; theta=%.2f deg)\n', ...
            a_flow, rad2deg(Ae(2)));
    else
        fprintf('  flow atan2(w,u)    = n/a  (|V| too small)\n');
    end
    ne = min(5, size(eta_e, 2));
    fprintf('  eta_e(1:%d)        = %s\n', ne, mat2str(eta_e(end, 1:ne), 3));
    if ~isempty(Te)
        fprintf('  T_engine          = %s N\n', mat2str(Te, 4));
    else
        warning('verify_aerodynamic_model:NoThrust', 'T_engine not in yout');
    end
    if use_trim
        fprintf('  (open-loop hover is not attitude-stable; ~1-2 s hold only)\n');
    end

    if use_trim
        ttl = sprintf('Hover trim open-loop  |  T_{front}=%.0f N  T_{aft}=%.0f N', ...
            report.T_front_N, report.T_aft_N);
    else
        ttl = 'Open-loop  |  throttle=0  |  tilt=\pi/2';
    end

    figure(21); clf;
    if ~isempty(Vb)
        plot(t, Vb(:, 1), t, Vb(:, 2), t, Vb(:, 3), 'LineWidth', 1.2);
        legend('u', 'v', 'w', 'Location', 'best');
    end
    grid on; xlabel('t (s)'); ylabel('m/s');
    title([ttl '  —  V_b']);

    figure(22); clf;
    if ~isempty(Wb)
        plot(t, Wb(:, 1), t, Wb(:, 2), t, Wb(:, 3), 'LineWidth', 1.2);
        legend('p', 'q', 'r', 'Location', 'best');
    end
    grid on; xlabel('t (s)'); ylabel('rad/s');
    title([ttl '  —  \Omega_b']);

    figure(23); clf;
    plot(t, rad2deg(att(:, 1)), t, rad2deg(att(:, 2)), t, rad2deg(att(:, 3)), ...
        'LineWidth', 1.2);
    legend('\phi', '\theta', '\psi', 'Location', 'best');
    grid on; xlabel('t (s)'); ylabel('deg');
    title('Attitude  (T_{m2body} \eta)');

    figure(24); clf;
    plot(t, eta_e(:, 1:ne), 'LineWidth', 1.2);
    leg = arrayfun(@(i) sprintf('\\eta_{e%d}', i), 1:ne, 'UniformOutput', false);
    legend(leg, 'Location', 'best');
    grid on; xlabel('t (s)'); ylabel('\eta_e');
    title('Elastic modes');

    figure(25); clf;
    if ~isempty(Teng)
        ncol = size(Teng, 2);
        nF = min(4, ncol);
        Tf = mean(Teng(:, 1:nF), 2);
        if ncol > 4
            Ta = mean(Teng(:, 5:ncol), 2);
        else
            Ta = Tf;
        end
        subplot(2, 1, 1);
        plot(t, Tf, 'LineWidth', 1.6);
        hold on;
        if use_trim
            yline(report.T_front_N, '--', 'Color', [0.3 0.3 0.3]);
            legend('front mean (E1–E4)', 'trim', 'Location', 'best');
        else
            legend('front mean (E1–E4)', 'Location', 'best');
        end
        grid on; ylabel('T (N)');
        title('Engine thrust');
        subplot(2, 1, 2);
        plot(t, Ta, 'LineWidth', 1.6);
        hold on;
        if use_trim
            yline(report.T_aft_N, '--', 'Color', [0.3 0.3 0.3]);
            legend('aft mean (E5–E8)', 'trim', 'Location', 'best');
        else
            legend('aft mean (E5–E8)', 'Location', 'best');
        end
        grid on; xlabel('t (s)'); ylabel('T (N)');
    else
        text(0.1, 0.5, 'T_{engine} not in yout');
    end

    fprintf('OK: V / Omega / attitude / modes / thrust  (figures 21-25)\n');
end

function report = load_hover_trim_()
    f = fullfile(fileparts(mfilename('fullpath')), 'last_hover_trim.mat');
    if evalin('base', 'exist(''trim_hover_report'',''var'')')
        report = evalin('base', 'trim_hover_report');
        return
    end
    if isfile(f)
        S = load(f, 'report');
        report = S.report;
    else
        report = trim_hover_static('mode', 'pitch2');
    end
    assignin('base', 'trim_hover_report', report);
end

function att = attitude_from_eta_(eta_r, eta_e)
    n = size(eta_e, 1);
    att = zeros(n, 3);
    if ~evalin('base', 'exist(''T_m2body'',''var'')')
        return
    end
    Tm = evalin('base', 'T_m2body');
    for k = 1:n
        nu = Tm * [eta_r(k, :).'; eta_e(k, :).'];
        att(k, :) = nu(4:6).';
    end
end

function X = as_time_rows_(X, nt)
    if isempty(X)
        return
    end
    if isvector(X)
        X = X(:);
        return
    end
    if size(X, 1) == nt
        return
    end
    if size(X, 2) == nt
        X = X.';
    end
end

function y = pick_(data, names)
    y = [];
    fns = fieldnames(data);
    for i = 1:numel(names)
        want = matlab.lang.makeValidName(names{i});
        if isfield(data, want) && ~isempty(data.(want))
            y = data.(want);
            return
        end
        for j = 1:numel(fns)
            if strcmpi(fns{j}, want) || contains(fns{j}, want, 'IgnoreCase', true)
                y = data.(fns{j});
                return
            end
        end
    end
end

function r = row_end_(X)
    r = [];
    if isempty(X)
        return
    end
    r = X(end, :);
end

function restore_verify_run_(ss, x0_prev, tf, ic_prev, tau_prev)
    try, set_param(ss, 'X0', x0_prev); catch, end
    try, set_param(tf, 'IC', ic_prev); catch, end
    try, set_param(tf, 'tau', tau_prev); catch, end
    if evalin('base', 'exist(''trim_hover_qS_backup'',''var'')')
        bak = evalin('base', 'trim_hover_qS_backup');
        names = {'wing_R', 'wing_L', 'v_tail_R', 'v_tail_L'};
        for k = 1:4
            nm = names{k};
            if isfield(bak, nm)
                assignin('base', ['qS_strip_' nm], bak.(nm));
            end
            if isfield(bak, [nm '_c'])
                assignin('base', ['qSc_strip_' nm], bak.([nm '_c']));
            end
        end
    end
end
