function out = plot_cruise_response(varargin)
%PLOT_CRUISE_RESPONSE  Rigid V/attitude + first-5 elastic eta / etadot curves.
%
%  out = plot_cruise_response
%  out = plot_cruise_response('StopTime', 2)

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

    wire_response_outs();

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

    load_system(mdl);
    ss = [mdl '/Flexible Aircraft Dynamics/Structure_SS'];
    tf = [mdl '/Propulsion Dynamics/Turbofan Engine System1'];
    x0_prev = get_param(ss, 'X0');
    ic_prev = get_param(tf, 'IC');
    tau_prev = get_param(tf, 'tau');
    set_param(ss, 'X0', 'x0_structure');
    set_param(tf, 'IC', num2str(ic_info.Fmax, 16));
    set_param(tf, 'tau', '0.01');
    cleanup = onCleanup(@() restore_params_(ss, x0_prev, tf, ic_prev, tau_prev)); %#ok<NASGU>

    fprintf('sim %s  T=%.3g s for response plots\n', mdl, Tstop);
    out = evalin('base', sprintf([ ...
        'sim(''%s'', ''StopTime'', ''%g'', ' ...
        '''LoadExternalInput'', ''on'', ''ExternalInput'', ''ds_cruise_trim'', ' ...
        '''SaveOutput'', ''on'', ''ReturnWorkspaceOutputs'', ''on'', ' ...
        '''SaveFormat'', ''Dataset'')'], mdl, Tstop));

    t = out.tout(:);
    yout = out.yout;
    data = struct();
    for i = 1:yout.numElements
        nm = yout.getElement(i).Name;
        Xi = squeeze(yout.getElement(i).Values.Data);
        data.(matlab.lang.makeValidName(nm)) = Xi;
        fprintf('  yout %-28s  %s\n', nm, mat2str(size(Xi)));
    end

    Vb = pick_(data, {'Velocity_b', 'Out_Vb', 'Vb'});
    Wb = pick_(data, {'Omega_b', 'Out_Wb', 'Wb'});
    eta_r = pick_(data, {'rigid_m_displacement', 'Out_eta_r', 'eta_r'});
    eta_e = pick_(data, {'elastic_displacement', 'Out_eta_e', 'eta_e'});
    etad_e = pick_(data, {'elastic_m_velocity', 'Out_etad_e', 'etad_e'});
    Alpha = pick_(data, {'Alpha', 'Out_Alpha'});

    % Attitude ~ body [phi; theta; psi] from T_m2body * eta (same as old Alltitude_b)
    Tm = evalin('base', 'T_m2body');
    n = size(eta_e, 1);
    att = zeros(n, 3);
    pos = zeros(n, 3);
    if isempty(eta_r)
        eta_r = zeros(n, 6);
    end
    for k = 1:n
        eta = [eta_r(k, :).'; eta_e(k, :).'];
        nu = Tm * eta;
        pos(k, :) = nu(1:3).';
        att(k, :) = nu(4:6).';
    end

    %% Figure 41: rigid velocity + attitude
    figure(41); clf;
    subplot(2, 1, 1);
    plot(t, Vb(:, 1), t, Vb(:, 2), t, Vb(:, 3));
    if ~isempty(Wb)
        hold on;
        plot(t, Wb(:, 1), '--', t, Wb(:, 2), '--', t, Wb(:, 3), '--');
        legend('u', 'v', 'w', 'p', 'q', 'r', 'Location', 'best');
    else
        legend('u', 'v', 'w', 'Location', 'best');
    end
    xlabel('t (s)'); ylabel('m/s or rad/s'); grid on;
    title('Rigid-body velocity (body): V_b and \Omega_b');

    subplot(2, 1, 2);
    plot(t, rad2deg(att(:, 1)), t, rad2deg(att(:, 2)), t, rad2deg(att(:, 3)));
    hold on;
    if ~isempty(Alpha)
        plot(t, rad2deg(Alpha(:)), 'k:', 'LineWidth', 1.2);
        legend('\phi', '\theta', '\psi', '\alpha', 'Location', 'best');
    else
        legend('\phi', '\theta', '\psi', 'Location', 'best');
    end
    xlabel('t (s)'); ylabel('deg'); grid on;
    title('Attitude from T_{m2body}\eta (Alltitude_b) + \alpha');

    %% Figure 42: first 5 elastic modes
    ne = min(5, size(eta_e, 2));
    figure(42); clf;
    subplot(2, 1, 1);
    plot(t, eta_e(:, 1:ne));
    leg = arrayfun(@(i) sprintf('\\eta_{e%d}', i), 1:ne, 'UniformOutput', false);
    legend(leg, 'Location', 'best');
    xlabel('t (s)'); ylabel('\eta_e'); grid on;
    title(sprintf('Elastic modal displacement (first %d)', ne));

    subplot(2, 1, 2);
    if ~isempty(etad_e)
        plot(t, etad_e(:, 1:ne));
        legv = arrayfun(@(i) sprintf('\\etȧ_{e%d}', i), 1:ne, 'UniformOutput', false);
        legend(legv, 'Location', 'best');
    else
        text(0.1, 0.5, 'elastic\_m\_velocity not in yout');
    end
    xlabel('t (s)'); ylabel('\etȧ_e'); grid on;
    title(sprintf('Elastic modal velocity (first %d)', ne));

    out.t = t;
    out.Velocity_b = Vb;
    out.Omega_b = Wb;
    out.attitude_rad = att;
    out.attitude_deg = rad2deg(att);
    out.Position_b = pos;
    out.eta_e = eta_e;
    out.etadot_e = etad_e;
    out.Alpha = Alpha;
    out.trim_report = report;
    assignin('base', 'out_cruise_response', out);

    mat_path = fullfile(root, 'trim', 'last_cruise_response.mat');
    save(mat_path, 'out', 't', 'Vb', 'Wb', 'att', 'eta_e', 'etad_e', 'Alpha');

    fig_dir = fullfile(root, 'trim', 'figures');
    if ~exist(fig_dir, 'dir'), mkdir(fig_dir); end
    saveas(41, fullfile(fig_dir, 'rigid_V_attitude.png'));
    saveas(42, fullfile(fig_dir, 'elastic_modes_1to5.png'));
    fprintf('Saved %s ; figures 41-42 (+PNG) ; out_cruise_response in base\n', mat_path);
end

function X = pick_(data, names)
    X = [];
    fn = fieldnames(data);
    for i = 1:numel(names)
        for j = 1:numel(fn)
            if contains(fn{j}, names{i}, 'IgnoreCase', true)
                X = data.(fn{j});
                if isvector(X), X = X(:); end
                return
            end
        end
    end
end

function restore_params_(ss, x0_prev, tf, ic_prev, tau_prev)
    try, set_param(ss, 'X0', x0_prev); catch, end
    try, set_param(tf, 'IC', ic_prev); catch, end
    try, set_param(tf, 'tau', tau_prev); catch, end
end
