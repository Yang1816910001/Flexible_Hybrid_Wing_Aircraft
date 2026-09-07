function ic_info = apply_hover_trim_ic(report)
%APPLY_HOVER_TRIM_IC  Write hover trim into bus IC + x0_structure (+ Wagner).
%
%  report from trim_hover_static (or load last_hover_trim.mat).

    if nargin < 1 || isempty(report)
        f = fullfile(fileparts(mfilename('fullpath')), 'last_hover_trim.mat');
        if ~isfile(f)
            report = trim_hover_static();
        else
            S = load(f, 'report');
            report = S.report;
        end
    end

    root = fileparts(fileparts(mfilename('fullpath')));
    addpath(root);
    addpath(fullfile(root, 'tests'));
    ensure_aero_workspace();

    Tm = evalin('base', 'T_m2body');
    n_modes = size(Tm, 2);
    V_b = zeros(3, 1);
    omega_b = zeros(3, 1);
    etadot = zeros(n_modes, 1);
    eta = zeros(n_modes, 1);
    x0_structure = [eta; etadot];

    mdl = 'aerodynamic_model';
    load_system(mdl);
    tf = [mdl '/Propulsion Dynamics/Turbofan Engine System1'];
    Fmax = str2double(get_param(tf, 'Fmax'));
    if ~(isfinite(Fmax) && Fmax > 0)
        Fmax = 45000;
    end
    Nt = str2double(get_param(tf, 'Nt'));
    if ~(isfinite(Nt) && Nt > 0)
        Nt = 1;
    end
    % Installed Tmax at pinned prop_Mach (hover: 0). Not bus |V|/a.
    prop_Mach = 0;
    [Tmax, tnd] = turbofan_tmax(Fmax, Nt, prop_Mach);
    thr = report.T_eng_N(:) / Tmax;
    thr = min(max(thr, 0), 1);
    assignin('base', 'prop_Mach', prop_Mach);

    ic = evalin('base', 'DynamicsAndFlexibleStates_IC');
    ic.Velocity_b = V_b;
    ic.Omega_b = omega_b;
    ic.Alpha = 0;
    ic.Beta = 0;
    ic.mach = 0;
    ic.Q = report.Q(:);
    ic.rigid_m_displacement = eta(1:6);
    ic.rigid_m_velocity = etadot(1:6);
    ic.elastic_displacement = eta(7:end);
    ic.elastic_m_velocity = etadot(7:end);

    % Wagner / alpha_du at zero inflow (CL=0)
    names = {'wing_R', 'wing_L', 'v_tail_R', 'v_tail_L'};
    qS_bak = struct();
    for k = 1:4
        nm = names{k};
        Ad = evalin('base', ['Ad_' nm]);
        Bd = evalin('base', ['Bd_' nm]);
        ns = size(Bd, 2);
        u0 = zeros(ns, 1);
        x0w = (eye(size(Ad, 1)) - Ad) \ (Bd * u0);
        assignin('base', ['x0_wagner_' nm], x0w);
        if evalin('base', sprintf('exist(''G_du_%s'',''var'')', nm))
            Gdu = evalin('base', ['G_du_' nm]);
            assignin('base', ['alpha_du0_' nm], Gdu * u0);
        end
        % Freeze cruise qbar out of hover (workspace U still cruise).
        qn = ['qS_strip_' nm];
        qcn = ['qSc_strip_' nm];
        qS_bak.(nm) = evalin('base', qn);
        assignin('base', qn, zeros(size(qS_bak.(nm))));
        if evalin('base', sprintf('exist(''%s'',''var'')', qcn))
            qS_bak.([nm '_c']) = evalin('base', qcn);
            assignin('base', qcn, zeros(size(qS_bak.([nm '_c']))));
        end
    end
    assignin('base', 'trim_hover_qS_backup', qS_bak);

    assignin('base', 'x0_structure', x0_structure);
    assignin('base', 'DynamicsAndFlexibleStates_IC', ic);
    assignin('base', 'trim_hover_throttle', thr);
    assignin('base', 'trim_hover_Fmax', Fmax);
    assignin('base', 'trim_hover_Nt', Nt);
    assignin('base', 'trim_hover_Tmax', Tmax);
    assignin('base', 'trim_hover_Tmax_sls', Fmax * Nt);
    assignin('base', 'trim_hover_T_eng_N', report.T_eng_N(:));
    assignin('base', 'trim_hover_Velocity_b', V_b);
    if isfield(report, 'tilt_rad')
        tilt = report.tilt_rad(:);
    else
        tilt = (pi / 2) * ones(8, 1);
    end
    assignin('base', 'trim_hover_tilt_rad', tilt);

    ms = [mdl '/Propulsion Dynamics/Manual Switch'];
    try
        set_param(ms, 'CurrentSetting', '0');
    catch
    end

    ic_info = struct();
    ic_info.x0_structure = x0_structure;
    ic_info.etadot_rigid = etadot(1:6);
    ic_info.V_b = V_b;
    ic_info.throttle = thr;
    ic_info.Fmax = Fmax;
    ic_info.Nt = Nt;
    ic_info.prop_Mach = prop_Mach;
    ic_info.Tmax = Tmax;
    ic_info.Tmax_sls = Fmax * Nt;
    ic_info.T_eng_N = report.T_eng_N(:);
    ic_info.tilt_rad = tilt;
    ic_info.alpha_deg = 0;
    ic_info.Q_pitch = report.Q_pitch;
    ic_info.tf_block = tf;

    fprintf('Applied hover trim IC:\n');
    fprintf('  V_b=0  alpha=0  tilt=pi/2\n');
    fprintf('  T_eng=%s N\n', mat2str(report.T_eng_N.', 4));
    fprintf('  throttle=T/Tmax(M)  Fmax=%.0f Nt=%.2f M=%.3f tnd=%.3f Tmax=%.0f: %s\n', ...
        Fmax, Nt, prop_Mach, tnd, Tmax, mat2str(thr.', 4));
    fprintf('  Delay IC + Wagner x0/alpha_du0 updated; x0_structure in base\n');
    fprintf('  static Qpitch=%.3g  Qh=%.3g\n', report.Q_pitch, report.Q(3));
end
