function ic_info = apply_cruise_trim_ic(report)
%APPLY_CRUISE_TRIM_IC  Write cruise trim into bus IC + x0_structure (+ Wagner).
%
%  report from trim_cruise_static (or load last_cruise_trim.mat).

    if nargin < 1 || isempty(report)
        f = fullfile(fileparts(mfilename('fullpath')), 'last_cruise_trim.mat');
        if ~isfile(f)
            report = trim_cruise_static();
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
    V_b = report.pack.V_b(:);
    if isfield(report.pack, 'Omega_b') && ~isempty(report.pack.Omega_b)
        omega_b = report.pack.Omega_b(:);
    elseif isfield(report, 'q0')
        omega_b = [0; report.q0; 0];
    else
        omega_b = zeros(3, 1);
    end
    etadot = zeros(n_modes, 1);
    etadot(1:6) = Tm(:, 1:6) \ [V_b; omega_b];
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
    Tmax_sls = Fmax * Nt;
    if isfield(report, 'mach') && ~isempty(report.mach)
        prop_Mach = report.mach;
    else
        prop_Mach = hypot(V_b(1), hypot(V_b(2), V_b(3))) / 340.3;
    end
    [Tmax, tnd] = turbofan_tmax(Fmax, Nt, prop_Mach);
    thr = report.T_eng_N(:) / Tmax;
    thr = min(max(thr, 0), 1);
    assignin('base', 'prop_Mach', prop_Mach);

    ic = evalin('base', 'DynamicsAndFlexibleStates_IC');
    ic.Velocity_b = V_b;
    ic.Omega_b = omega_b;
    ic.Alpha = report.alpha_rad;
    ic.Beta = 0;
    ic.mach = report.mach;
    ic.Q = report.Q(:);
    ic.rigid_m_displacement = eta(1:6);
    ic.rigid_m_velocity = etadot(1:6);
    ic.elastic_displacement = eta(7:end);
    ic.elastic_m_velocity = etadot(7:end);

    % Wagner / alpha_du IC at trim local alphas (quasi-steady)
    polar_a = evalin('base', 'polar_alpha_deg');
    polar_CL = evalin('base', 'polar_CL');
    local_a = report.pack.local_alpha;
    names = {'wing_R', 'wing_L', 'v_tail_R', 'v_tail_L'};
    for k = 1:4
        nm = names{k};
        CL = interp1(polar_a, polar_CL, rad2deg(local_a(k)), 'linear', 'extrap');
        Ad = evalin('base', ['Ad_' nm]);
        Bd = evalin('base', ['Bd_' nm]);
        ns = size(Bd, 2);
        u0 = CL * ones(ns, 1);
        x0w = (eye(size(Ad, 1)) - Ad) \ (Bd * u0);
        assignin('base', ['x0_wagner_' nm], x0w);
        if evalin('base', sprintf('exist(''G_du_%s'',''var'')', nm))
            Gdu = evalin('base', ['G_du_' nm]);
            assignin('base', ['alpha_du0_' nm], Gdu * u0);
        end
    end

    assignin('base', 'x0_structure', x0_structure);
    assignin('base', 'DynamicsAndFlexibleStates_IC', ic);
    assignin('base', 'trim_cruise_throttle', thr);
    assignin('base', 'trim_cruise_Fmax', Fmax);
    assignin('base', 'trim_cruise_T_eng_N', report.T_eng_N(:));
    assignin('base', 'trim_cruise_alpha_rad', report.alpha_rad);
    assignin('base', 'trim_cruise_Velocity_b', V_b);

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
    ic_info.Tmax_sls = Tmax_sls;
    ic_info.T_eng_N = report.T_eng_N(:);
    ic_info.alpha_deg = report.alpha_deg;
    ic_info.Q_pitch = report.Q_pitch;
    ic_info.tf_block = tf;

    fprintf('Applied cruise trim IC:\n');
    fprintf('  alpha=%.4f deg  V_b=%s\n', report.alpha_deg, mat2str(V_b.', 4));
    fprintf('  etadot_rigid=%s\n', mat2str(etadot(1:6).', 4));
    fprintf('  T_eng=%s N\n', mat2str(report.T_eng_N.', 4));
    fprintf('  throttle=T/Tmax(M)  Fmax=%.0f Nt=%.2f M=%.3f tnd=%.3f Tmax=%.0f: %s\n', ...
        Fmax, Nt, prop_Mach, tnd, Tmax, mat2str(thr.', 4));
    fprintf('  Delay IC + Wagner x0/alpha_du0 updated; x0_structure in base\n');
    fprintf('  static Qpitch=%.3g\n', report.Q_pitch);
end
