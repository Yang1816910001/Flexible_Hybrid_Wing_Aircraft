function [res, pack] = cruise_sim_residual(x, ctx, opts)
%CRUISE_SIM_RESIDUAL  Short open-loop sim residual (same slx / Wagner).
%
%  x = [alpha_rad; T_tilt_N; T_front_lift_N]           (length 3)
%  or [alpha_rad; T_tilt_N; T_front_lift_N; q0_radps] (length 4)
%  opts.res_mode = 'Q' | 'rates' | 'mix'
%    mix -> [mean(Qx); mean(Qh); mean(Qpitch); mean(q)*scale]

    if nargin < 3, opts = struct(); end
    Tstop = getfield_def_(opts, 'StopTime', 0.15);
    Fmax = getfield_def_(opts, 'Fmax', 45000);
    res_mode = getfield_def_(opts, 'res_mode', 'Q');

    alpha = x(1);
    T_tilt = max(x(2), 0);
    T_Lf = max(x(3), 0);
    q0 = 0;
    if numel(x) >= 4, q0 = x(4); end

    T_eng = zeros(8, 1);
    T_eng(ctx.is_tilt) = T_tilt;
    T_eng(ctx.idx_lift_front) = T_Lf;

    U = ctx.U;
    V_b = U * [cos(alpha); 0; sin(alpha)];
    omega_b = [0; q0; 0];

    [Q_guess, pack0] = cruise_static_Q(alpha, T_eng, ctx);
    report = struct();
    report.alpha_rad = alpha;
    report.alpha_deg = rad2deg(alpha);
    report.T_eng_N = T_eng;
    report.Q = Q_guess;
    report.Q_pitch = Q_guess(5);
    report.mach = ctx.mach;
    report.pack = pack0;
    report.pack.V_b = V_b;
    report.pack.Omega_b = omega_b;
    report.q0 = q0;

    apply_cruise_trim_ic(report);
    % override omega in IC / structure x0 for nonzero q0
    if abs(q0) > 0
        Tm = evalin('base', 'T_m2body');
        etadot = zeros(size(Tm, 2), 1);
        etadot(1:6) = Tm(:, 1:6) \ [V_b; omega_b];
        x0 = [zeros(size(Tm, 2), 1); etadot];
        assignin('base', 'x0_structure', x0);
        ic = evalin('base', 'DynamicsAndFlexibleStates_IC');
        ic.Omega_b = omega_b;
        ic.rigid_m_velocity = etadot(1:6);
        assignin('base', 'DynamicsAndFlexibleStates_IC', ic);
    end

    Nt = getfield_def_(opts, 'Nt', 0.9);
    if isfield(ctx, 'mach') && ~isempty(ctx.mach)
        prop_Mach = ctx.mach;
    else
        prop_Mach = U / 340.3;
    end
    assignin('base', 'prop_Mach', prop_Mach);
    Tmax = turbofan_tmax(Fmax, Nt, prop_Mach);
    thr = min(max(T_eng / Tmax, 0), 1);
    mdl = 'aerodynamic_model';
    load_system(mdl);
    ss = [mdl '/Flexible Aircraft Dynamics/Structure_SS'];
    tf = [mdl '/Propulsion Dynamics/Turbofan Engine System1'];

    ds = Simulink.SimulationData.Dataset;
    ds = ds.addElement(timeseries(repmat(thr(:).', 2, 1), [0; Tstop]), 'Throttle');
    ds = ds.addElement(timeseries(zeros(2, 8), [0; Tstop]), 'Tilt_cmd');
    ds = ds.addElement(timeseries(zeros(2, 8), [0; Tstop]), 'dotTilt_cmd');
    assignin('base', 'ds_cruise_trim', ds);

    x0_prev = get_param(ss, 'X0');
    ic_prev = get_param(tf, 'IC');
    tau_prev = get_param(tf, 'tau');
    set_param(ss, 'X0', 'x0_structure');
    set_param(tf, 'IC', num2str(Fmax, 16));
    set_param(tf, 'tau', '0.01');
    try
        out = evalin('base', sprintf([ ...
            'sim(''%s'', ''StopTime'', ''%g'', ' ...
            '''LoadExternalInput'', ''on'', ''ExternalInput'', ''ds_cruise_trim'', ' ...
            '''SaveOutput'', ''on'', ''ReturnWorkspaceOutputs'', ''on'', ' ...
            '''SaveFormat'', ''Dataset'')'], mdl, Tstop));
    catch ME
        set_param(ss, 'X0', x0_prev);
        set_param(tf, 'IC', ic_prev);
        set_param(tf, 'tau', tau_prev);
        rethrow(ME);
    end
    set_param(ss, 'X0', x0_prev);
    set_param(tf, 'IC', ic_prev);
    set_param(tf, 'tau', tau_prev);

    t = out.tout(:);
    Vb = []; Wb = []; Alpha = []; Q = [];
    yout = out.yout;
    for i = 1:yout.numElements
        nm = yout.getElement(i).Name;
        Xi = squeeze(yout.getElement(i).Values.Data);
        if contains(nm, 'Velocity_b'), Vb = Xi;
        elseif contains(nm, 'Omega_b'), Wb = Xi;
        elseif contains(nm, 'Alpha') && ~contains(nm, 'elastic'), Alpha = Xi(:);
        elseif contains(nm, 'Q') && size(Xi, 2) == 21, Q = Xi;
        end
    end
    if isempty(Vb), error('cruise_sim_residual:NoVb', 'Velocity_b missing'); end
    if isempty(Alpha), Alpha = atan2(Vb(:, 3), Vb(:, 1)); end
    if isempty(Wb), Wb = zeros(size(Vb, 1), 3); end

    i0 = max(1, floor(0.5 * numel(t)));
    Qm = mean(Q(i0:end, [1 3 5]), 1).';
    du = (Vb(end, 1) - Vb(i0, 1)) / max(t(end) - t(i0), eps);
    dw = (Vb(end, 3) - Vb(i0, 3)) / max(t(end) - t(i0), eps);
    q_mean = mean(Wb(i0:end, 2));
    da = (Alpha(end) - Alpha(i0)) / max(t(end) - t(i0), eps);

    switch lower(res_mode)
        case 'q'
            res = Qm;
        case 'rates'
            res = [du; dw; q_mean];
        case 'mix'
            % scale q so comparable to Q (~ tens)
            res = [Qm; 50 * q_mean; 20 * da];
            if numel(x) < 4
                res = res(1:3); % drop extras if only 3 unk
            else
                res = res(1:4);
            end
        otherwise
            error('cruise_sim_residual:Mode', 'unknown res_mode');
    end

    pack = struct();
    pack.t = t; pack.Vb = Vb; pack.Wb = Wb; pack.Alpha = Alpha; pack.Q = Q;
    pack.T_eng = T_eng; pack.V_b0 = V_b; pack.omega_b0 = omega_b;
    pack.alpha = alpha; pack.q0 = q0;
    pack.report = report; pack.res = res;
end

function v = getfield_def_(s, name, default)
    if isfield(s, name), v = s.(name); else, v = default; end
end
