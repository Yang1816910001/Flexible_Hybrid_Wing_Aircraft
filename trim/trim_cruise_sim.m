function report = trim_cruise_sim(varargin)
%TRIM_CRUISE_SIM  Cruise trim via short-horizon slx residual (Wagner path).
%
%  report = trim_cruise_sim('res_mode','mix','StopTime',0.25)
%  Unknowns (mix): alpha, T_tilt, T_front_lift, q0

    p = inputParser;
    addParameter(p, 'StopTime', 0.25, @isnumeric);
    addParameter(p, 'res_mode', 'mix', @(s) ischar(s) || isstring(s));
    addParameter(p, 'alpha0_deg', [], @(x) isempty(x) || isnumeric(x));
    addParameter(p, 'T_tilt0_N', [], @(x) isempty(x) || isnumeric(x));
    addParameter(p, 'T_lift0_N', [], @(x) isempty(x) || isnumeric(x));
    addParameter(p, 'q0', 0, @isnumeric);
    addParameter(p, 'tol', 1.0, @isnumeric);
    addParameter(p, 'max_iter', 12, @isnumeric);
    addParameter(p, 'write_base', true, @islogical);
    parse(p, varargin{:});

    root = fileparts(fileparts(mfilename('fullpath')));
    addpath(root); addpath(fullfile(root, 'trim'));
    addpath(fullfile(root, 'thrust')); addpath(fullfile(root, 'tests'));
    cd(root);

    mode = char(p.Results.res_mode);
    fprintf('=== Cruise SIM trim (res_mode=%s, T=%.3gs) ===\n', mode, p.Results.StopTime);
    wire_response_outs();
    ctx = trim_build_context();

    load_system('aerodynamic_model');
    tf = 'aerodynamic_model/Propulsion Dynamics/Turbofan Engine System1';
    Fmax = str2double(get_param(tf, 'Fmax'));
    if ~(isfinite(Fmax) && Fmax > 0), Fmax = 45000; end

    use_q0 = strcmpi(mode, 'mix');
    x = [deg2rad(-0.27); 80; 8280];
    fcache = fullfile(root, 'trim', 'last_cruise_static_pitch3.mat');
    if isfile(fcache)
        S = load(fcache, 'report');
        rs = S.report;
        x = [rs.alpha_rad; rs.T_tilt_N; rs.T_lift_front_N];
    end
    % Prefer last successful Q sim trim for mix start
    fsim = fullfile(root, 'trim', 'last_cruise_sim_trim.mat');
    if isfile(fsim)
        S = load(fsim, 'report');
        if isfield(S.report, 'T_lift_front_N') && contains(S.report.mode, 'Q')
            x = [S.report.alpha_rad; S.report.T_tilt_N; S.report.T_lift_front_N];
            fprintf('  init from last Q-sim trim\n');
        end
    end
    if ~isempty(p.Results.alpha0_deg), x(1) = deg2rad(p.Results.alpha0_deg); end
    if ~isempty(p.Results.T_tilt0_N), x(2) = p.Results.T_tilt0_N; end
    if ~isempty(p.Results.T_lift0_N), x(3) = p.Results.T_lift0_N; end
    if use_q0, x = [x(:); p.Results.q0]; end
    n = numel(x);

    opts = struct('StopTime', p.Results.StopTime, 'Fmax', Fmax, 'res_mode', mode);
    [fval, pack] = eval_quiet_(x, ctx, opts);
    best.x = x; best.f = fval; best.pack = pack; best.n = norm(fval);
    fprintf('  it 0  |res|=%.4g  n=%d  a=%.3f Tt=%.1f TLf=%.1f q0=%.3g\n', ...
        best.n, n, rad2deg(x(1)), x(2), x(3), tern_q0_(x));

    exitflag = 0; it = 0;
    for it = 1:p.Results.max_iter
        if best.n < p.Results.tol
            exitflag = 1; break
        end
        x = best.x; fval = best.f;
        epsv = [deg2rad(0.15); 100; 500];
        if n >= 4, epsv(4) = 0.05; end
        J = zeros(numel(fval), n);
        for j = 1:n
            xp = x; xp(j) = xp(j) + epsv(j);
            fj = eval_quiet_(xp, ctx, opts);
            J(:, j) = (fj - fval) / epsv(j);
        end
        if size(J, 1) ~= size(J, 2)
            dx = -pinv(J) * fval;
        else
            if rcond(J) < 1e-16
                warning('trim_cruise_sim:Singular', 'bad J'); exitflag = -1; break
            end
            dx = -J \ fval;
        end
        dx(1) = max(min(dx(1), deg2rad(2.5)), -deg2rad(2.5));
        dx(2) = max(min(dx(2), 1000), -1000);
        dx(3) = max(min(dx(3), 3500), -3500);
        if n >= 4, dx(4) = max(min(dx(4), 0.3), -0.3); end

        accepted = false;
        for ls = 1:8
            x_try = x + dx;
            x_try(2:3) = max(x_try(2:3), 0);
            [f_try, pack_try] = eval_quiet_(x_try, ctx, opts);
            if norm(f_try) < best.n
                best.x = x_try; best.f = f_try; best.pack = pack_try;
                best.n = norm(f_try); accepted = true; break
            end
            dx = 0.5 * dx;
        end
        fprintf('  it %d  |res|=%.4g  a=%.3f Tt=%.1f TLf=%.1f q0=%.3g  %s\n', ...
            it, best.n, rad2deg(best.x(1)), best.x(2), best.x(3), tern_q0_(best.x), ...
            tern_(accepted, 'ok', 'no-improve'));
        if ~accepted, exitflag = -2; break; end
    end
    if exitflag == 0 && best.n < 30, exitflag = 1;
    elseif exitflag == 0, exitflag = -2; end

    x = best.x; fval = best.f; pack = best.pack;
    T_eng = pack.T_eng;
    report = pack.report;
    report.name = 'trim_cruise_sim';
    report.mode = ['sim_' mode];
    report.ok = exitflag > 0;
    report.exitflag = exitflag;
    report.alpha_rad = x(1);
    report.alpha_deg = rad2deg(x(1));
    report.T_eng_N = T_eng;
    report.T_tilt_N = mean(T_eng(ctx.is_tilt));
    report.T_lift_front_N = mean(T_eng(ctx.idx_lift_front));
    report.T_lift_aft_N = mean(T_eng(ctx.idx_lift_aft));
    report.T_lift_N = mean(T_eng(~ctx.is_tilt));
    report.T_total_N = sum(T_eng);
    report.q0 = tern_q0_(x);
    report.res = fval;
    report.res_norm = norm(fval);
    report.pack = pack.report.pack;
    report.pack.V_b = pack.V_b0;
    if isfield(pack, 'omega_b0'), report.pack.Omega_b = pack.omega_b0; end
    report.sim_pack = pack;
    report.U = ctx.U; report.mach = ctx.mach;
    report.Altitude_m = ctx.Altitude_m; report.ctx_mg = ctx.mg;
    report.Fmax = Fmax; report.StopTime_trim = p.Results.StopTime;
    report.note = 'Short-horizon slx residual (Wagner/turbofan).';

    fprintf('\n  exit=%d |res|=%.4g a=%.4fdeg Tt=%.1f TLf=%.1f q0=%.4g\n', ...
        exitflag, report.res_norm, report.alpha_deg, report.T_tilt_N, ...
        report.T_lift_front_N, report.q0);
    if report.ok, fprintf('Cruise SIM trim: OK\n');
    else, fprintf('Cruise SIM trim: best-effort\n'); end

    save(fullfile(root, 'trim', 'last_cruise_sim_trim.mat'), 'report');
    save(fullfile(root, 'trim', 'last_cruise_trim.mat'), 'report');
    if p.Results.write_base
        assignin('base', 'trim_cruise_report', report);
        assignin('base', 'trim_cruise_alpha_rad', report.alpha_rad);
        assignin('base', 'trim_cruise_alpha_deg', report.alpha_deg);
        assignin('base', 'trim_cruise_T_eng_N', report.T_eng_N);
        assignin('base', 'trim_cruise_Velocity_b', report.pack.V_b);
    end
end

function [res, pack] = eval_quiet_(x, ctx, opts)
    evalc('[res, pack] = cruise_sim_residual(x, ctx, opts);');
end
function s = tern_(c, a, b), if c, s = a; else, s = b; end, end
function q = tern_q0_(x), if numel(x)>=4, q = x(4); else, q = 0; end, end
