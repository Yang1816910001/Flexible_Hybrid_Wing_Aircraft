function report = trim_cruise_static(varargin)
%TRIM_CRUISE_STATIC  Cruise static trim: freeze tilt=0, Newton FD (no toolbox).
%
%  report = trim_cruise_static
%  report = trim_cruise_static('mode','pitch3')   % default: close Fx, heave, pitch
%  report = trim_cruise_static('mode','tilt_only')
%  report = trim_cruise_static('mode','shared')

    p = inputParser;
    addParameter(p, 'mode', 'pitch3', @(s) ischar(s) || isstring(s));
    addParameter(p, 'alpha0_deg', 1.0, @isnumeric);
    addParameter(p, 'T0_N', 100, @isnumeric);
    addParameter(p, 'T_lift0_N', 8000, @isnumeric);
    addParameter(p, 'write_base', true, @islogical);
    addParameter(p, 'tol', 1e-5, @isnumeric);
    addParameter(p, 'max_iter', 40, @isnumeric);
    parse(p, varargin{:});
    mode = char(p.Results.mode);

    fprintf('=== Cruise static trim (tilt=0, mode=%s) ===\n', mode);
    ctx = trim_build_context();

    switch lower(mode)
        case {'tilt_only', 'shared'}
            x = [deg2rad(p.Results.alpha0_deg); p.Results.T0_N];
            n = 2;
        case 'pitch3'
            x = [deg2rad(p.Results.alpha0_deg); p.Results.T0_N; p.Results.T_lift0_N];
            n = 3;
        otherwise
            error('trim_cruise_static:Mode', 'unknown mode %s', mode);
    end

    fun = @(xx) cruise_static_residual(xx, ctx, mode);
    fprintf('  Newton FD solve (n=%d)...\n', n);
    exitflag = 0;
    fval = fun(x);
    it = 0;
    fprintf('  it 0  |res|=%.3e  x=%s\n', norm(fval), mat2str(x_disp_(x), 4));

    for it = 1:p.Results.max_iter
        if norm(fval) < p.Results.tol
            exitflag = 1;
            break
        end
        J = zeros(n, n);
        epsv = [1e-6; max(1, 1e-4 * abs(x(2)))];
        if n >= 3
            epsv(3) = max(1, 1e-4 * abs(x(3)));
        end
        for j = 1:n
            xp = x;
            xp(j) = xp(j) + epsv(j);
            J(:, j) = (fun(xp) - fval) / epsv(j);
        end
        if rcond(J) < 1e-14
            warning('trim_cruise_static:Singular', 'Jacobian nearly singular');
            exitflag = -1;
            break
        end
        dx = -J \ fval;
        for ls = 1:10
            x_try = x + dx;
            x_try(2:end) = max(x_try(2:end), 0);
            f_try = fun(x_try);
            if norm(f_try) < norm(fval) || ls == 10
                x = x_try;
                fval = f_try;
                break
            end
            dx = 0.5 * dx;
        end
        fprintf('  it %d  |res|=%.3e  x=%s\n', it, norm(fval), mat2str(x_disp_(x), 4));
    end
    if exitflag == 0 && norm(fval) < 1e-3
        exitflag = 1;
    elseif exitflag == 0
        exitflag = -2;
    end
    output = struct('iterations', it, 'method', 'newton_fd');

    [~, Q, pack] = cruise_static_residual(x, ctx, mode);
    alpha_deg = rad2deg(x(1));
    T_eng = pack.T_eng;

    report = struct();
    report.name = 'trim_cruise_static';
    report.mode = mode;
    report.ok = (exitflag > 0) && (norm(fval) < 1e-3);
    report.exitflag = exitflag;
    report.alpha_rad = x(1);
    report.alpha_deg = alpha_deg;
    report.T_eng_N = T_eng;
    report.T_tilt_N = mean(T_eng(ctx.is_tilt));
    report.T_lift_N = mean(T_eng(~ctx.is_tilt));
    report.T_lift_front_N = mean(T_eng(ctx.idx_lift_front));
    report.T_lift_aft_N = mean(T_eng(ctx.idx_lift_aft));
    report.T_total_N = sum(T_eng);
    report.Q = Q;
    report.Q_rigid = Q(1:6);
    report.res = fval;
    report.res_Fx_heave = fval(1:min(2, numel(fval)));
    report.res_norm = norm(fval);
    report.Q_pitch = Q(5);
    report.pack = pack;
    report.ctx_mg = ctx.mg;
    report.U = ctx.U;
    report.mach = ctx.mach;
    report.Altitude_m = ctx.Altitude_m;
    report.fsolve_output = output;
    report.note = [ ...
        'Static strip aero (L+D+CM) + gravity + prop. ' ...
        'pitch3 uses front-lift differential (aft lift=0). ' ...
        'T_eng in Newtons; throttle = T/Tmax(M,h).'];

    fprintf('\n');
    fprintf('  exitflag=%d  |res|=%.3e  iters=%d\n', ...
        exitflag, report.res_norm, output.iterations);
    fprintf('  alpha = %.4f deg\n', alpha_deg);
    fprintf('  T_tilt=%.1f  T_liftF=%.1f  T_liftA=%.1f  T_tot=%.1f N\n', ...
        report.T_tilt_N, report.T_lift_front_N, report.T_lift_aft_N, report.T_total_N);
    fprintf('  Q_rigid = %s\n', mat2str(Q(1:6).', 4));
    fprintf('  Qx=%.3g  Qh=%.3g  Qpitch=%.3g\n', Q(1), Q(3), Q(5));
    if report.ok
        fprintf('Cruise trim: OK\n');
    else
        fprintf('Cruise trim: FAILED or weak convergence\n');
    end

    out_dir = fileparts(mfilename('fullpath'));
    out_mat = fullfile(out_dir, 'last_cruise_trim.mat');
    save(out_mat, 'report');
    if strcmpi(mode, 'pitch3')
        save(fullfile(out_dir, 'last_cruise_static_pitch3.mat'), 'report');
    end
    fprintf('  saved %s\n', out_mat);

    if p.Results.write_base
        assignin('base', 'trim_cruise_alpha_rad', report.alpha_rad);
        assignin('base', 'trim_cruise_alpha_deg', report.alpha_deg);
        assignin('base', 'trim_cruise_T_eng_N', report.T_eng_N);
        assignin('base', 'trim_cruise_Q', report.Q);
        assignin('base', 'trim_cruise_report', report);
        assignin('base', 'trim_cruise_Velocity_b', pack.V_b);
        assignin('base', 'trim_cruise_throttle_hint', report.T_eng_N);
    end
end

function v = x_disp_(x)
    v = x(:).';
    v(1) = rad2deg(v(1));
end
