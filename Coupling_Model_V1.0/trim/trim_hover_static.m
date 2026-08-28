function report = trim_hover_static(varargin)
%TRIM_HOVER_STATIC  Hover static trim: tilt=pi/2, U=0, Newton FD.
%
%  report = trim_hover_static
%  report = trim_hover_static('mode','pitch2')   % default: heave + pitch
%  report = trim_hover_static('mode','shared')   % equal T on all 8

    p = inputParser;
    addParameter(p, 'mode', 'pitch2', @(s) ischar(s) || isstring(s));
    addParameter(p, 'T0_N', [], @(x) isempty(x) || isnumeric(x));
    addParameter(p, 'write_base', true, @islogical);
    addParameter(p, 'tol', 1e-5, @isnumeric);
    addParameter(p, 'max_iter', 40, @isnumeric);
    parse(p, varargin{:});
    mode = char(p.Results.mode);

    fprintf('=== Hover static trim (tilt=pi/2, mode=%s) ===\n', mode);
    ctx = trim_build_hover_context();
    T_guess = ctx.mg / 8;
    if ~isempty(p.Results.T0_N)
        T_guess = p.Results.T0_N;
    end

    switch lower(mode)
        case 'shared'
            x = T_guess;
            n = 1;
        case 'pitch2'
            x = [T_guess; T_guess];
            n = 2;
        otherwise
            error('trim_hover_static:Mode', 'unknown mode %s', mode);
    end

    fun = @(xx) hover_static_residual(xx, ctx, mode);
    fprintf('  m=%.1f kg  mg=%.1f N  T0=%.1f N/eng\n', ctx.m, ctx.mg, T_guess);
    fprintf('  Newton FD solve (n=%d)...\n', n);
    exitflag = 0;
    fval = fun(x);
    it = 0;
    fprintf('  it 0  |res|=%.3e  x=%s\n', norm(fval), mat2str(x(:).', 4));

    for it = 1:p.Results.max_iter
        if norm(fval) < p.Results.tol
            exitflag = 1;
            break
        end
        J = zeros(n, n);
        epsv = max(1, 1e-4 * abs(x(:)));
        for j = 1:n
            xp = x;
            xp(j) = xp(j) + epsv(j);
            J(:, j) = (fun(xp) - fval) / epsv(j);
        end
        if rcond(J) < 1e-14
            warning('trim_hover_static:Singular', 'Jacobian nearly singular');
            exitflag = -1;
            break
        end
        dx = -J \ fval;
        for ls = 1:10
            x_try = x + dx;
            x_try = max(x_try, 0);
            f_try = fun(x_try);
            if norm(f_try) < norm(fval) || ls == 10
                x = x_try;
                fval = f_try;
                break
            end
            dx = 0.5 * dx;
        end
        fprintf('  it %d  |res|=%.3e  x=%s\n', it, norm(fval), mat2str(x(:).', 4));
    end
    if exitflag == 0 && norm(fval) < 1e-3
        exitflag = 1;
    elseif exitflag == 0
        exitflag = -2;
    end
    output = struct('iterations', it, 'method', 'newton_fd');

    [~, Q, pack] = hover_static_residual(x, ctx, mode);
    T_eng = pack.T_eng;

    report = struct();
    report.name = 'trim_hover_static';
    report.mode = mode;
    report.ok = (exitflag > 0) && (norm(fval) < 1e-3);
    report.exitflag = exitflag;
    report.alpha_rad = 0;
    report.alpha_deg = 0;
    report.T_eng_N = T_eng;
    report.T_tilt_N = mean(T_eng(ctx.is_tilt));
    report.T_lift_N = mean(T_eng(~ctx.is_tilt));
    report.T_front_N = mean(T_eng(1:4));
    report.T_aft_N = mean(T_eng(5:8));
    report.T_total_N = sum(T_eng);
    report.Q = Q;
    report.Q_rigid = Q(1:6);
    report.res = fval;
    report.res_norm = norm(fval);
    report.Q_pitch = Q(5);
    report.pack = pack;
    report.ctx_mg = ctx.mg;
    report.U = 0;
    report.mach = 0;
    report.Altitude_m = ctx.Altitude_m;
    report.tilt_rad = ctx.tilt;
    report.fsolve_output = output;
    report.note = [ ...
        'Hover: tilt=pi/2, U=0, aero neglected. ' ...
        'pitch2: front(1-4) vs aft(5-8) thrust for heave+pitch. ' ...
        'T_eng in Newtons; throttle = T/(Fmax*Nt).'];

    fprintf('\n');
    fprintf('  exitflag=%d  |res|=%.3e  iters=%d\n', ...
        exitflag, report.res_norm, output.iterations);
    fprintf('  T_front=%.1f  T_aft=%.1f  T_tot=%.1f N  Fz_prop=%.1f N\n', ...
        report.T_front_N, report.T_aft_N, report.T_total_N, pack.Fz_prop);
    fprintf('  Q_rigid = %s\n', mat2str(Q(1:6).', 4));
    fprintf('  Qx=%.3g  Qh=%.3g  Qpitch=%.3g\n', Q(1), Q(3), Q(5));
    if report.ok
        fprintf('Hover trim: OK\n');
    else
        fprintf('Hover trim: FAILED or weak convergence\n');
    end

    out_dir = fileparts(mfilename('fullpath'));
    out_mat = fullfile(out_dir, 'last_hover_trim.mat');
    save(out_mat, 'report');
    fprintf('  saved %s\n', out_mat);

    if p.Results.write_base
        assignin('base', 'trim_hover_alpha_rad', 0);
        assignin('base', 'trim_hover_T_eng_N', report.T_eng_N);
        assignin('base', 'trim_hover_Q', report.Q);
        assignin('base', 'trim_hover_report', report);
        assignin('base', 'trim_hover_Velocity_b', pack.V_b);
        assignin('base', 'trim_hover_tilt_rad', report.tilt_rad);
    end
end
