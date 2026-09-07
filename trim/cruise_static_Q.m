function [Q, pack] = cruise_static_Q(alpha_rad, T_eng, ctx)
%CRUISE_STATIC_Q  Static modal GF: aero(alpha) + gravity + prop(T), tilt=0.
%
%  W_strip = Tq*L + Tq_D*D + Tq_CM*(qSc*CM)  (matches Strip Aerodynamics)
%  V_b = U * [cos(alpha); 0; sin(alpha)]  (body, z down)

    U = ctx.U;
    V_b = U * [cos(alpha_rad); 0; sin(alpha_rad)];
    [awr, awl, atr, atl] = strip_local_alphas(V_b, ctx.cant);
    local_a = [awr, awl, atr, atl];

    Qa = zeros(size(ctx.Qg));
    for k = 1:4
        nm = ctx.surface_names{k};
        surf = ctx.aero.(nm);
        a = local_a(k);
        CL = interp1(ctx.polar_a, ctx.polar_CL, rad2deg(a), 'linear', 'extrap');
        CD = interp1(ctx.polar_a, ctx.polar_CD, rad2deg(a), 'linear', 'extrap');
        CM = interp1(ctx.polar_a, ctx.polar_CM, rad2deg(a), 'linear', 'extrap');
        qS = evalin('base', ['qS_strip_' nm]);
        if evalin('base', sprintf('exist(''qSc_strip_%s'',''var'')', nm))
            qSc = evalin('base', ['qSc_strip_' nm]);
        else
            qSc = qS(:) * 2.56;
        end
        Qa = Qa + surf.Tq * (qS(:) * CL) + surf.Tq_D * (qS(:) * CD) ...
            + surf.Tq_CM * (qSc(:) * CM);
    end

    T_eng = T_eng(:);
    if numel(T_eng) == 1
        T_eng = T_eng * ones(8, 1);
    end
    W_prop = engine_pack_wrenches(T_eng, ctx.e_s, ctx.p_eng, ctx.p_eng0);
    Qp = ctx.Tq_prop * W_prop;
    Q = Qa + ctx.Qg + Qp;

    pack = struct();
    pack.Qa = Qa;
    pack.Qg = ctx.Qg;
    pack.Qp = Qp;
    pack.W_prop = W_prop;
    pack.T_eng = T_eng;
    pack.V_b = V_b;
    pack.local_alpha = local_a;
    pack.Alpha_body = alpha_rad;
end
