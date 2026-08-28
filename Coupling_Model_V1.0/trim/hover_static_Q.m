function [Q, pack] = hover_static_Q(T_eng, ctx)
%HOVER_STATIC_Q  Hover static modal GF: gravity + prop (tilt=pi/2). Aero neglected (U=0).

    T_eng = T_eng(:);
    if numel(T_eng) == 1
        T_eng = T_eng * ones(8, 1);
    end
    W_prop = engine_pack_wrenches(T_eng, ctx.e_s, ctx.p_eng, ctx.p_eng0);
    Qp = ctx.Tq_prop * W_prop;
    Qa = zeros(size(ctx.Qg));
    Q = Qa + ctx.Qg + Qp;

    pack = struct();
    pack.Qa = Qa;
    pack.Qg = ctx.Qg;
    pack.Qp = Qp;
    pack.W_prop = W_prop;
    pack.T_eng = T_eng;
    pack.V_b = [0; 0; 0];
    pack.Omega_b = [0; 0; 0];
    pack.local_alpha = [0, 0, 0, 0];
    pack.Alpha_body = 0;
    pack.Fz_prop = sum(W_prop(3:6:end));
end
