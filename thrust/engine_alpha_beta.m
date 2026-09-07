function [alpha, beta,Va,Vl] = engine_alpha_beta(C9, V_pt)
%ENGINE_ALPHA_BETA  发动机轴系迎角 / 侧滑（rad）
%
% V_eng = C_eb * V_pt
% alpha = atan2(w_eng, u_eng)
% beta  = asin(v_eng / |V|)

    n = size(V_pt, 2);
    alpha = zeros(n, 1);
    beta = zeros(n, 1);
    Va = zeros(n, 1);
    Vl = zeros(n, 1);
    for i = 1:n
        C_eb = reshape(C9(:, i), 3, 3);
        Vn = C_eb * V_pt(:, i);
        Va(i) = Vn(1);
        Vl(i) = hypot(Vn(2), Vn(3));
        spd = norm(Vn);
        if spd < 1e-6
            continue
        end
        alpha(i) = atan2(Vn(3), Vn(1));
        beta(i) = asin(max(-1, min(1, Vn(2) / spd)));
    end
end
