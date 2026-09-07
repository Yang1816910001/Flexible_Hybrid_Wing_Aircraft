function W_prop = engine_pack_wrenches(T_or_FM, e_s, p_eng, p_eng0, C9, R_sb)
%ENGINE_PACK_WRENCHES  结构系节点扳手 6n×1（折算到网格发动机节点）
%
% 旧接口（配平仍用）：T 为 8×1 标量推力，F = T * e_s。
% 查表接口：FM 为 6×8（或 48×1），桨轴系 [Fx Fy Fz Mx My Mz]，再经 C_eb、R_sb
% 转到结构系。力矩还要加 (p(θ)−p0)×F。不要再加 r_cg×F。

    n = size(e_s, 2);
    d = p_eng - p_eng0;
    if nargin < 5 || isempty(C9)
        T = T_or_FM(:).';
        F = e_s .* T;
        M = cross_cols_(d, F);
    else
        FM = T_or_FM;
        if isvector(FM)
            FM = reshape(FM(:), 6, n);
        end
        F_e = FM(1:3, :);
        M_e = FM(4:6, :);
        F = zeros(3, n);
        M_prop = zeros(3, n);
        for i = 1:n
            C_eb = reshape(C9(:, i), 3, 3);
            F_b = C_eb.' * F_e(:, i);
            M_b = C_eb.' * M_e(:, i);
            F(:, i) = R_sb.' * F_b;
            M_prop(:, i) = R_sb.' * M_b;
        end
        M = cross_cols_(d, F) + M_prop;
    end
    W_prop = zeros(6 * n, 1);
    for i = 1:n
        W_prop((i - 1) * 6 + (1:3)) = F(:, i);
        W_prop((i - 1) * 6 + (4:6)) = M(:, i);
    end
end

function M = cross_cols_(a, b)
    M = [a(2, :).*b(3, :) - a(3, :).*b(2, :);
         a(3, :).*b(1, :) - a(1, :).*b(3, :);
         a(1, :).*b(2, :) - a(2, :).*b(1, :)];
end
